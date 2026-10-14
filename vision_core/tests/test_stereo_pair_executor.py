"""Test actual live matcher dispatch, timing accounting and worker lifecycle."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2/sie_ros2/sie_ros2"))
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from stereo_pair_executor import StereoPairExecutor
from diagnostic_timing import StageTimings
from benchmark_stereo_sgbm_parallel import runtime_parameters
from vision_core.tests.test_live_shadow_local_surfaces import node_methods


class StereoPairExecutorTests(unittest.TestCase):
    def test_same_matcher_cannot_be_shared(self):
        matcher = object()
        with self.assertRaises(ValueError):
            StereoPairExecutor(matcher, matcher)

    def test_actual_live_lr_path_matches_and_records_parallel_wall_once(self):
        configs, _ = runtime_parameters()
        before = cv2.getNumThreads()
        worker = None
        try:
            cv2.setNumThreads(1)
            left = np.random.default_rng(14).integers(0, 256, (80, 320), dtype=np.uint8)
            right = np.roll(left, -8, axis=1)
            node = node_methods()()
            node.np = np
            node.sgbm, node.right_sgbm = [cv2.StereoSGBM_create(**p) for p in configs]
            node.lr_consistency_max_delta_px = 1.5
            node._stage_timings = StageTimings()
            sequential = node._left_right_disparity(left, right)
            seq_stages = node._stage_timings.snapshot()["stages"]
            self.assertIn("stereo_match_left", seq_stages)
            self.assertIn("stereo_match_right", seq_stages)
            worker = StereoPairExecutor(node.sgbm, node.right_sgbm)
            node._stereo_pair_executor = worker
            node._stage_timings = StageTimings()
            for _ in range(3):
                parallel = node._left_right_disparity(left, right)
                for a, b in zip(sequential, parallel):
                    np.testing.assert_array_equal(a, b)
            stages = node._stage_timings.snapshot()["stages"]
            self.assertEqual(stages["stereo_match_pair_parallel"]["calls"], 3)
            self.assertNotIn("stereo_match_left", stages)
            self.assertNotIn("stereo_match_right", stages)
            self.assertTrue(node._last_stereo_worker_timings["overlap_possible"])
        finally:
            if worker is not None:
                worker.close()
            cv2.setNumThreads(before)

    def test_failure_drains_other_task_before_returning(self):
        second_started, release, first_failed = threading.Event(), threading.Event(), threading.Event()
        def fail(*_):
            first_failed.set()
            raise ValueError("synthetic matcher error")
        def finish(*_):
            second_started.set()
            if not release.wait(3):
                raise RuntimeError("test release timeout")
            return "finished"
        worker = StereoPairExecutor(SimpleNamespace(compute=fail), SimpleNamespace(compute=finish))
        try:
            with ThreadPoolExecutor(max_workers=1) as caller:
                result = caller.submit(worker.compute, object(), object())
                try:
                    self.assertTrue(second_started.wait(2))
                    self.assertTrue(first_failed.wait(2))
                    self.assertFalse(result.done())
                finally:
                    release.set()
                with self.assertRaises(RuntimeError) as caught:
                    result.result(timeout=3)
                self.assertIsInstance(caught.exception.__cause__, ValueError)
        finally:
            release.set()
            worker.close()

    def test_shutdown_is_idempotent_and_closed_pool_refuses(self):
        left = SimpleNamespace(compute=lambda a, b: a)
        right = SimpleNamespace(compute=lambda a, b: a)
        worker = StereoPairExecutor(left, right)
        a, b, times = worker.compute("left", "right")
        self.assertEqual((a, b), ("left", "right"))
        self.assertGreaterEqual(times["left_compute_s"], 0)
        worker.close()
        worker.close()
        with self.assertRaises(RuntimeError):
            worker.compute(None, None)


if __name__ == "__main__":
    unittest.main()
