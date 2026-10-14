"""Real SGBM parity and source-preserving offline CLI, without ROS/cameras."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from benchmark_stereo_sgbm_parallel import runtime_parameters, compute_pair, consistency, compare


class ParallelBenchmarkTests(unittest.TestCase):
    def test_configuration_is_extracted_from_both_actual_matchers(self):
        configs, hashes = runtime_parameters()
        self.assertEqual([c["minDisparity"] for c in configs], [0, -128])
        self.assertEqual([c["numDisparities"] for c in configs], [128, 128])
        self.assertEqual(configs[0]["P1"], 1176)
        self.assertEqual(configs[0]["mode"], cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.assertEqual(len(hashes), 2)

    def test_real_sequential_parallel_maps_and_masks_match_at_one_and_two_threads(self):
        configs, _ = runtime_parameters()
        left = np.random.default_rng(7).integers(0, 256, (80, 320), dtype=np.uint8)
        right = np.roll(left, -8, axis=1)
        originals = (left.copy(), right.copy())
        old_threads = cv2.getNumThreads()
        try:
            reference = None
            with ThreadPoolExecutor(max_workers=2) as executor:
                for threads in (1, 2):
                    cv2.setNumThreads(threads)
                    matchers = [cv2.StereoSGBM_create(**p) for p in configs]
                    sequential, _ = compute_pair(matchers, left, right)
                    parallel, _ = compute_pair(matchers, left, right, executor)
                    self.assertTrue(compare(sequential, parallel)["exact_match"])
                    if reference is not None:
                        self.assertTrue(compare(reference, parallel)["exact_match"])
                    reference = sequential
                    self.assertGreater(np.count_nonzero(consistency(parallel)), 100)
        finally:
            cv2.setNumThreads(old_threads)
        np.testing.assert_array_equal(left, originals[0])
        np.testing.assert_array_equal(right, originals[1])

    def test_one_disparity_change_is_detected(self):
        left = np.full((8, 32), 8 * 16, dtype=np.int16)
        right = -left.copy()
        changed = right.copy()
        changed[3, 7] += 32
        result = compare((left, right), (left, changed))
        self.assertFalse(result["exact_match"])
        self.assertEqual(result["right_different_pixels"], 1)
        self.assertEqual(result["lr_mask_different_pixels"], 1)

    def test_full_cli_uses_saved_sources_and_refuses_existing_output(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            case = root / "synthetic-00000001"
            case.mkdir()
            left = np.random.default_rng(9).integers(0, 256, (800, 1280, 3), dtype=np.uint8)
            right = np.roll(left, -8, axis=1)
            for side, image in (("left", left), ("right", right)):
                self.assertTrue(cv2.imwrite(str(case / f"ov9281_physical_{side}_rectified.jpg"), image))
            (case / "audit.json").write_text(json.dumps({"cycle_id": case.name, "calibration": {"synthetic": True}}))
            originals = {p: p.read_bytes() for p in case.iterdir()}
            output = root / "comparison.json"
            cmd = [sys.executable, str(ROOT / "vision_core/tools/benchmark_stereo_sgbm_parallel.py"),
                   "--evidence-root", str(root), "--output", str(output),
                   "--cycles", "1", "--repeats", "1", "--opencv-thread-counts", "1"]
            run = subprocess.run(cmd, text=True, capture_output=True, env=os.environ.copy())
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            report = json.loads(output.read_text())
            self.assertTrue(report["all_exact"])
            self.assertTrue(report["inputs_unchanged"])
            self.assertEqual(len(report["rows"]), 2)
            self.assertEqual(originals, {p: p.read_bytes() for p in originals})
            before = output.read_bytes()
            retry = subprocess.run(cmd, text=True, capture_output=True)
            self.assertNotEqual(retry.returncode, 0)
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
