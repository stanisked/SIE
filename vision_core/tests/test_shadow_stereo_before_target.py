"""The evidence audit must retain stereo even when RGB selection refuses."""
from __future__ import annotations

import ast
from datetime import datetime, timezone
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


SOURCE = (Path(__file__).resolve().parents[2] /
          "ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py")


def cycle_node():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in ("_cycle", "_prepare_stereo_pair", "_frustum_associate")]
    namespace = {"np": np, "datetime": datetime, "timezone": timezone,
                 "time": __import__("time"),
                 "StageTimings": lambda: SimpleNamespace(),
                 "ContractError": ValueError,
                 "rclpy": SimpleNamespace(ok=lambda: True),
                 "validate_ar0234_observation": lambda raw: raw,
                 "ar0234_temporal_target_candidate": lambda *a, **k:
                     {"disposition": "REFUSED", "reason": "NO_TARGET",
                      "temporal_track": {}}}
    module = ast.Module(body=[ast.ImportFrom(
        module="__future__", names=[ast.alias(name="annotations")], level=0), cls],
        type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace[cls.name]()


class StereoBeforeTargetTests(unittest.TestCase):
    def test_rejected_rgb_keeps_same_cycle_stereo_without_association(self):
        node = cycle_node()
        events = []
        frame = np.zeros((1200, 1920, 3), dtype=np.uint8)
        combined = np.zeros((800, 2560, 3), dtype=np.uint8)
        node.sequence = 0
        node.run_id = "test"
        node.stereo_size = (1280, 800)
        node.max_pair_skew_ms = 50
        node._confirmed_target_track = None
        node.temporal_target_track_max_age_s = 5
        node.temporal_target_minimum_iou = .2
        node.temporal_target_maximum_center_displacement_relative = .75
        node.temporal_target_minimum_match_margin = .1
        node.evidence_capture_enabled = True
        node.ar_capture, node.stereo_capture = "ar", "stereo"
        node._read = lambda source: (frame if source == "ar" else combined, 1_000_000)
        node._timed_call = lambda name, fn, *a, **kw: fn(*a, **kw)
        node._audit_base = lambda cycle, *args: {"cycle_id": cycle}
        node._begin_cycle_evidence = lambda *args: events.append("begin")
        node._prepare_stereo_pair = lambda *args: (events.append("stereo") or
            ("left", "right", "disparity", "mask"))
        node.observer = SimpleNamespace(observe=lambda *a, **k: {"detections": []})
        node._publish_observation = lambda *args: events.append("rgb")
        node._render_ar_live_preview = lambda *args, **kw: frame
        node._write_live_ar_preview = lambda *args: None
        node._frustum_associate = lambda *a, **kw: self.fail("association called after refusal")
        audits = []
        node._publish_audit = audits.append
        node._cycle()
        self.assertEqual(events, ["begin", "stereo", "rgb"])
        self.assertEqual(audits[0]["status"], "TARGET_UNAVAILABLE")
        self.assertIsNone(audits[0]["stereo_geometry"])
        self.assertEqual(audits[0]["stereo_pair_evidence"]["source_cycle_id"],
                         audits[0]["cycle_id"])
        self.assertFalse(audits[0]["stereo_pair_evidence"]["metric_measurement_authorized"])

    def test_stereo_preparation_saves_one_pair_and_one_disparity(self):
        node = cycle_node()
        calls = []
        node.stereo_size = (4, 2)
        node.map_left_1 = node.map_left_2 = node.map_right_1 = node.map_right_2 = None
        node.cv2 = SimpleNamespace(
            INTER_LINEAR=1, COLOR_BGR2GRAY=2,
            remap=lambda frame, *_: frame,
            cvtColor=lambda frame, *_: frame[:, :, 0])
        node._timed_call = lambda _, fn, *a: fn(*a)
        node._persist_rectified_evidence = lambda *a: calls.append("rectified")
        node._left_right_disparity = lambda *a: (
            calls.append("matcher") or (np.ones((2, 4)), np.ones((2, 4), bool)))
        node._persist_disparity_evidence = lambda *a: calls.append("disparity")
        result = node._prepare_stereo_pair(np.zeros((2, 8, 3), np.uint8))
        self.assertEqual(calls, ["rectified", "matcher", "disparity"])
        self.assertEqual(len(result), 4)

    def test_association_reuses_prepared_stereo_without_second_match(self):
        node = cycle_node()
        node.np = np
        node._ar_inner_roi = lambda _: (0, 0, 64, 64)
        node._stereo_frustum_search_roi = lambda _: (0, 0, 64, 64)
        node._left_right_disparity = lambda *a: self.fail("second matcher call")
        node._persist_disparity_evidence = lambda *a: self.fail("duplicate evidence")
        node._write_frustum_debug = lambda *a: None
        node._depth_range_diagnostic = lambda *a: {}
        node.frustum_sample_stride_px = 4
        node.p2 = np.zeros((3, 4)); node.p2[0, 3] = -75954.188
        node.range_min_m, node.range_max_m, node.min_depth_samples = .5, 2., 100
        blank = np.zeros((64, 64, 3), np.uint8)
        association, reason, _ = node._frustum_associate(
            {}, blank, blank, prepared_stereo=(
                np.full((64, 64), 33., dtype=np.float32), np.ones((64, 64), bool)))
        self.assertIsNone(association)
        self.assertEqual(reason, "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE")


if __name__ == "__main__":
    unittest.main()
