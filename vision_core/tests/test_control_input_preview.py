"""Control-path scheduling, journal backpressure and task-budget preview tests."""
import ast
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2/sie_ros2/sie_ros2"))
from latest_diagnostic_writer import LatestDiagnosticWriter
from approach_control_preview import approach_control_preview


def preview_node_class():
    path = ROOT / "ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py"
    tree = ast.parse(path.read_text())
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ("_cycle", "_left_right_disparity")]
    scope = {"rclpy": SimpleNamespace(ok=lambda: True), "time": time,
             "datetime": datetime, "timezone": timezone,
             "String": SimpleNamespace, "encode": json.dumps,
             "validate_ar0234_observation": lambda v: v,
             "ar0234_temporal_target_candidate": lambda raw, **kw: raw,
             "approach_control_preview": approach_control_preview}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), "exec"), scope)
    return scope[cls.name]


class ControlPreviewTests(unittest.TestCase):
    def test_slow_writer_drops_pending_record_without_blocking_producer(self):
        started, release = threading.Event(), threading.Event()
        recorded = []
        def slow_write(value):
            if not recorded:
                started.set()
                if not release.wait(3):
                    raise RuntimeError("test release timeout")
            recorded.append(value)
        with tempfile.TemporaryDirectory() as name:
            writer = LatestDiagnosticWriter(Path(name) / "records", write=slow_write)
            try:
                writer.submit({"cycle_id": "one"})
                self.assertTrue(started.wait(2))
                writer.submit({"cycle_id": "two"})
                last = {"cycle_id": "three", "value": [3]}
                writer.submit(last)
                last["value"][0] = 99
                self.assertEqual(writer.pending.qsize(), 1)
                self.assertEqual(writer.dropped, 1)
            finally:
                release.set()
                writer.close()
            self.assertEqual([r["cycle_id"] for r in recorded], ["one", "three"])
            self.assertEqual(recorded[-1]["value"], [3])
            self.assertFalse(writer.submit({"cycle_id": "after-close"}))

    def test_journal_error_is_counted_not_a_foreground_exception(self):
        with tempfile.TemporaryDirectory() as name:
            def fail(_):
                raise OSError("disk unavailable")
            writer = LatestDiagnosticWriter(Path(name) / "records", write=fail)
            writer.submit({"cycle_id": "one"})
            writer.close()
            self.assertEqual(writer.errors, 1)
            self.assertEqual(writer.last_error, "OSError")

    def test_unknown_inputs_never_become_an_approach(self):
        result = approach_control_preview()
        self.assertEqual(result["status"], "STOP")
        self.assertIn("EXPOSURE_AGE_UNKNOWN_OR_INVALID", result["reasons"])
        self.assertFalse(result["motor_command_performed"])

    def test_admitted_synthetic_input_drives_working_speed_preview_and_stale_stops(self):
        values = dict(admitted_measurement=True, measurement_id="synthetic",
                      clearance_m=1.5, clearance_error_bound_m=.1, exposure_age_s=.25,
                      closing_speed_bound_m_s=.15, target_closing_speed_bound_m_s=0.,
                      measured_stop_distance_bound_m=.025, measured_stop_time_bound_s=.5,
                      aligned_in_robot_frame=True)
        result = approach_control_preview(**values)
        self.assertEqual(result["status"], "APPROACH")
        self.assertEqual(result["recommended_speed_m_s"], .15)
        self.assertAlmostEqual(result["required_clearance_m"], 1.2625)
        self.assertFalse(result["motor_command_performed"])
        self.assertEqual(approach_control_preview(**{**values, "exposure_age_s": .251})["status"], "STOP")
        self.assertEqual(approach_control_preview(**{**values, "clearance_m": 1.26})["status"], "STOP")
        moving = approach_control_preview(**{**values, "closing_speed_bound_m_s": 1., "target_closing_speed_bound_m_s": .85})
        self.assertEqual(moving["status"], "STOP")

    def test_new_cycle_never_uses_previous_disparity_and_publish_precedes_journal(self):
        node = preview_node_class()()
        with self.assertRaises(RuntimeError):
            node._cached_stereo = None
            node._left_right_disparity(None, None)
        node.sequence, node.run_id = 0, "test"
        node.stereo_size = (1280, 800)
        node.max_pair_skew_ms = 80
        node.range_min_m, node.range_max_m, node._activated_range = .5, 4.5, [.5, 2.]
        node._confirmed_target_track = None
        node.temporal_target_track_max_age_s = 5
        node.temporal_target_minimum_iou = .2
        node.temporal_target_maximum_center_displacement_relative = .75
        node.temporal_target_minimum_match_margin = .1
        node.ar_capture, node.stereo_capture = "ar", "stereo"
        node._read = lambda cap: (np.zeros((1200,1920,3) if cap == "ar" else (800,2560,3), np.uint8), time.monotonic_ns())
        node._audit_base = lambda cid, *_: {"cycle_id": cid, "stereo_execution": {},
            "metric_measurement_authorized": False, "navigation_eligible": False, "actuator_access": False}
        stereo_started, detector_started = threading.Event(), threading.Event()
        maps = object()
        def stereo(_):
            stereo_started.set()
            if not detector_started.wait(2):
                raise RuntimeError("detector did not overlap stereo")
            return object(), object(), maps, .1
        def detect(*_, **kw):
            detector_started.set()
            if not stereo_started.wait(2):
                raise RuntimeError("stereo did not overlap detector")
            return {"disposition": "OBSERVED", "candidate": {}}
        node._compute_stereo = stereo
        node.observer = SimpleNamespace(observe=detect)
        node._publish_observation = lambda _: None
        node._dynamic_target_geometry = lambda *_: {"accepted": True}
        node._selected_observation = lambda *args: {}
        def associate(*args):
            self.assertIs(node._left_right_disparity(None, None), maps)
            return {"range_m": 2.5}, "SUCCESS", {}
        node._frustum_associate = associate
        node._confirm_current_target_track = lambda *_: None
        events, messages = [], []
        node.preview_publisher = SimpleNamespace(publish=lambda msg: (events.append("publish"), messages.append(json.loads(msg.data))))
        node._writer = SimpleNamespace(dropped=0, errors=0, submit=lambda _: events.append("journal"))
        node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(events, ["publish", "journal"])
        self.assertEqual(messages[0]["status"], "ASSOCIATED_DIAGNOSTIC_ONLY")
        self.assertEqual(messages[0]["decision_preview"]["status"], "STOP")
        self.assertFalse(messages[0]["actuator_access"])
        self.assertIsNone(node._cached_stereo)
        # A global-MAD refusal must not prevent current-frame shape analysis.
        node.np = np
        node.p1 = node.p2 = np.eye(3,4)
        node.r1 = node.r_ar_to_left = node.k_ar = np.eye(3)
        node.t_ar_to_left_m = np.zeros((3,1))
        node.d_ar = np.zeros(5)
        node._ar_inner_roi = lambda _: [10,10,100,100]
        maps = (np.ones((4,4),np.float32), np.ones((4,4),bool))
        node._frustum_associate = lambda *args: (None, "DEPTH_MAD_EXCEEDED:0.06m", {})
        calls = []
        def masked(*args, **kwargs):
            calls.append((args, kwargs))
            return {"status":"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY"}
        node._mask_client = SimpleNamespace(evaluate=masked)
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(len(calls),1)
        self.assertEqual(calls[0][0][0],messages[-1]["cycle_id"])
        self.assertEqual(messages[-1]["original_association"]["reason"],"DEPTH_MAD_EXCEEDED:0.06m")
        self.assertEqual(messages[-1]["shape_diagnostic"]["status"],"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY")
        self.assertEqual(messages[-1]["decision_preview"]["status"],"STOP")
        # Half maps must bypass full-resolution association entirely, while
        # keeping current masks and the scale explicit in the worker request.
        node._stereo_scale = .5
        node._resolution_mode = "half_640x400"
        def forbidden(*args):
            raise AssertionError("small disparity entered full-resolution association")
        node._frustum_associate = forbidden
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(calls[-1][1], {"scale":.5})
        self.assertEqual(messages[-1]["stereo_resolution"]["matching_size_px"],[640,400])
        self.assertEqual(messages[-1]["original_association"]["reason"],"NOT_EVALUATED_IN_HALF_RESOLUTION_MODE")
        self.assertEqual(messages[-1]["status"],"SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY")
        self.assertIsNone(messages[-1]["association"])
        self.assertEqual(messages[-1]["decision_preview"]["status"],"STOP")
        # A later no-target cycle must not reuse the previous association.
        node.observer = SimpleNamespace(observe=lambda *args, **kw: {"disposition": "UNAVAILABLE", "reason": "NO_TARGET"})
        with ThreadPoolExecutor(max_workers=1) as worker:
            node._perception_worker = worker
            node._cycle()
        self.assertEqual(messages[-1]["status"], "TARGET_UNAVAILABLE")
        self.assertIsNone(messages[-1]["association"])
        self.assertIsNone(node._cached_stereo)


if __name__ == "__main__":
    unittest.main()
