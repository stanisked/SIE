"""Test independent-range execution and refusal preservation without ROS/cameras."""
from __future__ import annotations

import ast
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from shadow_local_surface_request import process_surface_request
from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from vision_core.tests.test_shadow_depth_regions import fixture


def node_methods():
    # Execute the actual changed methods without importing unavailable ROS.
    source = ROOT / "ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    names = {"_shadow_local_surfaces", "_publish_audit", "_frustum_associate"}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[])
    scope = {"time": time, "Path": Path, "json": json, "hashlib": hashlib,
             "encode": json.dumps, "String": SimpleNamespace,
             "rclpy": SimpleNamespace(ok=lambda: True)}
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), scope)
    return scope[cls.name]


def bound_fixture(base):
    cid = "synthetic-00000003"
    cycle = base / cid
    cycle.mkdir()
    path, _, cal = fixture(cycle)
    runtime = base / "runtime"
    runtime.mkdir()
    ar, stereo, ex = [runtime / n for n in ("ar.json", "stereo.npz", "ex.npz")]
    ar.write_text(json.dumps({"camera_matrix": cal["K"].tolist(), "distortion_coefficients": cal["D"].tolist()}))
    np.savez(stereo, camera_1_semantics="physical_left", camera_2_semantics="physical_right",
             size=[64, 64], R1=cal["R1"], P1=cal["P1"], P2=cal["P2"])
    np.savez(ex, source_stereo_calibration_sha256=sha256(stereo), source_ar_intrinsic_sha256=sha256(ar),
             R_ar0234_to_physical_left=cal["R"], T_ar0234_to_physical_left_mm=cal["T"] * 1000)
    cal = load_calibrations(ar, stereo, ex)
    model = "a" * 64
    (cycle / f"{cid}.json").write_text(json.dumps({"cycle_id": cid, "reference_frame": "ar0234_image_frame",
        "source_ar0234_sha256": sha256(cycle / "ar0234_source.jpg"),
        "mask_sha256": sha256(cycle / f"{cid}.png"), "model_sha256": model}))
    audit = json.loads(path.read_text())
    audit.update(calibration=cal["sha"], status="ASSOCIATION_UNAVAILABLE", reason="FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE",
                 shadow_instance_mask={"status": "MASK_SAVED_DIAGNOSTIC_ONLY", "worker_model_sha256": model})
    path.write_text(json.dumps(audit))
    snapshot = cycle / "shadow_surface_input.json"
    snapshot.write_text(json.dumps(audit))
    activation = runtime / "activation.json"
    activation.write_text(json.dumps({"authorization": {"metric_measurement_range_m": [.5, 2.]},
        "calibration_profile": {"source_sha256": {"ar0234_intrinsic": sha256(ar), "ov9281_stereo_calibration": sha256(stereo)},
            "ar0234_to_ov9281_physical_left_extrinsic": {"path": str(ex), "sha256": sha256(ex)}}}))
    req = {"cycle_id": cid, "snapshot": str(snapshot), "snapshot_sha256": sha256(snapshot),
           "disparity_sha256": sha256(cycle / "disparity_lr_consistent_px.npy"),
           "diagnostic_depth_range_m": [.5, 4.5], "activation": str(activation),
           "activation_sha256": sha256(activation), "ar_intrinsic": str(ar), "stereo_calibration": str(stereo)}
    return req, path, audit


class LiveShadowTests(unittest.TestCase):
    def test_original_range_refusal_does_not_block_surface_analysis(self):
        with tempfile.TemporaryDirectory() as name:
            req, path, audit = bound_fixture(Path(name))
            originals = {p: p.read_bytes() for p in Path(name).rglob("*") if p.is_file()}
            result = process_surface_request(req)
            self.assertEqual(result["activated_depth_range_m"], [.5, 2.])
            self.assertEqual(result["diagnostic_depth_range_m"], [.5, 4.5])
            self.assertGreater(result["mask_depth"]["candidate_depth_m"], 2.)
            self.assertEqual(result["local_surface"]["fitted_patch_count"], 4)
            self.assertEqual(result["local_surface"]["unfitted_patch_fraction"], 0.)
            self.assertEqual(originals, {p: p.read_bytes() for p in originals})
            self.assertEqual(json.loads(path.read_text()), audit)
            self.assertFalse(result["metric_measurement_authorized"])
            self.assertFalse(result["actuator_access"])
            with self.assertRaises(FileExistsError):
                process_surface_request(req)

    def test_corrupt_disparity_or_mask_and_invalid_range_refuse(self):
        for kind in ("disparity", "mask", "range", "cycle", "activation"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as name:
                req, path, _ = bound_fixture(Path(name))
                if kind == "disparity":
                    req["disparity_sha256"] = "bad"
                elif kind == "mask":
                    (path.parent / f"{req['cycle_id']}.png").write_bytes(b"bad")
                elif kind == "range":
                    req["diagnostic_depth_range_m"] = [.5, float("nan")]
                elif kind == "cycle":
                    req["cycle_id"] = "other"
                else:
                    req["activation_sha256"] = "bad"
                with self.assertRaises(ValueError):
                    process_surface_request(req)

    def test_worker_timeout_does_not_change_original_outcome(self):
        with tempfile.TemporaryDirectory() as name:
            req, path, audit = bound_fixture(Path(name))
            Path(req["snapshot"]).unlink()  # Node creates its own immutable snapshot.
            node = node_methods()()
            node.shadow_local_surfaces_enabled = True
            node._active_evidence_cycle_dir = path.parent
            node._shadow_mask_file = path.parent / f"{req['cycle_id']}.png"
            node._shadow_process = SimpleNamespace(stdin=io.StringIO())
            node._active_evidence_files = []
            node.shadow_diagnostic_range = [.5, 4.5]
            node.activation_path = Path(req["activation"])
            node.ar_intrinsic_path = Path(req["ar_intrinsic"])
            node.stereo_path = Path(req["stereo_calibration"])
            node.shadow_segmenter_timeout_s = .1
            node._shadow_read = lambda _: (_ for _ in ()).throw(TimeoutError("test timeout"))
            stopped = []
            node._stop_shadow_worker = lambda: stopped.append(True)
            before = json.dumps(audit)
            result = node._shadow_local_surfaces(audit)
            self.assertEqual(result["status"], "LOCAL_SURFACE_UNAVAILABLE")
            self.assertEqual(json.dumps(audit), before)
            self.assertEqual(stopped, [True])

    def test_original_range_gate_still_refuses_same_samples(self):
        node = node_methods()()
        node.np, node.cv2 = np, cv2
        node._ar_inner_roi = lambda _: (0, 0, 64, 64)
        node._stereo_frustum_search_roi = lambda _: (0, 0, 64, 64)
        node._left_right_disparity = lambda *_: (np.full((64, 64), 33., dtype=np.float32), np.ones((64, 64), bool))
        node._persist_disparity_evidence = lambda *_: None
        node._write_frustum_debug = lambda *_: None
        node._depth_range_diagnostic = lambda *_: {}
        node.frustum_sample_stride_px = 4
        node.p2 = np.zeros((3, 4)); node.p2[0, 3] = -75954.188
        node.range_min_m, node.range_max_m, node.min_depth_samples = .5, 2., 100
        image = np.zeros((64, 64, 3), np.uint8)
        association, reason, geometry = node._frustum_associate({}, image, image)
        self.assertIsNone(association)
        self.assertEqual(reason, "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE")
        self.assertEqual(geometry["in_range_depth_sample_count"], 0)

    def test_early_refusal_records_timing_and_not_run_without_a_measurement(self):
        node = node_methods()()
        node.shadow_local_surfaces_enabled = True
        node._cycle_started_mono = time.monotonic() - .02
        node._attach_evidence_record = lambda a: None
        messages = []
        node.audit_publisher = SimpleNamespace(publish=lambda m: messages.append(json.loads(m.data)))
        node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
        node._publish_audit({"cycle_id": "test-00000020", "status": "TARGET_UNAVAILABLE", "reason": "NO_TARGET"})
        self.assertEqual(messages[0]["status"], "TARGET_UNAVAILABLE")
        self.assertEqual(messages[0]["shadow_local_surfaces"]["status"], "NOT_RUN")
        self.assertGreater(messages[0]["processing_timing"]["host_capture_start_to_result_ready_s"], .019)
        self.assertFalse(messages[0]["processing_timing"]["exposure_age_known"])

    def test_disabled_path_never_contacts_worker(self):
        node = node_methods()()
        node.shadow_local_surfaces_enabled = False
        self.assertEqual(node._shadow_local_surfaces({}), {"status": "DISABLED"})


if __name__ == "__main__":
    unittest.main()
