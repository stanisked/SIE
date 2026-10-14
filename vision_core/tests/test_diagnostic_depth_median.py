"""Offline checks for ownership boundaries, missing data and replay reproducibility."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from vision_core.person_localization.diagnostic_depth_median import masked_median3x3


class MedianDepthTests(unittest.TestCase):
    def test_isolated_spike_corrected_without_mutating_source(self):
        depth = np.full((7, 7), 2.3)
        depth[3, 3] = 4.0
        original = depth.copy()
        filtered, changed, stats = masked_median3x3(depth, np.ones(depth.shape, dtype=bool))
        np.testing.assert_array_equal(depth, original)
        self.assertEqual(filtered[3, 3], 2.3)
        self.assertEqual(int(changed.sum()), 1)
        self.assertEqual(stats["evaluated_center_count"], 25)

    def test_missing_points_and_ownership_boundary_are_preserved(self):
        depth = np.full((7, 9), 2.3)
        depth[3, 3] = 4.0
        depth[3, 4] = np.nan
        depth[3, 6] = 4.0
        mask = np.ones(depth.shape, dtype=bool)
        mask[:, 7:] = False
        filtered, changed, _ = masked_median3x3(depth, mask)
        self.assertTrue(np.isnan(filtered[3, 4]))
        self.assertEqual(filtered[3, 3], 4.0)  # Window touches an invalid point.
        self.assertEqual(filtered[3, 6], 4.0)  # Window crosses instance ownership.
        np.testing.assert_array_equal(filtered[:, 7:], depth[:, 7:])
        self.assertFalse(changed[:, 7:].any())

    def test_straight_depth_edge_and_linear_slope_remain(self):
        depth = np.full((7, 8), 2.0)
        depth[:, 4:] = 4.0
        filtered, _, _ = masked_median3x3(depth, np.ones(depth.shape, dtype=bool))
        np.testing.assert_array_equal(filtered, depth)
        slope = np.add.outer(np.arange(7) * 0.01, np.arange(8) * 0.02) + 2
        filtered, _, _ = masked_median3x3(slope, np.ones(slope.shape, dtype=bool))
        np.testing.assert_array_equal(filtered, slope)

    def test_thin_internal_surface_can_be_erased(self):
        # Deliberately documents a limitation: lower MAD is not accuracy proof.
        depth = np.full((7, 7), 3.0)
        depth[1:6, 3] = 2.0
        filtered, _, _ = masked_median3x3(depth, np.ones(depth.shape, dtype=bool))
        self.assertEqual(filtered[3, 3], 3.0)

    def test_tiny_map_and_invalid_values_are_unchanged(self):
        depth = np.array([[np.nan, 0.0], [-1.0, np.inf]])
        filtered, changed, stats = masked_median3x3(depth, np.ones(depth.shape, dtype=bool))
        np.testing.assert_array_equal(filtered, depth)
        self.assertFalse(changed.any())
        self.assertEqual(stats["eligible_pixel_count"], 0)

    def test_reject_nonboolean_ownership(self):
        with self.assertRaises(ValueError):
            masked_median3x3(np.ones((3, 3)), np.ones((3, 3), dtype=np.uint8))


class ReplayIntegrationTests(unittest.TestCase):
    def test_cli_replays_baseline_and_preserves_sources(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV is required for geometric replay")
        from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
        from replay_shadow_mask_depth_range import replay, summary

        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            ar = base / "ar.json"
            stereo = base / "stereo.npz"
            extrinsic = base / "extrinsic.npz"
            p1 = np.array([[100., 0, 32, 0], [0, 100, 32, 0], [0, 0, 1, 0]])
            p2 = p1.copy()
            p2[0, 3] = -75954.18816496264
            k = [[100., 0, 960], [0, 100, 600], [0, 0, 1]]
            ar.write_text(json.dumps({"camera_matrix": k, "distortion_coefficients": [0.] * 5}))
            np.savez(stereo, camera_1_semantics="physical_left", camera_2_semantics="physical_right",
                     size=[64, 64], R1=np.eye(3), P1=p1, P2=p2)
            np.savez(extrinsic, source_stereo_calibration_sha256=sha256(stereo),
                     source_ar_intrinsic_sha256=sha256(ar), R_ar0234_to_physical_left=np.eye(3),
                     T_ar0234_to_physical_left_mm=np.zeros(3))
            cal = load_calibrations(ar, stereo, extrinsic)
            activation = base / "activation.json"
            activation.write_text(json.dumps({"authorization": {"metric_measurement_range_m": [0.5, 2.0]},
                "calibration_profile": {"source_sha256": {"ar0234_intrinsic": sha256(ar),
                    "ov9281_stereo_calibration": sha256(stereo)},
                    "ar0234_to_ov9281_physical_left_extrinsic": {"path": str(extrinsic),
                                                               "sha256": sha256(extrinsic)}}}))
            root = base / "source"
            cycle = root / "cycle1"
            cycle.mkdir(parents=True)
            cid = "synthetic-00000001"
            rgb = np.zeros((1200, 1920, 3), dtype=np.uint8)
            self.assertTrue(cv2.imwrite(str(cycle / "ar0234_source.jpg"), rgb))
            mask = np.zeros((1200, 1920), dtype=np.uint8)
            mask[550:650, 900:1030] = 255
            mask_path = cycle / f"{cid}.png"
            self.assertTrue(cv2.imwrite(str(mask_path), mask))
            cv2.imwrite(str(cycle / "ov9281_physical_left_rectified.jpg"),
                        np.zeros((64, 64, 3), dtype=np.uint8))
            disparity = np.full((64, 64), 75.95418816496264 / 2.3, dtype=np.float32)
            disparity[32, 32] = 75.95418816496264 / 3.8
            np.save(cycle / "disparity_lr_consistent_px.npy", disparity)
            model_sha = "1" * 64
            (cycle / f"{cid}.json").write_text(json.dumps({"cycle_id": cid,
                "reference_frame": "ar0234_image_frame", "model_sha256": model_sha,
                "source_ar0234_sha256": sha256(cycle / "ar0234_source.jpg"),
                "mask_sha256": sha256(mask_path)}))
            audit = {"cycle_id": cid, "status": "ASSOCIATION_UNAVAILABLE",
                     "reason": "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE", "calibration": cal["sha"],
                     "shadow_instance_mask": {"status": "MASK_SAVED_DIAGNOSTIC_ONLY",
                                              "worker_model_sha256": model_sha},
                     "stereo_geometry": {"sample_stride_px": 4,
                                         "ar_inner_roi_xyxy_px": [900, 550, 1030, 650]}}
            (cycle / "audit.json").write_text(json.dumps(audit))
            no_target = root / "cycle2"
            no_target.mkdir()
            (no_target / "audit.json").write_text(json.dumps({"cycle_id": "synthetic-00000002",
                "status": "TARGET_UNAVAILABLE", "reason": "NO_TARGET", "calibration": cal["sha"],
                "stereo_geometry": None}))
            rows = [replay(p, cal, .5, 4.5) for p in sorted(root.rglob("audit.json"))]
            baseline = base / "baseline.json"
            baseline.write_text(json.dumps({"source_root": str(root), "diagnostic_depth_range_m": [.5, 4.5],
                "calibration_sha256": cal["sha"], "summary": summary(rows), "cycles": rows}))
            hashes = {p: sha256(p) for p in root.rglob("*") if p.is_file()}
            output = base / "comparison.json"
            command = [sys.executable, str(ROOT / "vision_core/tools/compare_shadow_mask_depth_median.py"),
                       "--baseline-report", str(baseline), "--ar-intrinsic", str(ar),
                       "--stereo-calibration", str(stereo), "--activation", str(activation),
                       "--diagnostic-range-m", ".5", "4.5", "--output", str(output)]
            run = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            report = json.loads(output.read_text())
            self.assertTrue(report["baseline_replay_verified"])
            self.assertFalse(report["actuator_access"])
            self.assertEqual(report["cycles"][0]["filter"]["changed_grid_point_count"], 1)
            self.assertEqual(report["cycles"][0]["filter"]["filled_invalid_pixel_count"], 0)
            self.assertEqual(report["cycles"][0]["mask_depth"], rows[0]["mask_depth"])
            self.assertEqual(report["cycles"][1]["filtered_mask_depth"]["status"], "NO_STEREO_GEOMETRY")
            self.assertEqual(hashes, {p: sha256(p) for p in root.rglob("*") if p.is_file()})
            # Tampered baseline must refuse before creating any comparison artifacts.
            rows[0]["mask_depth"]["candidate_depth_m"] += .01
            bad = base / "bad.json"
            altered = json.loads(baseline.read_text())
            altered["cycles"] = rows
            bad.write_text(json.dumps(altered))
            bad_output = base / "bad_comparison.json"
            command[command.index(str(baseline))] = str(bad)
            command[-1] = str(bad_output)
            run = subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())
            self.assertNotEqual(run.returncode, 0)
            self.assertIn("baseline numerical replay mismatch", run.stderr)
            self.assertFalse(bad_output.exists())


if __name__ == "__main__":
    unittest.main()
