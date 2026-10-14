"""Shape, outlier, sparse-coverage and source-preservation checks for local fits."""

from __future__ import annotations

from pathlib import Path
import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from vision_core.person_localization.local_surface_diagnostic import analyze_local_surfaces


class LocalSurfaceTests(unittest.TestCase):
    def setUp(self):
        v, u = np.mgrid[0:32:4, 0:32:4]
        self.u, self.v = u.flatten(), v.flatten()
        self.fb = 75.95418816496264

    def test_sloped_surface_has_depth_extent_but_zero_model_residual(self):
        disparity = 30 + .2 * self.u + .1 * self.v
        original = disparity.copy()
        report, values = analyze_local_surfaces(self.u, self.v, disparity, self.fb)
        self.assertGreater(report["raw_depth"]["mad_m"], .05)
        self.assertLess(report["pooled_held_out_axial_residual"]["absolute_max_m"], 1e-9)
        self.assertEqual(report["fitted_sample_fraction"], 1.)
        self.assertEqual(report["pooled_held_out_axial_residual"]["sample_count"], 32)
        np.testing.assert_array_equal(disparity, original)
        self.assertEqual(len(values["axial_residual_m"]), 64)

    def test_test_outlier_remains_in_report(self):
        disparity = np.full(64, 33.)
        disparity[1] += 10  # Odd chessboard cell: never used by the fit.
        report, values = analyze_local_surfaces(self.u, self.v, disparity, self.fb)
        self.assertTrue(values["held_out"][1])
        self.assertGreater(abs(values["axial_residual_m"][1]), .4)
        self.assertGreater(report["pooled_held_out_axial_residual"]["absolute_max_m"], .4)
        self.assertEqual(report["pooled_held_out_axial_residual"]["sample_count"], 32)

    def test_huber_limits_training_outlier_influence(self):
        disparity = 33 + .02 * self.u + .01 * self.v
        disparity[0] += 10  # Training sample; all reported residuals are retained.
        report, values = analyze_local_surfaces(self.u, self.v, disparity, self.fb)
        held = report["pooled_held_out_axial_residual"]
        self.assertLess(abs(held["median_m"]), .02)
        self.assertGreater(abs(values["axial_residual_m"][0]), .4)

    def test_sparse_and_degenerate_tiles_do_not_gain_confidence(self):
        report, values = analyze_local_surfaces(self.u[:16], self.v[:16], np.full(16, 33.), self.fb)
        self.assertEqual(report["fitted_sample_fraction"], 0.)
        self.assertEqual(report["patches"][0]["status"], "TOO_FEW_POINTS")
        self.assertTrue(np.isnan(values["axial_residual_m"]).all())
        report, _ = analyze_local_surfaces(np.zeros(24), np.arange(24) * 4, np.full(24, 33.),
                                          self.fb, tile_px=128)
        self.assertEqual(report["fitted_sample_fraction"], 0.)
        self.assertEqual(report["patches"][0]["status"], "DEGENERATE_IMAGE_SUPPORT")

    def test_consistent_scale_bias_is_not_detected_by_surface_residual(self):
        disparity = (33 + .01 * self.u + .02 * self.v) / 1.1
        report, _ = analyze_local_surfaces(self.u, self.v, disparity, self.fb)
        self.assertLess(report["pooled_held_out_axial_residual"]["absolute_max_m"], 1e-9)
        self.assertEqual(report["status"], "LOCAL_SURFACE_DIAGNOSTIC_ONLY")

    def test_invalid_disparities_refuse(self):
        with self.assertRaises(ValueError):
            analyze_local_surfaces(self.u, self.v, np.full(64, np.nan), self.fb)

    def test_masked_replay_exports_original_geometry_without_mutating_sources(self):
        from vision_core.tests.test_shadow_depth_regions import fixture
        from analyze_shadow_local_surfaces import analyze_cycle
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            path, raw, cal = fixture(base)
            raw["mask_sha256"] = "verified-in-baseline-fixture"
            originals = {p: p.read_bytes() for p in base.iterdir()}
            output = base / "output"
            output.mkdir()
            row = analyze_cycle(path, raw, cal, .5, 4.5, output, True)
            local = row["local_surface"]
            self.assertEqual(local["sample_count"], 256)
            self.assertEqual(local["fitted_patch_count"], 4)
            self.assertGreater(local["raw_depth"]["mad_m"], .05)
            self.assertLess(local["pooled_held_out_axial_residual"]["absolute_max_m"], 1e-6)
            for key in ("mask_area_in_observed_bins_fraction", "mask_area_in_fitted_bins_fraction"):
                self.assertTrue(0 <= local["rgb_coverage"][key] <= 1)
            cloud = (output / row["artifacts"]["rgb_cloud"]).read_text()
            self.assertIn("element vertex 256", cloud)
            points = np.array([list(map(float, line.split()[:3]))
                               for line in cloud.split("end_header\n")[1].strip().splitlines()])
            original_d = np.load(base / "disparity_lr_consistent_px.npy")[::4, ::4].flatten()
            original_z = abs(float(cal["P2"][0, 3])) / original_d / 1000.
            np.testing.assert_allclose(points[:, 2], original_z, rtol=0, atol=1e-9)
            self.assertEqual(originals, {p: p.read_bytes() for p in originals})

            # Full command-line replay, including activated-artifact hashes.
            from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
            from replay_shadow_mask_depth_range import replay
            artifacts = base / "runtime"
            artifacts.mkdir()
            ar, stereo, extrinsic = (artifacts / n for n in ("ar.json", "stereo.npz", "extrinsic.npz"))
            ar.write_text(json.dumps({"camera_matrix": cal["K"].tolist(),
                                      "distortion_coefficients": cal["D"].tolist()}))
            np.savez(stereo, camera_1_semantics="physical_left", camera_2_semantics="physical_right",
                     size=[64, 64], R1=cal["R1"], P1=cal["P1"], P2=cal["P2"])
            np.savez(extrinsic, source_stereo_calibration_sha256=sha256(stereo),
                     source_ar_intrinsic_sha256=sha256(ar), R_ar0234_to_physical_left=cal["R"],
                     T_ar0234_to_physical_left_mm=cal["T"] * 1000)
            loaded = load_calibrations(ar, stereo, extrinsic)
            cid = raw["cycle_id"]
            model_sha = "1" * 64
            metadata = {"cycle_id": cid, "reference_frame": "ar0234_image_frame",
                        "model_sha256": model_sha, "mask_sha256": sha256(base / f"{cid}.png"),
                        "source_ar0234_sha256": sha256(base / "ar0234_source.jpg")}
            (base / f"{cid}.json").write_text(json.dumps(metadata))
            audit = json.loads(path.read_text())
            audit.update({"calibration": loaded["sha"], "status": "ASSOCIATION_UNAVAILABLE",
                          "reason": "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE",
                          "shadow_instance_mask": {"status": "MASK_SAVED_DIAGNOSTIC_ONLY",
                                                   "worker_model_sha256": model_sha}})
            path.write_text(json.dumps(audit))
            baseline = base / "baseline.json"
            baseline.write_text(json.dumps({"source_root": str(base), "calibration_sha256": loaded["sha"],
                                            "diagnostic_depth_range_m": [.5, 4.5],
                                            "cycles": [replay(path, loaded, .5, 4.5)]}))
            activation = artifacts / "activation.json"
            activation.write_text(json.dumps({"calibration_profile": {
                "source_sha256": {"ar0234_intrinsic": sha256(ar), "ov9281_stereo_calibration": sha256(stereo)},
                "ar0234_to_ov9281_physical_left_extrinsic": {"path": str(extrinsic), "sha256": sha256(extrinsic)}}}))
            command = [sys.executable, str(ROOT / "vision_core/tools/analyze_shadow_local_surfaces.py"),
                       "--baseline-report", str(baseline), "--ar-intrinsic", str(ar),
                       "--stereo-calibration", str(stereo), "--activation", str(activation),
                       "--export-cycles", "00000003", "--output-dir", str(base / "cli_output")]
            run = subprocess.run(command, text=True, capture_output=True, env=os.environ.copy())
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            result = json.loads((base / "cli_output/report.json").read_text())
            self.assertTrue(result["baseline_replay_verified"])
            self.assertFalse(result["metric_measurement_authorized"])
            self.assertEqual(result["cycles"][0]["local_surface"]["fitted_patch_count"], 4)


if __name__ == "__main__":
    unittest.main()
