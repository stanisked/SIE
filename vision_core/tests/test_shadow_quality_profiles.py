"""Integrity, incomplete evidence and conservative partial ordering checks."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "vision_core/tools"))
from analyze_shadow_quality_profiles import profile, digest, dominates, summarize


def fixture(root):
    cid = "quality-00000001"
    folder = root / "run" / cid
    folder.mkdir(parents=True)
    local = {"pooled_held_out_axial_residual": {"sample_count": 200, "mad_m": .02,
             "absolute_p90_m": .06, "absolute_max_m": .3, "median_m": -.001},
             "rgb_coverage": {"mask_area_in_fitted_bins_fraction": .98},
             "fitted_sample_fraction": .96, "unfitted_patch_fraction": .02,
             "patch_status_counts": {"FIT_DIAGNOSTIC_ONLY": 49, "TOO_FEW_POINTS": 1}}
    depth = {"status": "DEPTH_MAD_EXCEEDED", "candidate_depth_m": 2.5,
             "candidate_mad_m": .06, "selected_sample_count": 500,
             "candidate_sample_count": 480, "plausible_layer_count": 1}
    source = {"cycle_id": cid, "source_status": "ASSOCIATION_UNAVAILABLE",
              "source_reason": "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE",
              "mask_depth": depth, "local_surface": local}
    for key, filename in {"source_audit_sha256": "shadow_surface_input.json",
                          "source_mask_sha256": cid + ".png",
                          "source_disparity_sha256": "disparity_lr_consistent_px.npy",
                          "source_rgb_sha256": "ar0234_source.jpg"}.items():
        p = folder / filename
        p.write_bytes(b"untouched fixture source")
        source[key] = digest(p)
    report = {"cycle_id": cid, "calibration_sha256": {"stereo": "a" * 64}, "result": source,
              "reference_frame": "rectified_left_optical_frame", "diagnostic_depth_range_m": [.5, 4.5],
              "activated_depth_range_m": [.5, 2.], "method": {"tile_px": 32},
              "code_sha256": {"fixture": "b" * 64}}
    rp = folder / "shadow_local_surfaces/report.json"
    rp.parent.mkdir()
    rp.write_text(json.dumps(report))
    shadow = {"status": "SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY", "mask_depth": depth,
              "local_surface": local, "report_file": "shadow_local_surfaces/report.json",
              "report_sha256": digest(rp)}
    audit = {"cycle_id": cid, "calibration": report["calibration_sha256"],
             "status": source["source_status"], "reason": source["source_reason"],
             "shadow_local_surfaces": shadow, "processing_timing": {"host_capture_start_to_result_ready_s": 2.3},
             "pairing": {"exposure_synchronization_proven": False}}
    path = folder / "audit.json"
    path.write_text(json.dumps(audit))
    return path


class QualityProfileTests(unittest.TestCase):
    def test_complete_refused_profile_is_not_promoted(self):
        with tempfile.TemporaryDirectory() as name:
            path = fixture(Path(name))
            before = {p: p.read_bytes() for p in Path(name).rglob("*") if p.is_file()}
            row = profile(path)
            self.assertEqual(row["comparison_status"], "PROFILE_COMPLETE_DIAGNOSTIC_ONLY")
            self.assertEqual(row["mask_status"], "DEPTH_MAD_EXCEEDED")
            self.assertFalse(row["metric_measurement_authorized"])
            self.assertFalse(row["navigation_eligible"])
            self.assertEqual(before, {p: p.read_bytes() for p in before})

    def test_source_tamper_and_report_tamper_are_explicit(self):
        for filename in ("disparity_lr_consistent_px.npy", "shadow_local_surfaces/report.json"):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as name:
                path = fixture(Path(name))
                (path.parent / filename).write_bytes(b"changed")
                self.assertEqual(profile(path)["comparison_status"], "EVIDENCE_INTEGRITY_ERROR")

    def test_no_target_is_incomplete_not_zero_residual(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "audit.json"
            path.write_text(json.dumps({"cycle_id": "missing", "status": "TARGET_UNAVAILABLE"}))
            row = profile(path)
            self.assertEqual(row["comparison_status"], "NOT_COMPARABLE")
            self.assertNotIn("local_mad_m", row)

    def test_partial_order_requires_all_axes_same_run_and_one_strict_improvement(self):
        with tempfile.TemporaryDirectory() as name:
            a = profile(fixture(Path(name)))
            b = deepcopy(a)
            b["cycle_id"] = "accepted"
            b["mask_status"] = "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"
            self.assertFalse(dominates(a, b))  # Equal is not better.
            b["local_mad_m"] += .001
            self.assertTrue(dominates(a, b))
            self.assertEqual(len(summarize([a, b])["refused_dominates_accepted"]), 1)
            b["local_max_absolute_m"] -= .001  # Tradeoff, not domination.
            self.assertFalse(dominates(a, b))
            b["local_max_absolute_m"] = a["local_max_absolute_m"]
            b["run_directory"] = "different run"
            self.assertFalse(dominates(a, b))

    def test_cli_preserves_inputs_and_does_not_create_an_acceptance_policy(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            fixture(base)
            output = base / "quality.json"
            command = [sys.executable, str(ROOT / "vision_core/tools/analyze_shadow_quality_profiles.py"),
                       "--evidence-root", str(base), "--output", str(output)]
            run = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            report = json.loads(output.read_text())
            self.assertEqual(report["acceptance_policy"], "NOT_DEFINED_OR_VALIDATED")
            self.assertFalse(report["actuator_access"])
            retry = subprocess.run(command, text=True, capture_output=True)
            self.assertNotEqual(retry.returncode, 0)


if __name__ == "__main__":
    unittest.main()
