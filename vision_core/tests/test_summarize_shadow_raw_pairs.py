"""Check that a raw-pair report names only cycle-bound evidence."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from vision_core.tools.summarize_shadow_raw_pairs import summarize


class RawPairSummaryTests(unittest.TestCase):
    def test_complete_pair_and_rejected_binding(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle = "cycle-0001"
            directory = root / "run-id" / cycle
            directory.mkdir(parents=True)
            mask = directory / (cycle + ".png")
            rgb = directory / "ar0234_source.jpg"
            mask.write_bytes(b"mask")
            rgb.write_bytes(b"rgb")
            (directory / "ov9281_combined_source.jpg").write_bytes(b"stereo")
            (directory / "disparity_lr_consistent_px.npy").write_bytes(b"disparity")
            digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
            metadata = {"cycle_id": cycle, "mask_sha256": digest(mask),
                        "source_ar0234_sha256": digest(rgb), "model_sha256": "model"}
            (directory / (cycle + ".json")).write_text(json.dumps(metadata))
            audit = {"cycle_id": cycle, "shadow_instance_mask": {"worker_model_sha256": "model"},
                     "stereo_geometry": {"shadow_mask_depth": {"status": "DEPTH_MAD_EXCEEDED"}}}
            (directory / "audit.json").write_text(json.dumps(audit))
            report = summarize(root)
            self.assertEqual(report["mask_depth_status_counts"], {"DEPTH_MAD_EXCEEDED": 1})
            self.assertEqual(report["stereo_pair_evidence_status_counts"],
                             {"LEGACY_NOT_RECORDED": 1})
            self.assertEqual(len(report["examples"]["DEPTH_MAD_EXCEEDED"]), 1)
            mask.write_bytes(b"different mask")
            with self.assertRaisesRegex(ValueError, "binding mismatch"):
                summarize(root)

    def test_no_rgb_target_still_has_cycle_bound_stereo_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle = "cycle-0002"
            directory = root / "run-id" / cycle
            directory.mkdir(parents=True)
            names = ("ov9281_physical_left_rectified.jpg",
                     "ov9281_physical_right_rectified.jpg", "disparity_raw_px.npy",
                     "disparity_lr_consistent_px.npy", "lr_consistency_mask.png")
            for name in names:
                (directory / name).write_bytes(b"diagnostic")
            row = {"cycle_id": cycle, "status": "TARGET_UNAVAILABLE",
                   "stereo_geometry": None,
                   "stereo_pair_evidence": {"status": "SAVED_DIAGNOSTIC_ONLY",
                                            "source_cycle_id": cycle,
                                            "metric_measurement_authorized": False},
                   "evidence_capture": {"files": list(names)}}
            (directory / "audit.json").write_text(json.dumps(row))
            report = summarize(root)
            self.assertEqual(report["stereo_pair_evidence_status_counts"],
                             {"SAVED_DIAGNOSTIC_ONLY": 1})
            self.assertEqual(report["missing_declared_stereo_evidence"], [])
            (directory / "disparity_lr_consistent_px.npy").unlink()
            report = summarize(root)
            self.assertEqual(report["missing_declared_stereo_evidence"][0]["cycle_id"],
                             cycle)


if __name__ == "__main__":
    unittest.main()
