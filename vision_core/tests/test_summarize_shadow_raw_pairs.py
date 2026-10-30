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
            self.assertEqual(len(report["examples"]["DEPTH_MAD_EXCEEDED"]), 1)
            mask.write_bytes(b"different mask")
            with self.assertRaisesRegex(ValueError, "binding mismatch"):
                summarize(root)


if __name__ == "__main__":
    unittest.main()
