"""Offline Shadow RGB score accounting and frozen-evidence checks."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from vision_core.tools.analyze_shadow_rgb_detector import summarize, verified_rows


class ShadowRgbDetectorAnalysisTest(unittest.TestCase):
    def test_selected_positives_have_no_false_positive_rate(self):
        rows = [dict(cycle_id="a", person_visible=True, image_path=Path("x")),
                dict(cycle_id="b", person_visible=True, image_path=Path("y"))]
        report = summarize(rows, {"a": {"max_confidence": .71},
                                  "b": {"max_confidence": .25}},
                           source="reported", model_sha=None)
        self.assertEqual(report["summary"]["positive_frames_above_threshold"],
                         {"0.7": 1, "0.4": 1, "0.1": 2})
        self.assertIsNone(report["summary"]["false_positive_rate"])
        self.assertNotIn("image_path", report["frames"][0])

    def test_missing_scores_are_not_counted_as_failures(self):
        rows = [dict(cycle_id="a", person_visible=True, image_path=Path("x")),
                dict(cycle_id="b", person_visible=True, image_path=Path("y"))]
        report = summarize(rows, {"a": {"max_confidence": 0.0}},
                           source="partial", model_sha=None)
        self.assertEqual(report["summary"]["evaluated_frames"], 1)
        self.assertIsNone(report["frames"][1]["above_threshold"])

    def test_hash_check_rejects_changed_image(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cid = "ar0234-ov9281-dynamic-audit-session-00000001"
            cycle = root / "session" / cid
            cycle.mkdir(parents=True)
            img = cycle / "ar0234_source.jpg"
            audit = cycle / "audit.json"
            img.write_bytes(b"frozen jpeg")
            audit.write_text(json.dumps({"cycle_id": cid, "actuator_access": False,
                                         "metric_measurement_authorized": False,
                                         "ar0234_dynamic_target": {"source_target_status": "NO_TARGET"}}))
            label = {"cycle_id": cid, "person_visible": True,
                     "image_sha256": hashlib.sha256(img.read_bytes()).hexdigest(),
                     "audit_sha256": hashlib.sha256(audit.read_bytes()).hexdigest()}
            manifest = {"schema_version": "sie.shadow_rgb_visual_labels.v1", "frames": [label]}
            self.assertEqual(len(verified_rows(root, manifest)), 1)
            img.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "FROZEN_EVIDENCE_HASH_MISMATCH"):
                verified_rows(root, manifest)


if __name__ == "__main__":
    unittest.main()
