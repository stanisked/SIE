"""Verify frozen Shadow timeline binding and sampled-gap semantics."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from vision_core.tools.analyze_shadow_temporal_depth import analyze


class ShadowTemporalDepthTests(unittest.TestCase):
    def make_capture(self, root: Path) -> Path:
        calibration = {"ov9281_stereo_sha256": "v7"}
        states = ["NO_STEREO_GEOMETRY", "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                  "DEPTH_MAD_EXCEEDED", "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                  "NO_STEREO_GEOMETRY"]
        cycles = []
        for index, state in enumerate(states):
            cid = f"capture-0000000{index}"
            directory = root / "run" / cid
            directory.mkdir(parents=True)
            person = index in (1, 2, 3)
            audit = {
                "cycle_id": cid, "calibration": calibration,
                "timestamp": (datetime(2026, 10, 30, tzinfo=timezone.utc)
                              + timedelta(seconds=index * 3)).isoformat(),
                "status": "ASSOCIATION_UNAVAILABLE" if person else "TARGET_UNAVAILABLE",
                "reason": "FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE" if person else "NO_TARGET",
                "metric_measurement_authorized": False, "navigation_eligible": False,
                "actuator_access": False,
                "pairing": {"capture_pair_skew_ms": 13.0,
                            "exposure_synchronization_proven": False},
                "ar0234_dynamic_target": {"candidate": {"center_x_px": 900.0 + index * 20}} if person else {},
            }
            (directory / "audit.json").write_text(json.dumps(audit))
            depth = {"status": state}
            if person:
                mask = directory / f"{cid}.png"
                mask.write_bytes(b"frozen mask" + bytes([index]))
                depth["candidate_depth_m"] = 2.0 + index * .1
            row = {"cycle_id": cid, "source_status": audit["status"],
                   "source_reason": audit["reason"], "mask_depth": depth}
            if person:
                row["mask_sha256"] = hashlib.sha256(mask.read_bytes()).hexdigest()
            cycles.append(row)
        report = {"status": "OFFLINE_DIAGNOSTIC_ONLY", "source_root": str(root),
                  "calibration_sha256": calibration, "diagnostic_depth_range_m": [.5, 4.5],
                  "activated_depth_range_m": [.5, 2.], "cycles": cycles}
        path = root / "shadow_mask_replay.json"
        path.write_text(json.dumps(report))
        return path

    def test_gaps_are_sampled_and_person_identity_is_not_claimed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            replay = self.make_capture(root)
            report = analyze(root, replay, "walking sideways")
            summary = report["summary"]
            self.assertEqual(summary["cycle_count"], 5)
            self.assertEqual(summary["rgb_person_candidate_cycles"], 3)
            self.assertEqual(summary["coherent_depth_cycles"], 2)
            self.assertEqual(summary["median_intercycle_s"], 3)
            self.assertEqual(summary["max_gap_between_coherent_observations_s"], 6)
            self.assertEqual(summary["gaps_between_coherent_observations"][0]["intervening_refusal_count"], 1)
            self.assertEqual(summary["prefix_without_coherent_cycles"], 1)
            self.assertEqual(summary["suffix_without_coherent_cycles"], 1)
            self.assertEqual(summary["rgb_center_x_span_px"], 40)
            self.assertFalse(report["actuator_access"])

    def test_modified_mask_and_missing_audit_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            replay = self.make_capture(root)
            mask = root / "run/capture-00000001/capture-00000001.png"
            mask.write_bytes(b"replaced")
            with self.assertRaisesRegex(ValueError, "replayed mask changed"):
                analyze(root, replay, "walking")
            mask.unlink()
            (root / "run/capture-00000001/audit.json").unlink()
            with self.assertRaisesRegex(ValueError, "every captured audit"):
                analyze(root, replay, "walking")


if __name__ == "__main__":
    unittest.main()
