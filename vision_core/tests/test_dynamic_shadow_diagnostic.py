"""Dynamic RGB/depth diagnostic keeps static refusal and motor isolation."""
from copy import deepcopy
import unittest

from vision_core.person_localization.dynamic_shadow_diagnostic import (
    STEREO_SHA256, continue_after_static_gate, dynamic_shadow_diagnostic,
    select_edge_candidate, allow_near_edge_geometry, DiagnosticContinuity,
)


def audit():
    return {
        "cycle_id": "current-01",
        "calibration": {"ov9281_stereo_sha256": STEREO_SHA256},
        "existing_profile_static_gate": {"status": "REFUSED", "reason": "MOTION_DETECTED"},
        "ar0234_dynamic_target": {
            "disposition": "OBSERVED", "candidate": {"instance_index": 0}},
        "target_geometry": {"accepted": True},
        "segmentation_observation": {"cycle_id": "current-01", "arrays_sha256": "rgb01"},
        "frame_freshness": {"capture_backend": "latest_v4l2", "frames": {
            "ar0234": {"driver_timestamp_s": 10.0},
            "ov9281": {"driver_timestamp_s": 10.03}}},
        "shape_diagnostic": {"cycle_id": "current-01", "mask": {
            "status": "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY",
            "instance_index": 0, "source_rgb_arrays_sha256": "rgb01"},
            "mask_depth": {"status": "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                           "candidate_depth_m": 2.1}},
    }


class TestDynamicDiagnostic(unittest.TestCase):
    def test_single_current_edge_person_is_diagnostic_only(self):
        person = {"instance_index": 0, "confidence": .9,
                  "bbox_xyxy_px": [0., 10., 500., 1100.],
                  "truncated_left": True, "truncated_right": False, "truncated_top": False}
        legacy = {"disposition": "UNAVAILABLE", "reason": "NO_CURRENT_NON_EDGE_TARGET"}
        picked, override = select_edge_candidate({"instances": [person]}, legacy, min_confidence=.7)
        self.assertTrue(override)
        self.assertEqual(picked["candidate"], person)
        self.assertFalse(picked["metric_measurement_authorized"])
        for instances in ([], [person, person], [{**person, "confidence": .2}]):
            self.assertEqual(select_edge_candidate({"instances": instances}, legacy,
                                                   min_confidence=.7), (legacy, False))

    def test_continuity_resets_on_loss_and_never_proves_identity(self):
        track = DiagnosticContinuity()
        first = {"bbox_xyxy_px": [0., 10., 500., 1100.]}
        nearby = {"bbox_xyxy_px": [50., 10., 550., 1100.]}
        a = track.observe(first, now_s=1.)
        b = track.observe(nearby, now_s=1.2)
        lost = track.observe(None, now_s=1.4)
        reacquired = track.observe(first, now_s=1.6)
        self.assertEqual(a["status"], "NEW_SEGMENT_IDENTITY_UNVERIFIED")
        self.assertEqual(b["status"], "ADJACENT_BOX_MATCH_DIAGNOSTIC_ONLY")
        self.assertEqual(lost["status"], "NO_SINGLE_CURRENT_PERSON")
        self.assertEqual(reacquired["segment"], a["segment"] + 1)
        self.assertFalse(reacquired["same_person_proven"])

    def test_near_edge_geometry_override_requires_one_exact_person(self):
        person = {"instance_index": 0, "confidence": .85,
                  "bbox_xyxy_px": [1695., 120., 1917., 1198.]}
        geometry = {"accepted": False,
                    "reason": "BBOX_TOO_NARROW_FOR_UPPER_BODY+BBOX_NEAR_RIGHT_IMAGE_EDGE"}
        self.assertTrue(allow_near_edge_geometry({"instances": [person]}, person,
                                            geometry, min_confidence=.7))
        self.assertFalse(allow_near_edge_geometry({"instances": [person, person]}, person,
                                             geometry, min_confidence=.7))
        self.assertFalse(allow_near_edge_geometry({"instances": [person]}, person,
                                             {"accepted": False, "reason": "BAD_CALIBRATION"},
                                             min_confidence=.7))

    def test_static_refusal_only_bypassed_when_explicit(self):
        gate = {"status": "REFUSED", "reason": "MOTION_DETECTED"}
        self.assertFalse(continue_after_static_gate(gate, dynamic_diagnostic_enabled=False))
        self.assertTrue(continue_after_static_gate(gate, dynamic_diagnostic_enabled=True))

    def test_current_masked_depth_is_diagnostic_not_motion_range(self):
        result = dynamic_shadow_diagnostic(audit(), now_s=10.2)
        self.assertEqual(result["status"], "CURRENT_RGB_MASKED_DEPTH_DIAGNOSTIC_ONLY")
        self.assertEqual(result["candidate_optical_z_m"], 2.1)
        self.assertEqual(result["legacy_static_gate_reason"], "MOTION_DETECTED")
        self.assertFalse(result["motor_command_authorized"])
        self.assertFalse(result["front_clearance_established"])
        self.assertFalse(result["stable_person_identity_established"])

    def test_stale_or_wrong_mask_cannot_report_range(self):
        for change in ({"frame_freshness": {"capture_backend": "latest_v4l2",
                         "frames": {"ar0234": {"driver_timestamp_s": 8.0},
                                    "ov9281": {"driver_timestamp_s": 10.03}}}},
                       {"shape_diagnostic": {"cycle_id": "old", "mask": {
                           "status": "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY",
                           "instance_index": 0, "source_rgb_arrays_sha256": "rgb01"},
                           "mask_depth": {"status": "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY",
                                          "candidate_depth_m": 2.1}}}):
            record = deepcopy(audit())
            record.update(change)
            result = dynamic_shadow_diagnostic(record, now_s=10.2)
            self.assertEqual(result["status"], "RGB_CURRENT_DEPTH_UNAVAILABLE")
            self.assertIsNone(result["candidate_optical_z_m"])


if __name__ == "__main__":
    unittest.main()
