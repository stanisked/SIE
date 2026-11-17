"""RGB identity survives a depth refusal only while frames still match."""
import unittest

from vision_core.person_localization.shadow_rgb_continuity import (
    continued_track, remember_track, retain_short_gap,
    retain_current_rgb_on_depth_refusal, match_current_edge_track,
)
from vision_core.tests.test_shadow_fast_adapter import source, convert
from vision_core.tools.shadow_fast_adapter import AR_INTRINSIC_SHA256
from vision_core.tools.shadow_range_gate import select_target_range
from vision_core.tools.shadow_supervisor import ShadowSupervisor


class ShadowRgbContinuityTests(unittest.TestCase):
    def test_edge_track_keeps_identity_for_current_single_clipped_person(self):
        prior = remember_track("person-1",
            {"bbox_xyxy_px": [990, 0, 1845, 1190]},
            "2026-11-17T12:00:00+00:00")
        for when, x2 in zip(("12:00:00.25", "12:00:00.50",
                              "12:00:00.75", "12:00:01.00"),
                            (1918, 1914, 1916, 1911)):
            instance = {"instance_index": 0, "confidence": .90,
                        "bbox_xyxy_px": [1060, 0, x2, 1184],
                        "truncated_right": x2 >= 1917}
            matched = match_current_edge_track(
                prior, [instance],
                f"2026-11-17T{when}+00:00",
                refusal_reason=("NO_CURRENT_NON_EDGE_TARGET"
                                if x2 >= 1917 else "BBOX_NEAR_RIGHT_IMAGE_EDGE"),
                minimum_confidence=.7, minimum_iou=.2,
                maximum_center_displacement_relative=.75)
            self.assertIsNotNone(matched)
            prior = matched[0]
            self.assertEqual(prior["track_id"], "person-1")

    def test_edge_guidance_rejects_other_person_ambiguity_and_stale_rgb(self):
        prior = remember_track("person-1",
            {"bbox_xyxy_px": [990, 0, 1845, 1190]},
            "2026-11-17T12:00:00+00:00")
        instance = {"instance_index": 0, "confidence": .9,
                    "bbox_xyxy_px": [1060, 0, 1918, 1184],
                    "truncated_right": True}
        def check(items, when, reason="NO_CURRENT_NON_EDGE_TARGET"):
            return match_current_edge_track(
                prior, items, when, refusal_reason=reason,
                minimum_confidence=.7, minimum_iou=.2,
                maximum_center_displacement_relative=.75)
        self.assertIsNone(check([instance, instance],
            "2026-11-17T12:00:00.30+00:00"))
        self.assertIsNone(check([instance],
            "2026-11-17T12:00:01.0+00:00"))
        self.assertIsNone(check([{**instance,
            "bbox_xyxy_px": [0, 0, 50, 1100]}],
            "2026-11-17T12:00:00.3+00:00"))
        self.assertIsNone(check([instance],
            "2026-11-17T12:00:00.3+00:00", reason="UNKNOWN"))

    def test_depth_only_multilayer_refusal_retains_matched_identity_not_range(self):
        for depth_status in ("DEPTH_MAD_EXCEEDED",
                             "MULTIPLE_OR_NO_PLAUSIBLE_DEPTH_LAYERS"):
            with self.subTest(depth_status=depth_status):
                self.assertTrue(retain_current_rgb_on_depth_refusal(
                    rgb_matched_current=True,
                    audit_status="MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
                    depth_status=depth_status))
                self.assertFalse(retain_current_rgb_on_depth_refusal(
                    rgb_matched_current=False,
                    audit_status="MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
                    depth_status=depth_status))
        self.assertFalse(retain_current_rgb_on_depth_refusal(
            rgb_matched_current=True,
            audit_status="PROCESSING_REFUSED",
            depth_status="MULTIPLE_OR_NO_PLAUSIBLE_DEPTH_LAYERS"))
        self.assertFalse(retain_current_rgb_on_depth_refusal(
            rgb_matched_current=True,
            audit_status="MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
            depth_status="UNKNOWN"))

    def test_multi_layer_gap_pauses_then_matching_rgb_can_resume(self):
        supervisor = ShadowSupervisor("boot-2", "shadow-2")
        decide = lambda observation: select_target_range(
            observation.target, list(observation.ranges), 10.,
            expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
            minimum_confidence=.4)
        first, _ = convert(source())
        start = supervisor.observe(first.target, decide(first))
        self.assertEqual(start.operation, "START")
        supervisor.acknowledge(start, boot_session_id="boot-2",
            active_command_id="shadow-2", controller_state="DRIVING",
            right_pwm=50, left_pwm=50, fault_latched=False,
            transition_seq=start.transition_seq)
        before = remember_track(first.target.track_id,
            {"bbox_xyxy_px": [400, 200, 800, 900]},
            "2026-11-13T10:00:00+00:00")
        bad = source()
        bad["shape_diagnostic"]["mask_depth"]["status"] = (
            "MULTIPLE_OR_NO_PLAUSIBLE_DEPTH_LAYERS")
        empty, _ = convert(bad)
        self.assertIsNone(empty.target)
        pause = supervisor.observe(empty.target, decide(empty))
        self.assertEqual(pause.operation, "PAUSE")
        supervisor.acknowledge(pause, boot_session_id="boot-2",
            active_command_id="shadow-2", controller_state="PAUSED",
            right_pwm=0, left_pwm=0, fault_latched=False,
            transition_seq=pause.transition_seq)
        self.assertTrue(retain_current_rgb_on_depth_refusal(
            rgb_matched_current=True,
            audit_status="MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY",
            depth_status=bad["shape_diagnostic"]["mask_depth"]["status"]))
        retained = continued_track(before,
            {"bbox_xyxy_px": [420, 200, 820, 900]},
            "2026-11-13T10:00:00.27+00:00",
            minimum_iou=.2, maximum_center_displacement_relative=.75)
        self.assertEqual(retained["track_id"], first.target.track_id)
        recovered = source()
        recovered["shadow_track_confirmation"]["track_id"] = retained["track_id"]
        observation, _ = convert(recovered)
        self.assertEqual(supervisor.observe(
            observation.target, decide(observation)).operation, "RESUME")

    def test_current_rgb_continues_across_mad_without_depth(self):
        bbox = {"bbox_xyxy_px": [400, 200, 800, 900]}
        prior = remember_track("person-1", bbox, "2026-11-13T10:00:00+00:00")
        for time, x in (("10:00:00.27", 420), ("10:00:00.54", 440)):
            candidate = {"bbox_xyxy_px": [x, 200, x + 400, 900]}
            prior = continued_track(prior, candidate,
                f"2026-11-13T{time}+00:00", minimum_iou=.2,
                maximum_center_displacement_relative=.75)
            self.assertEqual(prior["track_id"], "person-1")

    def test_missing_time_or_unrelated_box_cannot_inherit_id(self):
        prior = remember_track("person-1", {"bbox_xyxy_px": [400, 200, 800, 900]},
                               "2026-11-13T10:00:00+00:00")
        def match(box, at):
            return continued_track(prior, {"bbox_xyxy_px": box}, at,
                minimum_iou=.2, maximum_center_displacement_relative=.75)
        self.assertIsNone(match([420, 200, 820, 900], "2026-11-13T10:00:01+00:00"))
        self.assertIsNone(match([1200, 200, 1600, 900], "2026-11-13T10:00:00.27+00:00"))
        self.assertIsNone(match([420, 200, 820, 900], "invalid"))

    def test_short_unavailable_gap_holds_only_identity_until_fresh_match(self):
        prior = remember_track("person-1", {"bbox_xyxy_px": [400, 200, 800, 900]},
                               "2026-11-15T09:33:56+00:00")
        self.assertTrue(retain_short_gap("TARGET_UNAVAILABLE",
                         "BBOX_TOO_NARROW_FOR_UPPER_BODY", prior))
        self.assertFalse(retain_short_gap("PROCESSING_REFUSED",
                          "BBOX_TOO_NARROW_FOR_UPPER_BODY", prior))
        self.assertFalse(retain_short_gap("TARGET_UNAVAILABLE",
                          "MULTIPLE_TARGETS_NO_CONFIRMED_TRACK", prior))
        def match(box, delay):
            return continued_track(prior, {"bbox_xyxy_px": box},
                f"2026-11-15T09:33:56.{delay:03d}+00:00",
                minimum_iou=.2, maximum_center_displacement_relative=.75)
        self.assertEqual(match([420, 200, 820, 900], 600)["track_id"], "person-1")
        self.assertIsNone(match([1200, 200, 1600, 900], 600))
        self.assertIsNone(match([420, 200, 820, 900], 800))

    def test_mad_pauses_then_same_rgb_track_can_resume(self):
        supervisor = ShadowSupervisor("boot-1", "shadow-1")
        decide = lambda observation: select_target_range(
            observation.target, list(observation.ranges), 10.,
            expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
            minimum_confidence=.4)
        first, _ = convert(source())
        start = supervisor.observe(first.target, decide(first))
        self.assertEqual(start.operation, "START")
        supervisor.acknowledge(start, boot_session_id="boot-1",
            active_command_id="shadow-1", controller_state="DRIVING",
            right_pwm=50, left_pwm=50, fault_latched=False,
            transition_seq=start.transition_seq)
        bbox = {"bbox_xyxy_px": [400, 200, 800, 900]}
        prior = remember_track(first.target.track_id, bbox,
                               "2026-11-13T10:00:00+00:00")
        bad = source()
        bad["shape_diagnostic"]["mask_depth"]["status"] = "DEPTH_MAD_EXCEEDED"
        empty, _ = convert(bad)
        self.assertIsNone(empty.target)
        pause = supervisor.observe(empty.target, decide(empty))
        self.assertEqual(pause.operation, "PAUSE")
        supervisor.acknowledge(pause, boot_session_id="boot-1",
            active_command_id="shadow-1", controller_state="PAUSED",
            right_pwm=0, left_pwm=0, fault_latched=False,
            transition_seq=pause.transition_seq)
        continued = continued_track(prior, {"bbox_xyxy_px": [420, 200, 820, 900]},
            "2026-11-13T10:00:00.27+00:00", minimum_iou=.2,
            maximum_center_displacement_relative=.75)
        self.assertEqual(continued["track_id"], first.target.track_id)
        resumed = source()
        resumed["shadow_track_confirmation"]["track_id"] = continued["track_id"]
        observation, _ = convert(resumed)
        self.assertEqual(supervisor.observe(observation.target, decide(observation)).operation,
                         "RESUME")


if __name__ == "__main__":
    unittest.main()
