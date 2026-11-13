"""RGB identity survives a depth refusal only while frames still match."""
import unittest

from vision_core.person_localization.shadow_rgb_continuity import (
    continued_track, remember_track,
)
from vision_core.tests.test_shadow_fast_adapter import source, convert
from vision_core.tools.shadow_fast_adapter import AR_INTRINSIC_SHA256
from vision_core.tools.shadow_range_gate import select_target_range
from vision_core.tools.shadow_supervisor import ShadowSupervisor


class ShadowRgbContinuityTests(unittest.TestCase):
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
