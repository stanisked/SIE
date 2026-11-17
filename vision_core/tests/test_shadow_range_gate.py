import unittest

from vision_core.tools.shadow_range_gate import (
    ACTIVE_STEREO_SHA256, FRONT_FRAME, RgbTarget, TargetRange,
    STEREO_SUPERVISED_MIN_M, SHADOW_STOP_DISTANCE_M,
    SHADOW_RESUME_MARGIN_M, SHADOW_COMMAND_CLEARANCE_M,
    SHADOW_MIN_SEGMENT_M, admit_supervised_stereo_distance,
    RangeDecision, select_target_range,
)


class ShadowRangeGateTests(unittest.TestCase):
    def setUp(self):
        self.target = RgbTarget("person-7", 10.0, 8.0, .95, "rgb-v5")
        self.stereo = TargetRange("OV9281", "person-7", 10.03, 1.5, .9,
                                  FRONT_FRAME, ACTIVE_STEREO_SHA256, "pair-1")
        self.lidar = TargetRange("LIDAR", "person-7", 10.04, 1.48, .94,
                                 FRONT_FRAME, "lidar-cal-1", "association-1")

    def decide(self, ranges, target=None, now=10.1):
        return select_target_range(self.target if target is None else target,
                                   ranges, now,
                                   expected_rgb_calibration_id="rgb-v5",
                                   expected_lidar_calibration_id="lidar-cal-1")

    def test_stereo_and_lidar_failover(self):
        self.assertEqual(self.decide([self.stereo]).selected.source, "OV9281")
        self.assertEqual(self.decide([self.lidar]).selected.source, "LIDAR")
        self.assertEqual(self.decide([self.stereo, self.lidar]).selected.source,
                         "LIDAR")

    def test_no_range_pauses_even_with_rgb(self):
        result = self.decide([])
        self.assertEqual((result.action, result.reason),
                         ("PAUSE", "NO_FRESH_ASSOCIATED_RANGE"))

    def test_wrong_person_and_missing_association_pause(self):
        from dataclasses import replace
        self.assertEqual(self.decide([replace(self.lidar, target_track_id="other")]).action,
                         "PAUSE")
        self.assertEqual(self.decide([replace(self.lidar, association_evidence_id="")]).action,
                         "PAUSE")

    def test_stale_or_unbound_calibration_pauses(self):
        from dataclasses import replace
        self.assertEqual(self.decide([self.lidar], now=10.7).action, "PAUSE")
        self.assertEqual(self.decide([replace(self.lidar, calibration_id="unknown")]).action,
                         "PAUSE")
        self.assertEqual(self.decide([self.lidar], target=replace(self.target,
                         track_id="person-8")).action, "PAUSE")

    def test_stop_distance_and_sensor_conflict(self):
        from dataclasses import replace
        self.assertEqual(self.decide([replace(self.lidar, distance_m=.78)]).action,
                         "HOLD")
        self.assertEqual(self.decide([replace(self.lidar, distance_m=.91)]).action,
                         "HOLD")
        self.assertEqual(self.decide([self.stereo,
                         replace(self.lidar, distance_m=.7)]).reason,
                         "RANGE_SOURCES_DISAGREE")

    def test_supervised_hold_overlaps_admitted_stereo_range(self):
        from dataclasses import replace
        self.assertLessEqual(STEREO_SUPERVISED_MIN_M, SHADOW_STOP_DISTANCE_M)
        for distance in (1.0, 1.10, 1.15):
            with self.subTest(distance=distance):
                result = self.decide([replace(self.stereo, distance_m=distance)])
                self.assertEqual(result.action, "HOLD")
        self.assertEqual(self.decide([replace(self.stereo, distance_m=1.19)]).reason,
                         "NEXT_SEGMENT_CLEARANCE_TOO_SMALL")
        self.assertEqual(self.decide([replace(self.stereo, distance_m=1.20)]).action,
                         "FOLLOW_ELIGIBLE")

    def test_follow_always_has_minimum_bounded_segment_clearance(self):
        from dataclasses import replace
        for distance in (1.16, 1.18, 1.19, 1.19001, 1.20, 1.5, 2.5):
            with self.subTest(distance=distance):
                result = self.decide([replace(self.stereo, distance_m=distance)])
                if result.action == "FOLLOW_ELIGIBLE":
                    clearance = (distance - SHADOW_STOP_DISTANCE_M -
                                 SHADOW_RESUME_MARGIN_M - SHADOW_COMMAND_CLEARANCE_M)
                    self.assertGreaterEqual(clearance, SHADOW_MIN_SEGMENT_M)
                else:
                    self.assertEqual(result.action, "HOLD")

    def test_close_range_is_hold_only_not_forward_admission(self):
        from dataclasses import replace
        close = self.decide([replace(self.stereo, distance_m=.865)])
        self.assertEqual(close.action, "HOLD")
        self.assertTrue(admit_supervised_stereo_distance(.865, close))
        self.assertFalse(admit_supervised_stereo_distance(.865,
            RangeDecision("FOLLOW_ELIGIBLE", "INVALID_OVERRIDE")))
        self.assertFalse(admit_supervised_stereo_distance(.865,
            RangeDecision("PAUSE", "NO_FRESH_ASSOCIATED_RANGE")))
        self.assertFalse(admit_supervised_stereo_distance(.49, close))
        self.assertFalse(admit_supervised_stereo_distance(4.51, close))
        follow = self.decide([replace(self.stereo, distance_m=1.5)])
        self.assertEqual(follow.action, "FOLLOW_ELIGIBLE")
        self.assertTrue(admit_supervised_stereo_distance(1.5, follow))
        # Don't cherry-pick a near range: admit the entire calibrated
        # diagnostic interval while leaving the close-range HOLD intact.
        for distance in (2.51, 3.0, 4.0, 4.5):
            with self.subTest(distance=distance):
                candidate = self.decide([replace(self.stereo, distance_m=distance)])
                self.assertEqual(candidate.action, "FOLLOW_ELIGIBLE")
                self.assertTrue(admit_supervised_stereo_distance(distance, candidate))

    def test_unconfigured_lidar_cannot_be_admitted(self):
        self.assertEqual(select_target_range(self.target, [self.lidar], 10.1,
                         expected_rgb_calibration_id="rgb-v5").action, "PAUSE")


if __name__ == "__main__":
    unittest.main()
