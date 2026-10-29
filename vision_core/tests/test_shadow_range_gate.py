import unittest

from vision_core.tools.shadow_range_gate import (
    ACTIVE_STEREO_SHA256, FRONT_FRAME, RgbTarget, TargetRange,
    select_target_range,
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

    def test_unconfigured_lidar_cannot_be_admitted(self):
        self.assertEqual(select_target_range(self.target, [self.lidar], 10.1,
                         expected_rgb_calibration_id="rgb-v5").action, "PAUSE")


if __name__ == "__main__":
    unittest.main()
