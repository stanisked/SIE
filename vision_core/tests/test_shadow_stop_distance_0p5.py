"""Regression: 0.5 m is the observed front-edge FOLLOW stop threshold.

These tests use a fake UART transport; no motors are energized.
"""
import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_shadow_edge_turn_v4 import Clock, normal
from vision_core.tests.test_shadow_edge_turn_v5 import V5Controller
from vision_core.tools.run_shadow_uart import run_session
from vision_core.tools.shadow_range_gate import (
    SHADOW_STOP_DISTANCE_M, STEREO_SUPERVISED_MIN_M,
    STEREO_SUPERVISED_MAX_M,
)


class ShadowStopHalfMeterTests(unittest.TestCase):
    def test_configured_stereo_domain_and_stop_distance(self):
        self.assertEqual(SHADOW_STOP_DISTANCE_M, .5)
        self.assertEqual(STEREO_SUPERVISED_MIN_M, .5)
        self.assertEqual(STEREO_SUPERVISED_MAX_M, 4.5)

    def run_one(self, distance):
        clock, controller = Clock(), V5Controller()
        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=[normal(clock, distance=distance)],
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=False)
        return report, controller.posts

    def test_half_meter_hold_never_starts_motor(self):
        report, posts = self.run_one(.5)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], [])
        self.assertEqual(posts, [])

    def test_hysteresis_zone_does_not_start_motor(self):
        report, posts = self.run_one(.65)
        self.assertEqual(report["operations"], [])
        self.assertEqual(posts, [])

    def test_slightly_above_clearance_starts_bounded_short_segment(self):
        report, posts = self.run_one(.72)
        self.assertEqual(report["operations"], ["START"])
        starts = [args for path, args in posts if path == "/shadow-start"]
        self.assertEqual(len(starts), 1)
        # 0.72 - 0.50 stop - 0.15 resume margin - 0.02 command reserve
        self.assertEqual(starts[0]["distance_m"], "0.050")
        self.assertEqual(posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
