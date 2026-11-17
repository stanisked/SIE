"""ESP32 v7 short steered-forward brake: source and UART contract tests.

No hardware actions or motor I/O. Arduino compilation and floor calibration
remain separate from these regression tests.
"""
from pathlib import Path
import unittest

from vision_core.tests.test_shadow_edge_turn_v5 import V5Controller
from vision_core.tests.test_shadow_edge_turn_v4 import Clock, normal
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD_V7, EDGE_TURN_CONTRACTS, run_session,
)
import tempfile


SOURCE = (Path(__file__).resolve().parents[2] /
          "firmware/esp32/traction_control_v4_2_uart_shadow_v7_short_forward_brake" /
          "traction_control_v4_2_uart_shadow_v7_short_forward_brake.ino")
OLD_SOURCE = (Path(__file__).resolve().parents[2] /
              "firmware/esp32/traction_control_v4_2_uart_shadow_v5_edge_turn_guard" /
              "traction_control_v4_2_uart_shadow_v5_edge_turn_guard.ino")


class V7Controller(V5Controller):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)
        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V7
        return code, body


class ShortSteeredForwardBrakeV7Tests(unittest.TestCase):
    def test_v7_uses_existing_uart_contract(self):
        self.assertIn((EDGE_TURN_BUILD_V7, "v2_paused_edge_turn"),
                      EDGE_TURN_CONTRACTS)
        clock, controller = Clock(), V7Controller()
        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller, observations=[normal(clock, distance=.732)],
                journal_path=Path(d) / "bridge.jsonl",
                rgb_calibration_id="test-rgb",
                clock=clock, lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=False)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START"])
        starts = [args for path, args in controller.posts
                  if path == "/shadow-start"]
        self.assertEqual(len(starts), 1)
        self.assertEqual(starts[0]["distance_m"], "0.062")
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_v7_contains_independent_predictive_brake_during_starting(self):
        src = SOURCE.read_text()
        self.assertIn('v4_2_uart_shadow_v7_short_forward_brake', src)
        self.assertIn('steeredShortGuardActive.load() && steeredSafetyActive.load()', src)
        self.assertIn('steeredShortBrakeRequested.compare_exchange_strong(', src)
        self.assertIn('activeSteeredForward && steeredShortBrakeRequested.load()', src)
        self.assertIn('beginBoundedTargetSettling();', src)
        self.assertIn('STEERED_SHORT_BRAKE_RESERVE_M = 0.030f', src)
        self.assertIn('STEERED_SHORT_BRAKE_MIN_TRIGGER_M = 0.005f', src)
        self.assertIn('steeredShortBrakeRequested.load()) {\n    brakeMotors();', src)
        self.assertIn('short_forward_brake_trigger_counts', src)

    def test_42mm_has_guard_ahead_of_unchanged_hard_center_limit(self):
        # Policy calculation is a diagnostic sanity check; firmware still
        # needs Arduino compile and real motor stopping-distance verification.
        target = .042
        hard_center = target + .020
        predictive_trigger = max(.005, target - .030)
        self.assertAlmostEqual(predictive_trigger, .012, places=6)
        self.assertAlmostEqual(hard_center, .062, places=6)
        self.assertLess(predictive_trigger, target)
        self.assertLess(predictive_trigger + .017, hard_center)
        self.assertLessEqual(target, .100)

    def test_hard_safety_envelope_and_existing_shadow_turn_unchanged(self):
        old = OLD_SOURCE.read_text()
        new = SOURCE.read_text()
        for unchanged in (
            'STEERED_CENTER_EXTRA_M = 0.020f',
            'STEERED_WHEEL_EXTRA_M = 0.075f',
            'STEERED_MAX_HEADING_RAD = 0.350f',
            'STEERED_LOOP_STALL_MS = 300',
            'right + left >= steeredSafetyCenterLimitSum.load()',
            'if (activeBoundedBrakeStarted) {',
            'shadowTurnGuardActive.load() &&',
            'STEERED_SAFETY_CENTER_LIMIT',
        ):
            with self.subTest(unchanged=unchanged):
                self.assertIn(unchanged, old)
                self.assertIn(unchanged, new)
        self.assertIn('if (activeSteeredForward && steeredShortBrakeRequested.load()', new)
        self.assertNotIn('steeredShortBrakeRequested', old)


if __name__ == "__main__":
    unittest.main()
