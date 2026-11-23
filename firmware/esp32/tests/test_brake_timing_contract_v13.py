"""Static source-level checks for diagnostic-only v13 continuity candidate.

These tests do not establish Arduino compilation or physical brake behavior.
"""
import re
import unittest
from pathlib import Path

NAME = "traction_control_v4_2_uart_shadow_v13_brake_continuity"
SKETCH = Path(__file__).resolve().parents[1] / NAME / (NAME + ".ino")


class BrakeTimingContractV12(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s = SKETCH.read_text()

    def test_hardware_and_limits_unchanged(self):
        expected = {
            "SHADOW_TURN_GUARD_MAX_DRIVE_MS": "280",
            "BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS": "6",
            "BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES": "0",
            "BOUNDED_ACTIVE_BRAKE_HOLD_MS": "400",
            "ZK_D0": "25",
            "ZK_D1": "33",
            "ZK_D2": "27",
            "ZK_D3": "14",
        }
        for key, val in expected.items():
            with self.subTest(key=key):
                self.assertRegex(self.s, rf"constexpr\s+[^;\n]+\s+{key}\s*=\s*{val}\s*;")
        self.assertIn("Serial2.begin(115200, SERIAL_8N1, 16, 17);", self.s)

    def test_trace_is_bound_to_brake_guard_generation(self):
        self.assertIn("shadowTurnBrakeTrace = ShadowTurnBrakeTrace{};", self.s)
        self.assertIn("shadowTurnBrakeTrace = trace;", self.s)
        self.assertIn("shadowTurnGuardGeneration.load() == eventGeneration", self.s)
        self.assertIn("shadowTurnBrakePublishedGeneration.store(", self.s)
        self.assertIn("portENTER_CRITICAL(&shadowTurnDiagnosticMux);", self.s)

    def test_output_timestamps_bracket_driver_calls(self):
        f = self.s.split("void brakeMotorsWithTrace(", 1)[1].split("void brakeMotors()", 1)[0]
        positions = [f.index(s) for s in (
            "lockMotorOutputs();",
            "trace->mutexAcquiredUs = micros();",
            "analogWrite(ZK_D0, 255);",
            "analogWrite(ZK_D1, 255);",
            "analogWrite(ZK_D2, 255);",
            "analogWrite(ZK_D3, 255);",
            "trace->outputCommandsIssuedUs = micros();",
            "unlockMotorOutputs();",
        )]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("brakeMotorsWithTrace(&trace);", self.s)

    def test_read_only_json_extension(self):
        handler = self.s.split("void handleFaultDiagnostics()", 1)[1].split("String buildUartStatus()", 1)[0]
        for field in (
            "shadow_turn_request_to_lock_us",
            "shadow_turn_lock_to_write_us",
            "shadow_turn_request_to_write_us",
            "shadow_turn_right_count_at_write",
            "shadow_turn_left_count_at_write",
        ):
            with self.subTest(field=field):
                self.assertIn(field, handler)
        for action in ("brakeMotors(", "startMotion(", "handleAckFault(", "stopMotors("):
            self.assertNotIn(action, handler)

    def test_build_id_is_explicit(self):
        self.assertIn('"v4_2_uart_shadow_v13_brake_continuity"', self.s)


if __name__ == "__main__":
    unittest.main()
