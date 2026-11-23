"""Static, source-bound regression policy for ESP32 v13 BRAKE handoff.

This does NOT validate electrical pin levels, motor stopping distance or
Arduino compilation. It checks the exact intended delta from v12 and
pinpoints that BRAKE remains asserted at the main-loop handoff.
"""
import hashlib
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V12_NAME = "traction_control_v4_2_uart_shadow_v12_brake_timing"
V13_NAME = "traction_control_v4_2_uart_shadow_v13_brake_continuity"
V12 = ROOT / V12_NAME / (V12_NAME + ".ino")
V13 = ROOT / V13_NAME / (V13_NAME + ".ino")

OLD_ID = 'constexpr char FIRMWARE_BUILD_ID[] = "v4_2_uart_shadow_v12_brake_timing";'
NEW_ID = 'constexpr char FIRMWARE_BUILD_ID[] = "v4_2_uart_shadow_v13_brake_continuity";'
OLD_BLOCK = """    // Explicitly cut directed drive before engaging the known TA6586 brake
    // state. From here until completion no path may issue directed drive PWM.
    stopMotors();
    brakeMotors();
"""
NEW_BLOCK = """    // The independent SHADOW_TURN guard may already have asserted BRAKE.
    // Do not release BRAKE to COAST during the main-loop handoff.
    // brakeMotors() serializes motor writes and overrides directed PWM.
    brakeMotors();
"""
V12_SHA256 = "eaad601bfd4a04002fcd7f33e3cfa89d394d51b2f21f21205bce62a4d5633324"


def body(source, begin, end):
    return source.split(begin, 1)[1].split(end, 1)[0]


class BrakeContinuityContractV13(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v12 = V12.read_text()
        cls.v13 = V13.read_text()

    def test_v12_exact_baseline_source(self):
        self.assertEqual(hashlib.sha256(V12.read_bytes()).hexdigest(), V12_SHA256)

    def test_intended_minimal_diff_only(self):
        self.assertEqual(self.v12.count(OLD_ID), 1)
        self.assertEqual(self.v12.count(OLD_BLOCK), 1)
        self.assertEqual(self.v13, self.v12.replace(OLD_ID, NEW_ID).replace(OLD_BLOCK, NEW_BLOCK))

    def test_first_brake_handoff_has_no_coast(self):
        f = self.v13.split(
            'void beginBoundedBraking(bool faultPending, const String &reason)\n{', 1
        )[1].split('\nbool enforceBoundedEncoderLimit()', 1)[0]
        handoff = f.split('if (!activeBoundedBrakeStarted) {', 1)[1].split(
            'activeBoundedBrakeStarted = true;', 1)[0]
        self.assertNotIn('stopMotors();', handoff)
        self.assertNotIn('setDrivePwm(', handoff)
        self.assertNotIn('applyMicroTurnOutputs(', handoff)
        self.assertEqual(handoff.count('brakeMotors();'), 1)
        self.assertLess(handoff.index('brakeMotors();'), handoff.index('readMotionCounts('))

    def test_brake_guard_output_write_is_serialized(self):
        guard = self.v13.split('void steeredSafetyTask(void*)', 1)[1].split('\nvoid ', 1)[0]
        self.assertIn('shadowTurnGuardBrakeRequested.store(true)', guard)
        self.assertIn('brakeMotorsWithTrace(&trace);', guard)
        brake = self.v13.split('void brakeMotorsWithTrace(', 1)[1].split(
            '\nvoid brakeMotors()', 1)[0]
        for item in ('lockMotorOutputs();', 'analogWrite(ZK_D0, 255);',
                     'analogWrite(ZK_D1, 255);', 'analogWrite(ZK_D2, 255);',
                     'analogWrite(ZK_D3, 255);', 'unlockMotorOutputs();'):
            self.assertIn(item, brake)

    def test_active_brake_hold_and_intentional_coast_unchanged(self):
        code = self.v13.split('void updateBoundedBraking(uint32_t now)', 1)[1].split(
            '\nvoid ', 1)[0]
        self.assertIn('now - activeBoundedBrakeStartTime < BOUNDED_ACTIVE_BRAKE_HOLD_MS', code)
        self.assertIn('brakeMotors();', code)
        self.assertIn('stopMotors();', code)  # Intentional coast AFTER hold, not yet changed.
        self.assertLess(code.index('brakeMotors();'), code.index('stopMotors();'))
        self.assertIn('BOUNDED_ACTIVE_BRAKE_HOLD_MS = 400;', self.v13)

    def test_no_limit_or_pin_changes(self):
        for token in ('BOUNDED_MICRO_TURN_BRAKE_RESERVE_COUNTS = 6;',
                      'BOUNDED_MICRO_TURN_MAX_CORRECTION_PULSES = 0;',
                      'SHADOW_TURN_GUARD_MAX_DRIVE_MS = 280;',
                      'ZK_D0 = 25;', 'ZK_D1 = 33;',
                      'ZK_D2 = 27;', 'ZK_D3 = 14;',
                      'Serial2.begin(115200, SERIAL_8N1, 16, 17);'):
            self.assertIn(token, self.v13)

    def test_no_new_fault_ack_or_motion_path(self):
        self.assertIn('boundedFaultLatched = true;', self.v13)
        self.assertIn('captureBoundedFault(', self.v13)
        self.assertIn('shadowTurnGuardBrakeRequested.store(true)', self.v13)
        self.assertNotIn('autoAckFault(', self.v13)

    def test_headers_unchanged_from_v12(self):
        for name in ('brake_timing.h', 'command_replay_guard.h',
                     'fault_snapshot.h', 'remote_stop_observer.h'):
            with self.subTest(name=name):
                self.assertEqual((ROOT / V12_NAME / name).read_bytes(),
                                 (ROOT / V13_NAME / name).read_bytes())


if __name__ == '__main__':
    unittest.main()
