"""Source contract for the isolated v5 motor-power candidate."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
V4 = ROOT / 'traction_control_v4_2_loaded_start_v4' / 'traction_control_v4_2_loaded_start_v4.ino'
V5 = ROOT / 'traction_control_v4_2_micro_turn_power_v5' / 'traction_control_v4_2_micro_turn_power_v5.ino'


def function(source, signature):
    start = source.index(signature)
    end = source.index('\n}\n', start) + 3
    return source[start:end]


class MicroTurnPowerCandidateTests(unittest.TestCase):
    def test_bounded_guards_and_stop_path_match_v4(self):
        old, new = V4.read_text(), V5.read_text()
        for name in ('bool enforceBoundedEncoderLimit()',
                     'void beginBoundedBraking(bool',
                     'void handleStop()'):
            with self.subTest(name=name):
                self.assertEqual(function(old, name), function(new, name))

    def test_stronger_start_is_micro_turn_only(self):
        source = V5.read_text()
        self.assertIn('FIRMWARE_BUILD_ID[] = "v4_2_micro_turn_power_v5"', source)
        self.assertIn('constexpr int MICRO_TURN_BREAKAWAY_START_PWM = 210;', source)
        self.assertIn('constexpr int MICRO_TURN_BREAKAWAY_CEILING_PWM = 230;', source)
        self.assertIn('constexpr int BREAKAWAY_START_PWM = 180;', source)
        startup = function(source, 'void updateStarting(uint32_t now)')
        self.assertIn('const int startPwm = microTurn', startup)
        self.assertIn('? MICRO_TURN_BREAKAWAY_START_PWM : BREAKAWAY_START_PWM;', startup)
        self.assertIn('microTurn ? max(rightSustainPwm(), rightBreakawayPwm)', startup)
        self.assertIn('microTurn ? max(leftSustainPwm(), leftBreakawayPwm)', startup)
        self.assertIn('BREAKAWAY_TIMEOUT_MS', startup)


if __name__ == '__main__':
    unittest.main()
