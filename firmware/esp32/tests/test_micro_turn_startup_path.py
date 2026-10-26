"""Guard the bounded turn startup path against bypassing its timeout."""

import unittest
from pathlib import Path


SKETCH = (Path(__file__).resolve().parents[1] /
          'traction_control_v4_2_bounded_motion_api' /
          'traction_control_v4_2_bounded_motion_api.ino')


class MicroTurnStartupPathTests(unittest.TestCase):
    def test_starting_reaches_breakaway_ramp_and_timeout(self):
        source = SKETCH.read_text()
        guard = source.split('bool enforceBoundedEncoderLimit()', 1)[1].split(
            'int32_t bounded', 1)[0]
        self.assertLess(guard.index('rightCount >= activeBoundedRightLimitCounts'),
                        guard.index('if (motionState == MotionState::STARTING)'))
        self.assertLess(guard.index('if (motionState == MotionState::STARTING)'),
                        guard.index('BoundedMotionProfile::BOUNDED_MICRO_TURN_V1'))
        self.assertIn('return false;', guard.split(
            'if (motionState == MotionState::STARTING)', 1)[1].split('}', 1)[0])

        update = source.split('void updateMotion()', 1)[1].split('void updateSquareSequence()', 1)[0]
        self.assertIn('if (enforceBoundedEncoderLimit())', update)
        self.assertIn('case MotionState::STARTING:\n      updateStarting(now);', update)
        starting = source.split('void updateStarting(uint32_t now)', 1)[1].split(
            'float loadedFeedForward(', 1)[0]
        self.assertIn('elapsed >= BREAKAWAY_TIMEOUT_MS', starting)
        self.assertIn('boundedBreakawayPwmCeiling()', starting)

    def test_micro_turn_drive_cannot_bypass_deadline_and_stall(self):
        source = SKETCH.read_text()
        update = source.split('void updateMotion()', 1)[1].split(
            'void updateSquareSequence()', 1)[0]
        self.assertLess(update.index('now - motionStartTime >= activeTimeoutMs'),
                        update.index('if (enforceBoundedEncoderLimit())'))
        guard = source.split('bool enforceBoundedEncoderLimit()', 1)[1].split(
            'void ', 1)[0]
        unbraked = guard.split('// When neither wheel has reached its brake point', 1)[1]
        self.assertIn('return false;', unbraked.split('const int32_t syncErrorCounts', 1)[0])
        drive = source.split('void updateDriving(uint32_t now)', 1)[1].split(
            'void updateMotion()', 1)[0]
        self.assertIn('rightTargetSpeed > STALL_CHECK_MIN_TARGET_M_S || microTurnBeforeBrake', drive)
        self.assertIn('leftTargetSpeed > STALL_CHECK_MIN_TARGET_M_S || microTurnBeforeBrake', drive)


if __name__ == '__main__':
    unittest.main()
