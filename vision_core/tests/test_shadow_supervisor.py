import unittest

from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange, RangeDecision,
)
from vision_core.tools.shadow_supervisor import ShadowSupervisor


class ShadowSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.runner = ShadowSupervisor("boot", "shadow-one")
        self.rgb = RgbTarget("person-7", 10., 3., .95, "rgb-v5")
        rng = TargetRange("LIDAR", "person-7", 10., 1.5, .9,
                          FRONT_FRAME, "lidar-cal-1", "association-1")
        self.go = RangeDecision("FOLLOW_ELIGIBLE", "FRESH_ASSOCIATED_RANGE", rng)
        self.pause = RangeDecision("PAUSE", "NO_FRESH_ASSOCIATED_RANGE")

    def confirm(self, intent, state, pwm=0):
        self.runner.acknowledge(intent, boot_session_id="boot",
                                active_command_id=None if state == "READY" else "shadow-one",
                                controller_state=state, right_pwm=pwm,
                                left_pwm=pwm, fault_latched=False,
                                transition_seq=intent.transition_seq,
                                last_command_id="shadow-one" if state == "READY" else None)

    def test_loss_reacquire_with_same_id(self):
        start = self.runner.observe(self.rgb, self.go, bias_pwm=10)
        self.assertEqual(start.operation, "START")
        self.confirm(start, "DRIVING", 170)
        pause = self.runner.observe(None, self.pause)
        self.assertEqual(pause.operation, "PAUSE")
        self.assertIsNone(self.runner.observe(self.rgb, self.go))
        with self.assertRaisesRegex(ValueError, "BRAKE_NOT_CONFIRMED"):
            self.confirm(pause, "PAUSED", 10)
        self.confirm(pause, "PAUSED")
        resume = self.runner.observe(self.rgb, self.go, bias_pwm=-15)
        self.assertEqual(resume.operation, "RESUME")
        self.assertEqual({start.command_id, pause.command_id, resume.command_id},
                         {"shadow-one"})
        self.assertEqual([start.transition_seq, pause.transition_seq,
                          resume.transition_seq], [1, 2, 3])
        self.confirm(resume, "DRIVING", 180)
        self.assertEqual(self.runner.observe(self.rgb, self.go).operation, "STEER")

    def test_identity_change_does_not_resume(self):
        start = self.runner.observe(self.rgb, self.go)
        self.confirm(start, "DRIVING", 150)
        pause = self.runner.observe(None, self.pause)
        self.confirm(pause, "PAUSED")
        other = RgbTarget("other", 11., 0., .99, "rgb-v5")
        self.assertIsNone(self.runner.observe(other, self.go))

    def test_conflict_pauses_and_operator_stop_is_terminal(self):
        start = self.runner.observe(self.rgb, self.go)
        self.confirm(start, "DRIVING", 150)
        pause = self.runner.observe(self.rgb, self.go,
                                    camera_encoder_disagreement=True)
        self.assertEqual(pause.operation, "PAUSE")
        self.confirm(pause, "PAUSED")
        stop = self.runner.observe(None, self.pause, operator_stop=True)
        self.confirm(stop, "READY")
        self.assertIsNone(self.runner.observe(self.rgb, self.go))

    def test_no_range_never_starts(self):
        self.assertIsNone(self.runner.observe(self.rgb, self.pause))

    def test_wrong_boot_and_stale_ack_refused(self):
        start = self.runner.observe(self.rgb, self.go)
        with self.assertRaisesRegex(ValueError, "ACK_MISMATCH"):
            self.runner.acknowledge(start, boot_session_id="other",
                active_command_id="shadow-one", controller_state="DRIVING",
                right_pwm=150, left_pwm=150, fault_latched=False,
                transition_seq=1)
        with self.assertRaisesRegex(ValueError, "ACK_MISMATCH"):
            self.runner.acknowledge(start, boot_session_id="boot",
                active_command_id="shadow-one", controller_state="DRIVING",
                right_pwm=150, left_pwm=150, fault_latched=False,
                transition_seq=0)

    def test_start_heartbeat_has_separate_steer_sequence(self):
        start = self.runner.observe(self.rgb, self.go)
        heartbeat = self.runner.starting_heartbeat()
        self.assertEqual((start.transition_seq, heartbeat.transition_seq), (1, 1))
        self.confirm(start, "DRIVING", 160)
        steer = self.runner.observe(self.rgb, self.go)
        self.assertEqual(steer.transition_seq, 2)
        self.confirm(steer, "DRIVING", 160)
        pause = self.runner.observe(None, self.pause)
        self.assertEqual(pause.transition_seq, 2)

    def test_natural_segment_completion_keeps_id(self):
        start = self.runner.observe(self.rgb, self.go)
        self.confirm(start, "DRIVING", 170)
        self.runner.reconcile(boot_session_id="boot", active_command_id="shadow-one",
            last_command_id=None, controller_state="PAUSED", right_pwm=0,
            left_pwm=0, fault_latched=False)
        self.assertEqual(self.runner.observe(self.rgb, self.go).operation, "RESUME")


if __name__ == "__main__":
    unittest.main()
