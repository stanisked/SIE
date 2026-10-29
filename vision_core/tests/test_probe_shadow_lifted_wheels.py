"""Bench probe checks: same command ID, physical-stop telemetry, emergency stop."""
import io
import unittest
from urllib.parse import parse_qs, urlsplit

from vision_core.tools.probe_shadow_lifted_wheels import (
    BUILD, check_pause_settled, probe,
)


class Controller:
    def __init__(self, fail=False, starts_before_drive=False,
                 reversed_encoders=False):
        self.fail = fail
        self.starts_before_drive = starts_before_drive
        self.reversed_encoders = reversed_encoders
        self.calls = []
        self.state = "READY"
        self.id = None
        self.seq = None
        self.segments = 0
        self.raw = 0
        self.last = None

    def __call__(self, method, url, timeout):
        parsed = urlsplit(url)
        path = parsed.path
        query = {k: v[0] for k, v in parse_qs(parsed.query).items()}
        self.calls.append((method, path, query))
        if path == "/status":
            driving = self.state == "DRIVING"
            return 200, {"firmware_build_id": BUILD,
                         "shadow_api_version": "v1_paused_segment",
                         "boot_session_id": "TESTBOOT", "state": self.state,
                         "active_command_id": self.id,
                         "last_command_id": self.last,
                         "last_command_state": "STOPPED" if self.last else None,
                         "shadow_transition_seq": self.seq,
                         "shadow_segments_started": self.segments,
                         "right_pwm": 120 if driving else 0,
                         "left_pwm": 120 if driving else 0,
                         "right_encoder_raw_count": (self.raw if self.reversed_encoders
                                                     else -self.raw),
                         "left_encoder_raw_count": (-self.raw if self.reversed_encoders
                                                    else self.raw),
                         "right_distance_m": .008 if driving else 0,
                         "left_distance_m": .008 if driving else 0,
                         "bounded_fault_latched": self.fail and self.id is not None,
                         "steered_safety_latched": False,
                         "remote_stop_pending": False}
        if path == "/shadow-start":
            self.id = query["command_id"]
            self.seq = 1
            self.segments = 1
            self.raw = 10
            self.state = "STARTING" if self.starts_before_drive else "DRIVING"
            return 202, {"accepted": True}
        if path == "/shadow-resume":
            assert query["command_id"] == self.id
            self.seq = 3
            self.segments = 2
            self.raw = 20
            self.state = "STARTING" if self.starts_before_drive else "DRIVING"
            return 202, {"accepted": True}
        if path == "/steer-update":
            self.state = "DRIVING"
            return 200, {"accepted": True}
        if path == "/shadow-pause":
            self.seq = int(query["seq"])
            self.state = "PAUSED"
            return 202, {"accepted": True}
        if path in ("/shadow-stop", "/stop"):
            self.last = self.id
            self.id = None
            self.state = "READY"
            return 202, {"accepted": True}
        raise AssertionError(path)


class TestLiftedProbe(unittest.TestCase):
    def test_two_segments_one_id_and_terminal_stop(self):
        fake = Controller()
        ticks = iter(i * .05 for i in range(1000))
        result = probe(fake, io.StringIO(), clock=lambda: next(ticks),
                       sleep=lambda _: None)
        self.assertEqual(result["result"], "TWO_SEGMENTS_PAUSED_AND_RESUMED")
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(len(result["segments"]), 2)
        self.assertEqual([p for _, p, _ in fake.calls if p.startswith("/shadow-")],
                         ["/shadow-start", "/shadow-pause", "/shadow-resume",
                          "/shadow-pause", "/shadow-stop"])
        ids = {q["command_id"] for _, p, q in fake.calls if p.startswith("/shadow-")}
        self.assertEqual(ids, {result["command_id"]})

    def test_fault_requests_terminal_stop(self):
        fake = Controller(fail=True)
        ticks = iter(i * .05 for i in range(1000))
        result = probe(fake, io.StringIO(), clock=lambda: next(ticks),
                       sleep=lambda _: None)
        self.assertEqual(result["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("CONTROLLER_FAULT", result["error"])
        self.assertIn("/stop", [p for _, p, _ in fake.calls])

    def test_zero_bias_heartbeat_during_starting(self):
        fake = Controller(starts_before_drive=True)
        ticks = iter(i * .05 for i in range(1000))
        result = probe(fake, io.StringIO(), clock=lambda: next(ticks),
                       sleep=lambda _: None)
        self.assertTrue(result["stop_confirmed"])
        heartbeats = [q for _, path, q in fake.calls if path == "/steer-update"]
        self.assertEqual(len(heartbeats), 2)
        self.assertEqual([h["bias_pwm"] for h in heartbeats], ["0", "0"])

    def test_reversed_encoder_signs_do_not_pass_and_stop_is_sent(self):
        fake = Controller(reversed_encoders=True)
        ticks = iter(i * .05 for i in range(2000))
        result = probe(fake, io.StringIO(), clock=lambda: next(ticks),
                       sleep=lambda _: None)
        self.assertEqual(result["result"], "STOPPED_ON_REFUSAL")
        self.assertEqual(result["segments"], [])
        self.assertTrue(result["stop_confirmed"])
        self.assertIn("/stop", [p for _, p, _ in fake.calls])

    def test_nonzero_pause_drift_is_rejected(self):
        a = {"state": "PAUSED", "remote_stop_pending": False,
             "right_pwm": 0, "left_pwm": 0,
             "right_encoder_raw_count": 5, "left_encoder_raw_count": 7}
        b = {**a, "left_encoder_raw_count": 10}
        with self.assertRaisesRegex(RuntimeError, "WHEEL_MOVED_AFTER_PAUSE"):
            check_pause_settled(a, b)


if __name__ == "__main__":
    unittest.main()
