"""SHADOW v4: fail-closed UART turn inside an existing paused RGB session."""
import json
import tempfile
import unittest
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

from vision_core.tools.run_shadow_uart import (
    BUILD, ShadowObservation, run_session, select_edge_turn,
)
from vision_core.tools.shadow_range_gate import (
    RgbTarget, TargetRange, FRONT_FRAME, ACTIVE_STEREO_SHA256,
)
from vision_core.tools.shadow_supervisor import ShadowSupervisor
from vision_core.tools.uart_http_transport import UartHttp


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class V4Controller:
    def __init__(self):
        self.active = None
        self.last = None
        self.state = "READY"
        self.seq = 0
        self.steer_seq = 0
        self.turn_result = "NONE"
        self.posts = []
        self.last_turn_args = None

    def __call__(self, method, url, timeout):
        path = urlsplit(url).path
        args = {k: v[0] for k, v in parse_qs(urlsplit(url).query).items()}
        if method == "POST":
            self.posts.append((path, args))
            if path == "/shadow-start":
                assert self.state == "READY"
                self.active = args["command_id"]
                self.seq = int(args["seq"])
                self.state = "DRIVING"
            elif path == "/shadow-pause":
                assert self.state == "DRIVING"
                self.state = "PAUSED"
                self.seq = int(args["seq"])
            elif path == "/shadow-turn":
                assert self.state == "PAUSED"
                assert self.active == args["command_id"]
                assert self.seq + 1 == int(args["seq"])
                assert 1 <= float(args["angle_deg"]) <= 10
                self.seq = int(args["seq"])
                self.last_turn_args = args
                # Simulate the encoder-limited STARTING/DRIVING/braking sequence
                # as already finished, with zero PWM and matching turn result.
                self.turn_result = "SUCCESS"
            elif path == "/shadow-resume":
                assert self.state == "PAUSED"
                self.seq = int(args["seq"])
                self.state = "DRIVING"
                self.turn_result = "NONE"
            elif path == "/steer-update":
                assert self.state == "DRIVING"
                self.steer_seq = int(args["seq"])
                return 200, {"accepted": True}
            elif path == "/stop":
                self.last = self.active
                self.active = None
                self.state = "READY"
            else:
                raise AssertionError(path)
            return 202, {"accepted": True}
        assert method == "GET" and path == "/status"
        return 200, {
            "firmware_build_id": BUILD,
            "shadow_api_version": "v2_paused_edge_turn",
            "boot_session_id": "test-boot",
            "state": self.state,
            "active_command_id": self.active,
            "last_command_id": self.last,
            "shadow_transition_seq": self.seq,
            "active_steer_update_seq": self.steer_seq,
            "shadow_turn_result": self.turn_result,
            "right_pwm": 170 if self.state == "DRIVING" else 0,
            "left_pwm": 170 if self.state == "DRIVING" else 0,
            "bounded_fault_latched": False,
            "steered_safety_latched": False,
            "remote_stop_pending": False,
            "heading_rad": -0.130,
            "right_encoder_raw_count": -28,
            "left_encoder_raw_count": -29,
        }


def normal(clock, *, distance=1.5, observed_s=None):
    t = clock() if observed_s is None else observed_s
    target = RgbTarget("person-1", t, 0., .95, "test-rgb")
    measured = TargetRange("OV9281", "person-1", t, distance, .9,
                           FRONT_FRAME, ACTIVE_STEREO_SHA256, "pair-" + str(t))
    return ShadowObservation(target, (measured,))


def edge(clock):
    return ShadowObservation(RgbTarget("person-1", clock(), 13., .95, "test-rgb"),
                             (), edge_recovery_bearing_deg=13.)


class ShadowEdgeTurnV4Tests(unittest.TestCase):
    def test_recent_same_person_clearance_required(self):
        t = RgbTarget("p", 10.2, 13., .9, "rgb")
        self.assertEqual(select_edge_turn(t, 13., ("p", 10., 1.5), 10.3),
                         ("RIGHT", 8.0))
        self.assertEqual(select_edge_turn(t, -4., ("p", 10., 1.5), 10.3),
                         ("LEFT", 4.0))
        self.assertIsNone(select_edge_turn(t, 13., ("p", 10., 1.2), 10.3))
        self.assertIsNone(select_edge_turn(t, 13., ("other", 10., 1.5), 10.3))
        self.assertIsNone(select_edge_turn(t, 13., ("p", 8., 1.5), 10.3))
        self.assertIsNone(select_edge_turn(t, 13., ("p", 10., 1.5), 11.0))

    def test_supervisor_turn_preserves_track_and_transition_sequence(self):
        s = ShadowSupervisor("boot", "cmd")
        s.track_id, s.state, s.seq = "person-1", "PAUSED", 3
        i = s.turn("RIGHT", 8.)
        self.assertEqual((i.operation, i.transition_seq, i.command_id),
                         ("TURN", 4, "cmd"))
        self.assertEqual((i.turn_direction, i.turn_angle_deg), ("RIGHT", 8.))
        s.acknowledge(i, boot_session_id="boot", active_command_id="cmd",
                      controller_state="PAUSED", right_pwm=0, left_pwm=0,
                      fault_latched=False, transition_seq=4)
        self.assertEqual(s.state, "PAUSED")
        self.assertEqual(s.track_id, "person-1")
        self.assertEqual(s.seq, 4)
        with self.assertRaises(ValueError):
            s.turn("LEFT", 11.)

    def test_uart_turn_encoding(self):
        fields = UartHttp._fields("POST", "http://uart.local/shadow-turn?"
            "boot_session_id=test-boot&command_id=cmd&seq=4&direction=RIGHT&angle_deg=8.00")
        self.assertEqual(fields, ["SHADOW_TURN", "test-boot", "cmd", "4", "RIGHT", "8.00"])
        with self.assertRaises(ValueError):
            UartHttp._fields("POST", "http://uart.local/shadow-turn?"
                "boot_session_id=test-boot&command_id=cmd&seq=4&angle_deg=8.00")

    def test_follow_pause_turn_pause_fresh_follow(self):
        clock, device = Clock(), V4Controller()

        def samples():
            yield normal(clock)
            clock.now += .15
            yield edge(clock)     # active forward must PAUSE first
            clock.now += .15
            yield edge(clock)     # now turn once from PAUSED
            clock.now += .10
            yield normal(clock, observed_s=10.20)  # pre-turn pair must be refused
            clock.now += .20
            yield normal(clock)   # genuinely fresh pair resumes

        with tempfile.TemporaryDirectory() as d:
            journal = Path(d) / "bridge.jsonl"
            report = run_session(request=device, observations=samples(),
                journal_path=journal, rgb_calibration_id="test-rgb",
                clock=clock, lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=True)
            events = [json.loads(s) for s in journal.read_text().splitlines()]
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["edge_turns_started"], 1)
        self.assertEqual(report["operations"], ["START", "PAUSE", "TURN", "RESUME"])
        self.assertEqual(device.last_turn_args["direction"], "RIGHT")
        self.assertEqual(device.last_turn_args["angle_deg"], "8.00")
        self.assertEqual(len([r for r in events if r.get("event") == "EDGE_TURN_SETTLED"]), 1)
        self.assertEqual(len([r for r in events if r.get("reason") == "POST_TURN_FRESH_PAIR_REQUIRED"]), 1)
        self.assertEqual(device.posts[-1][0], "/stop")
        self.assertEqual(report["final_status"]["state"], "READY")
        self.assertEqual(report["final_status"]["left_pwm"], 0)

    def test_clearance_refuses_turn_but_keeps_pause(self):
        clock, device = Clock(), V4Controller()
        def samples():
            yield normal(clock, distance=1.22)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)
        with tempfile.TemporaryDirectory() as d:
            report = run_session(request=device, observations=samples(),
                journal_path=Path(d) / "bridge.jsonl", rgb_calibration_id="test-rgb",
                clock=clock, lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=True)
        self.assertEqual(report["operations"], ["START", "PAUSE"])
        self.assertEqual(report["edge_turns_started"], 0)
        self.assertFalse(any(path == "/shadow-turn" for path, _ in device.posts))
        self.assertEqual(report["final_status"]["state"], "READY")


if __name__ == "__main__":
    unittest.main()
