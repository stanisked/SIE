import tempfile
from pathlib import Path
import unittest
from urllib.parse import urlsplit, parse_qs

from vision_core.tools.run_shadow_uart import BUILD, ShadowObservation, run_session
from vision_core.tools.probe_shadow_motor_off import probe
from vision_core.tools.shadow_range_gate import FRONT_FRAME, RgbTarget, TargetRange
from vision_core.tools.uart_http_transport import UartHttp


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class FakeController:
    def __init__(self):
        self.state = "READY"
        self.active = None
        self.last = None
        self.transition_seq = None
        self.steer_seq = None
        self.posts = []

    def __call__(self, method, url, timeout):
        p = urlsplit(url)
        args = {k: v[0] for k, v in parse_qs(p.query).items()}
        if method == "POST":
            self.posts.append((p.path, args))
            if p.path == "/shadow-start":
                assert self.state == "READY"
                self.active = args["command_id"]
                self.state = "DRIVING"
                self.transition_seq = int(args["seq"])
            elif p.path == "/shadow-pause":
                assert self.state == "DRIVING"
                self.state = "PAUSED"
                self.transition_seq = int(args["seq"])
            elif p.path == "/shadow-resume":
                assert self.state == "PAUSED"
                self.state = "DRIVING"
                self.transition_seq = int(args["seq"])
            elif p.path == "/steer-update":
                assert self.state == "DRIVING"
                self.steer_seq = int(args["seq"])
                return 200, {"accepted": True, "seq": self.steer_seq}
            elif p.path == "/stop":
                self.last = self.active
                self.active = None
                self.state = "READY"
            else:
                raise AssertionError(p.path)
            return 202, {"accepted": True}
        return 200, {"firmware_build_id": BUILD,
                     "shadow_api_version": "v1_paused_segment",
                     "boot_session_id": "boot-1", "state": self.state,
                     "active_command_id": self.active,
                     "last_command_id": self.last,
                     "right_pwm": 160 if self.state == "DRIVING" else 0,
                     "left_pwm": 160 if self.state == "DRIVING" else 0,
                     "bounded_fault_latched": False,
                     "steered_safety_latched": False,
                     "remote_stop_pending": False,
                     "shadow_transition_seq": self.transition_seq,
                     "active_steer_update_seq": self.steer_seq}


class ShadowRunnerTests(unittest.TestCase):
    def test_experimental_confidence_and_segment_fit(self):
        clock, controller = FakeClock(), FakeController()
        rgb = RgbTarget("person-7", clock(), 0., .5, "rgb-v5")
        rng = TargetRange("OV9281", "person-7", clock(), 1.02, .5,
            FRONT_FRAME, "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52",
            "association-1")
        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller,
                observations=[ShadowObservation(rgb, (rng,))],
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb-v5", minimum_confidence=.4,
                segment_m=.1, lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["operations"], ["START"])
        start = next(args for path, args in controller.posts if path == "/shadow-start")
        self.assertEqual(start["distance_m"], "0.050")

    def test_pause_resume_same_id_and_lidar_failover(self):
        clock = FakeClock()
        controller = FakeController()

        def sample(source):
            rgb = RgbTarget("person-7", clock(), 3.0, .95, "rgb-v5")
            rng = TargetRange(source, "person-7", clock(), 1.5, .9,
                FRONT_FRAME, "lidar-cal-1" if source == "LIDAR" else
                "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52",
                "association-1")
            return ShadowObservation(rgb, (rng,))

        def stream():
            yield sample("OV9281")
            clock.now += .20
            yield sample("OV9281")
            clock.now += .20
            yield ShadowObservation(None, ())
            clock.now += .20
            yield sample("LIDAR")

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller, observations=stream(),
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb-v5", lidar_calibration_id="lidar-cal-1",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "STEER", "PAUSE", "RESUME"])
        ids = {args["command_id"] for path, args in controller.posts if "command_id" in args}
        self.assertEqual(ids, {report["command_id"]})
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_no_associated_range_never_moves(self):
        clock, controller = FakeClock(), FakeController()
        rgb = RgbTarget("person-7", 10., 0., .95, "rgb-v5")
        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller,
                observations=[ShadowObservation(rgb, ())],
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb-v5",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["operations"], [])
        self.assertFalse(any(path in ("/shadow-start", "/shadow-resume")
                             for path, _ in controller.posts))
        self.assertEqual(controller.posts, [])

    def test_start_heartbeat_refusal_after_segment_pause_is_reconciled(self):
        class PauseBeforeHeartbeat(FakeController):
            def __call__(self, method, url, timeout):
                path = urlsplit(url).path
                if path == "/shadow-start":
                    code, body = super().__call__(method, url, timeout)
                    self.state = "STARTING"
                    return code, body
                if path == "/steer-update":
                    self.posts.append((path, {}))
                    self.state = "PAUSED"
                    return 409, {"accepted": False,
                                 "reason": "NO_MATCHING_ACTIVE_STEERED_FORWARD"}
                return super().__call__(method, url, timeout)

        clock, controller = FakeClock(), PauseBeforeHeartbeat()
        rgb = RgbTarget("p", clock(), 0., .9, "rgb")
        rng = TargetRange("LIDAR", "p", clock(), 1.5, .9,
                          FRONT_FRAME, "lidar", "association-1")
        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller,
                observations=[ShadowObservation(rgb, (rng,))],
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START"])
        self.assertEqual(controller.state, "READY")
        self.assertEqual([path for path, _ in controller.posts].count("/shadow-start"), 1)
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_start_heartbeat_refusal_waits_for_matching_brake_to_pause(self):
        class BrakingBeforePause(FakeController):
            def __init__(self):
                super().__init__()
                self.braking_reads = 0

            def __call__(self, method, url, timeout):
                path = urlsplit(url).path
                if path == "/shadow-start":
                    code, body = super().__call__(method, url, timeout)
                    self.state = "STARTING"
                    return code, body
                if path == "/steer-update":
                    self.posts.append((path, {}))
                    self.state = "BRAKING"
                    return 409, {"accepted": False,
                                 "reason": "NO_MATCHING_ACTIVE_STEERED_FORWARD"}
                if method == "GET" and self.state == "BRAKING":
                    self.braking_reads += 1
                    if self.braking_reads == 3:
                        self.state = "PAUSED"
                return super().__call__(method, url, timeout)

        clock, controller = FakeClock(), BrakingBeforePause()
        rgb = RgbTarget("p", clock(), 0., .9, "rgb")
        rng = TargetRange("LIDAR", "p", clock(), 1.5, .9,
                          FRONT_FRAME, "lidar", "association-1")
        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller,
                observations=[ShadowObservation(rgb, (rng,))],
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(controller.braking_reads, 3)
        self.assertEqual(report["operations"], ["START"])
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_start_heartbeat_refusal_wrong_command_stops(self):
        class WrongPausedCommand(FakeController):
            def __call__(self, method, url, timeout):
                path = urlsplit(url).path
                if path == "/shadow-start":
                    code, body = super().__call__(method, url, timeout)
                    self.state = "STARTING"
                    return code, body
                if path == "/steer-update":
                    self.posts.append((path, {}))
                    self.state = "PAUSED"
                    return 409, {"accepted": False,
                                 "reason": "NO_MATCHING_ACTIVE_STEERED_FORWARD"}
                code, body = super().__call__(method, url, timeout)
                if method == "GET" and self.state == "PAUSED":
                    body["active_command_id"] = "other-command"
                return code, body

        clock, controller = FakeClock(), WrongPausedCommand()
        rgb = RgbTarget("p", clock(), 0., .9, "rgb")
        rng = TargetRange("LIDAR", "p", clock(), 1.5, .9,
                          FRONT_FRAME, "lidar", "association-1")
        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller,
                observations=[ShadowObservation(rgb, (rng,))],
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("SHADOW_START_HEARTBEAT_REFUSED", report["error"])
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_automatic_segment_pause_then_resume(self):
        class AutoPause(FakeController):
            def __call__(self, method, url, timeout):
                code, body = super().__call__(method, url, timeout)
                if urlsplit(url).path == "/steer-update":
                    self.state = "PAUSED"
                if method == "GET" and self.state == "PAUSED":
                    body["state"] = "PAUSED"
                    body["right_pwm"] = body["left_pwm"] = 0
                return code, body

        clock, controller = FakeClock(), AutoPause()

        def stream():
            for _ in range(3):
                rgb = RgbTarget("p", clock(), 0., .9, "rgb")
                rng = TargetRange("LIDAR", "p", clock(), 1.5, .9,
                                   FRONT_FRAME, "lidar", "a")
                yield ShadowObservation(rgb, (rng,))
                clock.now += .2

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller, observations=stream(),
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["operations"], ["START", "STEER", "RESUME"])
        self.assertEqual(report["result"], "SESSION_ENDED")

    def test_settled_segment_requires_new_rgb_and_range_pair(self):
        class AutoPause(FakeController):
            def __call__(self, method, url, timeout):
                code, body = super().__call__(method, url, timeout)
                if urlsplit(url).path == "/steer-update":
                    self.state = "PAUSED"
                return code, body

        clock, controller = FakeClock(), AutoPause()

        def stream():
            for _ in range(3):
                rgb = RgbTarget("p", 10., 0., .9, "rgb")
                rng = TargetRange("LIDAR", "p", 10., 1.5, .9,
                                  FRONT_FRAME, "lidar", "old-pair")
                yield ShadowObservation(rgb, (rng,))
                clock.now += .2

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller, observations=stream(),
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["operations"], ["START", "STEER"])
        self.assertEqual(report["segments_started"], 1)
        self.assertEqual(report["reused_pair_refusals"], 1)
        self.assertFalse(any(path == "/shadow-resume" for path, _ in controller.posts))
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_segment_limit_stops_after_first_settled_step(self):
        class AutoPause(FakeController):
            def __call__(self, method, url, timeout):
                code, body = super().__call__(method, url, timeout)
                if urlsplit(url).path == "/steer-update":
                    self.state = "PAUSED"
                return code, body

        clock, controller = FakeClock(), AutoPause()

        def stream():
            for index in range(3):
                rgb = RgbTarget("p", clock(), 0., .9, "rgb")
                rng = TargetRange("LIDAR", "p", clock(), 1.5, .9,
                                  FRONT_FRAME, "lidar", f"pair-{index}")
                yield ShadowObservation(rgb, (rng,))
                clock.now += .2

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(request=controller, observations=stream(),
                journal_path=Path(directory) / "shadow.jsonl", clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                max_segments=1, lock_path=Path(directory) / "operator.lock")
        self.assertEqual(report["operations"], ["START", "STEER"])
        self.assertEqual(report["reason"], "SEGMENT_LIMIT_REACHED")
        self.assertEqual(report["segments_started"], 1)
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_shadow_uart_fields(self):
        fields = UartHttp._fields("POST", "http://uart.local/shadow-resume?"
            "boot_session_id=boot-1&command_id=one&seq=2&distance_m=0.300")
        self.assertEqual(fields, ["SHADOW_RESUME", "boot-1", "one", "2", "0.300"])
        self.assertEqual(UartHttp._fields("POST", "http://uart.local/shadow-pause?"
            "boot_session_id=boot-1&command_id=one&seq=3"),
            ["SHADOW_PAUSE", "boot-1", "one", "3"])

    def test_motor_off_probe_protocol(self):
        class MotorOff(FakeController):
            def __call__(self, method, url, timeout):
                p = urlsplit(url)
                args = {k: v[0] for k, v in parse_qs(p.query).items()}
                if p.path == "/shadow-start":
                    self.posts.append((p.path, args))
                    self.active = args["command_id"]
                    self.transition_seq = int(args["seq"])
                    self.state = "STARTING"
                    return 202, {"accepted": True}
                if p.path == "/shadow-resume":
                    self.posts.append((p.path, args))
                    assert self.state == "PAUSED"
                    self.state = "STARTING"
                    self.transition_seq = int(args["seq"])
                    return 202, {"accepted": True}
                if p.path == "/shadow-pause":
                    self.posts.append((p.path, args))
                    seq = int(args["seq"])
                    if self.state == "PAUSED" and seq == self.transition_seq:
                        return 200, {"duplicate": True}
                    assert self.state == "STARTING"
                    self.state = "PAUSED"
                    self.transition_seq = seq
                    return 202, {"accepted": True}
                if p.path == "/shadow-stop":
                    self.posts.append((p.path, args))
                    assert self.state == "PAUSED"
                    self.last = self.active
                    self.active = None
                    self.state = "READY"
                    self.transition_seq = int(args["seq"])
                    return 202, {"accepted": True}
                code, body = super().__call__(method, url, timeout)
                if method == "GET":
                    body["last_command_state"] = "STOPPED" if self.last else None
                    body["right_pwm"] = body["left_pwm"] = 0
                return code, body

        clock = FakeClock()
        controller = MotorOff()
        result = probe(controller, clock=clock, sleep=lambda _: None)
        self.assertTrue(result["same_id_resumed"])
        ids = {args["command_id"] for _, args in controller.posts}
        self.assertEqual(ids, {result["command_id"]})


if __name__ == "__main__":
    unittest.main()
