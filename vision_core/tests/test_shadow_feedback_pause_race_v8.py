"""Regression for post-START freshness priority and ESP32 feedback pause.

These tests make no UART connections and never enable physical motors.
"""
import json
import tempfile
import unittest
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import FRONT_FRAME, RgbTarget, TargetRange


def sample(stamp):
    person = RgbTarget("person-1", stamp, 2.0, .95, "rgb")
    depth = TargetRange("LIDAR", "person-1", stamp, 1.5, .95,
                        FRONT_FRAME, "lidar", f"pair-{stamp}")
    return ShadowObservation(person, (depth,))


class DelayedStatusController(FakeController):
    def __init__(self, clock, delay):
        super().__init__()
        self.clock, self.delay = clock, delay
        self.started_at = None
        self.first_steer_elapsed = None

    def __call__(self, method, url, timeout):
        path = urlsplit(url).path
        if method == "GET":
            self.clock.now += self.delay
        if method == "POST" and path == "/shadow-start":
            code, body = super().__call__(method, url, timeout)
            self.started_at = self.clock.now
            return code, body
        if method == "POST" and path == "/steer-update":
            self.first_steer_elapsed = self.clock.now - self.started_at
            # An extra blocking status at 360 ms would reach the 700 ms guard.
            if self.first_steer_elapsed >= .700:
                self.posts.append((path, parse_qs(urlsplit(url).query)))
                self.state = "BRAKING"
                return 409, {"accepted": False,
                             "reason": "NO_MATCHING_ACTIVE_STEERED_FORWARD"}
        return super().__call__(method, url, timeout)


class FeedbackBrakeController(FakeController):
    def __init__(self, reason="FEEDBACK_TIMEOUT"):
        super().__init__()
        self.reason = reason
        self.remote_pending = False
        self.brake_polls = 0

    def __call__(self, method, url, timeout):
        path = urlsplit(url).path
        if method == "POST" and path == "/steer-update":
            args = {k: v[0] for k, v in parse_qs(urlsplit(url).query).items()}
            self.posts.append((path, args))
            self.state = "BRAKING"
            self.remote_pending = True
            return 409, {"accepted": False,
                         "reason": "NO_MATCHING_ACTIVE_STEERED_FORWARD"}
        if method == "GET" and self.state == "BRAKING":
            self.brake_polls += 1
            if self.brake_polls >= 3:
                self.state = "PAUSED"
                self.remote_pending = False
        code, body = super().__call__(method, url, timeout)
        if method == "GET":
            body["remote_stop_pending"] = self.remote_pending
            body["shadow_pause_reason"] = self.reason
        return code, body


class UartFeedbackRaceTests(unittest.TestCase):
    def run_case(self, controller, observations, latest):
        clock = getattr(controller, "clock", FakeClock())
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / "uart.jsonl"
            report = run_session(
                request=controller,
                observations=observations(clock),
                latest_observation=latest,
                journal_path=journal, clock=clock,
                rgb_calibration_id="rgb", lidar_calibration_id="lidar",
                minimum_confidence=.4, lock_path=Path(directory) / "lock",
            )
            rows = [json.loads(row) for row in journal.read_text().splitlines()]
        return report, rows

    def test_first_fresh_steer_has_priority_over_delayed_status(self):
        for latency in (.154, .360):
            with self.subTest(latency=latency):
                clock = FakeClock()
                controller = DelayedStatusController(clock, latency)
                seen = {"current": sample(10.0)}

                def stream(clock):
                    yield sample(10.0)
                    clock.now += .080
                    seen["current"] = sample(clock.now)
                    yield seen["current"]

                report, rows = self.run_case(
                    controller, stream, lambda: seen["current"])
                self.assertEqual(report["result"], "SESSION_ENDED")
                self.assertEqual(report["operations"], ["START", "STEER"])
                self.assertLess(controller.first_steer_elapsed, .700)
                self.assertEqual(controller.posts[-1][0], "/stop")
                self.assertTrue(any(
                    row.get("event") == "SHADOW_FIRST_STEER_PRIORITY"
                    for row in rows))

    def test_409_feedback_brake_settles_to_pause_without_abort(self):
        controller = FeedbackBrakeController()
        current = sample(10.10)

        def stream(clock):
            yield sample(10.0)

        report, rows = self.run_case(controller, stream, lambda: current)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["feedback_timeout_pauses"], 1)
        self.assertIn("STEER", report["operations"])
        self.assertGreaterEqual(controller.brake_polls, 3)
        self.assertTrue(any(row.get("event") == "SHADOW_FEEDBACK_TIMEOUT_SETTLED"
                            for row in rows))
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_unrelated_remote_stop_does_not_mask_rejection(self):
        controller = FeedbackBrakeController(reason="HOST_REQUEST")

        def stream(clock):
            yield sample(10.0)

        report, rows = self.run_case(controller, stream, lambda: sample(10.1))
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("SHADOW_OPERATION_REJECTED", report.get("error", ""))
        self.assertEqual(report["feedback_timeout_pauses"], 0)
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
