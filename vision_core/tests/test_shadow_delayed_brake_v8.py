"""Regression: delayed BRAKING must not abort a valid short segment."""
import tempfile
import time
import unittest
from pathlib import Path
from urllib.parse import urlsplit

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD_V7, ShadowObservation, run_session,
)
from vision_core.tools.shadow_range_gate import (
    ACTIVE_STEREO_SHA256, FRONT_FRAME, RgbTarget, TargetRange,
)


class DelayedBrakeController(FakeController):
    def __init__(self):
        super().__init__()
        self.start_at = None
        self.states_seen = []

    def __call__(self, method, url, timeout):
        path = urlsplit(url).path

        if method == "POST" and path == "/shadow-start":
            code, body = super().__call__(method, url, timeout)
            self.state = "BRAKING"
            self.start_at = time.monotonic()
            return code, body

        if method == "GET" and self.state == "BRAKING":
            if time.monotonic() - self.start_at >= 0.85:
                self.state = "PAUSED"

        code, body = super().__call__(method, url, timeout)

        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V7
            body["shadow_api_version"] = "v2_paused_edge_turn"
            if self.state == "PAUSED":
                body["shadow_pause_reason"] = "SEGMENT_PARTIAL"
                body["last_command_state"] = "PAUSED"
            self.states_seen.append(self.state)

        return code, body


class DelayedBrakeV8Tests(unittest.TestCase):
    def test_delayed_brake_allows_fresh_resume(self):
        clock = FakeClock()
        controller = DelayedBrakeController()

        def sample(timestamp, pair_id):
            target = RgbTarget(
                "person-1", timestamp, 0., .95, "rgb"
            )
            depth = TargetRange(
                "OV9281", "person-1", timestamp, 1.5, .9,
                FRONT_FRAME, ACTIVE_STEREO_SHA256, pair_id
            )
            return ShadowObservation(target, (depth,))

        def observations():
            yield sample(10.0, "pair-1")
            clock.now = 10.15
            yield sample(10.0, "pair-1")
            clock.now = 10.30
            yield sample(10.30, "pair-2")

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=observations(),
                journal_path=Path(d) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lock_path=Path(d) / "lock",
                minimum_confidence=.4,
                max_segments=2,
            )

        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "RESUME"])
        self.assertEqual(report["segments_started"], 2)
        self.assertGreaterEqual(report["reused_pair_refusals"], 1)
        self.assertIn("BRAKING", controller.states_seen)
        self.assertIn("PAUSED", controller.states_seen)
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
