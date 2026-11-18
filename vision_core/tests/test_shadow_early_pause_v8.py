"""START -> early PAUSED -> fresh RGB/Depth -> RESUME."""
import tempfile
import unittest
from pathlib import Path
from urllib.parse import urlsplit

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)


class EarlyPauseController(FakeController):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)

        if method == "POST" and urlsplit(url).path == "/shadow-start":
            self.state = "PAUSED"

        if method == "GET":
            body["state"] = self.state
            body["right_pwm"] = body["left_pwm"] = 0
            body["last_command_state"] = "PAUSED"
            body["shadow_pause_reason"] = "SEGMENT_PARTIAL"

        return code, body


class EarlyPauseV8Tests(unittest.TestCase):
    def test_resume_requires_fresh_pair(self):
        clock = FakeClock()
        controller = EarlyPauseController()

        def stream():
            for timestamp in (10.0, 10.0, 10.2):
                rgb = RgbTarget(
                    "person-1", timestamp, 0., .9, "rgb"
                )
                depth = TargetRange(
                    "LIDAR", "person-1", timestamp, 1.5, .9,
                    FRONT_FRAME, "lidar",
                    f"association-{timestamp}"
                )
                yield ShadowObservation(rgb, (depth,))
                clock.now += .2

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=stream(),
                journal_path=Path(d) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                lock_path=Path(d) / "lock",
            )

        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "RESUME"])
        self.assertGreaterEqual(report["reused_pair_refusals"], 1)
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
