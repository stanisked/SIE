"""Regression: a partial segment resumes only on a new observation."""
import tempfile
import unittest
from pathlib import Path
from urllib.parse import urlsplit

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD_V7, ShadowObservation, run_session,
)
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)


class PartialController(FakeController):
    def __init__(self):
        super().__init__()
        self.partial_count = 0

    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)

        if method == "POST" and urlsplit(url).path == "/steer-update":
            self.state = "PAUSED"
            self.partial_count += 1

        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V7
            body["shadow_api_version"] = "v2_paused_edge_turn"

            if self.state == "PAUSED":
                body["state"] = "PAUSED"
                body["right_pwm"] = 0
                body["left_pwm"] = 0
                body["shadow_pause_reason"] = "SEGMENT_PARTIAL"
                body["last_command_state"] = "PARTIAL_PROGRESS"

        return code, body


class PartialResumeTests(unittest.TestCase):
    def test_fresh_pair_resumes_after_partial(self):
        clock = FakeClock()
        controller = PartialController()

        def observations():
            for index in range(3):
                timestamp = clock()
                target = RgbTarget("person-1", timestamp, 0., .9, "rgb")
                distance = TargetRange(
                    "LIDAR", "person-1", timestamp, 1.5, .9,
                    FRONT_FRAME, "lidar", f"association-{index}"
                )
                yield ShadowObservation(target, (distance,))
                clock.now += .2

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(
                request=controller,
                observations=observations(),
                journal_path=Path(directory) / "partial.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                lock_path=Path(directory) / "operator.lock",
            )

        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "STEER", "RESUME"])
        self.assertEqual(controller.partial_count, 1)
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
