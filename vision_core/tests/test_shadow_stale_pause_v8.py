"""Reject PAUSED acknowledgement with a stale transition sequence."""
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


class StalePauseController(FakeController):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)

        if method == "POST" and urlsplit(url).path == "/shadow-start":
            self.state = "PAUSED"
            self.transition_seq = 0  # stale; START expects seq=1

        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V7
            body["shadow_api_version"] = "v2_paused_edge_turn"
            body["state"] = self.state
            body["shadow_transition_seq"] = self.transition_seq
            body["right_pwm"] = body["left_pwm"] = 0

        return code, body


class StalePauseV8Tests(unittest.TestCase):
    def test_stale_pause_is_refused_and_stopped(self):
        clock = FakeClock()
        controller = StalePauseController()

        target = RgbTarget("person-1", 10.0, 0., .95, "rgb")
        depth = TargetRange(
            "LIDAR", "person-1", 10.0, 1.5, .9,
            FRONT_FRAME, "lidar", "pair-1"
        )

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=[ShadowObservation(target, (depth,))],
                journal_path=Path(d) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                lock_path=Path(d) / "lock",
                minimum_confidence=.4,
            )

        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("SHADOW_STATE_CONFIRMATION_TIMEOUT", report["error"])
        self.assertEqual(report["operations"], ["START"])
        self.assertFalse(any(
            path == "/shadow-resume" for path, _ in controller.posts
        ))
        self.assertEqual(controller.posts[-1][0], "/stop")
        self.assertEqual(report["final_status"]["state"], "READY")


if __name__ == "__main__":
    unittest.main()
