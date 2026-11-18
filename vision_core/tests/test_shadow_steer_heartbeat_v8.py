import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)


def sample(t):
    target = RgbTarget("person-1", t, 0., .95, "rgb")
    depth = TargetRange(
        "LIDAR", "person-1", t, 1.5, .9,
        FRONT_FRAME, "lidar", f"pair-{t}"
    )
    return ShadowObservation(target, (depth,))


class SteeringHeartbeatTests(unittest.TestCase):
    def run_case(self, stream_factory):
        clock = FakeClock()
        controller = FakeController()

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(
                request=controller,
                observations=stream_factory(clock),
                journal_path=Path(directory) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                minimum_confidence=.4,
                lock_path=Path(directory) / "lock",
            )

        return report, controller.posts

    def test_fresh_same_bias_sends_heartbeat(self):
        def stream(clock):
            for t in (10.0, 10.10, 10.31, 10.60):
                clock.now = t
                yield sample(t)

        report, posts = self.run_case(stream)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(
            [path for path, _ in posts].count("/steer-update"), 2
        )
        self.assertEqual(
            [args["bias_pwm"] for path, args in posts
             if path == "/steer-update"], ["0", "0"]
        )
        self.assertEqual(posts[-1][0], "/stop")

    def test_reused_pair_cannot_refresh_feedback(self):
        def stream(clock):
            old = sample(10.0)
            yield old
            clock.now = 10.35
            yield old

        report, posts = self.run_case(stream)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertFalse(
            any(path == "/steer-update" for path, _ in posts)
        )
        self.assertEqual(posts[-1][0], "/stop")

    def test_target_loss_pauses_instead_of_heartbeat(self):
        def stream(clock):
            yield sample(10.0)
            clock.now = 10.35
            yield ShadowObservation(None, ())

        report, posts = self.run_case(stream)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertIn("/shadow-pause", [path for path, _ in posts])
        self.assertFalse(
            any(path == "/steer-update" for path, _ in posts)
        )
        self.assertEqual(posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
