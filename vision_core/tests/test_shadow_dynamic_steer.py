import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)


class DynamicSteerTests(unittest.TestCase):
    def test_duplicate_ignored_real_change_sent(self):
        clock = FakeClock()
        controller = FakeController()

        def observations():
            for now_s, observed_s, bearing in (
                (10.0, 10.0, 0.0),  # Initial START
                (10.2, 10.2, 0.0),  # Fresh pair: first same-bias heartbeat
                (10.5, 10.2, 0.0),  # Reused pair: must not send STEER
                (10.7, 10.7, 3.0),  # Fresh pair: actual steering change
            ):
                clock.now = now_s
                target = RgbTarget(
                    "person-1", observed_s, bearing, .95, "rgb"
                )
                depth = TargetRange(
                    "LIDAR", "person-1", observed_s, 1.5, .9,
                    FRONT_FRAME, "lidar", f"pair-{observed_s}"
                )
                yield ShadowObservation(target, (depth,))

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=observations(),
                journal_path=Path(d) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                lock_path=Path(d) / "lock",
                minimum_confidence=.4,
            )

        commands = [
            (path, args.get("bias_pwm"))
            for path, args in controller.posts
            if path in ("/shadow-start", "/steer-update")
        ]

        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(commands, [
            ("/shadow-start", "0"),
            ("/steer-update", "0"),
            ("/steer-update", "12"),
        ])
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
