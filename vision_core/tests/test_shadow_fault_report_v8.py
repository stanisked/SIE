import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, RgbTarget, TargetRange,
)


class FaultController(FakeController):
    def __init__(self):
        super().__init__()
        self.driving_reads = 0
        self.fault_latched = False

    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)

        if method == "GET":
            if self.state == "DRIVING":
                self.driving_reads += 1
                if self.driving_reads >= 2:
                    self.fault_latched = True

            if self.fault_latched:
                body["state"] = "FAULT"
                body["bounded_fault_latched"] = True
                body["bounded_fault_reason"] = "STEERED_CENTER_LIMIT"
                body["right_pwm"] = 0
                body["left_pwm"] = 0

        return code, body


class FaultReportTests(unittest.TestCase):
    def test_controller_fault_is_not_session_ended(self):
        clock = FakeClock()
        controller = FaultController()

        target = RgbTarget("person-1", 10.0, 0., .95, "rgb")
        depth = TargetRange(
            "LIDAR", "person-1", 10.0, 1.5, .9,
            FRONT_FRAME, "lidar", "pair-1"
        )

        with tempfile.TemporaryDirectory() as directory:
            report = run_session(
                request=controller,
                observations=[
                    ShadowObservation(target, (depth,)),
                    None,
                ],
                journal_path=Path(directory) / "test.jsonl",
                clock=clock,
                rgb_calibration_id="rgb",
                lidar_calibration_id="lidar",
                lock_path=Path(directory) / "lock",
                minimum_confidence=.4,
            )

        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("STEERED_CENTER_LIMIT", report["error"])
        self.assertEqual(report["final_status"]["state"], "FAULT")
        self.assertEqual(report["final_status"]["right_pwm"], 0)
        self.assertEqual(report["final_status"]["left_pwm"], 0)
        self.assertEqual(controller.posts[-1][0], "/stop")


if __name__ == "__main__":
    unittest.main()
