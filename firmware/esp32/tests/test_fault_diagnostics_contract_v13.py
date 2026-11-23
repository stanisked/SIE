"""Static safety-contract checks for the v13 UART fault diagnostics."""

import unittest
from pathlib import Path


NAME = "traction_control_v4_2_uart_shadow_v13_brake_continuity"
SKETCH = Path(__file__).resolve().parents[1] / NAME / (NAME + ".ino")


class FaultDiagnosticsContractTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.source = SKETCH.read_text()

    def test_uart_dispatch_exposes_diagnostics(self):
        dispatch = self.source.split(
            "void handleUartFrame(const String &line)", 1
        )[1].split("void pollUart()", 1)[0]

        self.assertIn('fields[2] == "FAULT_DIAGNOSTICS"', dispatch)
        self.assertIn("handleFaultDiagnostics();", dispatch)

    def test_diagnostics_handler_is_read_only(self):
        handler = self.source.split(
            "void handleFaultDiagnostics()", 1
        )[1].split("\n}", 1)[0]

        forbidden = (
            "startMotion(",
            "stopMotors(",
            "brakeMotors(",
            "handleAckFault(",
            "beginBoundedBraking(",
            "queueBoundedMotionRequest(",
            "boundedFaultLatched =",
            "motionState =",
            "activeCommandIndex =",
        )

        import re

        for operation in forbidden:
            with self.subTest(operation=operation):
                if operation == "motionState =":
                    self.assertIsNone(
                        re.search(r"\bmotionState\s*=(?!=)", handler)
                    )
                else:
                    self.assertNotIn(operation, handler)


    def test_terminal_evidence_requires_matching_fault(self):
        handler = self.source.split(
            "void handleFaultDiagnostics()\n{", 1
        )[1].split("\nString buildUartStatus()", 1)[0]

        required = (
            "boundedFaultSnapshot.valid",
            "boundedFaultLatched",
            "motionState == MotionState::FAULT",
            "boundedFaultSnapshot.bootSessionId == bootSessionId",
            "last != nullptr",
            "last->state == CommandState::FAULT",
            "last->commandId == boundedFaultSnapshot.commandId",
        )

        for condition in required:
            with self.subTest(condition=condition):
                self.assertIn(condition, handler)

        self.assertIn("if (snapshotMatchesLast)", handler)
        self.assertIn("snapshot_matches_last_fault", handler)

        for field in (
            "rightFinalCount",
            "leftFinalCount",
            "rightOvershootCounts",
            "leftOvershootCounts",
        ):
            with self.subTest(field=field):
                self.assertIn("last->" + field, handler)


if __name__ == "__main__":
    unittest.main()
