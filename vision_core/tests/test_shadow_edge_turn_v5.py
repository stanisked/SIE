"""Regression: v5 guard build changes no SHADOW UART protocol or FOLLOW eligibility."""
import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_shadow_edge_turn_v4 import (
    Clock, V4Controller, edge, normal,
)
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD, EDGE_TURN_BUILD_V5, EDGE_TURN_CONTRACTS, run_session,
)


class V5Controller(V4Controller):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)
        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V5
        return code, body


class UnknownController(V5Controller):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)
        if method == "GET":
            body["firmware_build_id"] = "unknown-future-build"
        return code, body


class ShadowEdgeTurnV5Tests(unittest.TestCase):
    def test_previous_v4_build_contract_is_still_accepted(self):
        self.assertIn((EDGE_TURN_BUILD, "v2_paused_edge_turn"),
                      EDGE_TURN_CONTRACTS)
        self.assertIn((EDGE_TURN_BUILD_V5, "v2_paused_edge_turn"),
                      EDGE_TURN_CONTRACTS)

    def test_v5_keeps_track_and_resumes_only_with_fresh_pair(self):
        clock, controller = Clock(), V5Controller()

        def observations():
            yield normal(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .10
            yield normal(clock, observed_s=10.20)  # stale depth: no RESUME
            clock.now += .20
            yield normal(clock)  # new depth allows RESUME

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller, observations=observations(),
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=True)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"],
                         ["START", "PAUSE", "TURN", "RESUME"])
        self.assertEqual(report["edge_turns_started"], 1)
        self.assertEqual(controller.posts[-1][0], "/stop")
        self.assertEqual(report["final_status"]["state"], "READY")

    def test_v5_follows_without_turn_when_not_enabled(self):
        clock, controller = Clock(), V5Controller()
        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller,
                observations=[normal(clock)],
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=False)
        self.assertEqual(report["operations"], ["START"])
        self.assertFalse(any(path == "/shadow-turn"
                             for path, _ in controller.posts))
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_unknown_firmware_rejected_before_motion(self):
        clock, controller = Clock(), UnknownController()
        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller, observations=[normal(clock)],
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=True)
        self.assertEqual(report["result"], "STOPPED_ON_REFUSAL")
        self.assertIn("SHADOW_CONTROLLER_STATUS_INVALID", report["error"])
        self.assertFalse(controller.posts)


if __name__ == "__main__":
    unittest.main()
