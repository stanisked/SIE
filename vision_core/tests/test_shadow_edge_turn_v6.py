"""SHADOW v6: the short-turn brake guard must not change FOLLOW protocol."""
import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_shadow_edge_turn_v5 import V5Controller
from vision_core.tests.test_shadow_edge_turn_v4 import Clock, edge, normal
from vision_core.tools.run_shadow_uart import (
    EDGE_TURN_BUILD, EDGE_TURN_BUILD_V5, EDGE_TURN_BUILD_V6,
    EDGE_TURN_CONTRACTS, run_session,
)


class V6Controller(V5Controller):
    def __call__(self, method, url, timeout):
        code, body = super().__call__(method, url, timeout)
        if method == "GET":
            body["firmware_build_id"] = EDGE_TURN_BUILD_V6
            body["shadow_turn_fast_brake_requested"] = False
        return code, body


class ShadowEdgeTurnV6Tests(unittest.TestCase):
    def test_contracts_preserved(self):
        for build in (EDGE_TURN_BUILD, EDGE_TURN_BUILD_V5, EDGE_TURN_BUILD_V6):
            self.assertIn((build, "v2_paused_edge_turn"),
                          EDGE_TURN_CONTRACTS)

    def test_v6_follow_unchanged_without_edge_turn(self):
        clock, controller = Clock(), V6Controller()
        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller, observations=[normal(clock)],
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock", minimum_confidence=.4,
                enable_edge_turn=False)
        self.assertEqual(report["operations"], ["START"])
        self.assertFalse(any(path == "/shadow-turn"
                             for path, _ in controller.posts))
        self.assertEqual(controller.posts[-1][0], "/stop")

    def test_v6_turn_preserves_session_and_fresh_depth_requirement(self):
        clock, controller = Clock(), V6Controller()

        def samples():
            yield normal(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .10
            yield normal(clock, observed_s=10.20)
            clock.now += .20
            yield normal(clock)

        with tempfile.TemporaryDirectory() as d:
            report = run_session(
                request=controller, observations=samples(),
                journal_path=Path(d) / "journal.jsonl",
                rgb_calibration_id="test-rgb", clock=clock,
                lock_path=Path(d) / "lock", minimum_confidence=.4,
                enable_edge_turn=True)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "PAUSE", "TURN", "RESUME"])
        self.assertEqual(controller.posts[-1][0], "/stop")
        self.assertEqual(report["edge_turns_started"], 1)


if __name__ == "__main__":
    unittest.main()
