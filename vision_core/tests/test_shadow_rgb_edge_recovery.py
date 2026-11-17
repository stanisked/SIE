"""RGB edge orientation recovery with existing ESP32 TURN, no firmware edits.

This is mock-UART execution. It never starts physical motors.
"""
import json
import tempfile
import unittest
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

from vision_core.tests.test_shadow_edge_turn_v4 import Clock, edge, normal
from vision_core.tests.test_shadow_edge_turn_v5 import V5Controller
from vision_core.tools.run_shadow_uart import (
    ShadowObservation, run_session, select_edge_turn,
    EDGE_TURN_MIN_PRIOR_CLEARANCE_M, EDGE_TURN_MAX_PRIOR_AGE_S,
)
from vision_core.tools.shadow_range_gate import (
    RgbTarget, TargetRange, FRONT_FRAME, ACTIVE_STEREO_SHA256,
    SHADOW_STOP_DISTANCE_M,
)


class ConfirmingStopController(V5Controller):
    """The existing V5 mock lacks /shadow-stop; model its terminal ACK."""
    def __call__(self, method, url, timeout):
        if method == "POST" and urlsplit(url).path == "/shadow-stop":
            args = {k: v[0] for k, v in parse_qs(urlsplit(url).query).items()}
            self.posts.append(("/shadow-stop", args))
            self.last = self.active
            self.active = None
            self.seq = int(args["seq"])
            self.state = "READY"
            return 202, {"accepted": True}
        return super().__call__(method, url, timeout)


class ShadowRgbEdgeRecoveryTests(unittest.TestCase):
    def run_recovery(self, observations, controller_cls=V5Controller,
                     latest_observation=None):
        clock, controller = Clock(), controller_cls()
        with tempfile.TemporaryDirectory() as d:
            journal = Path(d) / "bridge.jsonl"
            report = run_session(
                request=controller, observations=observations(clock),
                journal_path=journal, rgb_calibration_id="test-rgb",
                clock=clock, lock_path=Path(d) / "lock",
                minimum_confidence=.4, enable_edge_turn=True,
                latest_observation=latest_observation)
            events = [json.loads(line) for line in journal.read_text().splitlines()]
        return report, controller.posts, events

    def test_floor_distance_and_pause_settling_allow_one_bounded_pivot(self):
        self.assertEqual(SHADOW_STOP_DISTANCE_M, .5)
        self.assertLess(EDGE_TURN_MIN_PRIOR_CLEARANCE_M, 1.145)
        self.assertEqual(EDGE_TURN_MAX_PRIOR_AGE_S, 3.0)
        t = RgbTarget("person-1", 12.15, -14.7, .93, "test-rgb")
        self.assertEqual(select_edge_turn(t, -14.7,
                         ("person-1", 10.0, 1.145), 12.15), ("LEFT", 8.0))
        self.assertIsNone(select_edge_turn(t, -14.7,
                          ("person-other", 10.0, 1.145), 12.15))
        self.assertIsNone(select_edge_turn(t, -14.7,
                          ("person-1", 9.0, 1.145), 12.15))
        self.assertEqual(select_edge_turn(t, -14.7,
                         ("person-1", 9.434, 1.061), 12.15), ("LEFT", 8.0))
        self.assertIsNone(select_edge_turn(t, -14.7,
                          ("person-1", 9.149, 1.061), 12.15))
        self.assertIsNone(select_edge_turn(t, -14.7,
                          ("person-1", 10.0, .68), 12.15))
        weak = RgbTarget("person-1", 12.15, -14.7, .3, "test-rgb")
        self.assertIsNone(select_edge_turn(weak, -14.7,
                          ("person-1", 10.0, 1.145), 12.15))

    def test_floor_2p716s_range_and_targetless_latest_tick_allow_one_pivot(self):
        # At the 20:32 floor trial, the first gate saw a 2.716s-old
        # same-ID range and a fresh RGB edge. The async camera may already
        # have published a targetless tick during the STATUS round trip.
        def left_edge(clock):
            # Actual floor edge was at -12.89 degrees, not the +13 degree
            # RIGHT-edge fixture used by unrelated regression scenarios.
            return ShadowObservation(
                RgbTarget("person-1", clock(), -12.89, .95, "test-rgb"),
                (), edge_recovery_bearing_deg=-12.89)

        def samples(clock):
            yield normal(clock, distance=1.061)
            clock.now += 2.57
            yield left_edge(clock)  # PAUSE
            clock.now += .146
            yield left_edge(clock)  # settled PAUSED, age = 2.716
        report, posts, events = self.run_recovery(
            samples, latest_observation=lambda: ShadowObservation(None, ()))
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "PAUSE", "TURN"])
        self.assertEqual(report["edge_turns_started"], 1)
        turns = [args for path, args in posts if path == "/shadow-turn"]
        self.assertEqual(len(turns), 1)
        self.assertEqual(turns[0]["direction"], "LEFT")
        self.assertEqual(turns[0]["angle_deg"], "8.00")
        self.assertTrue(any(e.get("event") == "EDGE_TURN_SETTLED"
                            for e in events))
        self.assertFalse(any(path == "/shadow-resume" for path, _ in posts))

    def test_newer_different_rgb_track_refuses_cached_edge_pivot(self):
        def samples(clock):
            yield normal(clock, distance=1.061)
            clock.now += 2.57
            yield edge(clock)
            clock.now += .146
            yield edge(clock)
        def other_target():
            return ShadowObservation(RgbTarget("different-person", 12.716,
                                               1.0, .95, "test-rgb"), ())
        report, posts, events = self.run_recovery(
            samples, latest_observation=other_target)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["edge_turns_started"], 0)
        self.assertEqual(report["operations"], ["START", "PAUSE"])
        self.assertFalse(any(path == "/shadow-turn" for path, _ in posts))

    def test_edge_retains_track_but_never_starts_forward_without_depth(self):
        def samples(clock):
            yield normal(clock, distance=1.145)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)   # TURN from confirmed PAUSED only
            clock.now += .10
            yield edge(clock)   # still no depth; cannot RESUME
            clock.now += .10
            yield normal(clock, observed_s=10.20)  # paired before pivot
            clock.now += .20
            yield normal(clock, distance=1.05)  # new same-person pair
        report, posts, events = self.run_recovery(samples)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "PAUSE", "TURN", "RESUME"])
        self.assertEqual(report["edge_turns_started"], 1)
        self.assertEqual(len([p for p, _ in posts if p == "/shadow-turn"]), 1)
        self.assertEqual(posts[-1][0], "/stop")
        self.assertEqual(len([e for e in events
                              if e.get("event") == "EDGE_RECOVERY_RGB_DEPTH_REACQUIRED"]), 1)
        self.assertTrue(any(e.get("reason") == "POST_TURN_FRESH_PAIR_REQUIRED"
                            for e in events))

    def test_second_pivot_requires_new_pair_between_turns(self):
        def samples(clock):
            yield normal(clock, distance=1.145)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .10
            yield edge(clock)  # don't pivot again on the same old range
            clock.now += .20
            yield normal(clock, distance=1.0)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)  # second bounded pivot after reacquisition
        report, posts, _ = self.run_recovery(samples)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"],
                         ["START", "PAUSE", "TURN", "RESUME", "PAUSE", "TURN"])
        self.assertEqual(report["edge_turns_started"], 2)
        self.assertEqual(len([p for p, _ in posts if p == "/shadow-turn"]), 2)
        self.assertEqual(posts[-1][0], "/stop")

    def test_different_reidentified_track_stops_instead_of_resuming(self):
        def samples(clock):
            yield normal(clock, distance=1.145)
            clock.now += .15
            yield edge(clock)
            clock.now += .15
            yield edge(clock)
            clock.now += .30
            target = RgbTarget("person-2", clock(), -2., .95, "test-rgb")
            rng = TargetRange("OV9281", "person-2", clock(), 1.0, .9,
                              FRONT_FRAME, ACTIVE_STEREO_SHA256, "fresh-other")
            yield ShadowObservation(target, (rng,))
        report, posts, _ = self.run_recovery(samples, ConfirmingStopController)
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["reason"], "NEW_TRACK_REQUIRES_NEW_SESSION")
        self.assertEqual(report["operations"], ["START", "PAUSE", "TURN", "STOP"])
        self.assertFalse(any(p == "/shadow-resume" for p, _ in posts))
        self.assertEqual(posts[-1][0], "/shadow-stop")


if __name__ == "__main__":
    unittest.main()
