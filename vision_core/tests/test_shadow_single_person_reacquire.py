"""Motor-off tests for opt-in single-person Shadow reacquisition."""
import json
import tempfile
import unittest
from pathlib import Path

from vision_core.tests.test_run_shadow_uart import FakeClock, FakeController
from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    FRONT_FRAME, ACTIVE_STEREO_SHA256, RgbTarget, TargetRange)
from vision_core.tools.shadow_single_person_reacquire import SinglePersonReacquireGate


RGB_CAL = "rgb-v5"


def pair(track, at, *, bearing=4.0, distance=1.50, unique=True):
    rgb = RgbTarget(track, at, bearing, .96, RGB_CAL)
    rng = TargetRange("OV9281", track, at, distance, .95,
        FRONT_FRAME, ACTIVE_STEREO_SHA256, f"pair-{track}-{at}")
    return ShadowObservation(rgb, (rng,), single_person_rgb=unique)


class ReacquireGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = SinglePersonReacquireGate()
        self.prior = ("old", 10.0, 1.50)

    def check(self, observation, now, **kwargs):
        return self.gate.consider(
            target=observation.target, selected=observation.ranges[0],
            single_person_rgb=observation.single_person_rgb,
            old_track_id="old", last_verified=self.prior,
            last_bearing_deg=4.0, now_s=now, **kwargs)

    def test_two_distinct_single_person_pairs(self):
        self.assertEqual(self.check(pair("new", 10.50), 10.50)[0], "WAIT")
        self.assertEqual(self.check(pair("new", 10.50), 10.55)[0], "WAIT")
        self.assertEqual(self.check(pair("new", 10.75), 10.75)[0], "MATCH")

    def test_non_unique_frame_never_counts(self):
        self.check(pair("new", 10.50), 10.50)
        self.assertEqual(self.check(pair("new", 10.70, unique=False), 10.70)[0], "WAIT")
        self.assertEqual(self.check(pair("new", 10.85), 10.85)[0], "WAIT")

    def test_wrong_position_is_refused(self):
        self.assertEqual(self.check(pair("other", 10.50, bearing=30), 10.50)[0],
                         "REFUSE")
        self.assertEqual(self.check(pair("other", 10.50, distance=2.60), 10.50)[0],
                         "REFUSE")

    def test_expired_identity_is_refused(self):
        self.assertEqual(self.check(pair("new", 13.20), 13.20)[0], "REFUSE")

    def test_repeated_or_conflicting_second_pair_does_not_confirm(self):
        self.check(pair("new", 10.50), 10.50)
        self.assertEqual(self.check(pair("new", 10.70, bearing=-9.0), 10.70)[0],
                         "REFUSE")
        self.assertEqual(self.check(pair("new", 10.90), 10.90)[0], "WAIT")


class ReacquireRunnerTests(unittest.TestCase):
    def run_case(self, observations, *, enabled=True):
        clock = FakeClock()
        controller = FakeController()
        with tempfile.TemporaryDirectory() as directory:
            def stream():
                for at, observation in observations:
                    clock.now = at
                    yield observation
            journal = Path(directory) / "journal.jsonl"
            report = run_session(
                request=controller, observations=stream(),
                journal_path=journal, clock=clock,
                rgb_calibration_id=RGB_CAL, max_segments=5,
                minimum_confidence=.4,
                enable_single_person_reacquire=enabled,
                lock_path=Path(directory) / "lock")
            rows = [json.loads(row) for row in journal.read_text().splitlines()]
        return report, rows, controller

    def test_reacquire_keeps_same_command_then_resumes_on_third_pair(self):
        report, rows, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (10.50, pair("new", 10.50)),
            (10.65, pair("new", 10.65)),
            (10.80, pair("new", 10.80)),
        ])
        self.assertEqual(report["result"], "SESSION_ENDED")
        self.assertEqual(report["operations"], ["START", "PAUSE", "RESUME"])
        self.assertEqual(report["single_person_reacquires"], 1)
        self.assertEqual([path for path, _ in ctrl.posts if path == "/shadow-resume"],
                         ["/shadow-resume"])
        event = next(x for x in rows if x.get("event") == "SHADOW_SINGLE_PERSON_TRACK_REBOUND")
        self.assertEqual(event["old_track_id"], "old")
        self.assertEqual(event["new_track_id"], "new")
        self.assertIs(event["person_identity_verified"], False)
        self.assertIs(event["resume_authorized"], False)
        self.assertEqual(ctrl.posts[-1][0], "/stop")

    def test_no_resuming_from_confirmation_pair(self):
        report, _, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (10.50, pair("new", 10.50)),
            (10.65, pair("new", 10.65)),
            (10.65, pair("new", 10.65)),
        ])
        self.assertEqual(report["single_person_reacquires"], 1)
        self.assertNotIn("RESUME", report["operations"])
        self.assertEqual(ctrl.posts[-1][0], "/stop")

    def test_default_track_change_still_stops(self):
        report, _, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (10.50, pair("new", 10.50)),
        ], enabled=False)
        self.assertEqual(report["reason"], "NEW_TRACK_REQUIRES_NEW_SESSION")
        self.assertEqual(report["operations"], ["START", "PAUSE", "STOP"])
        self.assertEqual(report["single_person_reacquires"], 0)
        self.assertEqual(ctrl.posts[-1][0], "/shadow-stop")

    def test_nonunique_rgb_does_not_handoff(self):
        report, _, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (10.50, pair("new", 10.50, unique=False)),
            (10.70, pair("new", 10.70, unique=False)),
        ])
        self.assertEqual(report["single_person_reacquires"], 0)
        self.assertNotIn("RESUME", report["operations"])
        self.assertEqual(ctrl.posts[-1][0], "/stop")

    def test_wrong_person_geometry_stops_without_resume(self):
        report, _, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (10.50, pair("other", 10.50, bearing=33)),
        ])
        self.assertEqual(report["reason"], "NEW_TRACK_REQUIRES_NEW_SESSION")
        self.assertEqual(report["single_person_reacquires"], 0)
        self.assertNotIn("RESUME", report["operations"])
        self.assertEqual(ctrl.posts[-1][0], "/shadow-stop")

    def test_expired_loss_stops_without_resume(self):
        report, _, ctrl = self.run_case([
            (10.0, pair("old", 10.0)),
            (10.25, ShadowObservation(None, ())),
            (13.25, pair("new", 13.25)),
        ])
        self.assertEqual(report["reason"], "NEW_TRACK_REQUIRES_NEW_SESSION")
        self.assertEqual(report["single_person_reacquires"], 0)
        self.assertNotIn("RESUME", report["operations"])
        self.assertEqual(ctrl.posts[-1][0], "/shadow-stop")


if __name__ == "__main__":
    unittest.main()
