"""Identity continuity tests. No images, faces, ROS or motors used."""
from dataclasses import replace
import unittest

from vision_core.person_localization.shadow_face_anchored_target import (
    FaceAnchoredTarget, FaceBodyEvidence, FACE_MATCH, FACE_MISMATCH, FACE_NONE)


MASK = "d" * 64
BODY_SHA = "b" * 64
VECTOR = tuple([1.0] + [0.0] * 15)
OTHER = tuple([0.0, 1.0] + [0.0] * 14)


def observation(track=":1", when=10., *, face=FACE_NONE, model=True,
                embedding=VECTOR, sole=True):
    return FaceBodyEvidence(
        cycle_id=f"cycle-{when:.2f}", captured_s=when,
        camera_track_id=track, mask_sha256=MASK,
        unique_body_binding=True, sole_visible_person=sole,
        face_result=face,
        face_profile_id=("my-face-v1" if face == FACE_MATCH else None),
        face_cycle_id=(f"cycle-{when:.2f}" if face == FACE_MATCH else None),
        face_mask_sha256=(MASK if face == FACE_MATCH else None),
        face_verifier_calibrated=(face == FACE_MATCH),
        body_model_sha256=(BODY_SHA if model else None),
        body_embedding=(embedding if model else None))


def new_state(calibrated=False):
    return FaceAnchoredTarget(
        entity_id="person_001", face_profile_id="my-face-v1",
        body_model_sha256=BODY_SHA,
        calibrated_body_min_cosine=(0.9 if calibrated else None),
        body_calibration_id=("synthetic-only-calibration" if calibrated else None))


class TargetIdentityTests(unittest.TestCase):
    def anchor(self, calibrated=False):
        state = new_state(calibrated)
        result = state.observe(observation(face=FACE_MATCH), now_s=10.)
        self.assertEqual(result.status, "FACE_ANCHORED")
        self.assertEqual(result.entity_id, "person_001")
        self.assertFalse(result.motor_authorized)
        return state

    def test_cannot_choose_target_by_body_alone_before_face(self):
        state = new_state()
        result = state.observe(observation(), now_s=10.)
        self.assertEqual(result.status, "AWAITING_FACE")
        self.assertIsNone(result.entity_id)

    def test_enrolled_face_anchors_body_under_entity_id(self):
        state = self.anchor()
        self.assertEqual(state.camera_track_id, ":1")
        self.assertEqual(len(state.gallery), 1)

    def test_face_lost_but_same_body_track_continues(self):
        state = self.anchor()
        for n in (10.25, 10.50, 10.75, 11.0):
            result = state.observe(observation(when=n), now_s=n)
            self.assertEqual(result.status, "BODY_TRACKED")
            self.assertEqual(result.entity_id, "person_001")
            self.assertFalse(result.face_visible)
            self.assertFalse(result.motor_authorized)

    def test_full_loss_then_same_track_after_long_gap_needs_reid(self):
        state = self.anchor()
        self.assertEqual(state.observe(None, now_s=11.).status, "TARGET_LOST")
        result = state.observe(observation(when=13.), now_s=13.)
        self.assertEqual(result.status, "TARGET_LOST")
        self.assertEqual(state.camera_track_id, ":1")

    def test_new_camera_id_cannot_take_over_without_body_calibration(self):
        state = self.anchor()
        state.observe(None, now_s=11.)
        result = state.observe(observation(track=":2",when=13.), now_s=13.)
        self.assertEqual(result.status, "TARGET_LOST")
        self.assertEqual(state.camera_track_id, ":1")

    def test_same_face_can_rebind_new_camera_track(self):
        state = self.anchor()
        state.observe(None, now_s=11.)
        result = state.observe(observation(track=":3",when=14.,face=FACE_MATCH),
                               now_s=14.)
        self.assertEqual(result.status, "FACE_REVERIFIED")
        self.assertEqual(result.entity_id, "person_001")
        self.assertEqual(result.camera_track_id, ":3")
        self.assertFalse(result.motor_authorized)

    def test_face_from_wrong_mask_cannot_reanchor(self):
        state = self.anchor()
        bad = replace(observation(track=":2", when=13., face=FACE_MATCH),
                      face_mask_sha256="a"*64)
        result = state.observe(bad, now_s=13.)
        self.assertEqual(result.status, "TARGET_UNAVAILABLE")
        self.assertEqual(state.camera_track_id, ":1")

    def test_wrong_face_on_same_or_new_track_is_conflict(self):
        for track in (":1", ":2"):
            with self.subTest(track=track):
                state = self.anchor(calibrated=True)
                result = state.observe(observation(
                    track=track,when=13.,face=FACE_MISMATCH),now_s=13.)
                self.assertEqual(result.status, "TARGET_CONFLICT")
                self.assertEqual(state.camera_track_id, ":1")

    def test_body_reid_requires_two_distinct_frames_and_calibration(self):
        state = self.anchor(calibrated=True)
        state.observe(None, now_s=13.)
        first = state.observe(observation(track=":2",when=13.1),now_s=13.1)
        self.assertEqual(first.status, "BODY_REID_PENDING")
        same = state.observe(observation(track=":2",when=13.1),now_s=13.2)
        self.assertEqual(same.status, "BODY_REID_PENDING")
        second = state.observe(observation(track=":2",when=13.3),now_s=13.3)
        self.assertEqual(second.status, "BODY_REIDENTIFIED")
        self.assertEqual(second.camera_track_id, ":2")
        self.assertEqual(second.entity_id, "person_001")
        self.assertFalse(second.motor_authorized)

    def test_body_reid_rejects_different_visual_descriptor(self):
        state = self.anchor(calibrated=True)
        result = state.observe(observation(track=":2",when=13.,
                                           embedding=OTHER),now_s=13.)
        self.assertEqual(result.status, "TARGET_LOST")
        self.assertEqual(state.camera_track_id, ":1")

    def test_body_reid_rejects_multiple_people(self):
        state = self.anchor(calibrated=True)
        result = state.observe(observation(track=":2",when=13.,
                                           sole=False),now_s=13.)
        self.assertEqual(result.status, "TARGET_LOST")
        self.assertEqual(state.camera_track_id, ":1")

    def test_stale_or_unbound_sources_cannot_attach(self):
        state = self.anchor()
        stale = state.observe(observation(track=":2",when=11.),now_s=13.)
        self.assertEqual(stale.status, "TARGET_UNAVAILABLE")
        invalid = replace(observation(track=":2",when=13.),
                          unique_body_binding=False)
        self.assertEqual(state.observe(invalid,now_s=13.).status,
                         "TARGET_UNAVAILABLE")

    def test_no_model_threshold_without_calibration_record(self):
        with self.assertRaises(ValueError):
            FaceAnchoredTarget(entity_id="person_001",
                face_profile_id="my-face-v1", body_model_sha256=BODY_SHA,
                calibrated_body_min_cosine=.9)


if __name__ == "__main__":
    unittest.main()
