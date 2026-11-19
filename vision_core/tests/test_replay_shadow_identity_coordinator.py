"""Coordinator integration tests on synthetic motor-off evidence only."""
from copy import deepcopy
import unittest

from vision_core.tools.replay_shadow_identity_coordinator import replay
from vision_core.tools.audit_shadow_identity_timeline import construct_timeline
from vision_core.tests.test_audit_shadow_identity_timeline import test_rows


class IdentityCoordinatorReplayTests(unittest.TestCase):
    def fixture(self):
        journal,face,body=test_rows()
        timeline=construct_timeline(journal,face,body)
        return journal,face,body,timeline

    def test_unverified_faces_do_not_anchor_entity(self):
        journal,face,body,timeline=self.fixture()
        output=replay(journal,face,body,timeline)
        self.assertEqual(output["journal_cycles"],5)
        self.assertEqual(output["snapshot_inputs"],3)
        self.assertEqual(output["unverified_face_inputs"],1)
        self.assertEqual(output["cycles_without_retained_identity_image"],2)
        self.assertEqual(output["coordinator_status_counts"],
                         {"AWAITING_FACE":3,"TARGET_LOST":2})
        self.assertEqual(output["confirmed_identity_count"],0)
        self.assertEqual(output["body_embeddings_supplied"],0)
        self.assertFalse(output["motor_authorized"])
        self.assertEqual(output["events"][-1]["coordinator_input"],
                         "FACE_PROFILE_COMPATIBLE_UNCALIBRATED")
        self.assertTrue(output["events"][-1]["face_visible"])
        self.assertIsNone(output["events"][-1]["confirmed_entity_id"])

    def test_missing_snapshots_are_not_faked_as_body_observations(self):
        journal,face,body,timeline=self.fixture()
        output=replay(journal,face,body,timeline)
        self.assertEqual(output["events"][1]["coordinator_input"],
                         "NO_RETAINED_IMAGE_FOR_IDENTITY_COORDINATOR")
        self.assertEqual(output["events"][1]["coordinator_status"],
                         "TARGET_LOST")
        self.assertEqual(output["events"][2]["coordinator_status"],
                         "TARGET_LOST")

    def test_source_hash_change_aborts_before_state_machine(self):
        journal,face,body,timeline=self.fixture()
        face["rows"][0]["mask_sha256"]="f"*64
        with self.assertRaisesRegex(ValueError,"BINDING_INVALID"):
            replay(journal,face,body,timeline)

    def test_cannot_upgrade_policy_by_editing_timeline(self):
        journal,face,body,timeline=self.fixture()
        timeline["policy"]["face_identity_calibrated"]=True
        with self.assertRaisesRegex(ValueError,"POLICY_OR_SCOPE_INVALID"):
            replay(journal,face,body,timeline)

    def test_cannot_inject_confirmed_entity_into_event(self):
        journal,face,body,timeline=self.fixture()
        timeline["per_cycle"][-1]["confirmed_entity_id"]="person_001"
        with self.assertRaisesRegex(ValueError,"TIMELINE_EVENT_BINDING_INVALID"):
            replay(journal,face,body,timeline)

    def test_cannot_relabel_snapshot_track(self):
        journal,face,body,timeline=self.fixture()
        body["source_evidence"][0]["rgb_track_id"]="other-track"
        with self.assertRaisesRegex(ValueError,"BINDING_INVALID"):
            replay(journal,face,body,timeline)

    def test_candidate_requires_corresponding_source_evidence(self):
        journal,face,body,timeline=self.fixture()
        timeline["per_cycle"][0]["candidate_entity_id"]="person_001"
        with self.assertRaisesRegex(ValueError,"TIMELINE_EVENT_BINDING_INVALID"):
            replay(journal,face,body,timeline)

    def test_face_snapshot_cannot_be_swapped_between_cycles(self):
        journal,face,body,timeline=self.fixture()
        timeline["per_cycle"][0]["face_snapshot"]=(
            timeline["per_cycle"][-1]["face_snapshot"])
        with self.assertRaisesRegex(ValueError,"TIMELINE_EVENT_BINDING_INVALID"):
            replay(journal,face,body,timeline)

    def test_replay_leaves_original_identity_coordinator_unmodified(self):
        journal,face,body,timeline=self.fixture()
        initial=deepcopy(timeline)
        replay(journal,face,body,timeline)
        self.assertEqual(timeline,initial)


if __name__=="__main__":
    unittest.main()
