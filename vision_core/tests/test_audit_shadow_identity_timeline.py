"""Synthetic identity evidence timeline tests. Never labels identity as proven."""
import json
from pathlib import Path
import tempfile
import unittest

from vision_core.tools.audit_shadow_identity_timeline import (
    construct_timeline, load_journal, provenance_join, summarize_cross_track)


PREFIX="ar0234-ov9281-control-preview-aaaaaaaaaaaaaaaa-"
T1="ar0234-audit-track:aaaaaaaaaaaaaaaa:1"
T2="ar0234-audit-track:aaaaaaaaaaaaaaaa:2"
SHA1="a"*64
SHA2="b"*64


def cycle(i):
    return PREFIX+f"{i:08d}"


def test_rows():
    journal=[]
    for i,decision,track in [
            (1,"FOLLOW_ELIGIBLE",T1),
            (2,"PAUSE",T1),
            (3,"PAUSE",None),
            (4,"FOLLOW_ELIGIBLE",T2),
            (5,"FOLLOW_ELIGIBLE",T2)]:
        journal.append({
            "cycle":i,"cycle_id":cycle(i),"camera_track_id":track,
            "decision":decision,"mask_sha256":SHA1,
            "segmentation_instance_count":1 if decision=="FOLLOW_ELIGIBLE" else 0,
            "eligible_count":1 if decision=="FOLLOW_ELIGIBLE" else 0,
        })
    face_rows=[]
    body_rows=[]
    for i,track,face in [(1,T1,False),(4,T2,False),(5,T2,True)]:
        face_rows.append({
            "cycle_id":cycle(i),"cycle":i,"rgb_track_id":track,
            "mask_sha256":SHA1,"rgb_original_jpeg_sha256":SHA2,
            "same_cycle_rgb_mask_verified":True,
            "geometric_face_body_relation":(
                "OVERLAP_OBSERVED_DIAGNOSTIC_ONLY"
                if face else "NO_VALID_SELECTED_FACE"),
            "cosine_max":0.7234 if face else None,
            "face_box_mask_overlap_fraction":0.96 if face else None,
            "face_center_inside_mask":True if face else None,
        })
        body_rows.append({
            "cycle_id":cycle(i),"cycle":i,"rgb_track_id":track,
            "selected_mask_sha256":SHA1,"rgb_jpeg_sha256":SHA2,
            "same_cycle_mask_verified":True,
        })
    faces={
        "scope":"MOTOR_OFF_FACE_BODY_GEOMETRIC_AUDIT_ONLY",
        "target_entity_id":"person_001",
        "face_profile_id":"person_001_face_enrollment_v1",
        "source_snapshots":len(face_rows),"rows":face_rows,
    }
    body={
        "scope":"MOTOR_OFF_BODY_REID_EXPLORATION_NO_IDENTITY_DECISION",
        "snapshots":len(body_rows),"source_evidence":body_rows,
        "experiments":[{
            "preprocessing_variant":"mask_neutral_background",
            "pairs":[
                {"cycles":[1,4],"tracks":[T1,T2],
                 "group":"DIFFERENT_RGB_TRACK","cosine_similarity":0.8},
                {"cycles":[1,5],"tracks":[T1,T2],
                 "group":"DIFFERENT_RGB_TRACK","cosine_similarity":0.79},
            ],
        }],
    }
    return journal,faces,body


class IdentityTimelineTests(unittest.TestCase):
    def test_no_identity_assigned_until_calibration_even_with_face(self):
        rows,face,body=test_rows()
        report=construct_timeline(rows,face,body)
        self.assertEqual(report["frames"],5)
        self.assertEqual(report["first_rgb_track_id"],T1)
        self.assertEqual(report["reacquired_first_cycle"],4)
        self.assertEqual(report["first_face_compatible_cycle"],5)
        self.assertEqual(report["face_compatible_snapshots_after_reacquisition"],1)
        self.assertEqual(report["body_reid_cross_track"]["median"],0.795)
        self.assertEqual(report["per_cycle"][0]["identity_status"],
                         "PRELOSS_RGB_BODY_UNANCHORED")
        self.assertEqual(report["per_cycle"][2]["identity_status"],
                         "TARGET_NOT_CURRENTLY_ELIGIBLE")
        self.assertEqual(report["per_cycle"][3]["identity_status"],
                         "REACQUIRED_BODY_ONLY_NOT_FACE_VERIFIED")
        self.assertEqual(report["per_cycle"][4]["identity_status"],
                         "PERSON_001_FACE_CANDIDATE_ON_CURRENT_TRACK")
        self.assertEqual(report["per_cycle"][4]["candidate_entity_id"],
                         "person_001")
        self.assertTrue(all(x["confirmed_entity_id"] is None
                            and x["motor_authorized"] is False
                            for x in report["per_cycle"]))

    def test_refuse_face_and_body_frame_hash_mismatch(self):
        rows,face,body=test_rows()
        body["source_evidence"][0]["selected_mask_sha256"]="c"*64
        with self.assertRaisesRegex(ValueError,"BINDING_INVALID"):
            provenance_join(rows,face,body)

    def test_refuse_journal_track_mismatch(self):
        rows,face,body=test_rows()
        rows[3]["camera_track_id"]=T1
        with self.assertRaisesRegex(ValueError,"BINDING_INVALID"):
            provenance_join(rows,face,body)

    def test_reject_face_sim_without_geometry(self):
        rows,face,body=test_rows()
        face["rows"][0]["cosine_max"]=0.95
        with self.assertRaisesRegex(ValueError,"NO_FACE_HAS_SIMILARITY"):
            provenance_join(rows,face,body)

    def test_negative_control_no_match_threshold_is_inferred(self):
        rows,face,body=test_rows()
        score=summarize_cross_track(body,T1,T2)
        self.assertEqual(score["pairs"],2)
        self.assertEqual(score["interpretation"],
                         "BODY_SAME_PERSON_HYPOTHESIS_ONLY_NO_THRESHOLD")
        self.assertNotIn("identity_match",score)

    def test_journal_parser_skips_empty_timing_ticks(self):
        rows,_,_=test_rows()
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"bridge.jsonl"
            lines=[json.dumps(dict(
                source="fast_mask",
                cycle_id=r["cycle_id"],
                track_id=r["camera_track_id"],
                decision=r["decision"],
                mask_sha256=r["mask_sha256"],
            )) for r in rows]
            lines.insert(2,json.dumps({"source":"fast_mask","cycle_id":None}))
            path.write_text("\n".join(lines)+"\n")
            parsed=load_journal(path)
            self.assertEqual(len(parsed),5)
            self.assertEqual(parsed[3]["cycle"],4)
            self.assertEqual(parsed[3]["camera_track_id"],T2)


if __name__=="__main__":
    unittest.main()
