#!/usr/bin/env python3
"""Replay motor-off Face/Body evidence into the EXISTING FaceAnchoredTarget.

Important: The 11 source snapshots were individually SHA/binding checked by
upstream audits. All remaining 132 journal cycles lack retained RGB/body
embeddings, so they are NOT reconstructed, fabricated or inferred here.

SFace cosine is uncalibrated: seen faces are UNVERIFIED_FACE, never MATCHED.
OSNet reports pairwise similarities only, not per-frame embeddings: this
adapter never invents embeddings or configures body re-identification.
Thus no permanent entity can be established from this data.

Pure replay with logical time == cycle index * 0.1 seconds for the coordinator
freshness contract. NOT a measured camera timestamp, NOT an execution trace.
No ROS, camera, motor, or live vision processes are started.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path

from vision_core.person_localization.shadow_face_anchored_target import (
    FACE_NONE, FACE_UNKNOWN, FaceAnchoredTarget, FaceBodyEvidence)
from vision_core.tools.audit_shadow_identity_timeline import (
    FACE_PROFILE, PERSON, BODY_SCOPE, FACE_SCOPE,
    load_journal, provenance_join, read_json)


def replay(journal, face_report, body_report, timeline):
    # Recheck cross-source provenance, including exact JPEG & mask hashes,
    # selected RGB track IDs and snapshot cycle IDs.
    source = provenance_join(journal, face_report, body_report)
    if (timeline.get("scope") !=
            "MOTOR_OFF_PERSON_001_EVIDENCE_TIMELINE_NOT_IDENTITY_ADMISSION"
            or timeline.get("entity_id_under_test") != PERSON
            or timeline.get("face_profile_id") != FACE_PROFILE
            or timeline.get("frames") != len(journal)
            or timeline.get("source_snapshots") != len(source)
            or timeline.get("policy") != {
                "face_identity_calibrated":False,
                "body_reid_identity_calibrated":False,
                "confirmed_identity_assigned":False,
                "motor_authorized":False,
                "preloss_track_link_proven":False}):
        raise ValueError("IDENTITY_TIMELINE_POLICY_OR_SCOPE_INVALID")
    ledger=timeline.get("per_cycle")
    if (not isinstance(ledger,list) or len(ledger) != len(journal)):
        raise ValueError("TIMELINE_CYCLE_COVERAGE_MISMATCH")

    faces={row["cycle_id"]:row for row in face_report["rows"]}
    model=FaceAnchoredTarget(entity_id=PERSON,face_profile_id=FACE_PROFILE)
    events=[]
    observed=0
    unknown_faces=0
    for jr,context in zip(journal,ledger):
        cid=jr["cycle_id"]
        if (context.get("cycle_id")!=cid
                or context.get("cycle")!=jr["cycle"]
                or context.get("rgb_track_id")!=jr["camera_track_id"]
                or context.get("decision")!=jr["decision"]
                or context.get("confirmed_entity_id") is not None
                or context.get("motor_authorized") is not False
                or context.get("candidate_entity_id") != (
                    PERSON if context.get("identity_status")==
                    "PERSON_001_FACE_CANDIDATE_ON_CURRENT_TRACK"
                    else None)
                or context.get("face_snapshot") != source.get(cid)):
            raise ValueError("TIMELINE_EVENT_BINDING_INVALID")

        snapshot=source.get(cid)
        # If no snapshot, no input frame is available. PAUSE alone is
        # not a source image; do not synthesize FaceBodyEvidence.
        evidence=None
        state="NO_RETAINED_IMAGE_FOR_IDENTITY_COORDINATOR"
        if snapshot is not None:
            face=faces[cid]
            face_status=snapshot["face_observation"]
            if face_status not in (
                    "FACE_NOT_OBSERVED",
                    "FACE_PROFILE_COMPATIBLE_UNCALIBRATED",
                    "FACE_NOT_BOUND_TO_SELECTED_BODY"):
                raise ValueError("UNEXPECTED_FACE_DIAGNOSTIC")
            # When geometry is not bound, do not set face_visible.
            face_unknown=face_status=="FACE_PROFILE_COMPATIBLE_UNCALIBRATED"
            mask_sha=face["mask_sha256"]
            evidence=FaceBodyEvidence(
                cycle_id=cid,
                captured_s=jr["cycle"]*0.1, # LOGICAL time only
                camera_track_id=jr["camera_track_id"],
                mask_sha256=mask_sha,
                unique_body_binding=(
                    snapshot["same_cycle_source_provenance_verified"]
                    and jr["decision"]=="FOLLOW_ELIGIBLE"),
                sole_visible_person=(
                    jr["segmentation_instance_count"]==1
                    and jr["eligible_count"]==1),
                face_result=FACE_UNKNOWN if face_unknown else FACE_NONE,
                face_profile_id=FACE_PROFILE if face_unknown else None,
                face_cycle_id=cid if face_unknown else None,
                face_mask_sha256=mask_sha if face_unknown else None,
                face_verifier_calibrated=False,
                body_model_sha256=None,
                body_embedding=None,
            )
            observed+=1
            unknown_faces+=int(face_unknown)
            state=face_status
        decision=model.observe(evidence,now_s=jr["cycle"]*0.1)
        if (decision.motor_authorized or decision.identity_supported
                or decision.entity_id is not None
                or decision.status in ("FACE_ANCHORED","FACE_REVERIFIED",
                                       "FACE_CONFIRMED","BODY_TRACKED",
                                       "BODY_REIDENTIFIED","BODY_REID_PENDING")):
            raise AssertionError("UNCALIBRATED_EVIDENCE_PROMOTED_TO_IDENTITY")

        events.append({
            "cycle":jr["cycle"],
            "cycle_id":cid,
            "rgb_track_id":jr["camera_track_id"],
            "identity_timeline_status":context["identity_status"],
            "unverified_candidate_entity_id":context["candidate_entity_id"],
            "coordinator_input":state,
            "coordinator_status":decision.status,
            "coordinator_reason":decision.reason,
            "face_visible":decision.face_visible,
            "confirmed_entity_id":None,
            "motor_authorized":False,
        })

    if observed != len(source):
        raise ValueError("REPLAY_SOURCE_COVERAGE_INVALID")
    return {
        "scope":"MOTOR_OFF_UNCALIBRATED_IDENTITY_COORDINATOR_REPLAY",
        "coordinator":"FaceAnchoredTarget",
        "entity_id_under_test":PERSON,
        "face_profile_id":FACE_PROFILE,
        "journal_cycles":len(journal),
        "snapshot_inputs":observed,
        "unverified_face_inputs":unknown_faces,
        "cycles_without_retained_identity_image":len(journal)-observed,
        "coordinator_status_counts":dict(Counter(
            item["coordinator_status"] for item in events)),
        "confirmed_identity_count":0,
        "body_embeddings_supplied":0,
        "face_verifier_calibrated":False,
        "body_reid_calibrated":False,
        "motor_authorized":False,
        "events":events,
        "limits":[
            "Only retained snapshot frames enter identity coordinator.",
            "Logical replay timestamps are not real camera timestamps.",
            "Uncalibrated SFace similarity is reported as UNVERIFIED_FACE.",
            "OSNet pairwise score is not a body embedding or identity decision.",
            "Existing FaceAnchoredTarget remains unchanged; no motor access.",
        ],
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal",type=Path,required=True)
    parser.add_argument("--face-report",type=Path,required=True)
    parser.add_argument("--body-report",type=Path,required=True)
    parser.add_argument("--timeline",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    os.umask(0o077)
    path=args.output.expanduser()
    if path.exists():
        parser.error("Output exists; refusing overwrite")
    if not path.parent.is_dir():
        parser.error("Output directory unavailable")
    report=replay(load_journal(args.journal),
                  read_json(args.face_report),
                  read_json(args.body_report),
                  read_json(args.timeline))
    with path.open("x",encoding="utf-8") as stream:
        os.chmod(path,0o600)
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write("\n")
    compact={key:value for key,value in report.items() if key!="events"}
    compact["private_report"]=str(path)
    print(json.dumps(compact,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=="__main__":
    main()
