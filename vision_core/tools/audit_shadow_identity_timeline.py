#!/usr/bin/env python3
"""Join SIEE motor-off journal + face/mask + OSNet pairwise evidence.

This is a chronological evidence ledger, NOT an entity identity decision.
Face scores are uncalibrated positive-only observations. No unseen person
can be declared the same person merely because the RGB Track ID changed.
It never runs a camera, actuates a motor, or writes biometric embeddings.

Inputs: bridge.jsonl, audit_shadow_face_body.py JSON,
        audit_shadow_body_osnet.py JSON.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import statistics

BODY_SCOPE = "MOTOR_OFF_BODY_REID_EXPLORATION_NO_IDENTITY_DECISION"
FACE_SCOPE = "MOTOR_OFF_FACE_BODY_GEOMETRIC_AUDIT_ONLY"
FACE_PROFILE = "person_001_face_enrollment_v1"
PERSON = "person_001"
SHA_RE = re.compile(r"[0-9a-f]{64}")
BODY_VARIANT = "mask_neutral_background"


def read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def cycle_number(cycle_id: str) -> int:
    if not isinstance(cycle_id, str) or not re.fullmatch(
            r"ar0234-ov9281-control-preview-[a-f0-9]{16}-[0-9]{8}",cycle_id):
        raise ValueError("INVALID_CYCLE_ID")
    return int(cycle_id[-8:])


def load_journal(path: Path):
    rows=[]
    with Path(path).open(encoding="utf-8") as stream:
        for i,line in enumerate(stream,1):
            row=json.loads(line)
            if not isinstance(row,dict) or row.get("source")!="fast_mask":
                raise ValueError(f"JOURNAL_SOURCE_INVALID_LINE_{i}")
            if row.get("cycle_id") is None:
                continue
            cycle=cycle_number(row["cycle_id"])
            if (row.get("decision") not in ("FOLLOW_ELIGIBLE","PAUSE")
                    or row.get("track_id") is not None and not isinstance(
                        row["track_id"],str)):
                raise ValueError("JOURNAL_DECISION_OR_TRACK_INVALID")
            rows.append({
                "cycle":cycle,
                "cycle_id":row["cycle_id"],
                "camera_track_id":row.get("track_id"),
                "decision":row["decision"],
                "audit_status":row.get("audit_status"),
                "audit_reason":row.get("audit_reason"),
                "rgb_reason":row.get("rgb_reason"),
                "segmentation_instance_count":row.get("segmentation_instance_count"),
                "eligible_count":row.get("current_eligible_count"),
                "mask_sha256":row.get("mask_sha256"),
            })
    if len(rows)<2:
        raise ValueError("JOURNAL_TOO_SHORT")
    rows.sort(key=lambda row:row["cycle"])
    if len({row["cycle"] for row in rows})!=len(rows):
        raise ValueError("JOURNAL_DUPLICATE_CYCLES")
    if any(rows[i]["cycle"] != rows[i-1]["cycle"]+1
           for i in range(1,len(rows))):
        raise ValueError("JOURNAL_HAS_CYCLE_GAPS")
    if len({row["cycle_id"].rsplit("-",1)[0] for row in rows})!=1:
        raise ValueError("JOURNAL_MULTIPLE_CAPTURE_SESSIONS")
    return rows


def provenance_join(journal,face_report,body_report):
    if (face_report.get("scope")!=FACE_SCOPE or
            face_report.get("target_entity_id")!=PERSON or
            face_report.get("face_profile_id")!=FACE_PROFILE or
            body_report.get("scope")!=BODY_SCOPE):
        raise ValueError("UNEXPECTED_REPORT_SCOPE_OR_PROFILE")
    faces=face_report.get("rows")
    bodies=body_report.get("source_evidence")
    if not isinstance(faces,list) or not isinstance(bodies,list):
        raise ValueError("SOURCE_ROWS_UNAVAILABLE")
    if (len(faces)!=len(bodies) or
            face_report.get("source_snapshots")!=len(faces) or
            body_report.get("snapshots")!=len(bodies)):
        raise ValueError("SNAPSHOT_COUNTS_DIFFER")
    by_journal={r["cycle_id"]:r for r in journal}
    by_face={r["cycle_id"]:r for r in faces}
    by_body={r["cycle_id"]:r for r in bodies}
    if (len(by_face)!=len(faces) or len(by_body)!=len(bodies) or
            set(by_face)!=set(by_body) or
            not set(by_face).issubset(by_journal)):
        raise ValueError("SOURCE_CYCLE_BINDING_INVALID")
    merged={}
    for cid,face in by_face.items():
        body=by_body[cid]
        journal_row=by_journal[cid]
        if (face.get("cycle")!=cycle_number(cid) or
                body.get("cycle")!=cycle_number(cid) or
                journal_row["cycle"]!=cycle_number(cid) or
                face.get("rgb_track_id")!=body.get("rgb_track_id") or
                face.get("rgb_track_id")!=journal_row["camera_track_id"] or
                face.get("mask_sha256")!=body.get("selected_mask_sha256") or
                face.get("rgb_original_jpeg_sha256")!=body.get("rgb_jpeg_sha256") or
                (journal_row["mask_sha256"] is not None and
                 journal_row["mask_sha256"]!=face.get("mask_sha256")) or
                face.get("same_cycle_rgb_mask_verified") is not True or
                body.get("same_cycle_mask_verified") is not True or
                journal_row["decision"]!="FOLLOW_ELIGIBLE"):
            raise ValueError("SOURCE_JOURNAL_FACE_BODY_BINDING_INVALID")
        if (not SHA_RE.fullmatch(face["mask_sha256"]) or
                not SHA_RE.fullmatch(face["rgb_original_jpeg_sha256"])):
            raise ValueError("SOURCE_SHA256_MALFORMED")
        # Classification below is deliberately "compatible", not "matched":
        # positive-only SFace scores are not a calibrated identity decision.
        relation=face.get("geometric_face_body_relation")
        if relation == "OVERLAP_OBSERVED_DIAGNOSTIC_ONLY":
            score=face.get("cosine_max")
            overlap=face.get("face_box_mask_overlap_fraction")
            if (type(score) not in (float,int) or
                    type(overlap) not in (float,int) or
                    not math.isfinite(score) or not -1<=score<=1 or
                    not math.isfinite(overlap) or not 0<overlap<=1 or
                    face.get("face_center_inside_mask") is not True):
                raise ValueError("FACE_SCORE_OR_MASK_RELATION_INVALID")
            state="FACE_PROFILE_COMPATIBLE_UNCALIBRATED"
        elif relation=="NO_VALID_SELECTED_FACE":
            if face.get("cosine_max") is not None:
                raise ValueError("NO_FACE_HAS_SIMILARITY")
            state="FACE_NOT_OBSERVED"
        elif relation=="NO_MASK_OVERLAP":
            state="FACE_NOT_BOUND_TO_SELECTED_BODY"
        else:
            raise ValueError("FACE_GEOMETRY_STATE_UNKNOWN")
        merged[cid]={
            "cycle":cycle_number(cid),
            "camera_track_id":body["rgb_track_id"],
            "face_observation":state,
            "sface_cosine_max":face.get("cosine_max"),
            "face_mask_overlap":face.get("face_box_mask_overlap_fraction"),
            "same_cycle_source_provenance_verified":True,
        }
    return merged


def summarize_cross_track(body_report, pre_track, new_track):
    variants=body_report.get("experiments")
    if not isinstance(variants,list):
        raise ValueError("BODY_EXPERIMENTS_MISSING")
    options=[item for item in variants
             if item.get("preprocessing_variant")==BODY_VARIANT]
    if len(options)!=1:
        raise ValueError("BODY_VARIANT_UNAVAILABLE")
    pairs=options[0].get("pairs")
    if not isinstance(pairs,list):
        raise ValueError("BODY_PAIRS_UNAVAILABLE")
    scores=[]
    per_cycle={}
    for pair in pairs:
        tracks=pair.get("tracks")
        cycles=pair.get("cycles")
        sim=pair.get("cosine_similarity")
        if (not isinstance(tracks,list) or len(tracks)!=2 or
                not isinstance(cycles,list) or len(cycles)!=2 or
                type(sim) not in (int,float) or not math.isfinite(sim) or
                not -1<=sim<=1):
            raise ValueError("BODY_PAIR_INVALID")
        if (tracks[0]==pre_track and tracks[1]==new_track or
                tracks[1]==pre_track and tracks[0]==new_track):
            if pair.get("group")!="DIFFERENT_RGB_TRACK":
                raise ValueError("BODY_PAIR_WRONG_TRACK_GROUP")
            scores.append(float(sim))
            new_cycle=cycles[0] if tracks[0]==new_track else cycles[1]
            per_cycle.setdefault(new_cycle,[]).append(float(sim))
    if not scores:
        raise ValueError("NO_REACQUIRED_CROSS_TRACK_BODY_PAIRS")
    return {
        "preloss_track_id":pre_track,
        "reacquired_track_id":new_track,
        "body_variant":BODY_VARIANT,
        "pairs":len(scores),
        "min":round(min(scores),5),
        "median":round(statistics.median(scores),5),
        "max":round(max(scores),5),
        "per_reacquired_cycle":[{
            "cycle":cycle,
            "preloss_track_comparison_count":len(values),
            "cosine_median":round(statistics.median(values),5),
            "cosine_min":round(min(values),5),
        } for cycle,values in sorted(per_cycle.items())],
        "interpretation":"BODY_SAME_PERSON_HYPOTHESIS_ONLY_NO_THRESHOLD",
    }


def chronological_segments(rows):
    segments=[]
    for row in rows:
        key=(row["camera_track_id"],row["decision"])
        if segments and segments[-1]["key"]==key and (
                segments[-1]["end_cycle"]+1==row["cycle"]):
            segments[-1]["end_cycle"]=row["cycle"]
            segments[-1]["length"]+=1
        else:
            segments.append({
                "key":key, "start_cycle":row["cycle"],
                "end_cycle":row["cycle"],"length":1})
    return [{
        "first_cycle":item["start_cycle"],
        "last_cycle":item["end_cycle"],
        "cycles":item["length"],
        "rgb_track_id":item["key"][0],
        "decision":item["key"][1],
    } for item in segments]


def construct_timeline(journal,face_report,body_report):
    merged=provenance_join(journal,face_report,body_report)
    follows=[r for r in journal if r["decision"]=="FOLLOW_ELIGIBLE"
             and r["camera_track_id"]]
    if len(follows)<2:
        raise ValueError("NOT_ENOUGH_RGB_TARGET_OBSERVATIONS")
    first_track=follows[0]["camera_track_id"]
    later_tracks=[r["camera_track_id"] for r in follows
                  if r["camera_track_id"]!=first_track]
    if not later_tracks:
        raise ValueError("NO_RGB_TRACK_CHANGE_IN_JOURNAL")
    second_track=later_tracks[0]
    if len(set(later_tracks))!=1:
        raise ValueError("MULTIPLE_REACQUIRED_TRACKS_NEED_SEPARATE_AUDIT")
    first_new=min(r["cycle"] for r in follows
                  if r["camera_track_id"]==second_track)
    if any(r["camera_track_id"]==first_track for r in follows
           if r["cycle"]>=first_new):
        raise ValueError("INTERLEAVED_TRACKS_NOT_THIS_SCENARIO")
    link=summarize_cross_track(body_report,first_track,second_track)
    anchor_candidates=[
        r for r in merged.values()
        if (r["camera_track_id"]==second_track and
            r["face_observation"]=="FACE_PROFILE_COMPATIBLE_UNCALIBRATED")]
    first_candidate_cycle=min(
        (r["cycle"] for r in anchor_candidates),default=None)
    ledger=[]
    for row in journal:
        cycle=row["cycle"]
        observation=merged.get(row["cycle_id"])
        if row["decision"]=="PAUSE":
            state="TARGET_NOT_CURRENTLY_ELIGIBLE"
        elif row["camera_track_id"]==first_track:
            state="PRELOSS_RGB_BODY_UNANCHORED"
        elif row["camera_track_id"]==second_track:
            if (first_candidate_cycle is not None and
                    cycle>=first_candidate_cycle):
                state="PERSON_001_FACE_CANDIDATE_ON_CURRENT_TRACK"
            else:
                state="REACQUIRED_BODY_ONLY_NOT_FACE_VERIFIED"
        else:
            state="UNACCOUNTED_RGB_TRACK"
        # Candidate ID only after supporting face/mask source evidence;
        # NEVER fill confirmed_entity_id from uncalibrated SFace.
        ledger.append({
            "cycle":cycle,
            "cycle_id":row["cycle_id"],
            "decision":row["decision"],
            "rgb_track_id":row["camera_track_id"],
            "identity_status":state,
            "confirmed_entity_id":None,
            "candidate_entity_id":(
                PERSON if state=="PERSON_001_FACE_CANDIDATE_ON_CURRENT_TRACK"
                else None),
            "face_snapshot":observation,
            "motor_authorized":False,
        })
    return {
        "scope":"MOTOR_OFF_PERSON_001_EVIDENCE_TIMELINE_NOT_IDENTITY_ADMISSION",
        "entity_id_under_test":PERSON,
        "face_profile_id":FACE_PROFILE,
        "frames":len(journal),
        "source_snapshots":len(merged),
        "first_rgb_track_id":first_track,
        "reacquired_rgb_track_id":second_track,
        "reacquired_first_cycle":first_new,
        "first_face_compatible_cycle":first_candidate_cycle,
        "face_compatible_snapshots_after_reacquisition":len(anchor_candidates),
        "body_reid_cross_track":link,
        "segments":chronological_segments(journal),
        "identity_status_counts":dict(Counter(
            row["identity_status"] for row in ledger)),
        "snapshot_events":[row for row in ledger
                           if row["face_snapshot"] is not None],
        "per_cycle":ledger,
        "policy":{
            "face_identity_calibrated":False,
            "body_reid_identity_calibrated":False,
            "confirmed_identity_assigned":False,
            "motor_authorized":False,
            "preloss_track_link_proven":False,
        },
        "limitations":[
            "Pre-loss RGB track had no detected face on archived snapshots.",
            "Cross-track OSNet scores are a same-person hypothesis, not proof.",
            "Post-reacquisition faces match profile descriptors but false-accept "
            "rates are not measured; no calibrated face verification.",
            "The label person_001 is candidate-only, never a confirmed identity.",
            "Adjacent frames are correlated, and there is no negative-person control.",
        ],
    }


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--journal",type=Path,required=True)
    p.add_argument("--face-report",type=Path,required=True)
    p.add_argument("--body-report",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    os.umask(0o077)
    if args.output.exists():
        p.error("Refusing to overwrite timeline")
    if not args.output.parent.is_dir():
        p.error("Timeline parent directory missing")
    result=construct_timeline(load_journal(args.journal),
                              read_json(args.face_report),
                              read_json(args.body_report))
    with args.output.open("x",encoding="utf-8") as stream:
        os.chmod(args.output,0o600)
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write("\n")
    print(json.dumps({
        "scope":result["scope"],
        "frames":result["frames"],
        "source_snapshots":result["source_snapshots"],
        "reacquired_first_cycle":result["reacquired_first_cycle"],
        "first_face_compatible_cycle":result["first_face_compatible_cycle"],
        "face_compatible_snapshots_after_reacquisition":
            result["face_compatible_snapshots_after_reacquisition"],
        "body_reid_cross_track":result["body_reid_cross_track"],
        "segments":result["segments"],
        "identity_status_counts":result["identity_status_counts"],
        "policy":result["policy"],
        "private_report":str(args.output),
    },ensure_ascii=False,indent=2,allow_nan=False))


if __name__=="__main__":
    main()
