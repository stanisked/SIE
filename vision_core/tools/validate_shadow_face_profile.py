#!/usr/bin/env python3
"""Independent, motor-off SFace evaluation for a private AR0234 dataset.

Compares a previously enrolled person_001 face profile to a DIFFERENT
capture dataset. Never enrolls, edits the profile, opens cameras or connects
to ESP32. This is a positive-only evaluation, NOT a calibrated FAR/FRR.

SFace cosine of a novel face and previously enrolled templates is the
output. Mere YuNet detection does not mean an identified person; the
recorded validation dataset has no body mask to associate the face to.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np

from vision_core.person_localization.shadow_face_verify import (
    LocalFaceEncoder, normalized_feature)
from vision_core.tools.curate_shadow_face_dataset import read_source_rows
from vision_core.tools.shadow_face_offline import load_private_profile


def evaluate(dataset: Path, profile: Path, encoder: LocalFaceEncoder) -> dict:
    dataset = Path(dataset).resolve()
    profile = Path(profile).expanduser().resolve()
    features, metadata = load_private_profile(profile, encoder)
    if (metadata.get("target_entity_id") != "person_001"
            or metadata.get("face_profile_id") != "person_001_face_enrollment_v1"
            or len(features) != 23
            or 132 in metadata.get("sample_indices", [])):
        raise ValueError("WRONG_OR_UNVERIFIED_ENROLLMENT_PROFILE")
    if dataset == Path(metadata.get("source_dataset", "")).resolve():
        raise ValueError("ENROLLMENT_DATASET_IS_NOT_VALIDATION")
    if "validation_" not in dataset.name:
        raise ValueError("NON_VALIDATION_DATASET")
    rows = []
    first_mono = None
    count_original_faces = 0
    agreement = {"both_detect": 0, "both_empty": 0,
                 "original_only": 0, "redetection_only": 0,
                 "ambiguous_redetection": 0}
    for row, frame in read_source_rows(dataset):
        observed = float(row["host_monotonic_s"])
        if not math.isfinite(observed):
            raise ValueError("INVALID_SOURCE_MONOTONIC")
        if first_mono is None:
            first_mono = observed
        had_one_face = len(row["faces"]) == 1
        if had_one_face:
            count_original_faces += 1

        height,width=frame.shape[:2]
        scale=min(1.0,960.0/max(height,width))
        reduced=(cv2.resize(frame,(round(width*scale),round(height*scale)),
                 interpolation=cv2.INTER_AREA) if scale < 1. else frame)
        encoder.detector.setInputSize((reduced.shape[1],reduced.shape[0]))
        _,faces=encoder.detector.detect(reduced)
        face_count = 0 if faces is None else len(faces)

        result={"sample_index":int(row["sample_index"]),
                "time_s":round(observed-first_mono,3),
                "original_face_count":len(row["faces"]),
                "redetected_face_count":face_count}
        if face_count > 1:
            agreement["ambiguous_redetection"] += 1
            result["status"]="MULTIPLE_FACES_NOT_EVALUATED"
        elif face_count == 0:
            agreement["original_only" if had_one_face else "both_empty"]+=1
            result["status"]="NO_FACE_VISIBLE"
        elif not had_one_face:
            agreement["redetection_only"]+=1
            result["status"]="NOT_IN_ORIGINAL_FACE_DETECTIONS"
        else:
            agreement["both_detect"]+=1
            face=np.asarray(faces[0],dtype=np.float32).copy()
            face[:14]/=scale
            try:
                aligned=encoder.recognizer.alignCrop(frame,face)
                candidate=normalized_feature(encoder.recognizer.feature(aligned))
            except (ValueError, cv2.error):
                result["status"]="SFACE_FEATURE_REJECTED"
            else:
                cosines=features @ candidate
                if (not np.isfinite(cosines).all() or
                        cosines.shape != (23,)):
                    raise ValueError("INVALID_SFACE_SIMILARITY")
                ranked=np.sort(cosines)[::-1]
                result.update({
                    "status":"SFACE_SCORED_NOT_IDENTITY_PROOF",
                    "cosine_max":round(float(ranked[0]),5),
                    "cosine_top3_mean":round(float(ranked[:3].mean()),5),
                    "cosine_median":round(float(np.median(ranked)),5),
                    "face_width_px":round(float(face[2]),1),
                    "yunet_score":round(float(face[14]),4),
                })
        rows.append(result)
    if not rows:
        raise ValueError("NO_VALIDATION_FRAMES")

    scored = [row for row in rows if row["status"] ==
              "SFACE_SCORED_NOT_IDENTITY_PROOF"]
    def distribution(records,field):
        if not records:
            return None
        data=np.asarray([r[field] for r in records],dtype=float)
        return {
            "count":len(records),
            "min":round(float(data.min()),5),
            "p05":round(float(np.percentile(data,5)),5),
            "median":round(float(np.median(data)),5),
            "p95":round(float(np.percentile(data,95)),5),
            "max":round(float(data.max()),5)}
    windows=[]
    for start,end in ((0,20),(20,40),(40,60)):
        group=[r for r in scored if start <= r["time_s"] < end]
        windows.append({
            "start_s":start,"end_s":end,
            "scored_frames":len(group),
            "cosine_max":distribution(group,"cosine_max"),
            "cosine_top3_mean":distribution(group,"cosine_top3_mean")})
    return {
        "scope":"PRIVATE_POSITIVE_ONLY_SFACE_EVALUATION_NO_MOTOR_ADMISSION",
        "face_profile_id":"person_001_face_enrollment_v1",
        "target_entity_id":"person_001",
        "validation_dataset":str(dataset),
        "profile_source_is_different_dataset":True,
        "yunet_sha256":encoder.yunet_sha256,
        "sface_sha256":encoder.sface_sha256,
        "profile_count":len(features),
        "frames":len(rows),
        "original_faces":count_original_faces,
        "redetection":agreement,
        "scored_faces":len(scored),
        "cosine_max":distribution(scored,"cosine_max"),
        "cosine_top3_mean":distribution(scored,"cosine_top3_mean"),
        "cosine_median":distribution(scored,"cosine_median"),
        "by_time_window":windows,
        "lowest_max_similarity":[
            {"sample_index":r["sample_index"],
             "time_s":r["time_s"],
             "cosine_max":r["cosine_max"],
             "cosine_top3_mean":r["cosine_top3_mean"]}
            for r in sorted(scored,key=lambda r:r["cosine_max"])[:10]],
        "per_frame":rows,
        "warning":(
            "Only claimed same-person capture. Negative examples of other "
            "people are required to calibrate decision thresholds and estimate "
            "false-accept rates; adjacent video frames are correlated. "
            "No body-mask association or movement permitted."
        )
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--profile",type=Path,required=True)
    parser.add_argument("--yunet",type=Path,required=True)
    parser.add_argument("--sface",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    os.umask(0o077)
    output=args.output.expanduser()
    if output.exists():
        parser.error("Output exists (no overwrite)")
    if not output.parent.is_dir():
        parser.error("Output parent directory missing")
    encoder=LocalFaceEncoder(args.yunet,args.sface)
    result=evaluate(args.dataset,args.profile,encoder)
    with output.open("x",encoding="utf-8") as stream:
        os.chmod(output,0o600)
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write("\n")
    compact={key:value for key,value in result.items() if key != "per_frame"}
    print(json.dumps(compact,ensure_ascii=False,indent=2,allow_nan=False))
    print("PRIVATE_FULL_REPORT:",output)


if __name__=="__main__":
    main()
