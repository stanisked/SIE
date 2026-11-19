#!/usr/bin/env python3
"""Offline fixed-batch OSNet comparison on seven authenticated Shadow snapshots.

Source: original same-cycle RGB JPEG + selected segmentation mask, both bound
to the current audit through SHA256. Reports scores only, no raw embeddings.

No motor, live camera, SFace calibration, or target reassignment. A high
cosine score never constitutes a verified person identity in this script.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tarfile

import numpy as np

from vision_core.person_localization.shadow_body_osnet import (
    FixedBatchOsnet, SUPPORTED_CROPS,
    body_input_tensor, cosine_pairs, similarity_summary)
from vision_core.tools.audit_shadow_face_body import checked_mask_and_binding
from vision_core.tools.shadow_face_offline import selected_source_images


def analyze_archive(archive_path: Path, encoder: FixedBatchOsnet):
    verified=[]
    tensors={mode:[] for mode in SUPPORTED_CROPS}
    with tarfile.open(archive_path,"r:gz") as archive:
        files={info.name:info for info in archive.getmembers() if info.isfile()}
        if sum(info.size for info in files.values())>150_000_000:
            raise ValueError("SOURCE_EVIDENCE_TOO_LARGE")
        for source in selected_source_images(archive_path):
            cycle=source["cycle"]
            mask,candidate,manifest=checked_mask_and_binding(
                archive, files, source["cycle_id"])
            if (manifest.get("track_id")!=source["track_id"]
                    or candidate.get("bbox_xyxy_px")!=source["bbox"]):
                raise ValueError("SELECTED_RGB_TRACK_OR_BBOX_NOT_BOUND")
            metadata={
                "cycle":cycle,
                "cycle_id":source["cycle_id"],
                "rgb_track_id":source["track_id"],
                "rgb_jpeg_sha256":source["rgb_sha256"],
                "selected_mask_sha256":manifest["instance_mask_sha256"],
                "bbox_xyxy_px":[round(float(v),2) for v in source["bbox"]],
                "same_cycle_mask_verified":True,
            }
            for mode in SUPPORTED_CROPS:
                tensors[mode].append(body_input_tensor(
                    source["image"],mask,source["bbox"],variant=mode))
            verified.append(metadata)

    if len(verified)<2:
        raise ValueError("INSUFFICIENT_SHADOW_SOURCE_EVIDENCE")
    cycles=[row["cycle"] for row in verified]
    if len(set(cycles))!=len(cycles):
        raise ValueError("DUPLICATE_CYCLE_NUMBER")
    tracks=[row["rgb_track_id"] for row in verified]
    observations=[]
    for mode in SUPPORTED_CROPS:
        before=encoder.inference_calls
        features=encoder.encode(np.stack(tensors[mode],axis=0))
        pairwise=cosine_pairs(features,cycles,tracks)
        observations.append({
            "preprocessing_variant":mode,
            "input_tensors":len(features),
            "fixed_model_batch_size":16,
            "inference_batches":encoder.inference_calls-before,
            "inference_ms":encoder.inference_times_ms[before:],
            "group_statistics":similarity_summary(pairwise),
            "pairs":pairwise,
        })
    return {
        "scope":"MOTOR_OFF_BODY_REID_EXPLORATION_NO_IDENTITY_DECISION",
        "model":"osnet_x0_25_msmt17.onnx",
        "model_sha256":encoder.model_sha256,
        "input_shape":[16,3,256,128],
        "output_shape":[16,512],
        "model_preprocessing_assumption":
            "RGB/255 with ImageNet mean and std; original model card lacks contract",
        "snapshots":len(verified),
        "source_evidence":verified,
        "experiments":observations,
        "warnings":[
            "Changed RGB Track ID is NOT proof of a different person.",
            "Single-person positive-only sequence cannot measure false matches.",
            "Upper-35-percent occlusion is a proxy, not a real back view.",
            "No threshold selected; no target entity rebind; no motor command.",
            "Fixed batch 16 includes repeated padding ONLY during inference.",
            "The selected mask and original RGB JPEG are same-cycle hash verified.",
        ],
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive",required=True,type=Path)
    parser.add_argument("--model",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args()
    os.umask(0o077)
    output=args.output.expanduser()
    if output.exists():
        parser.error("Output already exists; will not overwrite")
    if not output.parent.is_dir():
        parser.error("Output parent directory does not exist")
    encoder=FixedBatchOsnet(args.model)
    report=analyze_archive(args.archive,encoder)
    with output.open("x",encoding="utf-8") as stream:
        os.chmod(output,0o600)
        json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write("\n")
    print(json.dumps({
        "scope":report["scope"],
        "model_sha256":report["model_sha256"],
        "snapshots":report["snapshots"],
        "cycles":[row["cycle"] for row in report["source_evidence"]],
        "track_ids":[row["rgb_track_id"] for row in report["source_evidence"]],
        "summary":[{
            "variant":exp["preprocessing_variant"],
            "inference_ms":exp["inference_ms"],
            "same_track":exp["group_statistics"]["SAME_RGB_TRACK"],
            "cross_track":exp["group_statistics"]["DIFFERENT_RGB_TRACK"],
            "untracked":exp["group_statistics"]["UNTRACKED_INVOLVED"],
        } for exp in report["experiments"]],
        "private_report":str(output)
    },ensure_ascii=False,indent=2,allow_nan=False))


if __name__=="__main__":
    main()
