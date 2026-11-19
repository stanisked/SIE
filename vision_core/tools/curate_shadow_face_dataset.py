#!/usr/bin/env python3
"""Offline SIEE face enrollment candidate curation: private, motor-off.

Reads one private AR0234 capture dataset. Verifies each JPEG against its
manifest SHA256. Re-runs YuNet for landmarks and extracts SFace embeddings
to choose diverse candidate frames. Writes a PRIVATE contact sheet and
candidate manifest, NOT a trusted identity profile or a match threshold.

Embedding novelty and YuNet landmark-derived pose proxy only diversify
examples. They do not prove identity, gaze, facial pose or liveness.
Operator must review all candidates before enrollment.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re

import cv2
import numpy as np

from vision_core.person_localization.shadow_face_verify import (
    LocalFaceEncoder, model_sha256, normalized_feature,
)


NAME_RE = re.compile(r"rgb/rgb_[0-9]{5}\.jpg")
TARGET_COUNT = 24
BINS = 6
MAX_MANIFEST_LINES = 1000
MAX_IMAGE_SIZE = 8_000_000


def read_source_rows(dataset_dir: Path):
    manifest = dataset_dir / "manifest.jsonl"
    if not manifest.is_file():
        raise FileNotFoundError("DATASET_MANIFEST_NOT_FOUND")
    rows = []
    with manifest.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            if len(rows) >= MAX_MANIFEST_LINES:
                raise ValueError("MANIFEST_TOO_LARGE")
            entry = json.loads(line)
            filename = entry.get("file")
            if not isinstance(filename,str) or not NAME_RE.fullmatch(filename):
                raise ValueError("UNSAFE_IMAGE_PATH")
            if entry.get("frame_width") != 1920 or entry.get("frame_height") != 1200:
                raise ValueError("UNEXPECTED_AR0234_SHAPE")
            image_path = dataset_dir / filename
            if not image_path.is_file() or image_path.stat().st_size > MAX_IMAGE_SIZE:
                raise ValueError("MISSING_OR_OVERSIZED_IMAGE")
            raw = image_path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != entry.get("jpeg_sha256"):
                raise ValueError("DATASET_IMAGE_SHA256_MISMATCH")
            image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape != (1200,1920,3):
                raise ValueError("DATASET_DECODED_FRAME_INVALID")
            if not isinstance(entry.get("faces"),list):
                raise ValueError("DATASET_FACE_METADATA_INVALID")
            rows.append((entry,image))
    if not rows:
        raise ValueError("DATASET_EMPTY")
    return rows


def extract_candidates(source_rows, encoder: LocalFaceEncoder):
    first = source_rows[0][0]["host_monotonic_s"]
    candidates=[]
    exclusions = {"no_single_face":0, "not_detected_again":0,
                  "low_quality":0, "encoding_error":0}
    for entry, image in source_rows:
        if len(entry["faces"]) != 1:
            exclusions["no_single_face"] += 1
            continue
        # This is operator's separate single-person enrollment dataset.
        # This bounding box is used only to gate face extraction in an
        # image, not as proof of face-to-body association.
        _, h_w = None, image.shape
        height, width = h_w[:2]
        scale=min(1., 960./max(width,height))
        small=(cv2.resize(image,(round(width*scale),round(height*scale)),
                          interpolation=cv2.INTER_AREA)
               if scale < 1 else image)
        encoder.detector.setInputSize((small.shape[1],small.shape[0]))
        _, faces=encoder.detector.detect(small)
        if faces is None or len(faces)!=1:
            exclusions["not_detected_again"]+=1
            continue
        face=np.asarray(faces[0],dtype=np.float32).copy()
        face[:14]/=scale
        x,y,w,h=[float(x) for x in face[:4]]
        if (float(face[14]) < .85 or w < 85 or h < 55
                or x+w <= 0 or y+h <= 0):
            exclusions["low_quality"]+=1
            continue
        x1,y1=max(0,int(x)),max(0,int(y))
        x2,y2=min(width,int(x+w)),min(height,int(y+h))
        crop=image[y1:y2,x1:x2]
        if crop.size==0:
            exclusions["low_quality"]+=1
            continue
        focus=float(cv2.Laplacian(cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY),
                                  cv2.CV_64F).var())
        if not math.isfinite(focus) or focus < 120:
            exclusions["low_quality"]+=1
            continue
        try:
            aligned=encoder.recognizer.alignCrop(image,face)
            feature=normalized_feature(encoder.recognizer.feature(aligned))
        except (cv2.error,ValueError):
            exclusions["encoding_error"]+=1
            continue
        t=float(entry["host_monotonic_s"])-float(first)
        eye_distance=abs(float(face[6])-float(face[4]))
        yaw_proxy=((float(face[8]) - (float(face[4])+float(face[6]))/2)
                    /max(eye_distance,1.))
        if not math.isfinite(t) or not math.isfinite(yaw_proxy):
            exclusions["low_quality"]+=1
            continue
        quality=(.55*min(1.,w/180.)+
                 .30*min(1.,focus/500.)+
                 .15*min(1.,float(face[14])))
        candidates.append({
            "sample_index":int(entry["sample_index"]),
            "filename":entry["file"],
            "jpeg_sha256":entry["jpeg_sha256"],
            "time_s":t,
            "width_px":w,
            "sharpness":focus,
            "yunet_score":float(face[14]),
            "yaw_proxy":yaw_proxy,
            "quality":quality,
            "feature":feature,
            "aligned":aligned,
        })
    return candidates,exclusions


def select_candidates(candidates, *, count=TARGET_COUNT, bins=BINS):
    """Equal time-bin coverage; novelty of embeddings/landmarks within bins."""
    if not candidates or count <= 0 or bins <= 0 or count % bins:
        raise ValueError("INVALID_CURATE_SELECTION")
    max_time=max(float(row["time_s"]) for row in candidates)
    if max_time <= 0:
        raise ValueError("DATASET_HAS_NO_TIME_SPAN")
    selected=[]
    for bin_index in range(bins):
        pool=[r for r in candidates if min(bins-1,
              int(float(r["time_s"])/max_time*bins))==bin_index]
        quota=count//bins
        if len(pool)<quota:
            raise ValueError(f"NOT_ENOUGH_QUALITY_CANDIDATES_IN_BIN_{bin_index}")
        chosen=[]
        while len(chosen)<quota:
            eligible=[r for r in pool if r not in chosen
                      and all(abs(r["time_s"]-prev["time_s"])>=1.0
                              for prev in chosen)]
            if not eligible:
                raise ValueError(f"NOT_ENOUGH_TEMPORALLY_DISTINCT_FACES_IN_BIN_{bin_index}")
            def rank(row):
                if not selected and not chosen:
                    novelty=0.0
                    pose=0.0
                else:
                    previous=selected+chosen
                    novelty=1.-max(float(np.dot(row["feature"],p["feature"]))
                                   for p in previous)
                    pose=min(1., min(abs(row["yaw_proxy"]-p["yaw_proxy"])
                                     for p in previous)/.4)
                return (.60*novelty+.15*pose+.25*float(row["quality"]),
                        float(row["quality"]), -int(row["sample_index"]))
            chosen.append(max(eligible,key=rank))
        selected.extend(chosen)
    return sorted(selected,key=lambda r:r["time_s"])


def save_private_review(dataset_dir, selected, exclusions, encoder):
    destination=dataset_dir / "curation_v1"
    destination.mkdir(mode=0o700)
    os.chmod(destination,0o700)
    rows=[]
    tile_w,tile_h=210,228
    sheet=np.full((6*tile_h,4*tile_w,3),245,dtype=np.uint8)
    for idx,row in enumerate(selected):
        x=(idx%4)*tile_w
        y=(idx//4)*tile_h
        aligned=cv2.resize(row["aligned"],(192,192),
                           interpolation=cv2.INTER_AREA)
        sheet[y:y+192,x+9:x+201]=aligned
        cv2.putText(sheet,f'#{row["sample_index"]:03d} t={row["time_s"]:.1f}s',
                    (x+9,y+210),cv2.FONT_HERSHEY_SIMPLEX,.43,(25,25,25),1,
                    cv2.LINE_AA)
        rows.append({
            "sample_index":row["sample_index"],
            "file":row["filename"],
            "jpeg_sha256":row["jpeg_sha256"],
            "time_s":round(float(row["time_s"]),3),
            "face_width_px":round(float(row["width_px"]),1),
            "face_sharpness":round(float(row["sharpness"]),1),
            "yunet_score":round(float(row["yunet_score"]),4),
            "yaw_landmark_proxy_not_degrees":round(float(row["yaw_proxy"]),4),
        })
    ok,compressed=cv2.imencode(".jpg",sheet,
                               [cv2.IMWRITE_JPEG_QUALITY,93])
    if not ok:
        raise RuntimeError("FACE_CONTACT_SHEET_ENCODING_FAILED")
    sheet_path=destination/"face_review_contact_sheet.jpg"
    with sheet_path.open("xb") as file:
        os.chmod(sheet_path,0o600)
        file.write(compressed.tobytes())
    summary={
        "scope":"PRIVATE_CURATION_CANDIDATES_ONLY_NOT_ENROLLMENT",
        "dataset_dir":str(dataset_dir),
        "candidate_count":len(selected),
        "exclusions":exclusions,
        "yu_net_model_sha256":encoder.yunet_sha256,
        "sface_model_sha256":encoder.sface_sha256,
        "contact_sheet":str(sheet_path),
        "operator_identity_review_required":True,
        "not_an_identity_accuracy_metric":True,
        "selected":rows,
    }
    report=destination/"candidates.json"
    with report.open("x",encoding="utf-8") as file:
        os.chmod(report,0o600)
        json.dump(summary,file,ensure_ascii=False,indent=2)
        file.write("\n")
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--yunet",type=Path,required=True)
    parser.add_argument("--sface",type=Path,required=True)
    args=parser.parse_args()
    os.umask(0o077)
    root=args.dataset.expanduser()
    if not root.is_dir():
        parser.error("Dataset directory unavailable")
    if (root/"curation_v1").exists():
        parser.error("curation_v1 already exists (refusing overwrite)")
    encoder=LocalFaceEncoder(args.yunet,args.sface)
    rows=read_source_rows(root)
    pool,exclusions=extract_candidates(rows,encoder)
    picked=select_candidates(pool)
    result=save_private_review(root,picked,exclusions,encoder)
    print(json.dumps({
        "scope":result["scope"],
        "source_frames":len(rows),
        "eligible_faces":len(pool),
        "selected":len(picked),
        "selected_indices":[x["sample_index"] for x in result["selected"]],
        "contact_sheet":result["contact_sheet"],
        "review_manifest":str(root/"curation_v1"/"candidates.json"),
        "exclusions":exclusions
    },indent=2,ensure_ascii=False))


if __name__=="__main__":
    main()
