#!/usr/bin/env python3
"""Motor-off SIEE same-cycle face <-> segmented body diagnostic.

Uses SHADOW source snapshot NPZ and same-cycle audit. An observed face
overlapping an RGB segmentation mask is geometric evidence only. Similarity
to enrolled SFace profile is a separate positive-only diagnostic, NOT
identity proof, not a calibrated face match and not motor authorization.

Does NOT mutate profile, run a camera, or issue ROS/UART commands.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import tarfile

import numpy as np

from vision_core.person_localization.shadow_face_verify import (
    LocalFaceEncoder, face_cosine)
from vision_core.tools.shadow_face_offline import (
    selected_source_images, load_private_profile)


def mask_face_geometry(mask: np.ndarray, xywh) -> dict:
    """Report face rectangle overlap with current same-cycle selected mask.

    No threshold here confers identity or motor permission. Face may lie
    outside the silhouette due to segmentation error or camera geometry.
    """
    if (not isinstance(mask,np.ndarray) or mask.dtype != np.bool_
            or mask.ndim != 2 or not isinstance(xywh,(list,tuple))
            or len(xywh) != 4):
        raise ValueError("INVALID_FACE_MASK_GEOMETRY")
    try:
        x,y,w,h=[float(v) for v in xywh]
    except (ValueError,TypeError):
        raise ValueError("INVALID_FACE_MASK_GEOMETRY") from None
    if not all(math.isfinite(v) for v in (x,y,w,h)) or w<=0 or h<=0:
        raise ValueError("INVALID_FACE_MASK_GEOMETRY")
    height,width=mask.shape
    x1=max(0,int(math.floor(x)))
    y1=max(0,int(math.floor(y)))
    x2=min(width,int(math.ceil(x+w)))
    y2=min(height,int(math.ceil(y+h)))
    if x2<=x1 or y2<=y1:
        return {"face_box_mask_overlap_fraction":0.0,
                "face_center_inside_mask":False,
                "face_box_inside_image":False}
    region=mask[y1:y2,x1:x2]
    center_x=int(x+w/2)
    center_y=int(y+h/2)
    center_valid=0<=center_x<width and 0<=center_y<height
    return {
        "face_box_mask_overlap_fraction":round(
            float(np.count_nonzero(region)/region.size),5),
        "face_center_inside_mask":bool(mask[center_y,center_x]) if center_valid else False,
        "face_box_inside_image":(x>=0 and y>=0 and x+w<=width and y+h<=height),
    }


def checked_mask_and_binding(tar, files, cycle_id):
    snapshot_path=f"shadow-source-snapshots/{cycle_id}"
    name=snapshot_path+".json"
    info=files.get(name)
    if info is None or info.size>2_000_000:
        raise ValueError("SOURCE_MANIFEST_NOT_FOUND")
    manifest=json.load(tar.extractfile(info))
    if manifest.get("cycle_id")!=cycle_id:
        raise ValueError("SOURCE_CYCLE_MISMATCH")
    matching=[key for key in files if key.startswith("control-preview-")
              and key.endswith("/"+cycle_id+".json")]
    if len(matching)!=1:
        raise ValueError("CURRENT_CYCLE_AUDIT_UNAVAILABLE")
    aud=json.load(tar.extractfile(files[matching[0]]))
    selected=(aud.get("ar0234_dynamic_target") or {})
    candidate=selected.get("candidate") or {}
    if (aud.get("cycle_id")!=cycle_id or
            selected.get("disposition")!="OBSERVED" or
            selected.get("current_eligible_count")!=1 or
            candidate.get("instance_index")!=manifest.get("selected_instance_index") or
            (aud.get("shape_diagnostic") or {}).get("mask_sha256") !=
                manifest.get("instance_mask_sha256") or
            (aud.get("segmentation_observation") or {}).get("model_sha256") !=
                manifest.get("segmentation_model_sha256")):
        raise ValueError("SOURCE_MASK_INSTANCE_BINDING_INVALID")
    datafile=files.get(snapshot_path+".npz")
    if datafile is None or datafile.size>32_000_000:
        raise ValueError("SOURCE_NPZ_MISSING_OR_LARGE")
    payload=tar.extractfile(datafile).read()
    if hashlib.sha256(payload).hexdigest()!=manifest.get("npz_sha256"):
        raise ValueError("SNAPSHOT_PAYLOAD_SHA256_MISMATCH")
    with np.load(io.BytesIO(payload),allow_pickle=False) as obj:
        mask=obj["instance_mask"]
        if mask.dtype!=np.bool_ or mask.shape!=(1200,1920):
            raise ValueError("SOURCE_MASK_SHAPE_INVALID")
        if hashlib.sha256(mask.tobytes()).hexdigest()!=manifest.get(
                "instance_mask_sha256"):
            raise ValueError("SOURCE_MASK_SHA256_MISMATCH")
    return mask,candidate,manifest


def analyze(archive: Path, profile: Path, encoder: LocalFaceEncoder) -> dict:
    features,metadata=load_private_profile(profile,encoder)
    if (len(features)!=23 or metadata.get("target_entity_id")!="person_001"
            or metadata.get("face_profile_id")!="person_001_face_enrollment_v1"):
        raise ValueError("UNEXPECTED_FACE_PROFILE")
    rows=[]
    with tarfile.open(archive,"r:gz") as tar:
        files={x.name:x for x in tar.getmembers() if x.isfile()}
        if sum(x.size for x in files.values())>150_000_000:
            raise ValueError("SHADOW_EVIDENCE_ARCHIVE_TOO_LARGE")
        for source in selected_source_images(archive):
            cycle_id=source["cycle_id"]
            mask,candidate,manifest=checked_mask_and_binding(
                tar,files,cycle_id)
            if (manifest.get("track_id")!=source.get("track_id")
                    or source["bbox"]!=candidate.get("bbox_xyxy_px")):
                raise ValueError("RGB_TRACK_OR_BBOX_BINDING_MISMATCH")
            row={
                "cycle_id":cycle_id,
                "cycle":source["cycle"],
                "rgb_track_id":source["track_id"],
                "selected_instance_index":candidate["instance_index"],
                "rgb_original_jpeg_sha256":source["rgb_sha256"],
                "mask_sha256":manifest["instance_mask_sha256"],
                "same_cycle_rgb_mask_verified":True,
                "face_body_identity_verified":False,
                "motor_authorized":False,
            }
            observation,reason=encoder.observe(source["image"],source["bbox"])
            row["face_reason"]=reason
            if observation is not None:
                geometry=mask_face_geometry(mask,observation.face_box_xywh)
                row.update(geometry)
                sim=sorted(
                    [face_cosine(observation.feature,f) for f in features],
                    reverse=True)
                row["cosine_max"]=round(float(sim[0]),5)
                row["cosine_top3_mean"]=round(float(np.mean(sim[:3])),5)
                row["face_score"]=round(float(observation.score),4)
                row["geometric_face_body_relation"]=(
                    "OVERLAP_OBSERVED_DIAGNOSTIC_ONLY"
                    if geometry["face_box_mask_overlap_fraction"]>0
                    else "NO_MASK_OVERLAP")
            else:
                row["geometric_face_body_relation"]="NO_VALID_SELECTED_FACE"
            rows.append(row)
    return {
        "scope":"MOTOR_OFF_FACE_BODY_GEOMETRIC_AUDIT_ONLY",
        "target_entity_id":"person_001",
        "face_profile_id":"person_001_face_enrollment_v1",
        "source_snapshots":len(rows),
        "face_present":sum("cosine_max" in r for r in rows),
        "geometric_overlap_observed":sum(
            r["geometric_face_body_relation"]==
            "OVERLAP_OBSERVED_DIAGNOSTIC_ONLY" for r in rows),
        "track_ids":sorted(set(r["rgb_track_id"]
                                for r in rows if r["rgb_track_id"])),
        "rows":rows,
        "warnings":[
            "Face-box/mask overlap is geometric evidence, not biometric identity.",
            "SFace scores are not calibrated against negative people.",
            "No body appearance embeddings, no persistent identity rebinding.",
            "No motor authorization or actuator access.",
        ],
    }


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--archive",type=Path,required=True)
    p.add_argument("--profile",type=Path,required=True)
    p.add_argument("--yunet",type=Path,required=True)
    p.add_argument("--sface",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    os.umask(0o077)
    if a.output.exists():
        p.error("Output exists; refusing overwrite")
    if not a.output.parent.is_dir():
        p.error("Output parent directory missing")
    result=analyze(a.archive,a.profile,
                   LocalFaceEncoder(a.yunet,a.sface))
    with a.output.open("x",encoding="utf-8") as file:
        os.chmod(a.output,0o600)
        json.dump(result,file,ensure_ascii=False,indent=2)
        file.write("\n")
    print(json.dumps({
        "scope":result["scope"],
        "snapshots":result["source_snapshots"],
        "face_present":result["face_present"],
        "geometric_overlap_observed":result["geometric_overlap_observed"],
        "rows":[{
            "cycle":r["cycle"],
            "track_id":r["rgb_track_id"],
            "cosine_max":r.get("cosine_max"),
            "mask_overlap":r.get("face_box_mask_overlap_fraction"),
            "center_in_mask":r.get("face_center_inside_mask"),
            "geometry":r["geometric_face_body_relation"],
        } for r in result["rows"]],
        "private_report":str(a.output)
    },ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
