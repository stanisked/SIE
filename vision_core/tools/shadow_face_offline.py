#!/usr/bin/env python3
"""Offline YuNet/SFace scan, opt-in enrollment and diagnostic comparison.

Never controls motors. Do not commit .npz face profiles or source JPEGs.
Enrollment uses explicitly selected cycle numbers of the user's own face.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile
import tempfile
import time

import cv2
import numpy as np

from vision_core.person_localization.shadow_face_verify import (
    LocalFaceEncoder, face_cosine)

DEFAULT_PROFILE = Path.home() / ".local/state/sie/private/face_profiles/target_v1.npz"
ARCHIVE_MAX_BYTES = 120_000_000
BUNDLE_MAX_BYTES = 12_000_000


def sha(data):
    return hashlib.sha256(data).hexdigest()


def selected_source_images(archive_path: Path):
    """Yield only hash-bound selected-person RGB frames, never extract tar."""
    with tarfile.open(archive_path, "r:gz") as archive:
        files = {m.name: m for m in archive.getmembers() if m.isfile()}
        if sum(m.size for m in files.values()) > ARCHIVE_MAX_BYTES:
            raise ValueError("ARCHIVE_TOO_LARGE")

        def load(name):
            info = files.get(name)
            if info is None or info.size > BUNDLE_MAX_BYTES:
                raise ValueError("MISSING_OR_OVERSIZED:" + name)
            return archive.extractfile(info).read()

        manifests = sorted(name for name in files
                           if name.startswith("shadow-source-snapshots/")
                           and name.endswith(".json"))
        if not manifests:
            raise ValueError("NO_SOURCE_SNAPSHOTS")
        for name in manifests:
            manifest = json.loads(load(name))
            cycle_id = manifest["cycle_id"]
            if ("/" in cycle_id or ".." in cycle_id or
                    name != f"shadow-source-snapshots/{cycle_id}.json"):
                raise ValueError("INVALID_SNAPSHOT_CYCLE")
            audits = [n for n in files
                      if n.startswith("control-preview-")
                      and n.endswith("/" + cycle_id + ".json")]
            if len(audits) != 1:
                raise ValueError("MISSING_OR_AMBIGUOUS_AUDIT")
            audit = json.loads(load(audits[0]))
            if audit.get("cycle_id") != cycle_id:
                raise ValueError("WRONG_CYCLE_AUDIT")
            selected = (audit.get("ar0234_dynamic_target") or {}).get("candidate") or {}
            model = (audit.get("segmentation_observation") or {}).get("model_sha256")
            shape = audit.get("shape_diagnostic") or {}
            if (selected.get("instance_index") != manifest.get("selected_instance_index")
                    or shape.get("mask_sha256") != manifest.get("instance_mask_sha256")
                    or model != manifest.get("segmentation_model_sha256")):
                raise ValueError("SELECTED_RGB_MASK_BINDING_INVALID")
            raw = load(f"shadow-source-snapshots/{cycle_id}.npz")
            if sha(raw) != manifest.get("npz_sha256"):
                raise ValueError("SOURCE_NPZ_SHA256_MISMATCH")
            with np.load(io.BytesIO(raw), allow_pickle=False) as data:
                jpeg = data["rgb_original_jpeg"].tobytes()
                if sha(jpeg) != manifest.get("rgb_original_jpeg_sha256"):
                    raise ValueError("ORIGINAL_RGB_SHA256_MISMATCH")
                frame = cv2.imdecode(np.frombuffer(jpeg,np.uint8),
                                     cv2.IMREAD_COLOR)
                if frame is None or frame.shape != (1200,1920,3):
                    raise ValueError("BAD_RGB_FRAME")
                if sha(frame.tobytes()) != manifest.get("segmentation_rgb_payload_sha256"):
                    raise ValueError("RGB_DECODED_SHA256_MISMATCH")
                mask = data["instance_mask"]
                if sha(mask.tobytes()) != manifest.get("instance_mask_sha256"):
                    raise ValueError("SELECTED_MASK_SHA256_MISMATCH")
            bbox = selected.get("bbox_xyxy_px")
            if not isinstance(bbox,list) or len(bbox)!=4:
                raise ValueError("NO_SELECTED_PERSON_BOX")
            yield {"cycle_id": cycle_id, "cycle": int(cycle_id[-8:]),
                   "track_id": manifest.get("track_id"),
                   "image": frame, "bbox": bbox,
                   "rgb_sha256": sha(jpeg)}


def write_private_profile(path: Path, features: list, cycles: list,
                          encoder: LocalFaceEncoder):
    if len(features) < 3:
        raise ValueError("NEED_THREE_CONFIRMED_FACE_IMAGES")
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    os.chmod(path.parent, 0o700)
    if path.exists():
        raise FileExistsError("FACE_PROFILE_EXISTS_NOT_OVERWRITTEN")
    descriptor = {
        "schema": "sie.shadow.face_profile.v1",
        "target_entity_id": "enrolled-target-1",
        "model": "OpenCV_SFace_2021dec",
        "yunet_sha256": encoder.yunet_sha256,
        "sface_sha256": encoder.sface_sha256,
        "enrolled_cycle_ids": cycles,
        "source": "EXPLICIT_USER_SELECTED_CYCLES",
        "person_identity_proven": False,
        "motor_admission": False,
    }
    fd, temp = tempfile.mkstemp(prefix=".face-profile-",suffix=".npz",
                                dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as file:
            np.savez_compressed(file,
                features=np.vstack(features).astype(np.float32),
                metadata=np.array(json.dumps(descriptor)))
            file.flush()
            os.fsync(file.fileno())
        os.chmod(temp, 0o600)
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def load_private_profile(path, encoder):
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError("FACE_PROFILE_NOT_FOUND")
    if path.stat().st_mode & 0o077:
        raise PermissionError("FACE_PROFILE_PERMISSIONS_TOO_OPEN")
    with np.load(path, allow_pickle=False) as profile:
        metadata = json.loads(str(profile["metadata"].item()))
        features = profile["features"].astype(np.float32)
    if (metadata["sface_sha256"] != encoder.sface_sha256
            or metadata["yunet_sha256"] != encoder.yunet_sha256
            or metadata["schema"] != "sie.shadow.face_profile.v1"
            or features.ndim != 2 or features.shape[1] != 128
            or len(features) < 3):
        raise ValueError("FACE_PROFILE_MODEL_OR_SHAPE_MISMATCH")
    return features, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("scan", "enroll", "compare"))
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--yunet", type=Path, required=True)
    parser.add_argument("--sface", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--cycles", type=int, nargs="*",
                        help="Explicit enrollment cycle numbers, e.g. 1 10 20")
    args = parser.parse_args()
    if args.operation == "enroll" and (not args.cycles or len(set(args.cycles)) < 3):
        parser.error("enroll requires at least three distinct --cycles personally reviewed by operator")
    engine = LocalFaceEncoder(args.yunet, args.sface)
    existing = None
    if args.operation == "compare":
        existing = load_private_profile(args.profile, engine)
    profile_features, profile_cycles = [], []
    results = []
    for source in selected_source_images(args.archive):
        cycle = source["cycle"]
        if args.operation == "enroll" and cycle not in args.cycles:
            continue
        started = time.monotonic()
        observation, reason = engine.observe(source["image"], source["bbox"])
        row = {"cycle": cycle, "camera_track_id": source["track_id"],
               "reason": reason, "face_confirmed": False,
               "extract_ms": round((time.monotonic()-started)*1000,1)}
        if observation is not None:
            row["face_score"] = round(observation.score,4)
            row["face_width_px"] = round(observation.face_box_xywh[2],1)
            if args.operation == "enroll":
                profile_features.append(observation.feature)
                profile_cycles.append(source["cycle_id"])
            if existing is not None:
                feature_matrix, metadata = existing
                scores = [face_cosine(x, observation.feature)
                          for x in feature_matrix]
                row["cosine_max"] = round(max(scores),4)
                row["cosine_median"] = round(float(np.median(scores)),4)
                row["overlaps_enrollment"] = source["cycle_id"] in (
                    metadata["enrolled_cycle_ids"])
                row["score_not_identity_decision"] = True
        results.append(row)
    if args.operation == "enroll":
        if len(profile_features) < 3:
            raise ValueError("ENROLLMENT_INSUFFICIENT_FACES: only " +
                             str(len(profile_features)) + " usable faces")
        write_private_profile(args.profile, profile_features, profile_cycles, engine)
    print(json.dumps({
        "operation": args.operation,
        "scope": "MOTOR_OFF_FACE_DIAGNOSTIC_NOT_IDENTITY_PROOF",
        "models": {"yunet_sha256": engine.yunet_sha256,
                   "sface_sha256": engine.sface_sha256},
        "profile_saved": args.operation == "enroll",
        "usable_faces": sum(r["reason"] == "CURRENT_SELECTED_FACE_EMBEDDED"
                           for r in results),
        "results": results,
        "warning": ("Same archive enrollment/verification is not independent evaluation; "
                    "positive and negative face samples and calibration required.")
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
