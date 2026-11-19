#!/usr/bin/env python3
"""Audit exact-cycle Snapshot/JSON evidence and print COLOR-ONLY distances.

Offline tool. Reads existing .tar.gz; never opens cameras, motor ports or
ROS nodes; does not infer physical identity or set motion thresholds.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import tarfile

import cv2
import numpy as np

from vision_core.person_localization.shadow_appearance_signature import (
    clothing_distance, clothing_signature)

CYCLE = re.compile(r"^ar0234-ov9281-control-preview-[A-Za-z0-9]+-\d{8}$")
MAX_BUNDLE_BYTES = 8_000_000
MAX_ARCHIVE_BYTES = 100_000_000


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def analyze_archive(path: Path) -> dict:
    results = []
    with tarfile.open(path, mode="r:gz") as tar:
        members = {m.name: m for m in tar.getmembers() if m.isfile()}
        if sum(m.size for m in members.values()) > MAX_ARCHIVE_BYTES:
            raise ValueError("ARCHIVE_SIZE_LIMIT")

        def read_member(name: str) -> bytes:
            member = members.get(name)
            if member is None or member.size > MAX_BUNDLE_BYTES:
                raise ValueError("MISSING_OR_OVERSIZED_EVIDENCE:" + name)
            return tar.extractfile(member).read()

        manifests = sorted(k for k in members
                           if k.startswith("shadow-source-snapshots/")
                           and k.endswith(".json"))
        if not manifests:
            raise ValueError("NO_SNAPSHOT_MANIFESTS")
        for manifest_path in manifests:
            meta = json.loads(read_member(manifest_path))
            cycle = meta["cycle_id"]
            if (not CYCLE.fullmatch(cycle)
                    or manifest_path != "shadow-source-snapshots/" + cycle + ".json"):
                raise ValueError("CYCLE_BINDING_INVALID")
            audit_dirs = [name for name in members if name.startswith("control-preview-")
                          and name.endswith("/" + cycle + ".json")]
            if len(audit_dirs) != 1:
                raise ValueError("CYCLE_AUDIT_NOT_UNIQUE")
            audit = json.loads(read_member(audit_dirs[0]))
            if audit.get("cycle_id") != cycle:
                raise ValueError("AUDIT_CYCLE_MISMATCH")
            shape = audit.get("shape_diagnostic") or {}
            if (meta.get("instance_mask_sha256") != shape.get("mask_sha256")
                    or meta.get("segmentation_model_sha256") != shape.get("model_sha256")
                    or meta.get("track_id") !=
                       ((audit.get("shadow_track_confirmation") or
                         audit.get("shadow_rgb_continuity") or {}).get("track_id"))):
                raise ValueError("SOURCE_CYCLE_BINDING_MISMATCH")
            raw = read_member("shadow-source-snapshots/" + cycle + ".npz")
            if _sha256(raw) != meta.get("npz_sha256"):
                raise ValueError("SNAPSHOT_SHA256_MISMATCH")
            with np.load(io.BytesIO(raw), allow_pickle=False) as obj:
                jpg = obj["rgb_original_jpeg"].tobytes()
                if _sha256(jpg) != meta.get("rgb_original_jpeg_sha256"):
                    raise ValueError("RGB_JPEG_SHA256_MISMATCH")
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.shape != (1200, 1920, 3):
                    raise ValueError("RGB_FRAME_SHAPE_MISMATCH")
                if _sha256(frame.tobytes()) != meta.get("segmentation_rgb_payload_sha256"):
                    raise ValueError("DECODED_RGB_BINDING_MISMATCH")
                mask = obj["instance_mask"]
                if _sha256(mask.tobytes()) != meta.get("instance_mask_sha256"):
                    raise ValueError("MASK_SHA256_MISMATCH")
                if (meta.get("selected_instance_index") !=
                        (audit.get("ar0234_dynamic_target", {}).get("candidate") or {}).get("instance_index")):
                    raise ValueError("SELECTED_INSTANCE_MISMATCH")
                person = (audit.get("ar0234_dynamic_target") or {}).get("candidate") or {}
                sig = clothing_signature(frame, mask, person["bbox_xyxy_px"])
            results.append((int(cycle[-8:]), meta.get("track_id"), sig))
    distances = []
    for i in range(len(results)):
        for j in range(i + 1, len(results)):
            ca, ida, a = results[i]
            cb, idb, b = results[j]
            distances.append({"cycles": [ca, cb], "tracks": [ida, idb],
                              "lab_ab_color_distance": round(clothing_distance(a,b), 4)})
    return {"scope": "MOTOR_OFF_COLOR_BASELINE_NOT_IDENTITY_PROOF",
            "snapshot_count": len(results),
            "snapshots": [{"cycle": n, "track_id": identity,
                           "clothing_pixels": s.clothing_pixels,
                           "coverage": round(s.coverage, 3)} for n,identity,s in results],
            "pair_distances": distances,
            "cautions": ["No identity match threshold calibrated",
                         "Other people in similar clothing may have similar histograms",
                         "Absent camera track labels cannot be reconstructed from images"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze_archive(args.archive), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
