#!/usr/bin/env python3
"""Create private SIEE person_001 SFace enrollment from operator-reviewed frames.

Never writes photos or biometrics to GitHub; offline, no camera or motors.
One explicitly approved set from one source dataset; NOT an accuracy test,
identity proof, body binding, motor admission, or calibrated face threshold.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from vision_core.person_localization.shadow_face_verify import LocalFaceEncoder
from vision_core.tools.curate_shadow_face_dataset import (
    extract_candidates, read_source_rows)

PROFILE_PATH = (Path.home() /
    ".local/state/sie/private/face_profiles/person_001_face_enrollment_v1.npz")
FACE_FEATURE_DIM = 128


def sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def load_reviewed_approval(dataset: Path, encoder: LocalFaceEncoder):
    curation_dir = dataset / "curation_v1"
    initial_path = curation_dir / "candidates.json"
    approval_path = curation_dir / "candidates_approved.json"
    initial_bytes = initial_path.read_bytes()
    approved_bytes = approval_path.read_bytes()
    original = json.loads(initial_bytes)
    approved = json.loads(approved_bytes)

    if (original.get("scope") != "PRIVATE_CURATION_CANDIDATES_ONLY_NOT_ENROLLMENT"
            or approved.get("scope") != "OPERATOR_REVIEWED_ENROLLMENT_CANDIDATES"
            or original.get("dataset_dir") != str(dataset)
            or approved.get("dataset_dir") != str(dataset)
            or approved.get("yu_net_model_sha256") != encoder.yunet_sha256
            or approved.get("sface_model_sha256") != encoder.sface_sha256
            or original.get("yu_net_model_sha256") != encoder.yunet_sha256
            or original.get("sface_model_sha256") != encoder.sface_sha256):
        raise ValueError("CURATION_SCOPE_OR_MODEL_MISMATCH")
    excluded = approved.get("excluded_by_operator")
    if excluded != {"132": "PARTIAL_FACE"}:
        raise ValueError("OPERATOR_REVIEW_NOT_AS_EXPECTED")
    full = original.get("selected")
    chosen = approved.get("selected")
    if (not isinstance(full,list) or len(full) != 24
            or not isinstance(chosen,list) or len(chosen) != 23
            or approved.get("candidate_count") != 23
            or original.get("candidate_count") != 24):
        raise ValueError("REVIEW_FRAME_COUNT_MISMATCH")

    # Require byte-for-byte metadata equality for all approved entries, not
    # only the sample index. No new or edited indices can be smuggled in.
    originals_by_id = {row["sample_index"]: row for row in full}
    if (len(originals_by_id) != 24
            or 132 not in originals_by_id
            or len({row["sample_index"] for row in chosen}) != 23):
        raise ValueError("SOURCE_CURATION_NOT_UNIQUE")
    for row in chosen:
        index = row["sample_index"]
        if index == 132 or originals_by_id.get(index) != row:
            raise ValueError("UNAPPROVED_OR_MODIFIED_CANDIDATE")
    if {row["sample_index"] for row in chosen} != set(originals_by_id)-{132}:
        raise ValueError("APPROVED_INDICES_NOT_EXACT_EXCLUSION")
    return chosen, sha256(approved_bytes)


def collect_enrollment(dataset: Path, approved, encoder: LocalFaceEncoder):
    approved_by_index = {row["sample_index"]:row for row in approved}
    def approved_source_rows():
        # read_source_rows verifies SHA256 and shape on all 169 inputs.
        for source,image in read_source_rows(dataset):
            index = source.get("sample_index")
            if index not in approved_by_index:
                continue
            expected = approved_by_index[index]
            if (source.get("file") != expected.get("file")
                    or source.get("jpeg_sha256") != expected.get("jpeg_sha256")
                    or len(source.get("faces",[])) != 1):
                raise ValueError("APPROVED_FRAME_SOURCE_BINDING_INVALID")
            yield source,image

    extracted,exclusions,source_count = extract_candidates(
        approved_source_rows(),encoder)
    if (source_count != 23 or len(extracted) != 23
            or any(exclusions.values())):
        raise ValueError("FACE_REDETECTION_OR_FEATURE_EXTRACTION_FAILED")
    result = {row["sample_index"]: row for row in extracted}
    if len(result) != 23 or set(result) != set(approved_by_index):
        raise ValueError("FACE_ENROLLMENT_SET_CHANGED")
    features=np.vstack([result[item["sample_index"]]["feature"]
                        for item in approved]).astype(np.float32)
    if (features.shape != (23,FACE_FEATURE_DIM)
            or not np.all(np.isfinite(features))
            or not np.allclose(np.linalg.norm(features,axis=1),1.,atol=1e-4)):
        raise ValueError("FACE_FEATURES_INVALID")
    return features


def save_private_enrollment(path: Path, *, features: np.ndarray,
                            approved, encoder: LocalFaceEncoder,
                            approval_sha256: str, dataset: Path):
    path=path.expanduser()
    path.parent.mkdir(parents=True,mode=0o700,exist_ok=True)
    os.chmod(path.parent,0o700)
    if path.exists():
        raise FileExistsError("ENROLLMENT_PROFILE_ALREADY_EXISTS")
    metadata={
        "schema": "sie.shadow.face_profile.v1",
        "target_entity_id": "person_001",
        "face_profile_id": "person_001_face_enrollment_v1",
        "source": "OPERATOR_APPROVED_23_OF_24_EXCLUDING_132",
        "source_dataset": str(dataset),
        "source_approval_sha256": approval_sha256,
        "sample_indices": [row["sample_index"] for row in approved],
        "source_jpeg_sha256": [row["jpeg_sha256"] for row in approved],
        "enrolled_cycle_ids": [
            f"face-dataset:{dataset.name}:{row['sample_index']}"
            for row in approved],
        "model": "OpenCV_SFace_2021dec",
        "yunet_sha256": encoder.yunet_sha256,
        "sface_sha256": encoder.sface_sha256,
        "person_identity_proven": False,
        "face_threshold_calibrated": False,
        "body_binding_verified": False,
        "motor_admission": False,
    }
    fd,tmp=tempfile.mkstemp(prefix=".person_001_face-",suffix=".npz",
                            dir=str(path.parent))
    try:
        with os.fdopen(fd,"wb") as handle:
            np.savez_compressed(handle,features=features,
                                metadata=np.array(json.dumps(metadata)))
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp,0o600)
        # Do not overwrite in a race. Hard link is atomic and refuses
        # existing destination even if another process created it.
        os.link(tmp,path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    with np.load(path,allow_pickle=False) as saved:
        assert saved["features"].shape == (23,128)
        assert json.loads(str(saved["metadata"].item()))["sample_indices"] == (
            metadata["sample_indices"])
    return metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--yunet",type=Path,required=True)
    parser.add_argument("--sface",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=PROFILE_PATH)
    args=parser.parse_args()
    os.umask(0o077)
    dataset=args.dataset.expanduser().resolve()
    encoder=LocalFaceEncoder(args.yunet,args.sface)
    approved,approval_hash=load_reviewed_approval(dataset,encoder)
    features=collect_enrollment(dataset,approved,encoder)
    meta=save_private_enrollment(args.output,features=features,
        approved=approved,encoder=encoder,
        approval_sha256=approval_hash,dataset=dataset)
    print(json.dumps({
        "scope":"PRIVATE_FACE_ENROLLMENT_NOT_IDENTITY_PROOF",
        "target_entity_id":meta["target_entity_id"],
        "face_profile_id":meta["face_profile_id"],
        "enrolled_faces":len(features),
        "excluded_sample_indices":[132],
        "output_path":str(args.output.expanduser()),
        "face_threshold_calibrated":False,
        "motor_admission":False,
    },ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
