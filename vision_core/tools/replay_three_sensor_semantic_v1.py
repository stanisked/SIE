#!/usr/bin/env python3
"""Replay bounded Step 71 Shadow source evidence. No hardware or ROS execution."""
import argparse
from collections import Counter
import io
import json
import os
from pathlib import Path
import re

import cv2
import numpy as np

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import LD19_FRAME, OPTICAL_FRAME, validate_rotation
from sie_core.geometry.camera_lidar_optical_datum_v1 import ESTIMATED_OPTICAL_FRAME, estimated_ld19_transform
from sie_ros2.live_temporal_association_v1 import BOUNDARY as TEMPORAL_BOUNDARY, SCHEMA as TEMPORAL_SCHEMA
from vision_core.stereo.local_stereo_lidar_correspondence_v1 import stereo_grid
from vision_core.stereo.stereo_lidar_static_v1 import residual_stats
from vision_core.stereo.three_sensor_semantic_spatial_v1 import (
    RGB_SHAPE, SCHEMA, associate_cycle, bind_semantic_target, temporal_eligibility,
)
from vision_core.tools import replay_stereo_lidar_local_v1 as local_replay
from vision_core.tools import stereo_lidar_joint_static_v1 as joint

ARTIFACT_SHA256 = {
    "ar0234_intrinsic_sha256": "e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096",
    "ov9281_stereo_sha256": "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52",
    "ar0234_to_physical_left_extrinsic_sha256": "10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc",
}
ACTIVATION_SHA256 = "c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9"
POLICY_SHA256 = "0a88bc3e5a9d79eaa658a5ddfd3c23fe18de21af5b46df0cab62eb5c1c894f35"
POLICY_FILE = "three_sensor_semantic_spatial_v1.json"
BOUNDARY = {**TEMPORAL_BOUNDARY, "decision_authorized": False,
            "extrinsic_independently_validated": False, "confidence": None}
CANDIDATE = "THREE_SENSOR_LOCAL_CANDIDATE_DIAGNOSTIC_ONLY"


def verify_sha(path, expected):
    actual = joint.sha256_file(path)
    if actual != expected:
        raise ValueError(f"ARTIFACT_SHA256_MISMATCH: {path.name}")
    return actual


def load_camera_inputs(ar_path, stereo_path, activation_path):
    from analyze_ar0234_stereo_component_ambiguity import load_calibrations

    verify_sha(activation_path, ACTIVATION_SHA256)
    activate = json.loads(activation_path.read_text())
    descriptor = activate["calibration_profile"]["ar0234_to_ov9281_physical_left_extrinsic"]
    extrinsic_path = Path(descriptor["path"]).expanduser()
    for key, path in zip(ARTIFACT_SHA256, (ar_path, stereo_path, extrinsic_path)):
        verify_sha(path, ARTIFACT_SHA256[key])
    if descriptor["sha256"] != ARTIFACT_SHA256["ar0234_to_physical_left_extrinsic_sha256"]:
        raise ValueError("ACTIVATION_EXTRINSIC_IDENTITY_MISMATCH")
    calibration = joint.load_calibration(stereo_path.read_bytes())
    calibration.update(load_calibrations(ar_path, stereo_path, extrinsic_path))
    validate_rotation(calibration["R"])
    policy_path = joint.CONFIG / POLICY_FILE
    verify_sha(policy_path, POLICY_SHA256)
    policy = json.loads(policy_path.read_text())
    if policy["local_stereo_policy_sha256"] != local_replay.POLICY_SHA256:
        raise ValueError("STEP69_POLICY_BINDING_MISMATCH")
    return calibration, policy


def load_inputs(ar_path, stereo_path, activation_path):
    calibration, policy = load_camera_inputs(ar_path, stereo_path, activation_path)
    profiles = []
    for key, filename in (("mechanical", "camera_lidar_mechanical_geometry_v1.json"),
                          ("optical_datum", "camera_lidar_optical_datum_v1.json")):
        path = joint.CONFIG / filename
        verify_sha(path, local_replay.PROFILE_DIGESTS[key])
        profiles.append(json.loads(path.read_text()))
    rotation, translation, axial = estimated_ld19_transform(*profiles)
    _, local_policy = local_replay.frozen_policy()
    return calibration, rotation, translation, axial, local_policy, policy


def read_snapshot(directory, cycle, expected_model_hash):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", cycle):
        raise ValueError("INVALID_CYCLE_ID")
    manifest_path = directory / f"{cycle}.json"
    manifest = json.loads(manifest_path.read_text())
    if (manifest.get("cycle_id") != cycle or manifest.get("status") != "SOURCE_SNAPSHOT_DIAGNOSTIC_ONLY"
            or manifest.get("calibration") != ARTIFACT_SHA256
            or manifest.get("segmentation_model_sha256") != expected_model_hash):
        raise ValueError("SOURCE_SNAPSHOT_CALIBRATION_OR_IDENTITY_MISMATCH")
    data = joint.checked_bytes(directory, {"path": f"{cycle}.npz", "sha256": manifest["npz_sha256"]})
    with np.load(io.BytesIO(data), allow_pickle=False) as values:
        rgb, stereo, mask = (values[key] for key in (
            "rgb_original_jpeg", "stereo_original_jpeg", "instance_mask"))
    for payload, key in ((rgb, "rgb_original_jpeg_sha256"), (stereo, "stereo_original_jpeg_sha256")):
        if payload.dtype != np.uint8 or payload.ndim != 1 or joint.digest(payload.tobytes()) != manifest.get(key):
            raise ValueError("SOURCE_JPEG_HASH_MISMATCH")
    decoded = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
    if (decoded is None or decoded.shape != (*RGB_SHAPE, 3)
            or joint.digest(decoded.tobytes()) != manifest.get("segmentation_rgb_payload_sha256")):
        raise ValueError("SEGMENTATION_RGB_INPUT_FINGERPRINT_MISMATCH")
    return manifest, stereo.tobytes(), mask, joint.sha256_file(manifest_path)


def scan_payload(row, identity):
    archive = row.get("ld19_payload_archive") or {}
    if archive.get("status") != "SOURCE_BOUND_DIAGNOSTIC_PAYLOAD":
        raise ValueError(archive.get("status", "LD19_PAYLOAD_UNAVAILABLE"))
    record = archive["record"]
    if (archive["receipt_sequence"] != identity.get("receipt_sequence")
            or archive["receipt_monotonic_ns"] != identity.get("received_monotonic_ns")
            or any(record.get("admission", {}).get(key) is not False for key in
                   ("measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))):
        raise ValueError("LD19_PAYLOAD_RECEIPT_OR_ADMISSION_MISMATCH")
    raw = record["payload"]["scan"]
    header = raw["header"]
    digest = joint.digest(joint.encoded(raw).encode())
    if (header["frame_id"] != LD19_FRAME
            or header["stamp_sec"] * 1_000_000_000 + header["stamp_nanosec"] != identity.get("timestamp_ns")
            or digest != record["evidence"]["raw_scan_sha256"]):
        raise ValueError("LD19_PAYLOAD_HEADER_OR_SHA256_MISMATCH")
    return raw, digest


def reconstruct_snapshot(jpeg, maps, calibration):
    """Use the existing offline rectification, SGBM7, LR check and metric grid."""
    from compare_shadow_saved_stereo_matcher import lr_pair

    left, right = joint.rectify_jpeg(jpeg, maps)
    disparity, lr, _ = lr_pair(left, right, "sgbm7")
    grid = stereo_grid(disparity, lr, calibration["Q"], calibration["R1"], np.ones_like(lr))
    return disparity, lr, grid


def replay(run, output, inputs, model_hash):
    calibration, rotation, translation, axial, local_policy, policy = inputs
    journals = sorted(run.glob("control-preview-*/temporal-association-shadow.jsonl"))
    if len(journals) != 1:
        raise ValueError("EXPECTED_ONE_STEP57_TEMPORAL_JOURNAL")
    journal, = journals
    snapshots = run / "shadow-source-snapshots"
    rows = joint.read_rows(journal)
    raw_cycles = [row for row in rows if "cycle_id" in row]
    cycles = {row["cycle_id"]: row for row in raw_cycles}
    if len(cycles) != len(raw_cycles):
        raise ValueError("DUPLICATE_TEMPORAL_CYCLE_ID")
    for row in cycles.values():
        if row.get("schema_version") != TEMPORAL_SCHEMA or any(row.get(key) is not False for key in (
                "measurement_authorized", "world_state_update_authorized", "motor_control_authorized")):
            raise ValueError("INVALID_STEP57_DIAGNOSTIC_JOURNAL")
    manifests = {path.stem for path in snapshots.glob("*.json")}
    hashes = {**ARTIFACT_SHA256, **local_replay.PROFILE_DIGESTS,
              "step69_policy_sha256": local_replay.POLICY_SHA256,
              "step71_policy_sha256": POLICY_SHA256, "segmentation_model_sha256": model_hash,
              "activation_provenance_sha256": ACTIVATION_SHA256}
    output.mkdir(mode=0o700)
    joint.write_json(output / "policy.json", {
        **BOUNDARY, "schema_version": SCHEMA, "sha256": hashes, "policy": policy,
        "source_temporal_journal_sha256": joint.sha256_file(journal),
        "reference_frames": {"rgb_pixels": "ar0234_image_frame", "rgb_optical": "ar0234_optical_frame",
                             "stereo_optical": OPTICAL_FRAME, "stereo_pixels": joint.STEREO_FRAME,
                             "ld19_sensor": LD19_FRAME, "estimated_ld19_optical": ESTIMATED_OPTICAL_FRAME,
                             "longitudinal": "robot_aligned_forward_from_physical_left_rim"},
        "calibration_ids": {"ar": "ar0234_intrinsic_v5_final_20260910",
                            "stereo": "ov9281_pi_stereo_candidate_v7",
                            "extrinsic": "ar0234_ov9281_physical_left_extrinsic_v1"},
        "units": {"xyz": "m", "residual": "m", "pixel": "px", "time": "ns", "angle": "rad"},
        "estimated_axial_offset_m": axial, "residual_sign": "LD19_MINUS_STEREO",
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
        "matcher": "STEP66_EXISTING_FULL_RES_SGBM7_LR_1P5PX",
    })
    maps = joint.rectification_maps(calibration)
    dispositions, temporal_counts, beam_counts = Counter(), Counter(), Counter()
    optical, longitudinal = [], []
    with (output / "cycles.jsonl").open("x") as stream:
        for cycle in dict.fromkeys([*cycles, *sorted(manifests)]):
            row = {**BOUNDARY, "schema_version": SCHEMA, "cycle_id": cycle, "sha256": hashes}
            try:
                if cycle not in manifests:
                    raise ValueError("CAMERA_MASK_SOURCE_SNAPSHOT_UNAVAILABLE")
                if cycle not in cycles:
                    raise ValueError("STEP57_TEMPORAL_ROW_UNAVAILABLE")
                manifest, jpeg, mask, sha = read_snapshot(snapshots, cycle, model_hash)
                row["camera_snapshot"] = {"manifest_sha256": sha, "npz_sha256": manifest["npz_sha256"],
                    "rgb_jpeg_sha256": manifest["rgb_original_jpeg_sha256"],
                    "stereo_jpeg_sha256": manifest["stereo_original_jpeg_sha256"]}
                temporal = temporal_eligibility(cycles[cycle], manifest["frame_freshness"])
                row["temporal"] = temporal
                temporal_counts[temporal["status"]] += 1
                if temporal["status"] != "THREE_SOURCE_TIMING_DIAGNOSTIC_ONLY":
                    raise ValueError(temporal["status"])
                bound = bind_semantic_target(manifest, mask)
                row["semantic_target"] = bound
                raw, digest = scan_payload(cycles[cycle], temporal["ld19"])
                row["ld19_raw_scan_sha256"] = digest
                row["ld19_callback_receipt_host_realtime_ns"] = cycles[cycle]["ld19_payload_archive"].get(
                    "receipt_host_realtime_ns")
                disparity, lr, grid = reconstruct_snapshot(jpeg, maps, calibration)
                report = associate_cycle(disparity, lr, grid, mask, bound, raw, calibration,
                                         rotation, translation, axial, local_policy, policy)
                row.update(report)
                beams = report["beams"]
                beam_counts.update(beam["semantic_status"] for beam in beams)
                candidates = [beam for beam in beams if beam["semantic_status"] == CANDIDATE]
                optical.extend(beam["ld19_minus_stereo_optical_z_m"] for beam in candidates)
                longitudinal.extend(beam["ld19_minus_stereo_longitudinal_m"] for beam in candidates)
                row["status"] = "LOCAL_CANDIDATES_PRESENT" if candidates else "NO_LOCAL_CANDIDATES"
            except (ValueError, KeyError, OSError, cv2.error, np.linalg.LinAlgError) as error:
                row.update(status="DIAGNOSTIC_REFUSAL", reason=str(error))
            dispositions[row.get("reason", row["status"])] += 1
            joint.append_row(stream, row)
    joint.write_json(output / "summary.json", {
        **BOUNDARY, "schema_version": SCHEMA, "source_cycles": len(cycles),
        "camera_snapshot_count": len(manifests), "cycle_dispositions": dict(dispositions),
        "temporal_dispositions_for_bound_snapshots": dict(temporal_counts),
        "source_journal_counters": [row["journal"] for row in rows if row.get("record_type") == "SESSION_SUMMARY"],
        "beam_dispositions": dict(beam_counts), "coherent_semantic_candidates": beam_counts[CANDIDATE],
        "optical_z_residual": residual_stats(optical), "longitudinal_residual": residual_stats(longitudinal),
        "residual_pooling": "PER_BEAM_PER_SOURCE_CYCLE; SCAN_REUSE_POSSIBLE",
    })
    return output / "summary.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    os.umask(0o077)
    cv2.setNumThreads(1)
    inputs = load_inputs(args.ar_intrinsic, args.stereo_calibration, args.activation)
    print(replay(args.run, args.output, inputs, joint.sha256_file(args.model)))


if __name__ == "__main__":
    main()
