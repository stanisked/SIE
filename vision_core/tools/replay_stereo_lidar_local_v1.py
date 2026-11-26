#!/usr/bin/env python3
"""Offline Step 69 replay of hash-checked Step 66 stereo/LD19 archives."""
import argparse
from collections import Counter
import io
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import LD19_FRAME, OPTICAL_FRAME
from sie_core.geometry.camera_lidar_optical_datum_v1 import ESTIMATED_OPTICAL_FRAME, estimated_ld19_transform
from vision_core.stereo.local_stereo_lidar_correspondence_v1 import (
    CANDIDATE, LocalTrialPolicy, compare_local_scan, stereo_grid,
)
from vision_core.stereo.stereo_lidar_static_v1 import (
    MAX_STEREO_PLANE_P95_M, compare_scan, fit_stereo_surface, residual_stats, target_mask,
)
from vision_core.tools import stereo_lidar_joint_static_v1 as joint

SCHEMA = "sie.stereo_lidar_local_correspondence.v1.diagnostic"
BOUNDARY = {**joint.BOUNDARY, "decision_authorized": False, "extrinsic_independently_validated": False}
POLICY_FILENAME = "local_stereo_lidar_correspondence_v1.json"
POLICY_SHA256 = "f2c8947121035c5ea8b87a7581099654b41e639163fb889fd026439965f77624"
PROFILE_DIGESTS = {
    "mechanical": "c439598397cde034c8475444d3ed5e1d1c341839d83e71e0fcaf9a74f2bd632d",
    "optical_datum": "4a5d851e7b274d1a5d788217a85488ea23bacee160aad01ebf945af1df14f56c",
}


def frozen_policy():
    data = (joint.CONFIG / POLICY_FILENAME).read_bytes()
    if joint.digest(data) != POLICY_SHA256:
        raise ValueError("frozen Step 69 local policy SHA-256 mismatch")
    fields = json.loads(data)
    policy = LocalTrialPolicy(**{key: fields[key] for key in LocalTrialPolicy.__dataclass_fields__})
    return fields, policy


def _verify_scan(capture, scan):
    joint.checked_bytes(capture, scan["payload_cdr"])
    if "record" in scan:
        raw = scan["record"]["payload"]["scan"]
        if joint.digest(joint.encoded(raw).encode()) != scan["record"]["evidence"]["raw_scan_sha256"]:
            raise ValueError("decoded scan evidence hash mismatch")
        header = raw["header"]
        if (header["stamp_sec"] * 1_000_000_000 + header["stamp_nanosec"] != scan["header_ns"]
                or header["frame_id"] != LD19_FRAME):
            raise ValueError("decoded scan header differs from captured timing or frame identity")


def _archive_arrays(global_analysis, archived, frame, scan, timing):
    artifact = archived.get("stereo_arrays")
    if artifact is None:
        return None
    if (archived["stereo_source"] != frame["payload_mjpg"]
            or archived["scan_source"] != scan["payload_cdr"]
            or archived["scan_header_ns"] != scan["header_ns"]
            or archived["timing"] != timing):
        raise ValueError(f"archived stereo array provenance mismatch for frame {frame['frame_index']}")
    with np.load(io.BytesIO(joint.checked_bytes(global_analysis, artifact)), allow_pickle=False) as arrays:
        disparity, lr = arrays["disparity_px"], arrays["lr_consistent"]
    if (disparity.shape != joint.SIZE[::-1] or disparity.dtype != np.float32
            or lr.shape != disparity.shape or lr.dtype != np.bool_):
        raise ValueError("archived disparity/LR arrays have unexpected shape or dtype")
    return disparity, lr, {"kind": "VERIFIED_STEP66_ARRAYS", **artifact}


def _global_comparison(disparity, lr, cal, region, raw_scan, rotation, translation, sector):
    # This is the original Step 66 algorithm and 40 mm gate, independent of the local path.
    try:
        surface = fit_stereo_surface(disparity, lr, cal["Q"], cal["R1"], region)
    except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
        return {"status": "STEREO_ANALYSIS_REFUSED", "reason": str(error)}
    comparison = compare_scan(raw_scan, surface, cal["R1"], cal["P1"], rotation, translation, sector)
    comparison.pop("beams")
    return {"status": "COMPARISON_DIAGNOSTIC_ONLY", **comparison}


def replay(capture, global_analysis, output):
    policy_record, policy = frozen_policy()
    capture_hashes = {
        "capture_session_sha256": joint.sha256_file(capture / "session.json"),
        "frame_journal_sha256": joint.sha256_file(capture / "frames.jsonl"),
        "scan_journal_sha256": joint.sha256_file(capture / "scans.jsonl"),
    }
    session = json.loads((capture / "session.json").read_text())
    original_policy = json.loads((global_analysis / "analysis-policy.json").read_text())
    if session.get("schema_version") != joint.SCHEMA or any(session.get(flag) is not False for flag in (
        "measurement_authorized", "world_state_update_authorized", "motor_control_authorized"
    )):
        raise ValueError("invalid diagnostic capture manifest")
    expected = {
        **capture_hashes, "schema_version": joint.SCHEMA, "calibration_sha256": joint.ACTIVE_V7_SHA256,
        "stereo_matcher": "sgbm7", "lr_max_delta_px": 1.5,
        "max_stereo_plane_p95_m": MAX_STEREO_PLANE_P95_M,
        "max_header_delta_ns": joint.PAIRING.max_rgb_ld19_header_delta_ns,
        "max_source_age_ns": joint.FRESHNESS.max_stereo_age_ns,
    }
    for key, value in expected.items():
        if original_policy.get(key) != value:
            raise ValueError(f"Step 66 analysis provenance mismatch: {key}")
    inputs = session["inputs"]
    cal = joint.load_calibration(joint.checked_bytes(capture, inputs["calibration"]))
    for name, sha in PROFILE_DIGESTS.items():
        if inputs[name]["sha256"] != sha:
            raise ValueError(f"Step 60/66 profile SHA-256 mismatch: {name}")
    rotation, translation, axial = estimated_ld19_transform(
        *(json.loads(joint.checked_bytes(capture, inputs[name])) for name in PROFILE_DIGESTS)
    )
    region = target_mask(joint.SIZE[::-1], original_policy["target_polygon_rectified_left_px"])
    sector = original_policy["ld19_sector_deg"]
    lo, hi = sector
    if not (math.isfinite(lo) and math.isfinite(hi) and -180 <= lo < hi <= 180):
        raise ValueError("invalid archived diagnostic sector")
    scans = joint.read_rows(capture / "scans.jsonl")
    for scan in scans:
        _verify_scan(capture, scan)
    prior = {}
    for row in joint.read_rows(global_analysis / "comparisons.jsonl"):
        index = row.get("frame_index")
        if index is not None:
            if index in prior:
                raise ValueError(f"duplicate Step 66 frame index: {index}")
            prior[index] = row
    hashes = {"calibration_sha256": joint.ACTIVE_V7_SHA256, "geometry_profile_sha256": PROFILE_DIGESTS,
              "local_policy_sha256": POLICY_SHA256}
    output.mkdir(mode=0o700)
    joint.write_json(output / "replay-policy.json", {
        **BOUNDARY, "schema_version": SCHEMA, **hashes, **capture_hashes,
        "global_analysis_policy_sha256": joint.sha256_file(global_analysis / "analysis-policy.json"),
        "global_comparisons_sha256": joint.sha256_file(global_analysis / "comparisons.jsonl"),
        "local_policy": policy_record, "estimated_axial_offset_m": axial,
        "global_plane_p95_limit_m": MAX_STEREO_PLANE_P95_M,
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
        "target_polygon_rectified_left_px": original_policy["target_polygon_rectified_left_px"],
        "ld19_sector_deg": sector, "confidence": None,
        "residual_sign": "LD19_MINUS_STEREO",
        "reference_frames": {
            "stereo_optical_z": OPTICAL_FRAME, "transformed_ld19": ESTIMATED_OPTICAL_FRAME,
            "image": joint.STEREO_FRAME, "longitudinal": "robot_aligned_forward_from_physical_left_rim",
        },
    })
    capture_counts, temporal_counts, spatial_counts = Counter(), Counter(), Counter()
    beam_counts, beam_totals, global_counts, prior_counts, array_sources = (Counter() for _ in range(5))
    beam_totals.update(dict.fromkeys(("total_beams", "projectable_beams", "inside_image_beams",
        "inside_target_polygon_beams", "beams_with_usable_local_stereo", "beams_with_sufficient_local_stereo_support",
        "coherent_local_surface_candidates", "ambiguous_beams"), 0))
    compared_scans, attempted_scans = set(), set()
    candidates_with_global_refusal = global_disposition_mismatches = 0
    optical, longitudinal = [], []
    maps = None
    with (output / "comparisons.jsonl").open("x") as out:
        for frame in joint.read_rows(capture / "frames.jsonl"):
            index = frame.get("frame_index")
            row = {"frame_index": index, "capture_status": frame["status"], **hashes}
            capture_counts[frame["status"]] += 1
            if frame["status"] != "PAYLOAD_RECORDED":
                row["temporal"] = {"status": "NOT_EVALUATED_CAPTURE_REFUSED"}
                joint.append_row(out, row)
                continue
            jpeg = joint.checked_bytes(capture, frame["payload_mjpg"])
            timing, scan = joint.select_scan(frame, scans, session["boot_id"])
            row.update(temporal=timing, camera=frame["camera"], stereo_source=frame["payload_mjpg"])
            temporal_counts[timing["status"]] += 1
            if scan is None:
                joint.append_row(out, row)
                continue
            attempted_scans.add(scan["receipt_sequence"])
            row["ld19_source"] = {
                "frame_id": LD19_FRAME, "receipt_sequence": scan["receipt_sequence"],
                "header_ns": scan["header_ns"], "receipt_monotonic_ns": scan["receipt_monotonic_ns"],
                "payload_cdr": scan["payload_cdr"], "raw_scan_sha256": scan["record"]["evidence"]["raw_scan_sha256"],
            }
            archived = prior.get(index, {})
            row["archived_global"] = {"status": archived.get("status", "UNAVAILABLE"), "reason": archived.get("reason")}
            prior_counts[row["archived_global"]["reason"] or row["archived_global"]["status"]] += 1
            # Provenance or byte mismatches in a present archive are hard failures,
            # never silently replaced by newly reconstructed arrays.
            stereo = _archive_arrays(global_analysis, archived, frame, scan, timing)
            raw_scan = scan["record"]["payload"]["scan"]
            grid = None
            try:
                if stereo is None:
                    from compare_shadow_saved_stereo_matcher import lr_pair
                    if maps is None:
                        maps = joint.rectification_maps(cal)
                    left, right = joint.rectify_jpeg(jpeg, maps)
                    disparity, lr, _ = lr_pair(left, right, "sgbm7")
                    source = {"kind": "RECONSTRUCTED_FROM_VERIFIED_MJPEG"}
                else:
                    disparity, lr, source = stereo
                row["stereo_arrays"] = source
                array_sources[source["kind"]] += 1
                grid = stereo_grid(disparity, lr, cal["Q"], cal["R1"], region)
            except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                row["stereo_frame_refusal_reason"] = str(error)
            row["global_plane"] = (
                _global_comparison(disparity, lr, cal, region, raw_scan, rotation, translation, sector)
                if grid is not None else {"status": "STEREO_FRAME_UNAVAILABLE"}
            )
            g = row["global_plane"]
            global_counts[g.get("reason", g["status"])] += 1
            beams, counts = compare_local_scan(
                raw_scan, grid, region, cal["R1"], cal["P1"], rotation, translation, axial, sector, policy,
            )
            row.update(beams=beams, local_counts=counts)
            beam_counts.update(counts["beam_status_counts"])
            beam_totals.update({key: value for key, value in counts.items() if key != "beam_status_counts"})
            if counts["coherent_local_surface_candidates"]:
                compared_scans.add(scan["receipt_sequence"])
                spatial_counts["LOCAL_CANDIDATES_PRESENT"] += 1
                candidates_with_global_refusal += g["status"] == "STEREO_ANALYSIS_REFUSED"
            else:
                spatial_counts["STEREO_FRAME_UNAVAILABLE" if grid is None else "NO_LOCAL_CANDIDATES"] += 1
            for beam in beams:
                if beam["status"] == CANDIDATE:
                    optical.append(beam["ld19_minus_stereo_optical_z_m"])
                    longitudinal.append(beam["ld19_minus_stereo_longitudinal_m"])
            if archived:
                row["global_disposition_reproduced"] = (
                    g["status"] == archived["status"] and g.get("reason") == archived.get("reason")
                )
                global_disposition_mismatches += not row["global_disposition_reproduced"]
            joint.append_row(out, row)
    matched = temporal_counts["DIAGNOSTIC_HEADER_MATCH"]
    joint.write_json(output / "summary.json", {
        **BOUNDARY, "schema_version": SCHEMA, **hashes,
        "result": "LOCAL_CANDIDATES_PRESENT" if optical else "NO_LOCAL_CANDIDATES",
        "capture_frame_status_counts": dict(capture_counts),
        "temporal_matches": matched, "temporal_refusals": sum(temporal_counts.values()) - matched,
        "temporal_status_counts": dict(temporal_counts), "spatial_frame_status_counts": dict(spatial_counts),
        "local_beams": dict(beam_totals), "beam_status_counts": dict(beam_counts),
        "stereo_array_source_counts": dict(array_sources),
        "distinct_temporally_paired_ld19_scans": len(attempted_scans),
        "distinct_compared_ld19_scans": len(compared_scans),
        "ld19_minus_stereo_optical_z": residual_stats(optical),
        "ld19_minus_stereo_longitudinal": residual_stats(longitudinal),
        "global_plane_p95_limit_m": MAX_STEREO_PLANE_P95_M,
        "global_plane_status_counts": dict(global_counts),
        "archived_step66_global_status_counts": dict(prior_counts),
        "frames_with_local_candidates_and_global_plane_refusal": candidates_with_global_refusal,
        "global_disposition_mismatches_vs_archive": global_disposition_mismatches,
        "residual_pooling": "PER_BEAM_PER_STEREO_FRAME; SCAN_REUSE_POSSIBLE",
    })
    return output / "summary.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--global-analysis", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    os.umask(0o077)
    cv2.setNumThreads(1)
    print(replay(args.capture, args.global_analysis, args.output))


if __name__ == "__main__":
    main()
