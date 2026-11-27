#!/usr/bin/env python3
"""Replay archived camera and LD19 evidence into an offline entity preview."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
import os
from pathlib import Path
import re
from time import perf_counter_ns
from zipfile import BadZipFile

import cv2
import numpy as np

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import OPTICAL_FRAME
from sie_ros2.live_temporal_association_v1 import FRESHNESS, PAIRING, RECENT, SCHEMA as TEMPORAL_SCHEMA
from sie_ros2.temporal_association_freshness_v1 import classify_source_age
from vision_core.person_localization.diagnostic_instance_selection import has_unique_eligible_instance_binding
from vision_core.stereo.entity_centric_diagnostic_v1 import (
    BOUNDARY, POLICY_ID, SCHEMA, evidence, integrate_cycle, lidar_observation,
)
from vision_core.stereo.three_sensor_semantic_spatial_v1 import (
    MATCH, bind_semantic_target, camera_packet_matches, semantic_snapshot_metadata, semantic_stereo_component,
)
from vision_core.tools import replay_three_sensor_semantic_v1 as step71
from vision_core.tools import stereo_lidar_joint_static_v1 as joint


INPUT_ERRORS = (ValueError, KeyError, TypeError, AttributeError, OSError, EOFError, BadZipFile, cv2.error)
FRESHNESS_REFUSALS = {"SOURCE_STALE", "STAMP_FROM_FUTURE", "FUTURE_WITHIN_TRIAL_SKEW",
                      "STAMP_AHEAD_OF_RECEIPT", "NOT_RECEIVED_BY_ENDPOINT"}


def freshness_status(reason):
    return "STALE" if reason == "SOURCE_STALE" else "REJECTED"


def camera_age(packet, cutoff, max_age):
    stamp, received = packet.get("timestamp_ns"), packet.get("received_monotonic_ns")
    if any(type(value) is not int or value <= 0 for value in (stamp, received, cutoff)):
        return "CAMERA_TIMING_UNAVAILABLE"
    if received > cutoff:
        return "NOT_RECEIVED_BY_ENDPOINT"
    if stamp > received:
        return "STAMP_AHEAD_OF_RECEIPT"
    return classify_source_age(cutoff - stamp, max_age, FRESHNESS.max_future_skew_ns)


def semantic_evidence(row, source, artifact_hash, *, mask=None):
    rgb = row.get("rgb") or {}
    cutoff = (row.get("endpoint_monotonic_ns") or {}).get("zero_wait")
    frames = (source.get("frame_freshness") or {}).get("frames") or {}
    meta = source.get("step71_semantic") or {}
    seg, target = meta.get("segmentation") or {}, meta.get("target") or {}
    cycle = source["cycle_id"]
    context = {
        "source_packet": rgb, "source_artifact_sha256": artifact_hash,
        "manifest_sha256": artifact_hash if mask is not None else None,
        "npz_sha256": source.get("npz_sha256"), "reference_frame": "ar0234_image_frame", "unit": "px",
        "producer_status": seg.get("status"), "target_selection": target,
        "track_confirmation": meta.get("track_confirmation"),
        "rgb_track_continuity": meta.get("rgb_track_continuity"),
        "observed_instances": seg.get("instances"), "mask_hashes": seg.get("instance_mask_sha256"),
    }
    if not rgb:
        return evidence("UNAVAILABLE", "RGB_PACKET_UNAVAILABLE", **context)
    if (not camera_packet_matches(rgb, frames.get("ar0234"), "ar0234")
            or cutoff != rgb.get("received_monotonic_ns")
            or row.get("freshness_policy") != asdict(FRESHNESS)):
        return evidence("REJECTED", "RGB_PACKET_OR_FRESHNESS_POLICY_MISMATCH", **context)
    age = camera_age(rgb, cutoff, FRESHNESS.max_rgb_age_ns)
    if age != RECENT:
        return evidence(freshness_status(age), age, **context)

    instances = seg.get("instances")
    if seg.get("cycle_id") != cycle or seg.get("status") != "CURRENT_INSTANCES_DIAGNOSTIC_ONLY":
        return evidence("UNAVAILABLE", "CURRENT_RGB_OBSERVATION_UNAVAILABLE", **context)
    if not has_unique_eligible_instance_binding(instances, target):
        eligible = target.get("current_eligible_count")
        status = "AMBIGUOUS" if type(eligible) is int and eligible > 1 else "UNAVAILABLE"
        return evidence(status, target.get("reason") or "UNIQUE_RGB_TARGET_UNAVAILABLE", **context)

    selected = target["candidate"]
    index = selected["instance_index"]
    masks = seg.get("instance_mask_sha256")
    mask_hash = masks[index] if isinstance(masks, list) and len(masks) == len(instances) else None
    if mask is not None and (index != source.get("selected_instance_index")
                            or mask_hash != joint.digest(mask.tobytes())
                            or source.get("instance_mask_sha256") != mask_hash):
        return evidence("REJECTED", "RGB_MASK_BINDING_MISMATCH", **context)
    track = None
    basis = None
    observation_id = f"segmentation-preview:{cycle}:{index}"
    confirmation = meta.get("track_confirmation") or {}
    continuity = meta.get("rgb_track_continuity") or {}
    if (confirmation.get("state") == "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION"
            and confirmation.get("confirmed_observation_id") == observation_id):
        track, basis = confirmation.get("track_id"), "CURRENT_CONFIRMATION"
    elif continuity.get("status") == "MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY":
        track, basis = continuity.get("track_id"), "CURRENT_RGB_CONTINUITY"
    observation = {
        "observation_id": observation_id, "semantic_class": selected.get("class_name") or "person",
        "track_id": track, "track_basis": basis, "observation_state": "CURRENT",
        "instance": selected, "confidence": selected.get("confidence"),
        "source_timestamp_ns": rgb["timestamp_ns"], "source_timestamp_basis": rgb["timestamp_basis"],
        "source_ref": rgb["evidence_ref"], "reference_frame": "ar0234_image_frame", "unit": "px",
        "mask_evidence": {"sha256": mask_hash, "source_artifact_sha256": artifact_hash,
                          "npz_sha256": source.get("npz_sha256"),
                          "payload_verified": mask is not None, "coordinates": meta.get("mask_coordinates")},
        "model_sha256": seg.get("model_sha256"), "snapshot_quality": source.get("mask_depth"),
    }
    return evidence("AVAILABLE", "CURRENT_RGB_DIAGNOSTIC_ONLY", observation=observation, **context)


def audit_semantics(row, path):
    if not path.is_file():
        return evidence("UNAVAILABLE", "CAMERA_SOURCE_EVIDENCE_UNAVAILABLE")
    data = path.read_bytes()
    audit = json.loads(data)
    if (audit.get("schema_version") != "sie.ar0234_ov9281.control_input_preview.v1"
            or audit.get("cycle_id") != row["cycle_id"]):
        raise ValueError("CAMERA_AUDIT_IDENTITY_MISMATCH")
    source = {"cycle_id": audit["cycle_id"], "frame_freshness": audit.get("frame_freshness"),
              "step71_semantic": semantic_snapshot_metadata(audit, {}),
              "mask_depth": (audit.get("shape_diagnostic") or {}).get("mask_depth")}
    return semantic_evidence(row, source, joint.digest(data))


def archived_semantics(row, path):
    try:
        return audit_semantics(row, path)
    except INPUT_ERRORS as error:
        return evidence("REJECTED", f"CAMERA_AUDIT: {error}")


def independent_camera_evidence(row, snapshot, calibration, maps, policy):
    """Validate RGB on its own, then bind stereo using the recorded camera link."""
    manifest, jpeg, mask, manifest_hash = snapshot
    semantic = semantic_evidence(row, manifest, manifest_hash, mask=mask)
    if semantic["status"] != "AVAILABLE":
        return semantic, evidence("UNAVAILABLE", "RGB_TARGET_UNAVAILABLE")
    rgb, cycle = row["rgb"], manifest["cycle_id"]
    frames = (manifest.get("frame_freshness") or {}).get("frames") or {}
    stereo = (row.get("actual_camera_pair") or {}).get("stereo") or {}
    link = (row.get("zero_wait") or {}).get("rgb_stereo") or {}
    cutoff = row["endpoint_monotonic_ns"]["zero_wait"]
    timing = {"cutoff_monotonic_ns": cutoff, "camera_link": link, "actual_stereo_packet": stereo}
    if not stereo:
        return semantic, evidence("UNAVAILABLE", "STEREO_PACKET_UNAVAILABLE", timing=timing)
    age = camera_age(stereo, cutoff, FRESHNESS.max_stereo_age_ns)
    if age != RECENT:
        return semantic, evidence(freshness_status(age), age, timing=timing)
    delta = stereo["timestamp_ns"] - rgb["timestamp_ns"]
    if (not camera_packet_matches(stereo, frames.get("ov9281"), "ov9281")
            or row.get("association_policy") != asdict(PAIRING)
            or link.get("status") != MATCH or link.get("candidate") != stereo
            or link.get("candidate_minus_rgb_ns") != delta
            or abs(delta) > PAIRING.max_rgb_stereo_delta_ns):
        return semantic, evidence("REJECTED", "ACTUAL_CAMERA_PAIR_NOT_BOUND_AT_ZERO_WAIT", timing=timing)
    try:
        binding = bind_semantic_target(manifest, mask)
    except ValueError as error:
        return semantic, evidence("REJECTED", str(error), timing=timing)

    try:
        _, _, grid = step71.reconstruct_snapshot(jpeg, maps, calibration)
        region, quality = semantic_stereo_component(grid, mask, binding, calibration, policy)
    except INPUT_ERRORS as error:
        return semantic, evidence("REJECTED", f"STEREO_RECONSTRUCTION: {error}", timing=timing)
    if not region.any():
        status = "AMBIGUOUS" if quality["status"].startswith("AMBIGUOUS") else "REJECTED"
        return semantic, evidence(status, quality["status"], quality=quality, timing=timing)

    points = grid[0][region]
    location = np.median(points, axis=0).tolist()
    component = {
        "observation_id": f"stereo-component:{cycle}:{quality['component_id']}",
        "semantic_observation_id": binding["observation_id"], "track_id": binding["track_id"],
        "source_ref": stereo["evidence_ref"], "source_timestamp_ns": stereo["timestamp_ns"],
        "source_timestamp_basis": stereo["timestamp_basis"],
        "component_mask_sha256": quality["component_mask_sha256"],
        "reference_frame": OPTICAL_FRAME, "unit": "m", "componentwise_median_xyz_m": location,
        "component_grid_reference_frame": joint.STEREO_FRAME, "component_grid_unit": "px",
        "rgb_roi_reference_frame": "ar0234_image_frame", "rgb_roi_unit": "px",
        "depth_m": location[2], "calibration": manifest["calibration"], "quality": quality,
    }
    return semantic, evidence("AVAILABLE", quality["status"], component=component, timing=timing)


def independent_ld19_evidence(row, link):
    archive = row.get("ld19_payload_archive") or {}
    candidate = link.get("candidate") or {}
    source = {"source_packet": candidate, "archive_status": archive.get("status"),
              "clock_evidence": row.get("ld19_clock_evidence"),
              "receipt_host_realtime_ns": archive.get("receipt_host_realtime_ns"),
              "freshness": link.get("candidate_freshness")}
    try:
        if archive.get("status") != "SOURCE_BOUND_DIAGNOSTIC_PAYLOAD":
            age = (source["freshness"] or {}).get("status")
            reason = archive.get("reason") or archive.get("status") or "LD19_PAYLOAD_UNAVAILABLE"
            if age in FRESHNESS_REFUSALS:
                return evidence(freshness_status(age), age, timing=source)
            status = "REJECTED" if archive.get("status") == "LD19_PAYLOAD_REFUSED" else "UNAVAILABLE"
            return evidence(status, reason, timing=source)
        step71.scan_payload(row, candidate)
        scan = lidar_observation(archive["record"], timing=source)
    except INPUT_ERRORS as error:
        return evidence("REJECTED", str(error), timing=source)
    age = (source["freshness"] or {}).get("status")
    if age in FRESHNESS_REFUSALS:
        return evidence(freshness_status(age), age, observation=scan)
    status = "AVAILABLE" if scan["valid_return_count"] else "REJECTED"
    reason = "LD19_SOURCE_LOCAL_GEOMETRY" if scan["valid_return_count"] else "NO_VALID_LD19_RETURNS"
    return evidence(status, reason, observation=scan)


def surface_context(surface, semantic, spatial, ld19):
    if surface is None:
        return evidence("UNAVAILABLE", "NO_STEP71_SURFACE_REPLAY_ROW")
    if surface.get("status") == "DIAGNOSTIC_REFUSAL":
        return evidence("REJECTED", surface.get("reason") or "STEP71_SURFACE_REFUSED")
    if surface.get("status") not in ("NO_LOCAL_CANDIDATES", "LOCAL_CANDIDATES_PRESENT"):
        return evidence("REJECTED", "INVALID_STEP71_SURFACE_DISPOSITION")
    component = spatial.get("component") or {}
    if ((surface.get("camera_snapshot") or {}).get("manifest_sha256") != semantic.get("manifest_sha256")
            or (surface.get("semantic_target") or {}).get("observation_id") != component.get("semantic_observation_id")
            or (surface.get("semantic_stereo") or {}).get("component_mask_sha256") != component.get("component_mask_sha256")
            or surface.get("ld19_raw_scan_sha256") != (ld19.get("observation") or {}).get("raw_scan_sha256")
            or not all(source["status"] == "AVAILABLE" for source in (semantic, spatial, ld19))):
        return evidence("REJECTED", "STEP71_SURFACE_EVIDENCE_BINDING_MISMATCH")
    dispositions = Counter(beam["semantic_status"] for beam in surface["beams"])
    count = dispositions[step71.CANDIDATE]
    if any(reason.startswith("AMBIGUOUS") for reason in dispositions):
        status = "AMBIGUOUS"
    elif count:
        status = "CANDIDATES"
    else:
        status = "NO_CANDIDATES"
    return evidence(status, "STEP71_SURFACE_LEVEL_ONLY", candidate_count=count, beam_dispositions=dict(dispositions))


def indexed_cycles(rows, schema):
    result = {}
    for row in rows:
        if "cycle_id" not in row:
            continue
        cycle = row["cycle_id"]
        if (not isinstance(cycle, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", cycle)
                or cycle in result or row.get("schema_version") != schema
                or any(row.get(key) is not False for key in
                       ("measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))):
            raise ValueError("INVALID_OR_DUPLICATE_DIAGNOSTIC_CYCLE")
        result[cycle] = row
    return result


def load_surface_replay(path, journal_hash, policy):
    descriptor = json.loads((path / "policy.json").read_text())
    if (descriptor.get("schema_version") != step71.SCHEMA
            or descriptor.get("source_temporal_journal_sha256") != journal_hash
            or (descriptor.get("sha256") or {}).get("step71_policy_sha256") != step71.POLICY_SHA256
            or descriptor.get("policy") != policy):
        raise ValueError("STEP71_SURFACE_REPLAY_PROVENANCE_MISMATCH")
    return indexed_cycles(joint.read_rows(path / "cycles.jsonl"), step71.SCHEMA)


def camera_cycles(cycles, journal, snapshots, manifests, inputs, model_hash, surfaces, surface_refusal):
    calibration, policy = inputs
    maps = joint.rectification_maps(calibration) if manifests else None
    for cycle in dict.fromkeys([*cycles, *sorted(manifests)]):
        row = cycles.get(cycle) or {}
        rgb_ld19 = (row.get("zero_wait") or {}).get("rgb_ld19_ros_header") or {}
        ld19 = independent_ld19_evidence(row, rgb_ld19)
        audit_path = journal.parent / f"{cycle}.json"
        if not row:
            semantic = spatial = evidence("UNAVAILABLE", "TEMPORAL_JOURNAL_ROW_UNAVAILABLE")
        elif cycle not in manifests:
            semantic = archived_semantics(row, audit_path)
            spatial = evidence("UNAVAILABLE", "STEREO_SOURCE_SNAPSHOT_UNAVAILABLE")
        else:
            try:
                snapshot = step71.read_snapshot(snapshots, cycle, model_hash)
                semantic, spatial = independent_camera_evidence(row, snapshot, calibration, maps, policy)
            except INPUT_ERRORS as error:
                spatial = evidence("REJECTED", f"CAMERA_SOURCE_SNAPSHOT: {error}")
                semantic = archived_semantics(row, audit_path)
                if semantic["status"] == "UNAVAILABLE":
                    semantic = spatial
        if surface_refusal:
            surface = evidence("REJECTED", surface_refusal)
        else:
            try:
                surface = surface_context(surfaces.get(cycle), semantic, spatial, ld19)
            except INPUT_ERRORS as error:
                surface = evidence("REJECTED", f"STEP71_SURFACE_REPLAY: {error}")
        result = integrate_cycle(cycle, semantic, spatial, ld19, rgb_ld19=rgb_ld19, surface_level=surface)
        result["source_packets"] = {"ar0234": row.get("rgb"),
                                    "ov9281": (row.get("actual_camera_pair") or {}).get("stereo"),
                                    "ld19": rgb_ld19.get("candidate")}
        result["camera_pair_timing"] = (row.get("zero_wait") or {}).get("rgb_stereo")
        result["temporal_endpoints_monotonic_ns"] = row.get("endpoint_monotonic_ns")
        clock = (row.get("clock") or {}).get("zero_wait") or {}
        result["zero_wait_clock"] = {key: clock.get(key) for key in ("clock_valid", "status")}
        yield result


def standalone_ld19_cycles(journal):
    if journal is None:
        return
    missing_camera = evidence("UNAVAILABLE", "INDEPENDENT_LD19_JOURNAL")
    with journal.open() as stream:
        for index, line in enumerate(stream):
            source_id = None
            try:
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise ValueError("LD19_DIAGNOSTIC_RECORD_INVALID")
                source_id = record.get("observation_id")
                scan = lidar_observation(record, timing={"status": "UNASSESSED_INDEPENDENT_CLOCK"})
                ld19 = evidence("AVAILABLE", "LD19_SOURCE_LOCAL_GEOMETRY", observation=scan) if (
                    scan["valid_return_count"]) else evidence("REJECTED", "NO_VALID_LD19_RETURNS", observation=scan)
            except INPUT_ERRORS as error:
                ld19 = evidence("REJECTED", str(error))
            result = integrate_cycle(f"independent-ld19:{index}", missing_camera, missing_camera, ld19,
                                     rgb_ld19={}, surface_level=missing_camera)
            result["source_packets"] = {"ar0234": None, "ov9281": None,
                                        "ld19": {"observation_id": source_id} if source_id else None}
            yield result


def write_cycles(output, camera_rows, ld19_rows):
    sources = {name: Counter() for name in ("semantic", "spatial", "ld19")}
    reasons = {name: Counter() for name in sources}
    packets, associations, surfaces, totals = Counter(), Counter(), Counter(), Counter()
    unique_scans = set()
    with (output / "cycles.jsonl").open("x") as stream:
        for kind, rows in (("camera_archive", camera_rows), ("standalone_ld19_records", ld19_rows)):
            for row in rows:
                joint.append_row(stream, row)
                totals[kind] += 1
                totals["entities"] += row["provisional_entity"] is not None
                packets.update(name for name, packet in row["source_packets"].items() if packet)
                for name in sources:
                    sources[name][row[name]["status"]] += 1
                    reasons[name][row[name]["reason"]] += 1
                if row["ld19"]["status"] == "AVAILABLE":
                    scan = row["ld19"]["observation"]
                    unique_scans.add(scan["observation_id"])
                    totals["valid_returns"] += scan["valid_return_count"]
                associations[row["ld19_entity_association"]["status"]] += 1
                surfaces[row["surface_level"]["status"]] += 1
    return {
        "standalone_ld19_records": totals["standalone_ld19_records"],
        "diagnostic_rows": totals["camera_archive"] + totals["standalone_ld19_records"],
        "source_availability_counts": {name: packets[name] for name in ("ar0234", "ov9281", "ld19")},
        "source_dispositions": sources, "source_reasons": reasons,
        "semantic_candidate_count": sources["semantic"]["AVAILABLE"],
        "spatial_evidence_count": sources["spatial"]["AVAILABLE"],
        "independent_ld19_obstacle_observation_count": sources["ld19"]["AVAILABLE"],
        "independent_ld19_unique_scan_count": len(unique_scans),
        "valid_ld19_return_instances": totals["valid_returns"],
        "provisional_entity_candidate_count": totals["entities"],
        "ld19_entity_associations": 0,
        "ld19_entity_association_dispositions": associations, "surface_level_dispositions": surfaces,
    }


def replay(run, output, inputs, model_hash, *, ld19_journal=None, surface_replay=None):
    started = perf_counter_ns()
    journal_hash = None
    surface_hashes = {}
    cycles, manifests, camera_rows = {}, set(), ()
    if run is not None:
        journals = sorted(run.glob("control-preview-*/temporal-association-shadow.jsonl"))
        if len(journals) != 1:
            raise ValueError("EXPECTED_ONE_STEP57_TEMPORAL_JOURNAL")
        journal, = journals
        journal_hash = joint.sha256_file(journal)
        cycles = indexed_cycles(joint.read_rows(journal), TEMPORAL_SCHEMA)
        surfaces, surface_refusal = {}, None
        if surface_replay:
            try:
                surface_hashes = {
                    "surface_replay_cycles_sha256": joint.sha256_file(surface_replay / "cycles.jsonl"),
                    "surface_replay_policy_sha256": joint.sha256_file(surface_replay / "policy.json"),
                }
                surfaces = load_surface_replay(surface_replay, journal_hash, inputs[1])
            except INPUT_ERRORS as error:
                surface_refusal = f"STEP71_SURFACE_REPLAY: {error}"
        snapshots = run / "shadow-source-snapshots"
        manifests = {path.stem for path in snapshots.glob("*.json")}
        camera_rows = camera_cycles(cycles, journal, snapshots, manifests, inputs, model_hash,
                                    surfaces, surface_refusal)
    elif ld19_journal is None or surface_replay is not None:
        raise ValueError("EXPECTED_CAMERA_ARCHIVE_OR_INDEPENDENT_LD19_JOURNAL")

    provenance = {
        "source_temporal_journal_sha256": journal_hash,
        "independent_ld19_journal_sha256": joint.sha256_file(ld19_journal) if ld19_journal else None,
        **surface_hashes,
        "camera_calibration_sha256": step71.ARTIFACT_SHA256 if run is not None else None,
        "segmentation_model_sha256": model_hash,
        "step71_policy_sha256": step71.POLICY_SHA256 if run is not None else None,
        "activation_sha256": step71.ACTIVATION_SHA256 if run is not None else None,
        "camera_pairing": asdict(PAIRING), "source_freshness": asdict(FRESHNESS),
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
    }
    output.mkdir(mode=0o700)
    summary = write_cycles(output, camera_rows, standalone_ld19_cycles(ld19_journal))
    joint.write_json(output / "summary.json", {
        **BOUNDARY, "schema_version": SCHEMA, "policy_id": POLICY_ID, "provenance": provenance,
        "source_cycles": len(cycles) + summary["standalone_ld19_records"],
        "source_temporal_cycles": len(cycles), "camera_snapshot_count": len(manifests), **summary,
    })
    joint.write_json(output / "processing_cost.json", {
        "scope": "offline replay including input reads, reconstruction and output writes",
        "wall_time_ns": perf_counter_ns() - started,
    })
    return output / "summary.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    for name in ("run", "ar-intrinsic", "stereo-calibration", "activation", "model"):
        parser.add_argument(f"--{name}", type=Path)
    parser.add_argument("--ld19-journal", type=Path, help="optional independent LD19 diagnostic JSONL")
    parser.add_argument("--surface-replay", type=Path, help="optional existing Step 71/72 replay directory")
    args = parser.parse_args()
    os.umask(0o077)
    cv2.setNumThreads(1)
    inputs, model_hash = None, None
    if args.run is not None:
        if not all((args.ar_intrinsic, args.stereo_calibration, args.activation, args.model)):
            parser.error("--run requires --ar-intrinsic, --stereo-calibration, --activation and --model")
        inputs = step71.load_camera_inputs(args.ar_intrinsic, args.stereo_calibration, args.activation)
        model_hash = joint.sha256_file(args.model)
    elif args.ld19_journal is None or args.surface_replay is not None:
        parser.error("provide --run or --ld19-journal; --surface-replay requires --run")
    print(replay(args.run, args.output, inputs, model_hash,
                 ld19_journal=args.ld19_journal, surface_replay=args.surface_replay))


if __name__ == "__main__":
    main()
