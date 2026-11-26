#!/usr/bin/env python3
"""Standalone Step 66 capture and offline analysis. Only subscribes to LD19."""
from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import time

import cv2
import numpy as np

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import OPTICAL_FRAME, validate_rotation
from sie_core.geometry.camera_lidar_optical_datum_v1 import (
    ESTIMATED_OPTICAL_FRAME, estimated_ld19_transform,
)
from sie_ros2.host_timebase_bridge_v1 import convert_monotonic_stamp, make_host_timebase_bridge
from sie_ros2.host_timebase_clock_probe_v1 import collect_host_clock_brackets
from sie_ros2.latest_v4l2_capture import LatestV4L2Capture
from sie_ros2.ld19_diagnostic_evidence import SOURCE_TOPIC, make_ld19_diagnostic_record
from sie_ros2.live_temporal_association_v1 import (
    BOUNDARY, BRIDGE_POLICY, FRESHNESS, PAIRING, RECENT, bridge_at, nanoseconds,
)
from sie_ros2.temporal_association_freshness_v1 import classify_source_age
from vision_core.stereo.stereo_lidar_static_v1 import (
    MAX_STEREO_PLANE_P95_M, MIN_STEREO_COVERAGE, MIN_STEREO_SUPPORT,
    compare_scan, fit_stereo_surface, residual_stats, target_mask,
)
from vision_core.tools.validate_ov9281_pi_depth_v7_pose_aware import sha256_file

SCHEMA = "sie.stereo_lidar_joint_static.v1.diagnostic"
ACTIVE_V7_SHA256 = "39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52"
ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "vision_core/config/diagnostics"
SIZE = (1280, 800)
STEREO_FRAME = "rectified_left_optical_frame"
CLOCK_REFERENCE = re.compile(r"sha256:[0-9a-f]{64}")
MAX_SCAN_PAYLOADS = 2000


def digest(data):
    return hashlib.sha256(data).hexdigest()


def encoded(row):
    return json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)


def write_json(path, row):
    with path.open("x") as stream:
        stream.write(encoded(row) + "\n")


def append_row(stream, row):
    stream.write(encoded(row) + "\n")
    stream.flush()


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def save_bytes(directory, name, data):
    with (directory / name).open("xb") as stream:
        stream.write(data)
    return {"path": name, "sha256": digest(data)}


def checked_bytes(directory, artifact):
    name = artifact["path"]
    if Path(name).name != name:
        raise ValueError("evidence paths must be file names within the capture directory")
    data = (directory / name).read_bytes()
    if digest(data) != artifact["sha256"]:
        raise ValueError(f"evidence digest mismatch for {name}")
    return data


def load_calibration(data):
    if digest(data) != ACTIVE_V7_SHA256:
        raise ValueError("active OV9281 v7 calibration SHA-256 mismatch")
    with np.load(io.BytesIO(data), allow_pickle=False) as npz:
        if (tuple(npz["size"]) != SIZE
                or str(npz["camera_1_semantics"].item()) != "physical_left"
                or str(npz["camera_2_semantics"].item()) != "physical_right"
                or str(npz["captured_stream_identity"].item()) != "captured_second_is_physical_left"):
            raise ValueError("calibration size or physical stream identity mismatch")
        cal = {name: npz[name] for name in ("K1", "K2", "D1", "D2", "R1", "R2", "P1", "P2", "Q")}
    for key, shape in {"K1": (3, 3), "K2": (3, 3), "R1": (3, 3), "R2": (3, 3),
                       "P1": (3, 4), "P2": (3, 4), "Q": (4, 4)}.items():
        if cal[key].shape != shape or not np.isfinite(cal[key]).all():
            raise ValueError(f"invalid {key} in v7 calibration")
    validate_rotation(cal["R1"])
    validate_rotation(cal["R2"])
    return cal


def rectification_maps(cal):
    return [cv2.initUndistortRectifyMap(cal[f"K{i}"], cal[f"D{i}"], cal[f"R{i}"],
                                      cal[f"P{i}"], SIZE, cv2.CV_32FC1)
            for i in (1, 2)]


def rectify_jpeg(jpeg, maps):
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape != (SIZE[1], SIZE[0] * 2):
        raise ValueError("invalid packed stereo MJPEG")
    right, left = np.split(image, 2, axis=1)
    return tuple(cv2.remap(raw, *mapping, cv2.INTER_LINEAR)
                 for raw, mapping in zip((left, right), maps))


def select_scan(frame, scans, boot_id):
    """Adapt the existing diagnostic 50 ms/250 ms policy to a stereo anchor."""
    when = nanoseconds(frame["camera"]["host_packet_received_monotonic_s"])
    stamp = nanoseconds(frame["camera"]["driver_timestamp_s"])
    clock = frame["clock"]
    if not clock["clock_valid"]:
        return {"status": clock["status"]}, None
    result = convert_monotonic_stamp(clock["bridge"], source_monotonic_ns=stamp,
                                     evaluation_monotonic_ns=when, boot_id=boot_id, policy=BRIDGE_POLICY)
    if result["estimated_realtime_ns"] is None:
        return {"status": result["status"]}, None
    anchor = result["estimated_realtime_ns"]
    stereo_age = classify_source_age(when - stamp, FRESHNESS.max_stereo_age_ns, FRESHNESS.max_future_skew_ns)
    if stereo_age != RECENT:
        return {"status": "STEREO_" + stereo_age}, None

    now_epoch = when + clock["bridge"]["offset_ns"]
    accepted = []
    refusals = Counter()
    for scan in scans:
        received = scan["receipt_monotonic_ns"]
        if received > when:
            continue
        reason = scan["status"] if scan["status"] != "PAYLOAD_RECORDED" else scan["clock_status"]
        if reason == "OPERATOR_ATTESTED_SAME_HOST_REALTIME":
            header = scan["header_ns"]
            age = now_epoch - header
            reason = classify_source_age(age, FRESHNESS.max_ld19_ros_header_age_ns,
                                         FRESHNESS.max_future_skew_ns)
            if reason == RECENT:
                if header > received + clock["bridge"]["offset_ns"]:
                    reason = "HEADER_AHEAD_OF_CALLBACK_RECEIPT"
                elif abs(header - anchor) > PAIRING.max_rgb_ld19_header_delta_ns:
                    reason = "HEADER_DELTA_EXCEEDED"
                else:
                    accepted.append((abs(header - anchor), header, scan["receipt_sequence"], scan))
                    continue
        refusals[reason] += 1
    if not accepted:
        return {"status": "NO_ELIGIBLE_CAUSAL_SCAN", "candidate_refusals": dict(refusals)}, None
    _, header, sequence, scan = min(accepted)
    return {"status": "DIAGNOSTIC_HEADER_MATCH", "scan_receipt_sequence": sequence,
            "stereo_driver_realtime_estimate_ns": anchor,
            "header_minus_stereo_stamp_ns": header - anchor,
            "header_age_at_stereo_receipt_ns": now_epoch - header,
            "stereo_stamp_age_at_receipt_ns": when - stamp,
            "candidate_refusals": dict(refusals)}, scan


def capture(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan

    if not (1 <= args.frames <= 100 and .2 <= args.interval_s <= 1.0):
        raise ValueError("capture requires 1–100 frames and an interval of 0.2–1.0 seconds")
    if not args.native_capture.is_file() or not os.access(args.native_capture, os.X_OK):
        raise ValueError("compile the repository native capture helper first")
    if args.ld19_clock_evidence and not CLOCK_REFERENCE.fullmatch(args.ld19_clock_evidence):
        raise ValueError("LD19 clock evidence must be sha256:<64 lowercase hex digits>")
    cal_data = args.calibration.read_bytes()
    cal = load_calibration(cal_data)
    maps = rectification_maps(cal)
    mechanical_data = (CONFIG / "camera_lidar_mechanical_geometry_v1.json").read_bytes()
    datum_data = (CONFIG / "camera_lidar_optical_datum_v1.json").read_bytes()
    _, _, axial = estimated_ld19_transform(json.loads(mechanical_data), json.loads(datum_data))
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if not boot_id:
        raise ValueError("local boot ID is unavailable")
    args.output.mkdir(mode=0o700)
    paths = {
        "calibration": save_bytes(args.output, "ov9281-v7.npz", cal_data),
        "mechanical": save_bytes(args.output, "mechanical.json", mechanical_data),
        "optical_datum": save_bytes(args.output, "optical-datum.json", datum_data),
    }
    write_json(args.output / "session.json", {
        **BOUNDARY, "schema_version": SCHEMA, "inputs": paths, "boot_id": boot_id,
        "ld19_topic": SOURCE_TOPIC, "ld19_clock_evidence": args.ld19_clock_evidence,
        "estimated_axial_offset_m": axial, "native_capture_sha256": sha256_file(args.native_capture),
        "device": args.device, "requested_frames": args.frames, "interval_s": args.interval_s,
        "camera_payload": "V4L2_MJPEG_PACKED_PHYSICAL_RIGHT_THEN_PHYSICAL_LEFT",
        "stereo_timestamp_basis": "V4L2_DRIVER_DECLARED_MONOTONIC_START_OF_EXPOSURE",
        "ld19_timestamp_basis": "ROS_SCAN_CONVERSION_PUBLICATION_NOT_LASER_ACQUISITION",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
    })
    rclpy.init(args=[])
    node = camera = None
    count, frame_count, unique_headers, omitted_scans = 0, 0, 0, 0
    last_header, first_publisher = 0, None
    windows, previous = deque(maxlen=16), None
    try:
        node = Node("siee_stereo_lidar_static_v1", use_global_arguments=False)
        with (args.output / "scans.jsonl").open("x") as scan_out, (args.output / "frames.jsonl").open("x") as frame_out:
            def on_scan(scan):
                nonlocal count, unique_headers, omitted_scans, last_header, first_publisher
                receipt = time.monotonic_ns()
                receipt_wall = time.time_ns()
                if count >= MAX_SCAN_PAYLOADS:
                    omitted_scans += 1
                    return
                count += 1
                artifact = save_bytes(args.output, f"scan-{count:05d}.cdr", serialize_message(scan))
                row = {"receipt_sequence": count, "receipt_monotonic_ns": receipt,
                       "receipt_host_realtime_ns": receipt_wall, "payload_cdr": artifact,
                       "frame_id": scan.header.frame_id,
                       "header_sec": scan.header.stamp.sec, "header_nanosec": scan.header.stamp.nanosec,
                       "clock_status": "ROS_CLOCK_DOMAIN_UNVERIFIED"}
                try:
                    row["record"] = make_ld19_diagnostic_record(
                        scan, received_at_utc=datetime.fromtimestamp(receipt_wall / 1e9, timezone.utc).isoformat())
                except (ValueError, OverflowError, OSError) as error:
                    append_row(scan_out, row | {"status": "INVALID_LD19_PAYLOAD", "reason": str(error)})
                    return

                header = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
                status = "PAYLOAD_RECORDED" if header > last_header else "DUPLICATE_OR_REVERSED_HEADER"
                unique_headers += status == "PAYLOAD_RECORDED"
                last_header = max(last_header, header)
                publishers = node.get_publishers_info_by_topic(SOURCE_TOPIC)
                gid = list(publishers[0].endpoint_gid) if len(publishers) == 1 else None
                if gid is not None and first_publisher is None:
                    first_publisher = gid
                if gid is None:
                    row["clock_status"] = "LD19_PUBLISHER_NOT_UNIQUE"
                elif gid != first_publisher:
                    row["clock_status"] = "LD19_PUBLISHER_CHANGED"
                elif args.ld19_clock_evidence:
                    row["clock_status"] = "OPERATOR_ATTESTED_SAME_HOST_REALTIME"
                append_row(scan_out, row | {"header_ns": header, "status": status,
                                            "sole_discovered_publisher_gid": gid})
            node.create_subscription(LaserScan, SOURCE_TOPIC, on_scan, qos_profile_sensor_data)
            camera = LatestV4L2Capture(args.native_capture, args.device, SIZE[0] * 2, SIZE[1], timeout=.5)
            start = time.monotonic_ns()
            next_clock, due, last_sequence = start, start + 2_000_000_000, None
            deadline = due + round((args.frames * args.interval_s + 5) * 1e9)
            while rclpy.ok() and frame_count < args.frames and time.monotonic_ns() < deadline:
                now = time.monotonic_ns()
                if now >= next_clock:
                    bridge = make_host_timebase_bridge(
                        collect_host_clock_brackets(count=3, interval_ms=0), boot_id=boot_id,
                        policy=BRIDGE_POLICY, previous_bridge=previous,
                    )
                    if bridge["status"] == "DIAGNOSTIC_BRIDGE_READY":
                        previous = bridge
                    windows.append((time.monotonic_ns(), bridge))
                    next_clock = now + 100_000_000
                rclpy.spin_once(node, timeout_sec=.02)
                if time.monotonic_ns() < due:
                    continue
                due = time.monotonic_ns() + round(args.interval_s * 1e9)
                try:
                    jpeg, metadata = camera.snapshot(last_sequence)
                except TimeoutError:
                    append_row(frame_out, {"status": "CAMERA_SNAPSHOT_TIMEOUT", "attempt_monotonic_ns": now})
                    continue
                last_sequence = metadata["sequence"]
                frame_count += 1
                artifact = save_bytes(args.output, f"frame-{frame_count:04d}.mjpg", jpeg)
                mono = nanoseconds(metadata["host_packet_received_monotonic_s"])
                append_row(frame_out, {"status": "PAYLOAD_RECORDED", "frame_index": frame_count,
                                       "camera": metadata, "payload_mjpg": artifact,
                                       "reference_frame": OPTICAL_FRAME,
                                       "clock": bridge_at(windows, mono, boot_id)})
                if not (args.output / "target-reference-left.png").exists():
                    try:
                        left, _ = rectify_jpeg(jpeg, maps)
                    except ValueError:
                        continue  # Original malformed packet remains archived and is reported by analysis.
                    if not cv2.imwrite(str(args.output / "target-reference-left.png"), left):
                        raise OSError("could not write target reference image")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if camera is not None:
                camera.release()
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    complete = frame_count == args.frames and unique_headers > 0 and not omitted_scans
    write_json(args.output / "capture-summary.json", {
        **BOUNDARY, "frames": frame_count, "archived_scan_callbacks": count,
        "strictly_increasing_ld19_headers": unique_headers,
        "scan_callbacks_omitted_at_capacity": omitted_scans,
        "status": "CAPTURE_COMPLETE" if complete else "CAPTURE_INCOMPLETE",
    })
    print(args.output)
    return 0 if complete else 1


def analyze(args):
    # Import the existing full-resolution reference and LR check from the
    # saved-pair diagnostic. The tool directory is on PYTHONPATH on Pi.
    from compare_shadow_saved_stereo_matcher import lr_pair

    session = json.loads((args.capture / "session.json").read_text())
    if session.get("schema_version") != SCHEMA or any(session.get(flag) is not False for flag in (
        "measurement_authorized", "world_state_update_authorized", "motor_control_authorized"
    )):
        raise ValueError("invalid diagnostic capture manifest")
    artifacts = {k: checked_bytes(args.capture, v) for k, v in session["inputs"].items()}
    cal = load_calibration(artifacts["calibration"])
    rotation, translation, axial = estimated_ld19_transform(
        json.loads(artifacts["mechanical"]), json.loads(artifacts["optical_datum"]),
    )
    maps = rectification_maps(cal)
    polygon = np.asarray(args.polygon, dtype=int).reshape(4, 2)
    region = target_mask(SIZE[::-1], polygon)
    lo, hi = args.sector_deg
    if not (math.isfinite(lo) and math.isfinite(hi) and -180 <= lo < hi <= 180):
        raise ValueError("sector must have increasing endpoints in [-180, 180] degrees")
    if not args.target_label.strip():
        raise ValueError("describe the physical target and static pose")
    scan_rows = read_rows(args.capture / "scans.jsonl")
    for scan in scan_rows:
        checked_bytes(args.capture, scan["payload_cdr"])
        if "record" in scan:
            raw = scan["record"]["payload"]["scan"]
            if digest(encoded(raw).encode()) != scan["record"]["evidence"]["raw_scan_sha256"]:
                raise ValueError("decoded scan evidence hash mismatch")
            header = raw["header"]
            if (header["stamp_sec"] * 1_000_000_000 + header["stamp_nanosec"] != scan["header_ns"]
                    or header["frame_id"] != "ld19_sensor"):
                raise ValueError("decoded scan header differs from captured timing or frame identity")
    args.output.mkdir(mode=0o700)
    write_json(args.output / "analysis-policy.json", {
        **BOUNDARY, "schema_version": SCHEMA, "target_label": args.target_label,
        "capture_session_sha256": sha256_file(args.capture / "session.json"),
        "frame_journal_sha256": sha256_file(args.capture / "frames.jsonl"),
        "scan_journal_sha256": sha256_file(args.capture / "scans.jsonl"),
        "calibration_sha256": ACTIVE_V7_SHA256, "target_polygon_rectified_left_px": polygon.tolist(),
        "ld19_sector_deg": list(args.sector_deg), "stereo_matcher": "sgbm7", "lr_max_delta_px": 1.5,
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
        "min_stereo_support": MIN_STEREO_SUPPORT, "min_stereo_coverage": MIN_STEREO_COVERAGE,
        "max_stereo_plane_p95_m": MAX_STEREO_PLANE_P95_M, "max_header_delta_ns": PAIRING.max_rgb_ld19_header_delta_ns,
        "max_source_age_ns": FRESHNESS.max_stereo_age_ns, "stereo_depth_frame": STEREO_FRAME,
        "stereo_comparison_frame": OPTICAL_FRAME, "ld19_transformed_frame": ESTIMATED_OPTICAL_FRAME,
        "estimated_axial_offset_m": axial, "extrinsic_independently_validated": False,
    })
    frame_counts, beam_counts, used_scans = Counter(), Counter(), set()
    infinite, finite = [], []
    with (args.output / "comparisons.jsonl").open("x") as stream:
        for frame in read_rows(args.capture / "frames.jsonl"):
            row = {"frame_index": frame.get("frame_index"), "status": frame["status"]}
            if frame["status"] == "PAYLOAD_RECORDED":
                jpeg = checked_bytes(args.capture, frame["payload_mjpg"])
                timing, scan = select_scan(frame, scan_rows, session["boot_id"])
                row.update(stereo_source=frame["payload_mjpg"], timing=timing, status=timing["status"])
                if scan is not None:
                    row.update(scan_source=scan["payload_cdr"], scan_header_ns=scan["header_ns"])
                    try:
                        left, right = rectify_jpeg(jpeg, maps)
                        disparity, lr, _ = lr_pair(left, right, "sgbm7")
                        buffer = io.BytesIO()
                        np.savez_compressed(buffer, disparity_px=disparity, lr_consistent=lr)
                        row["stereo_arrays"] = save_bytes(
                            args.output, f"stereo-{frame['frame_index']:04d}.npz", buffer.getvalue(),
                        )
                        surface = fit_stereo_surface(disparity, lr, cal["Q"], cal["R1"], region)
                    except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                        row.update(status="STEREO_ANALYSIS_REFUSED", reason=str(error))
                    else:
                        comparison = compare_scan(
                            scan["record"]["payload"]["scan"], surface, cal["R1"], cal["P1"],
                            rotation, translation, args.sector_deg,
                        )
                        row.update(status="COMPARISON_DIAGNOSTIC_ONLY", comparison=comparison)
                        beam_counts.update(comparison["beam_status_counts"])
                        used_scans.add(scan["receipt_sequence"])
                        for beam in comparison["beams"]:
                            if "signed_residual_mm" in beam:
                                infinite.append(beam["signed_residual_mm"] / 1000)
                                if beam["status"] == "FINITE_SURFACE_CANDIDATE":
                                    finite.append(beam["signed_residual_mm"] / 1000)
            frame_counts[row["status"]] += 1
            append_row(stream, row)
    write_json(args.output / "summary.json", {
        **BOUNDARY, "schema_version": SCHEMA, "frame_status_counts": dict(frame_counts),
        "beam_status_counts": dict(beam_counts), "distinct_compared_ld19_scans": len(used_scans),
        "infinite_plane_in_declared_sector": residual_stats(infinite),
        "finite_surface_candidates": residual_stats(finite),
        "pooled_statistics_weighting": "PER_BEAM_PER_STEREO_FRAME; LD19_SCAN_REUSE_POSSIBLE",
    })
    print(args.output / "summary.json")
    return 0 if finite else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    acquire = commands.add_parser("capture", help="archive raw sensor payloads to a new directory")
    acquire.add_argument("--native-capture", type=Path, required=True)
    acquire.add_argument("--calibration", type=Path, required=True)
    acquire.add_argument("--device", default="/dev/video2")
    acquire.add_argument("--frames", type=int, default=30)
    acquire.add_argument("--interval-s", type=float, default=.2)
    acquire.add_argument("--ld19-clock-evidence", default="")
    acquire.add_argument("--output", type=Path, required=True)
    offline = commands.add_parser("analyze", help="compare a manually marked static target offline")
    offline.add_argument("--capture", type=Path, required=True)
    offline.add_argument("--output", type=Path, required=True)
    offline.add_argument("--polygon", type=int, nargs=8, required=True,
                         help="four rectified-left pixel vertices in perimeter order, x1 y1 … x4 y4")
    offline.add_argument("--sector-deg", type=float, nargs=2, required=True,
                         help="predeclared LD19 angular window [min, max] in degrees")
    offline.add_argument("--target-label", required=True)
    os.umask(0o077)
    cv2.setNumThreads(1)
    args = parser.parse_args(argv)
    return capture(args) if args.command == "capture" else analyze(args)


if __name__ == "__main__":
    raise SystemExit(main())
