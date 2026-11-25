"""RGB-anchored time association of private Pi metadata: diagnostic only.

No ROS, image decoding, LiDAR packet parsing, World State, or motor I/O.
Input is the private sie.three_sensor_warmed.v1.diagnostic capture format.
A LiDAR ROS header is conversion/publication time, NOT laser acquisition.
This is not an approved Observation or Measurement Data Contract.
"""
from __future__ import annotations

from dataclasses import dataclass
import bisect
import re
from typing import Mapping, Sequence

SCHEMA = "sie.temporal_association.v1.diagnostic"
INPUT_SCHEMA = "sie.three_sensor_warmed.v1.diagnostic"
DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")

RGB_STAMP_BASIS = "V4L2_DRIVER_DECLARED_SOE_MONOTONIC_HOST_BRIDGE_DIAGNOSTIC"
LD19_STAMP_BASIS = "ROS_SCAN_CONVERSION_PUBLICATION_NOT_LASER_ACQUISITION"


@dataclass(frozen=True)
class TemporalAssociationTrialPolicy:
    """Explicit trial-only limits, never production acceptance criteria."""

    policy_id: str
    max_rgb_stereo_delta_ns: int
    max_rgb_ld19_header_delta_ns: int

    def __post_init__(self):
        if not isinstance(self.policy_id, str) or not self.policy_id.strip():
            raise ValueError("policy_id is required")
        for key in ("max_rgb_stereo_delta_ns", "max_rgb_ld19_header_delta_ns"):
            if type(getattr(self, key)) is not int or getattr(self, key) < 0:
                raise ValueError(f"{key} must be a nonnegative integer")


def _positive_int(value, name):
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nearest(entries: Sequence[dict], target_ns: int) -> dict | None:
    """Deterministic nearest; ties choose earlier timestamp then input index."""
    if not entries:
        return None
    timestamps = [item["timestamp_ns"] for item in entries]
    index = bisect.bisect_left(timestamps, target_ns)
    candidates = entries[max(0, index - 1):min(len(entries), index + 1)]
    return min(candidates, key=lambda item: (
        abs(item["timestamp_ns"] - target_ns),
        item["timestamp_ns"], item["index"],
    ))


def _candidate(item, *, source_id, timestamp_basis, reference_frame, digest, path):
    return {
        "source_id": source_id,
        "sequence": item.get("sequence"),
        "timestamp_ns": item["timestamp_ns"],
        "timestamp_basis": timestamp_basis,
        "reference_frame": reference_frame,
        "evidence_ids": [f"private-capture-sha256:{digest}#{path}[{item['index']}]"],
    }


def associate_warmed_metadata(
    report: Mapping,
    *,
    source_sha256: str,
    policy: TemporalAssociationTrialPolicy,
) -> dict:
    """Replay a bounded captured session. No current clocks or device I/O.

    RGB -> Stereo is the primary metadata association. The nearest LD19
    header is independently linked to RGB and never gates the Stereo pair.
    All emitted statuses are diagnostic, never canonical data admission.
    """
    if not isinstance(policy, TemporalAssociationTrialPolicy):
        raise ValueError("explicit experimental policy required")
    if not isinstance(source_sha256, str) or not DIGEST_RE.fullmatch(source_sha256):
        raise ValueError("source evidence SHA-256 required")
    if not isinstance(report, Mapping) or report.get("schema_version") != INPUT_SCHEMA:
        raise ValueError("unexpected evidence schema")
    if any(report.get(flag) is not False for flag in (
            "measurement_authorized",
            "world_state_update_authorized",
            "motor_control_authorized",
    )):
        raise ValueError("source file does not preserve diagnostic-only boundary")
    boot_id = report.get("boot_id")
    bridge = report.get("bridge")
    if not isinstance(boot_id, str) or not boot_id:
        raise ValueError("missing boot ID")
    if (not isinstance(bridge, Mapping)
            or bridge.get("boot_id") != boot_id
            or bridge.get("status") != "DIAGNOSTIC_BRIDGE_READY"
            or bridge.get("monotonic_clock") != "CLOCK_MONOTONIC"
            or bridge.get("realtime_clock") != "CLOCK_REALTIME"):
        raise ValueError("invalid or mismatched clock bridge")
    first = _positive_int(bridge.get("first_sample_monotonic_ns"), "bridge first")
    last = _positive_int(bridge.get("last_sample_monotonic_ns"), "bridge last")
    offset = _positive_int(bridge.get("offset_ns"), "bridge offset")
    if first >= last:
        raise ValueError("invalid bridge window")

    camera_frames = report.get("camera_frames")
    if not isinstance(camera_frames, Mapping):
        raise ValueError("missing camera source streams")

    def camera_stream(name):
        rows = camera_frames.get(name)
        if not isinstance(rows, list):
            raise ValueError(f"missing {name} metadata")
        result = []
        for index, row in enumerate(rows):
            if not isinstance(row, Mapping):
                raise ValueError("invalid frame record")
            stamp = _positive_int(row.get("driver_monotonic_ns"), "driver stamp")
            sequence = row.get("sequence")
            if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
                raise ValueError("invalid V4L2 sequence")
            received = _positive_int(row.get("received_monotonic_ns"), "packet receipt")
            if received < stamp:
                raise ValueError("V4L2 receipt before driver stamp")
            if first <= stamp <= last:
                result.append({
                    "index": index,
                    "timestamp_ns": stamp + offset,
                    "sequence": sequence,
                    "receipt_latency_ns": received - stamp,
                })
        result.sort(key=lambda item: (item["timestamp_ns"], item["index"]))
        if len({item["sequence"] for item in result}) != len(result):
            raise ValueError("repeated camera sequence")
        return result

    rgb = camera_stream("ar0234")
    stereo = camera_stream("ov9281")

    raw_lidar = report.get("ld19")
    if not isinstance(raw_lidar, list):
        raise ValueError("missing LD19 metadata")
    lidar = []
    for index, row in enumerate(raw_lidar):
        if not isinstance(row, Mapping) or row.get("frame_id") != "ld19_sensor":
            raise ValueError("unexpected LD19 frame")
        timestamp = _positive_int(row.get("header_ns"), "LD19 header stamp")
        received = _positive_int(row.get("receive_monotonic_ns"), "LD19 receipt")
        if first <= received <= last and first + offset <= timestamp <= last + offset:
            lidar.append({"index": index, "timestamp_ns": timestamp})
    lidar.sort(key=lambda item: (item["timestamp_ns"], item["index"]))

    # Match the Step 41 offline audit: restrict RGB anchors to LD19 coverage
    # if LD19 exists, and still permit camera-only pairs when it does not.
    anchors = (
        [item for item in rgb if lidar[0]["timestamp_ns"]
         <= item["timestamp_ns"] <= lidar[-1]["timestamp_ns"]]
        if lidar else rgb
    )

    matched_stereo = []
    linked_lidar = []
    rows = []

    for anchor in anchors:
        rgb_ref = _candidate(
            anchor, source_id="ar0234", timestamp_basis=RGB_STAMP_BASIS,
            reference_frame="UNVERIFIED_IN_CAPTURE", digest=source_sha256,
            path="camera_frames.ar0234",
        )
        nearest_stereo = _nearest(stereo, anchor["timestamp_ns"])
        nearest_lidar = _nearest(lidar, anchor["timestamp_ns"])

        stereo_ref = None
        stereo_status = "STEREO_UNAVAILABLE"
        rgb_stereo_delta_ns = None
        if nearest_stereo is not None:
            rgb_stereo_delta_ns = (nearest_stereo["timestamp_ns"]
                                   - anchor["timestamp_ns"])
            if abs(rgb_stereo_delta_ns) <= policy.max_rgb_stereo_delta_ns:
                stereo_status = "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY"
                matched_stereo.append(nearest_stereo["index"])
                stereo_ref = _candidate(
                    nearest_stereo, source_id="ov9281",
                    timestamp_basis=RGB_STAMP_BASIS,
                    reference_frame="UNVERIFIED_IN_CAPTURE",
                    digest=source_sha256, path="camera_frames.ov9281",
                )
            else:
                stereo_status = "RGB_STEREO_DELTA_EXCEEDED"

        lidar_ref = None
        lidar_status = "LD19_UNAVAILABLE"
        rgb_lidar_header_delta_ns = None
        stereo_lidar_header_delta_ns = None
        if nearest_lidar is not None:
            rgb_lidar_header_delta_ns = (nearest_lidar["timestamp_ns"]
                                         - anchor["timestamp_ns"])
            if abs(rgb_lidar_header_delta_ns) <= policy.max_rgb_ld19_header_delta_ns:
                lidar_status = "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY"
                linked_lidar.append(nearest_lidar["index"])
                lidar_ref = _candidate(
                    nearest_lidar, source_id="ld19_sensor",
                    timestamp_basis=LD19_STAMP_BASIS,
                    reference_frame="ld19_sensor",
                    digest=source_sha256, path="ld19",
                )
                # Report this third edge as context, never as a gate.
                if nearest_stereo is not None:
                    stereo_lidar_header_delta_ns = (
                        nearest_lidar["timestamp_ns"]
                        - nearest_stereo["timestamp_ns"]
                    )
            else:
                lidar_status = "LD19_ROS_HEADER_DELTA_EXCEEDED"

        rows.append({
            "status": stereo_status,
            "rgb": rgb_ref,
            "stereo": stereo_ref,
            "lidar_status": lidar_status,
            "ld19": lidar_ref,
            "rgb_stereo_delta_ns": rgb_stereo_delta_ns,
            "rgb_ld19_header_delta_ns": rgb_lidar_header_delta_ns,
            "stereo_ld19_header_delta_ns": stereo_lidar_header_delta_ns,
            "physical_laser_acquisition_time": "UNKNOWN",
            "physical_camera_exposure_time_independently_verified": False,
            "confidence": None,
            "confidence_basis": "NOT_ESTIMATED",
            "measurement_authorized": False,
            "world_state_update_authorized": False,
            "motor_control_authorized": False,
        })

    summary = {
        "rgb_anchors": len(anchors),
        "rgb_stereo_matches": len(matched_stereo),
        "rgb_stereo_unique_stereo": len(set(matched_stereo)),
        "rgb_ld19_header_links": len(linked_lidar),
        "rgb_ld19_unique_headers": len(set(linked_lidar)),
        "both_diagnostic_links": sum(
            row["stereo"] is not None and row["ld19"] is not None
            for row in rows
        ),
    }

    return {
        "schema_version": SCHEMA,
        "status": "DIAGNOSTIC_ONLY_NOT_ADMITTED",
        "policy_id": policy.policy_id,
        "policy_limits_ns": {
            "rgb_stereo": policy.max_rgb_stereo_delta_ns,
            "rgb_ld19_header": policy.max_rgb_ld19_header_delta_ns,
        },
        "boot_id": boot_id,
        "source_evidence_sha256": source_sha256,
        "timebase": "HOST_REALTIME_ESTIMATE_DIAGNOSTIC_ONLY",
        "ros_clock_domain_independently_verified": False,
        "physical_laser_acquisition_time": "UNKNOWN",
        "summary": summary,
        "associations": rows,
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }
