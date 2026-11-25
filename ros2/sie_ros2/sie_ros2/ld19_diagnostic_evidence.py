"""Pure, non-authoritative LD19 LaserScan evidence converter.

This is an internal diagnostic file format, NOT an approved SIE Data Contract.
It must not feed Measurement, World State or motor admission. Keep ROS types at
the adapter boundary; the returned record contains only JSON-compatible data.
"""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone


DIAGNOSTIC_SCHEMA = "sie.ld19.diagnostic_observation_candidate.v0"
EXPECTED_FRAME = "ld19_sensor"
SOURCE_TOPIC = "/sie/lidar/scan_raw"


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    return float(value)


def _finite(value, name: str) -> float:
    number = _number(value, name)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _raw_float(value, name: str):
    """Keep non-finite samples identifiable in strict JSON."""
    number = _number(value, name)
    if math.isnan(number):
        return "NaN"
    if number == math.inf:
        return "+Infinity"
    if number == -math.inf:
        return "-Infinity"
    return number


def _stamp(stamp) -> tuple[int, int, str]:
    sec = stamp.sec
    nsec = stamp.nanosec
    if (
        isinstance(sec, bool) or not isinstance(sec, int)
        or isinstance(nsec, bool) or not isinstance(nsec, int)
        or sec <= 0 or not 0 <= nsec < 1_000_000_000
    ):
        raise ValueError("LaserScan requires a non-zero valid source stamp")
    time = datetime.fromtimestamp(sec, timezone.utc)
    return sec, nsec, time.strftime("%Y-%m-%dT%H:%M:%S") + f".{nsec:09d}Z"


def make_ld19_diagnostic_record(scan, *, received_at_utc: str) -> dict:
    """Construct deterministic raw-scan evidence with no decision authority.

    The source timestamp identifies the scan. A separate host receive timestamp
    documents ingestion latency but does not replace sensor/header time.
    """
    if scan.header.frame_id != EXPECTED_FRAME:
        raise ValueError("unexpected scan reference_frame")

    sec, nsec, timestamp = _stamp(scan.header.stamp)
    received = datetime.fromisoformat(received_at_utc.replace("Z", "+00:00"))
    if received.tzinfo is None:
        raise ValueError("received_at_utc must include timezone")

    angle_min = _finite(scan.angle_min, "angle_min")
    angle_max = _finite(scan.angle_max, "angle_max")
    angle_increment = _finite(scan.angle_increment, "angle_increment")
    time_increment = _finite(scan.time_increment, "time_increment")
    scan_time = _finite(scan.scan_time, "scan_time")
    range_min = _finite(scan.range_min, "range_min")
    range_max = _finite(scan.range_max, "range_max")
    if angle_increment <= 0 or angle_max < angle_min:
        raise ValueError("invalid angular geometry")
    if time_increment < 0 or scan_time < 0 or not 0 <= range_min < range_max:
        raise ValueError("invalid timing or range limits")

    ranges = list(scan.ranges)
    intensities = list(scan.intensities)
    if not 0 < len(ranges) <= 5000:
        raise ValueError("unexpected scan sample count")
    if len(intensities) not in (0, len(ranges)):
        raise ValueError("intensity count must match ranges or be empty")

    valid = sum(
        math.isfinite(_number(distance, "range"))
        and range_min <= float(distance) <= range_max
        for distance in ranges
    )

    raw = {
        "header": {
            "stamp_sec": sec,
            "stamp_nanosec": nsec,
            "frame_id": EXPECTED_FRAME,
        },
        "angle_min_rad": angle_min,
        "angle_max_rad": angle_max,
        "angle_increment_rad": angle_increment,
        "time_increment_s": time_increment,
        "scan_time_s": scan_time,
        "range_min_m": range_min,
        "range_max_m": range_max,
        "ranges_m": [_raw_float(v, "range") for v in ranges],
        "intensities": [_raw_float(v, "intensity") for v in intensities],
    }
    encoded = json.dumps(raw, allow_nan=False, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    evidence_id = f"ld19-raw:{digest}"

    return {
        "schema_version": DIAGNOSTIC_SCHEMA,
        "status": "DIAGNOSTIC_ONLY_NOT_ADMITTED",
        "observation_id": f"ld19-diagnostic:{sec}:{nsec:09d}:{digest[:12]}",
        "source": "ldlidar_stl_ros2",
        "source_id": EXPECTED_FRAME,
        "source_topic": SOURCE_TOPIC,
        "timestamp": timestamp,
        "received_at_utc": received_at_utc,
        "cycle_id": f"ld19-scan:{sec}:{nsec:09d}",
        "observation_type": "lidar_2d_scan",
        "reference_frame": EXPECTED_FRAME,
        "confidence": 0.0,
        "quality": {
            "assessment": "NOT_QUALIFIED",
            "confidence_assessment": "NOT_ESTIMATED",
            "total_samples": len(ranges),
            "valid_samples": valid,
            "valid_fraction": valid / len(ranges),
        },
        "evidence_ids": [evidence_id],
        "evidence": {
            "raw_scan_sha256": digest,
            "hash_algorithm": "sha256",
            "encoding": "canonical_json",
        },
        "payload": {
            "distance_unit": "m",
            "angle_unit": "rad",
            "scan": raw,
        },
        "admission": {
            "measurement_authorized": False,
            "world_state_update_authorized": False,
            "motor_control_authorized": False,
        },
    }
