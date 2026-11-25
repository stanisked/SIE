"""Draft LD19 timestamp provenance and freshness checks (pure Python).

This is a DIAGNOSTIC policy, not an approved SIE Data Contract and not a
physical laser acquisition-time validator. No ROS runtime or motor interface.

LD19 driver bf668a8 (operator-inspected 2026-11-25):
- ROS LaserScan.header.stamp: node->now() at conversion/publication path.
- SDK per-point stamps: host system-clock packet-time interpolations, NOT
  exported in the currently published LaserScan.

Any source-vs-receive comparison requires independently verified comparable
clock domains. A passing host timing window NEVER admits a Measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import re
from typing import Any


SCHEMA = "sie.ld19.timestamp_provenance_freshness.v1.draft"
INPUT_SCHEMA = "sie.ld19.diagnostic_observation_candidate.v0"
EXPECTED_FRAME = "ld19_sensor"
EXPECTED_TOPIC = "/sie/lidar/scan_raw"
NANOSECONDS = 1_000_000_000
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class TimestampPolicyV1:
    """Explicit engineering-test limits, NOT approved runtime thresholds."""

    policy_id: str
    max_receive_age_ns: int
    max_future_skew_ns: int
    max_conversion_to_receive_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.policy_id, str) or not self.policy_id.strip():
            raise ValueError("policy_id must be nonempty")
        for name in (
            "max_receive_age_ns",
            "max_future_skew_ns",
            "max_conversion_to_receive_ns",
        ):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")


def _utc_ns(iso: str) -> int:
    if not isinstance(iso, str) or not iso:
        raise ValueError("timestamp is missing")
    try:
        moment = datetime.fromisoformat(iso.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("timestamp must be ISO 8601") from exc
    if moment.tzinfo is None:
        raise ValueError("timestamp must carry a timezone")
    moment = moment.astimezone(timezone.utc)
    delta = moment - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return ((delta.days * 86400 + delta.seconds) * NANOSECONDS
            + delta.microseconds * 1000)


def _positive_int(value: object, name: str, *, allow_zero: bool = False) -> int:
    if type(value) is not int or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f"{name} must be a valid integer")
    return value


def _read_provenance(record: dict[str, Any]) -> tuple[int, int, str]:
    if not isinstance(record, dict) or record.get("schema_version") != INPUT_SCHEMA:
        raise ValueError("unexpected input schema")
    if record.get("status") != "DIAGNOSTIC_ONLY_NOT_ADMITTED":
        raise ValueError("input must be diagnostic-only")
    if record.get("reference_frame") != EXPECTED_FRAME:
        raise ValueError("unexpected reference_frame")
    if record.get("source_topic") != EXPECTED_TOPIC:
        raise ValueError("unexpected source_topic")

    admission = record.get("admission")
    if not isinstance(admission, dict) or any(
        admission.get(key) is not False
        for key in (
            "measurement_authorized",
            "world_state_update_authorized",
            "motor_control_authorized",
        )
    ):
        raise ValueError("input admission boundary invalid")

    evidence = record.get("evidence")
    digest = evidence.get("raw_scan_sha256") if isinstance(evidence, dict) else None
    if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
        raise ValueError("raw evidence digest missing")
    if record.get("evidence_ids") != [f"ld19-raw:{digest}"]:
        raise ValueError("evidence linkage mismatch")

    try:
        header = record["payload"]["scan"]["header"]
        if header["frame_id"] != EXPECTED_FRAME:
            raise ValueError("unexpected scan header frame")
        sec = _positive_int(header["stamp_sec"], "stamp_sec")
        nsec = _positive_int(
            header["stamp_nanosec"], "stamp_nanosec", allow_zero=True
        )
        if nsec >= NANOSECONDS:
            raise ValueError("invalid stamp_nanosec")
        source_time = datetime.fromtimestamp(sec, tz=timezone.utc)
        expected_iso = (
            source_time.strftime("%Y-%m-%dT%H:%M:%S")
            + f".{nsec:09d}Z"
        )
        if record["timestamp"] != expected_iso:
            raise ValueError("scan/header timestamp mismatch")
        received_ns = _utc_ns(record["received_at_utc"])
    except (KeyError, TypeError, OverflowError, OSError) as exc:
        raise ValueError("invalid scan timestamp fields") from exc

    return sec * NANOSECONDS + nsec, received_ns, f"ld19-raw:{digest}"


def evaluate_ld19_timestamp(
    record: dict[str, Any],
    *,
    evaluation_time_utc_ns: int,
    policy: TimestampPolicyV1,
    previous_source_stamp_ns: int | None = None,
    same_host_clock_domain_verified: bool = False,
) -> dict[str, Any]:
    """Deterministic offline diagnostic decision, never production admission.

    Caller explicitly supplies decision time and the previous PASS source
    stamp. No implicit system clock, hidden state or auto clock-domain trust.
    On a failed check the previous PASS high-water mark is preserved.
    """
    if type(evaluation_time_utc_ns) is not int or evaluation_time_utc_ns <= 0:
        raise ValueError("evaluation_time_utc_ns must be positive integer")
    if previous_source_stamp_ns is not None:
        _positive_int(previous_source_stamp_ns, "previous_source_stamp_ns")
    if type(same_host_clock_domain_verified) is not bool:
        raise ValueError("same_host_clock_domain_verified must be boolean")
    if not isinstance(policy, TimestampPolicyV1):
        raise ValueError("policy must be TimestampPolicyV1")

    # The provenance properties describe THIS inspected driver revision only.
    result = {
        "schema_version": SCHEMA,
        "policy_id": policy.policy_id,
        "evaluation_time_utc_ns": evaluation_time_utc_ns,
        "source_clock": "ROS_NODE_CLOCK",
        "source_stamp_meaning": "ROS_SCAN_CONVERSION_TIME_NOT_ACQUISITION",
        "sdk_packet_clock": "HOST_SYSTEM_CLOCK_PACKET_PARSE_TIME",
        "sdk_per_point_stamp": "INTERPOLATED_NOT_EXPORTED_TO_ROS_LASERSCAN",
        "receive_clock": "UTC_HOST_SYSTEM_CLOCK",
        "physical_acquisition_time": "UNKNOWN",
        "same_host_clock_domain_verified": same_host_clock_domain_verified,
        "status": "INVALID_SOURCE",
        "source_stamp_ns": None,
        "receive_stamp_ns": None,
        "receive_age_ns": None,
        "conversion_to_receive_ns": None,
        "evidence_id": None,
        "last_accepted_source_stamp_ns": previous_source_stamp_ns,
        "next_accepted_source_stamp_ns": previous_source_stamp_ns,
        "diagnostic_time_window_passed": False,
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }

    try:
        source_ns, received_ns, evidence_id = _read_provenance(record)
    except (ValueError, AttributeError) as exc:
        result["reason"] = str(exc)
        return result

    result.update(
        source_stamp_ns=source_ns,
        receive_stamp_ns=received_ns,
        receive_age_ns=evaluation_time_utc_ns - received_ns,
        evidence_id=evidence_id,
    )

    if previous_source_stamp_ns is not None:
        if source_ns == previous_source_stamp_ns:
            result["status"] = "DUPLICATE_SOURCE_STAMP"
            return result
        if source_ns < previous_source_stamp_ns:
            result["status"] = "OUT_OF_ORDER_SOURCE_STAMP"
            return result

    receive_age_ns = evaluation_time_utc_ns - received_ns
    if receive_age_ns < -policy.max_future_skew_ns:
        result["status"] = "RECEIVE_TIME_IN_FUTURE"
        return result
    if receive_age_ns > policy.max_receive_age_ns:
        result["status"] = "STALE_RECEIPT"
        return result

    # Do not compare ROS node clock with UTC until it is independently proven
    # to represent the SAME host clock (e.g. not ROS simulation time).
    if not same_host_clock_domain_verified:
        result["status"] = "CLOCK_DOMAIN_UNVERIFIED"
        return result

    transport_ns = received_ns - source_ns
    result["conversion_to_receive_ns"] = transport_ns
    if transport_ns < -policy.max_future_skew_ns:
        result["status"] = "ROS_STAMP_AHEAD_OF_RECEIPT"
        return result
    if transport_ns > policy.max_conversion_to_receive_ns:
        result["status"] = "CONVERSION_TO_RECEIVE_TOO_OLD"
        return result

    result["status"] = "HOST_TIMING_WINDOW_PASS_DIAGNOSTIC_ONLY"
    result["diagnostic_time_window_passed"] = True
    result["next_accepted_source_stamp_ns"] = source_ns
    return result
