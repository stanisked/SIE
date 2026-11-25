"""Explicit-time freshness audit for offline RGB-anchored associations.

DRAFT diagnostic only. No implicit clock reads, ROS, cameras or motor code.
A recent LD19 ROS header is not proof of recent laser acquisition.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from sie_ros2.temporal_association_diagnostic_v1 import (
    SCHEMA as ASSOCIATION_SCHEMA,
    RGB_STAMP_BASIS,
    LD19_STAMP_BASIS,
)

SCHEMA = "sie.temporal_association_freshness.v1.diagnostic"


@dataclass(frozen=True)
class FreshnessTrialPolicy:
    """Experimental demonstration limits, NOT runtime safety thresholds."""

    policy_id: str
    max_rgb_age_ns: int
    max_stereo_age_ns: int
    max_ld19_ros_header_age_ns: int
    max_bridge_holdover_ns: int
    max_future_skew_ns: int

    def __post_init__(self):
        if not isinstance(self.policy_id, str) or not self.policy_id.strip():
            raise ValueError("policy_id must be nonempty")
        for name in (
            "max_rgb_age_ns", "max_stereo_age_ns",
            "max_ld19_ros_header_age_ns",
            "max_bridge_holdover_ns", "max_future_skew_ns",
        ):
            if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                raise ValueError(f"{name} must be a nonnegative integer")


def _int(value, name, *, allow_zero=False):
    if type(value) is not int or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f"{name} must be a valid integer")
    return value


def _no_admission(value):
    return isinstance(value, Mapping) and all(value.get(name) is False for name in (
        "measurement_authorized",
        "world_state_update_authorized",
        "motor_control_authorized",
    ))


def evaluate_association_freshness(
    associations: Mapping,
    bridge: Mapping,
    *,
    evaluation_monotonic_ns: int,
    boot_id: str,
    policy: FreshnessTrialPolicy,
    previous_evaluation_monotonic_ns: int | None = None,
) -> dict:
    """Evaluate recorded host timestamp recency, never spatial/motion admission.

    Evaluation time is caller-supplied *CLOCK_MONOTONIC on the source Pi boot*.
    A test can use an explicitly selected historical instant for replay.
    """
    if not isinstance(policy, FreshnessTrialPolicy):
        raise ValueError("explicit FreshnessTrialPolicy required")
    _int(evaluation_monotonic_ns, "evaluation_monotonic_ns")
    if not isinstance(boot_id, str) or not boot_id:
        raise ValueError("boot_id required")
    if previous_evaluation_monotonic_ns is not None:
        _int(previous_evaluation_monotonic_ns, "previous_evaluation_monotonic_ns")
    if not _no_admission(associations):
        raise ValueError("source association admission boundary invalid")
    if (associations.get("schema_version") != ASSOCIATION_SCHEMA
            or associations.get("status") != "DIAGNOSTIC_ONLY_NOT_ADMITTED"):
        raise ValueError("unexpected temporal association source")
    if associations.get("boot_id") != boot_id:
        raise ValueError("association boot ID mismatch")
    if associations.get("timebase") != "HOST_REALTIME_ESTIMATE_DIAGNOSTIC_ONLY":
        raise ValueError("unexpected association timebase")
    # A false/unknown ROS-clock-domain claim must not be promoted to verified.
    if associations.get("ros_clock_domain_independently_verified") is not False:
        raise ValueError("ROS clock verification state changed")
    rows = associations.get("associations")
    if not isinstance(rows, list):
        raise ValueError("missing source association rows")

    if (not isinstance(bridge, Mapping)
            or bridge.get("boot_id") != boot_id
            or bridge.get("status") != "DIAGNOSTIC_BRIDGE_READY"
            or bridge.get("monotonic_clock") != "CLOCK_MONOTONIC"
            or bridge.get("realtime_clock") != "CLOCK_REALTIME"):
        raise ValueError("unverified bridge or boot ID mismatch")
    first = _int(bridge.get("first_sample_monotonic_ns"), "bridge first")
    last = _int(bridge.get("last_sample_monotonic_ns"), "bridge last")
    offset = _int(bridge.get("offset_ns"), "bridge offset")
    if first >= last:
        raise ValueError("invalid bridge span")
    limits = bridge.get("policy_limits")
    if not isinstance(limits, Mapping):
        raise ValueError("missing original host bridge policy limits")
    source_max_age = _int(limits.get("max_age_ns"), "bridge max_age_ns", allow_zero=True)

    # Clock discontinuity/order/age is a whole-evaluation refusal, never
    # a fabricated fresh result. No implicit wall-clock conversion.
    refusal = None
    if (previous_evaluation_monotonic_ns is not None
            and evaluation_monotonic_ns < previous_evaluation_monotonic_ns):
        refusal = "EVALUATION_MONOTONIC_ROLLBACK"
    elif evaluation_monotonic_ns < last:
        refusal = "BRIDGE_FROM_FUTURE"
    elif evaluation_monotonic_ns - last > min(
            policy.max_bridge_holdover_ns, source_max_age):
        refusal = "BRIDGE_EXPIRED"

    result = {
        "schema_version": SCHEMA,
        "status": refusal or "EVALUATED_DIAGNOSTIC_ONLY",
        "boot_id": boot_id,
        "policy_id": policy.policy_id,
        "source_capture_sha256": associations.get("source_evidence_sha256"),
        "evaluation_monotonic_ns": evaluation_monotonic_ns,
        "evaluation_realtime_estimate_ns": None if refusal else evaluation_monotonic_ns + offset,
        "ros_clock_domain_independently_verified": False,
        "physical_laser_acquisition_time": "UNKNOWN",
        "rows": [],
        "summary": {"evaluated": 0, "rgb_stereo_recent": 0,
                    "ld19_header_recent": 0, "both_independent_links_recent": 0},
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }
    if refusal:
        # No per-row age may be asserted using expired or future mapping.
        return result

    def source(ref, expected_id, expected_basis, max_age, expected_frame):
        if ref is None:
            return {"status": "SOURCE_UNAVAILABLE", "age_ns": None}
        if not isinstance(ref, Mapping):
            raise ValueError("source reference must be a mapping")
        if (ref.get("source_id") != expected_id
                or ref.get("timestamp_basis") != expected_basis
                or ref.get("reference_frame") != expected_frame):
            raise ValueError("source provenance mismatch")
        evidence_ids = ref.get("evidence_ids")
        if (not isinstance(evidence_ids, list)
                or len(evidence_ids) != 1
                or not isinstance(evidence_ids[0], str)
                or not evidence_ids[0].startswith(
                    "private-capture-sha256:" + str(associations["source_evidence_sha256"]) + "#"
                )):
            raise ValueError("source evidence link mismatch")
        timestamp = _int(ref.get("timestamp_ns"), "source timestamp")
        age = result["evaluation_realtime_estimate_ns"] - timestamp
        if age < -policy.max_future_skew_ns:
            status = "STAMP_FROM_FUTURE"
        elif age < 0:
            status = "FUTURE_WITHIN_TRIAL_SKEW"
        elif age > max_age:
            status = "SOURCE_STALE"
        else:
            status = "RECENT_HOST_TIMESTAMP_DIAGNOSTIC_ONLY"
        return {"status": status, "age_ns": age}

    for row in rows:
        if not _no_admission(row):
            raise ValueError("association row admission boundary invalid")
        if row.get("status") not in (
            "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY",
            "STEREO_UNAVAILABLE", "RGB_STEREO_DELTA_EXCEEDED",
        ):
            raise ValueError("unexpected RGB-Stereo association status")
        if row.get("lidar_status") not in (
            "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY",
            "LD19_UNAVAILABLE", "LD19_ROS_HEADER_DELTA_EXCEEDED",
        ):
            raise ValueError("unexpected LD19 diagnostic link status")
        if (row.get("physical_laser_acquisition_time") != "UNKNOWN"
                or row.get("physical_camera_exposure_time_independently_verified") is not False
                or row.get("confidence") is not None):
            raise ValueError("source uncertainty boundary changed")
        if row.get("stereo") is not None and row["status"] != "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY":
            raise ValueError("stereo link/status mismatch")
        if row.get("ld19") is not None and row["lidar_status"] != "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY":
            raise ValueError("LD19 link/status mismatch")

        rgb = source(row.get("rgb"), "ar0234", RGB_STAMP_BASIS,
                     policy.max_rgb_age_ns, "UNVERIFIED_IN_CAPTURE")
        stereo = source(row.get("stereo"), "ov9281", RGB_STAMP_BASIS,
                        policy.max_stereo_age_ns, "UNVERIFIED_IN_CAPTURE")
        ld19 = source(row.get("ld19"), "ld19_sensor", LD19_STAMP_BASIS,
                      policy.max_ld19_ros_header_age_ns, "ld19_sensor")

        # Never use the third Stereo<->LD19 edge as a gate.
        recent_status = "RECENT_HOST_TIMESTAMP_DIAGNOSTIC_ONLY"
        pair_recent = (row["status"] == "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY"
                       and rgb["status"] == recent_status
                       and stereo["status"] == recent_status)
        ld19_recent = (row["lidar_status"] == "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY"
                       and rgb["status"] == recent_status
                       and ld19["status"] == recent_status)

        if row["status"] != "RGB_STEREO_MATCH_DIAGNOSTIC_ONLY":
            pair_status = "RGB_STEREO_ASSOCIATION_UNAVAILABLE"
        elif pair_recent:
            pair_status = "RGB_STEREO_RECENT_DIAGNOSTIC_ONLY"
        elif rgb["status"] != recent_status:
            pair_status = "RGB_" + rgb["status"]
        else:
            pair_status = "STEREO_" + stereo["status"]

        if row["lidar_status"] != "LD19_ROS_HEADER_NEAR_RGB_DIAGNOSTIC_ONLY":
            lidar_status = "LD19_DIAGNOSTIC_LINK_UNAVAILABLE"
        elif ld19_recent:
            lidar_status = "LD19_ROS_HEADER_RECENT_DIAGNOSTIC_ONLY"
        elif rgb["status"] != recent_status:
            lidar_status = "RGB_" + rgb["status"]
        else:
            lidar_status = "LD19_" + ld19["status"]

        result["rows"].append({
            "rgb_evidence_ids": row["rgb"]["evidence_ids"] if row.get("rgb") else [],
            "age_ns": {
                "rgb": rgb["age_ns"], "stereo": stereo["age_ns"],
                "ld19_ros_header": ld19["age_ns"],
            },
            "source_status": {
                "rgb": rgb["status"], "stereo": stereo["status"],
                "ld19_ros_header": ld19["status"],
            },
            "rgb_stereo_freshness_status": pair_status,
            "rgb_ld19_header_freshness_status": lidar_status,
            "rgb_stereo_recent": pair_recent,
            "rgb_ld19_header_recent": ld19_recent,
            "ld19_laser_acquisition_recency": "UNKNOWN",
            "measurement_authorized": False,
            "world_state_update_authorized": False,
            "motor_control_authorized": False,
        })
        result["summary"]["evaluated"] += 1
        result["summary"]["rgb_stereo_recent"] += int(pair_recent)
        result["summary"]["ld19_header_recent"] += int(ld19_recent)
        result["summary"]["both_independent_links_recent"] += int(
            pair_recent and ld19_recent
        )
    return result
