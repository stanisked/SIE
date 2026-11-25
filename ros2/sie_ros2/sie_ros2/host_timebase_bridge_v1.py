"""Pure diagnostic host CLOCK_MONOTONIC ↔ CLOCK_REALTIME bridge v1.

DRAFT, not an approved SIE Data Contract. No ROS, cameras, LiDAR or motor I/O.
Offsets are empirical host-clock samples, not hardware acquisition timestamps.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

BRIDGE_SCHEMA = "sie.host.timebase_bridge.v1.diagnostic"
NS = 1_000_000_000


@dataclass(frozen=True)
class HostBridgeTestPolicy:
    """All limits are explicit and experimental, never production defaults."""

    policy_id: str
    max_bracket_ns: int
    max_sample_gap_ns: int
    max_offset_span_ns: int
    max_consecutive_offset_jump_ns: int
    max_between_window_offset_change_ns: int
    max_age_ns: int
    max_extrapolation_ns: int
    max_future_skew_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.policy_id, str) or not self.policy_id.strip():
            raise ValueError("policy_id must be nonempty")
        for field in (
            "max_bracket_ns", "max_sample_gap_ns", "max_offset_span_ns",
            "max_consecutive_offset_jump_ns",
            "max_between_window_offset_change_ns",
            "max_age_ns", "max_extrapolation_ns", "max_future_skew_ns",
        ):
            value = getattr(self, field)
            if type(value) is not int or value < 0:
                raise ValueError(f"{field} must be a nonnegative integer")


def _integer(value: object, name: str, *, positive: bool = False) -> int:
    if type(value) is not int or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be a valid integer")
    return value


def _median_int(values: list[int]) -> int:
    values = sorted(values)
    midpoint = len(values) // 2
    if len(values) % 2:
        return values[midpoint]
    return (values[midpoint - 1] + values[midpoint]) // 2


def _policy_limits(policy: HostBridgeTestPolicy) -> dict:
    """Bind every threshold to the bridge, not just the human-readable ID."""
    return {
        field: getattr(policy, field)
        for field in (
            "max_bracket_ns", "max_sample_gap_ns", "max_offset_span_ns",
            "max_consecutive_offset_jump_ns",
            "max_between_window_offset_change_ns",
            "max_age_ns", "max_extrapolation_ns", "max_future_skew_ns",
        )
    }


def _boundary() -> dict:
    return {
        "diagnostic_only": True,
        "physical_acquisition_time": "UNKNOWN",
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }


def make_host_timebase_bridge(
    brackets: Sequence[Mapping[str, int]],
    *,
    boot_id: str,
    policy: HostBridgeTestPolicy,
    previous_bridge: Mapping | None = None,
) -> dict:
    """Construct a bounded empirical mapping from externally sampled clocks.

    Each bracket has monotonic_before_ns, realtime_ns, monotonic_after_ns.
    No implicit clock reads. A sample envelope is NOT a future error guarantee.
    """
    if not isinstance(policy, HostBridgeTestPolicy):
        raise ValueError("policy must be HostBridgeTestPolicy")
    if not isinstance(boot_id, str) or not boot_id.strip():
        raise ValueError("boot_id is required for reboot isolation")
    outcome = {
        **_boundary(),
        "schema_version": BRIDGE_SCHEMA,
        "policy_id": policy.policy_id,
        "policy_limits": _policy_limits(policy),
        "boot_id": boot_id,
        "monotonic_clock": "CLOCK_MONOTONIC",
        "realtime_clock": "CLOCK_REALTIME",
        "status": "INVALID_BRACKETS",
        "sample_count": 0,
        "offset_ns": None,
        "empirical_offset_envelope_ns": None,
        "first_sample_monotonic_ns": None,
        "last_sample_monotonic_ns": None,
        "max_bracket_observed_ns": None,
        "observed_offset_span_ns": None,
    }
    if not isinstance(brackets, (list, tuple)) or len(brackets) < 3:
        return outcome

    samples = []
    try:
        for entry in brackets:
            if not isinstance(entry, Mapping):
                raise ValueError("bracket must be a mapping")
            before = _integer(entry["monotonic_before_ns"], "monotonic_before_ns", positive=True)
            realtime = _integer(entry["realtime_ns"], "realtime_ns", positive=True)
            after = _integer(entry["monotonic_after_ns"], "monotonic_after_ns", positive=True)
            if after < before:
                raise ValueError("invalid bracket order")
            midpoint = (before + after) // 2
            samples.append((before, midpoint, after, realtime, realtime - midpoint))
    except (KeyError, ValueError, TypeError):
        return outcome

    if any(samples[i][1] <= samples[i - 1][1]
           or samples[i][0] <= samples[i - 1][2]
           for i in range(1, len(samples))):
        outcome["status"] = "MONOTONIC_SEQUENCE_INVALID"
        return outcome
    if any(samples[i][3] < samples[i - 1][3] for i in range(1, len(samples))):
        outcome["status"] = "REALTIME_CLOCK_ROLLBACK"
        return outcome

    widths = [after - before for before, _, after, _, _ in samples]
    if max(widths) > policy.max_bracket_ns:
        outcome["status"] = "BRACKET_TOO_WIDE"
        return outcome
    if any(samples[i][0] - samples[i - 1][2] > policy.max_sample_gap_ns
           for i in range(1, len(samples))):
        outcome["status"] = "SAMPLE_GAP_EXCEEDED"
        return outcome

    offsets = [sample[4] for sample in samples]
    offset_span = max(offsets) - min(offsets)
    if any(abs(offsets[i] - offsets[i - 1])
           > policy.max_consecutive_offset_jump_ns
           for i in range(1, len(offsets))):
        outcome["status"] = "CLOCK_STEP_SUSPECTED"
        return outcome
    if offset_span > policy.max_offset_span_ns:
        outcome["status"] = "OFFSET_SPAN_EXCEEDED"
        return outcome

    offset = _median_int(offsets)
    if previous_bridge is not None:
        if (not isinstance(previous_bridge, Mapping)
                or previous_bridge.get("schema_version") != BRIDGE_SCHEMA
                or previous_bridge.get("status") != "DIAGNOSTIC_BRIDGE_READY"
                or previous_bridge.get("policy_id") != policy.policy_id
                or previous_bridge.get("policy_limits") != _policy_limits(policy)
                or type(previous_bridge.get("offset_ns")) is not int
                or type(previous_bridge.get("last_sample_monotonic_ns")) is not int):
            outcome["status"] = "INVALID_PREVIOUS_BRIDGE"
            return outcome
        if previous_bridge.get("boot_id") != boot_id:
            outcome["status"] = "BOOT_ID_CHANGED"
            return outcome
        if samples[0][1] <= previous_bridge.get("last_sample_monotonic_ns", -1):
            outcome["status"] = "BRIDGE_WINDOWS_OUT_OF_ORDER"
            return outcome
        if abs(offset - previous_bridge["offset_ns"]) > policy.max_between_window_offset_change_ns:
            outcome["status"] = "CLOCK_STEP_SUSPECTED"
            return outcome

    # max(|sample offset - median| + half-bracket), rounded upward; empirical only.
    envelope = max(abs(s[4] - offset) + (s[2] - s[0] + 1) // 2 for s in samples)
    outcome.update(
        status="DIAGNOSTIC_BRIDGE_READY",
        sample_count=len(samples),
        offset_ns=offset,
        empirical_offset_envelope_ns=envelope,
        first_sample_monotonic_ns=samples[0][1],
        last_sample_monotonic_ns=samples[-1][1],
        max_bracket_observed_ns=max(widths),
        observed_offset_span_ns=offset_span,
    )
    return outcome


def convert_monotonic_stamp(
    bridge: Mapping,
    *,
    source_monotonic_ns: int,
    evaluation_monotonic_ns: int,
    boot_id: str,
    policy: HostBridgeTestPolicy,
) -> dict:
    """Map an external monotonic stamp to host UTC nanoseconds, diagnostic only.

    Source could be V4L2 driver-declared SOE. This does not prove SOE accuracy,
    nor LiDAR capture timing. No automatic ROS clock-domain assertion.
    """
    if not isinstance(policy, HostBridgeTestPolicy):
        raise ValueError("policy must be HostBridgeTestPolicy")
    _integer(source_monotonic_ns, "source_monotonic_ns", positive=True)
    _integer(evaluation_monotonic_ns, "evaluation_monotonic_ns", positive=True)
    if not isinstance(boot_id, str) or not boot_id.strip():
        raise ValueError("boot_id is required")
    result = {
        **_boundary(),
        "schema_version": BRIDGE_SCHEMA,
        "policy_id": policy.policy_id,
        "status": "BRIDGE_UNAVAILABLE",
        "source_clock": "CLOCK_MONOTONIC",
        "estimated_realtime_ns": None,
        "empirical_offset_envelope_ns": None,
        "source_monotonic_ns": source_monotonic_ns,
        "conversion_is_physical_acquisition_time": False,
    }
    if (not isinstance(bridge, Mapping)
            or bridge.get("schema_version") != BRIDGE_SCHEMA
            or bridge.get("status") != "DIAGNOSTIC_BRIDGE_READY"
            or bridge.get("policy_id") != policy.policy_id):
        return result
    if bridge.get("boot_id") != boot_id:
        result["status"] = "BOOT_ID_MISMATCH"
        return result
    if bridge.get("policy_limits") != _policy_limits(policy):
        result["status"] = "POLICY_LIMITS_MISMATCH"
        return result

    first = bridge.get("first_sample_monotonic_ns")
    last = bridge.get("last_sample_monotonic_ns")
    offset = bridge.get("offset_ns")
    envelope = bridge.get("empirical_offset_envelope_ns")
    if any(type(value) is not int for value in (first, last, offset, envelope)):
        result["status"] = "INVALID_BRIDGE_FIELDS"
        return result
    if first <= 0 or last < first or envelope < 0:
        result["status"] = "INVALID_BRIDGE_FIELDS"
        return result
    if evaluation_monotonic_ns < last:
        result["status"] = "BRIDGE_FROM_FUTURE"
        return result
    if evaluation_monotonic_ns - last > policy.max_age_ns:
        result["status"] = "BRIDGE_EXPIRED"
        return result
    if source_monotonic_ns > evaluation_monotonic_ns + policy.max_future_skew_ns:
        result["status"] = "STAMP_FROM_FUTURE"
        return result
    if (source_monotonic_ns < first - policy.max_extrapolation_ns
            or source_monotonic_ns > last + policy.max_extrapolation_ns):
        result["status"] = "STAMP_OUTSIDE_BRIDGE_WINDOW"
        return result

    result.update(
        status="HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY",
        estimated_realtime_ns=source_monotonic_ns + offset,
        empirical_offset_envelope_ns=envelope,
    )
    return result
