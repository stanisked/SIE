"""Offline-only candidate for the versioned Shadow UART compact status contract.

No motion command or UART I/O is implemented here. The live controller still
uses the existing full STATUS operation until a separate firmware/host migration
has been compiled, validated and explicitly authorized.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

SCHEMA_VERSION = 1
# Wire aliases are stable positions in the proposed v1 control-status contract.
# Full keys are reconstructed BEFORE the existing supervisor sees any status.
WIRE_FIELDS = (
    ("b", "firmware_build_id"),
    ("v", "shadow_api_version"),
    ("i", "boot_session_id"),
    ("s", "state"),
    ("a", "active_command_id"),
    ("j", "last_command_id"),
    ("k", "last_command_state"),
    ("q", "shadow_transition_seq"),
    ("p", "shadow_pause_reason"),
    ("t", "shadow_turn_result"),
    ("n", "shadow_segments_started"),
    ("u", "shadow_turns_started"),
    ("f", "bounded_fault_latched"),
    ("g", "steered_safety_latched"),
    ("z", "remote_stop_pending"),
    ("e", "fault"),
    ("y", "bounded_fault_reason"),
    ("r", "right_pwm"),
    ("l", "left_pwm"),
    ("h", "heading_rad"),
    ("x", "right_encoder_raw_count"),
    ("w", "left_encoder_raw_count"),
    ("d", "right_distance_m"),
    ("o", "left_distance_m"),
    ("m", "average_distance_m"),
    ("c", "last_steer_update_seq"),
    ("A", "active_steer_update_seq"),
)

_STATES = frozenset((
    "READY", "PAUSED", "STARTING", "DRIVING", "BRAKING", "CORRECTING",
    "COASTING", "SUCCESS", "FAULT", "UNKNOWN",
))
_REQUIRED_STRINGS = frozenset((
    "firmware_build_id", "shadow_api_version", "boot_session_id",
    "state", "shadow_turn_result",
))
_NULLABLE_STRINGS = frozenset((
    "active_command_id", "last_command_id", "last_command_state",
    "shadow_pause_reason", "fault", "bounded_fault_reason",
))
_BOOLS = frozenset((
    "bounded_fault_latched", "steered_safety_latched", "remote_stop_pending",
))
_NULLABLE_INTS = frozenset((
    "shadow_transition_seq", "last_steer_update_seq", "active_steer_update_seq",
))
_INTS = frozenset((
    "shadow_segments_started", "shadow_turns_started", "right_pwm",
    "left_pwm", "right_encoder_raw_count", "left_encoder_raw_count",
))
_FINITE_NUMBERS = frozenset((
    "heading_rad", "right_distance_m", "left_distance_m", "average_distance_m",
))


def _validate(fields: Mapping[str, object]) -> None:
    for _, name in WIRE_FIELDS:
        if name not in fields:
            raise ValueError("COMPACT_STATUS_MISSING_FIELD:" + name)
        value = fields[name]
        if name in _REQUIRED_STRINGS:
            good = type(value) is str and bool(value)
        elif name in _NULLABLE_STRINGS:
            good = value is None or (type(value) is str and bool(value))
        elif name in _BOOLS:
            good = type(value) is bool
        elif name in _NULLABLE_INTS:
            good = value is None or (type(value) is int and value >= 0)
        elif name in _INTS:
            good = type(value) is int
            if name in ("shadow_segments_started", "shadow_turns_started", "right_pwm", "left_pwm"):
                good = good and 0 <= value <= (255 if name.endswith("_pwm") else 1000000)
        elif name in _FINITE_NUMBERS:
            good = type(value) in (int, float) and math.isfinite(value)
        else:
            raise RuntimeError("COMPACT_STATUS_SCHEMA_INCOMPLETE:" + name)
        if not good:
            raise ValueError("COMPACT_STATUS_INVALID_FIELD:" + name)
    if fields["state"] not in _STATES:
        raise ValueError("COMPACT_STATUS_UNKNOWN_STATE")


def compress_status(full: Mapping[str, object]) -> dict[str, object]:
    """Model a future firmware reply from an existing full UART status."""
    if not isinstance(full, Mapping):
        raise ValueError("COMPACT_STATUS_INVALID_SOURCE")
    _validate(full)
    return {"_": SCHEMA_VERSION, **{alias: full[name] for alias, name in WIRE_FIELDS}}


def expand_status(wire: Mapping[str, object]) -> dict[str, object]:
    """Fail closed on missing, unexpected, mistyped, or unknown-version data."""
    if not isinstance(wire, Mapping) or wire.get("_") != SCHEMA_VERSION or type(wire.get("_")) is not int:
        raise ValueError("COMPACT_STATUS_UNSUPPORTED_VERSION")
    required_keys = {"_"} | {alias for alias, _ in WIRE_FIELDS}
    if set(wire) != required_keys:
        raise ValueError("COMPACT_STATUS_WIRE_KEYS_INVALID")
    full = {name: wire[alias] for alias, name in WIRE_FIELDS}
    _validate(full)
    return full
