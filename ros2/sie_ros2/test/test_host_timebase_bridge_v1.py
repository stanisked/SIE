"""Offline synthetic tests: host monotonic/realtime mapping, never sensor admission."""
from copy import deepcopy
import json

import pytest

from sie_ros2.host_timebase_bridge_v1 import (
    BRIDGE_SCHEMA, HostBridgeTestPolicy,
    make_host_timebase_bridge, convert_monotonic_stamp,
)

OFFSET = 1_795_611_548_984_341_504
START = 10_000_000_000


@pytest.fixture
def policy():
    # Artificial regression thresholds ONLY; not an SIE runtime policy.
    return HostBridgeTestPolicy(
        policy_id="test-host-timebase-v1",
        max_bracket_ns=2_000,
        max_sample_gap_ns=20_000_000,
        max_offset_span_ns=10_000,
        max_consecutive_offset_jump_ns=5_000,
        max_between_window_offset_change_ns=10_000,
        max_age_ns=250_000_000,
        max_extrapolation_ns=10_000_000,
        max_future_skew_ns=1_000_000,
    )


def brackets(*, start=START, noise=None, count=5):
    if noise is None:
        noise = [0] * count
    result = []
    for i in range(count):
        before = start + 10_000_000 * i
        after = before + 1_000
        result.append(dict(
            monotonic_before_ns=before,
            realtime_ns=(before + after) // 2 + OFFSET + noise[i],
            monotonic_after_ns=after,
        ))
    return result


def build(policy, samples=None, *, previous=None, boot="pi-boot-a"):
    return make_host_timebase_bridge(
        samples if samples is not None else brackets(),
        policy=policy, boot_id=boot, previous_bridge=previous,
    )


def convert(policy, bridge, *, source=None, now=None, boot="pi-boot-a"):
    if source is None:
        source = bridge["last_sample_monotonic_ns"]
    if now is None:
        now = bridge["last_sample_monotonic_ns"] + 10_000_000
    return convert_monotonic_stamp(
        bridge, source_monotonic_ns=source, evaluation_monotonic_ns=now,
        boot_id=boot, policy=policy,
    )


def test_bridge_is_exact_integer_and_serializable(policy):
    b = build(policy)
    assert b["schema_version"] == BRIDGE_SCHEMA
    assert b["status"] == "DIAGNOSTIC_BRIDGE_READY"
    assert b["sample_count"] == 5
    assert b["offset_ns"] == OFFSET
    assert b["empirical_offset_envelope_ns"] == 500
    assert b["observed_offset_span_ns"] == 0
    assert b["max_bracket_observed_ns"] == 1_000
    assert json.loads(json.dumps(b, allow_nan=False)) == b
    assert not b["measurement_authorized"]
    assert not b["world_state_update_authorized"]
    assert not b["motor_control_authorized"]


def test_successful_conversion_diagnostic_only(policy):
    b = build(policy)
    result = convert(policy, b)
    assert result["status"] == "HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY"
    assert result["estimated_realtime_ns"] == b["last_sample_monotonic_ns"] + OFFSET
    assert result["empirical_offset_envelope_ns"] == 500
    assert result["physical_acquisition_time"] == "UNKNOWN"
    assert result["conversion_is_physical_acquisition_time"] is False
    assert not result["measurement_authorized"]
    assert not result["world_state_update_authorized"]
    assert not result["motor_control_authorized"]


def test_even_median_uses_integer_math_avoiding_float_roundoff(policy):
    samples = brackets(count=4, noise=[-2, 0, 2, 10])
    b = build(policy, samples)
    assert b["status"] == "DIAGNOSTIC_BRIDGE_READY"
    assert b["offset_ns"] == OFFSET + 1


def test_slight_offset_variation_is_observed_not_accepted_accuracy(policy):
    b = build(policy, brackets(noise=[0, 10, -10, 20, -20]))
    assert b["status"] == "DIAGNOSTIC_BRIDGE_READY"
    assert b["observed_offset_span_ns"] == 40
    assert b["empirical_offset_envelope_ns"] == 520


@pytest.mark.parametrize("variant,expected", [
    ("insufficient", "INVALID_BRACKETS"),
    ("wrong_type", "INVALID_BRACKETS"),
    ("no_field", "INVALID_BRACKETS"),
    ("float", "INVALID_BRACKETS"),
    ("negative", "INVALID_BRACKETS"),
    ("reverse", "INVALID_BRACKETS"),
    ("overlap", "MONOTONIC_SEQUENCE_INVALID"),
    ("rollback", "REALTIME_CLOCK_ROLLBACK"),
    ("wide", "BRACKET_TOO_WIDE"),
    ("gap", "SAMPLE_GAP_EXCEEDED"),
    ("step", "CLOCK_STEP_SUSPECTED"),
    ("drift", "OFFSET_SPAN_EXCEEDED"),
])
def test_invalid_samples_fail_closed(policy, variant, expected):
    s = brackets()
    if variant == "insufficient":
        s = s[:2]
    elif variant == "wrong_type":
        s = [1, 2, 3]
    elif variant == "no_field":
        del s[0]["realtime_ns"]
    elif variant == "float":
        s[0]["monotonic_before_ns"] = float(START)
    elif variant == "negative":
        s[0]["monotonic_before_ns"] = -1
    elif variant == "reverse":
        s[0]["monotonic_after_ns"] = s[0]["monotonic_before_ns"] - 1
    elif variant == "overlap":
        s[1]["monotonic_before_ns"] = s[0]["monotonic_after_ns"]
    elif variant == "rollback":
        s[1]["realtime_ns"] = s[0]["realtime_ns"] - 1
    elif variant == "wide":
        s[0]["monotonic_after_ns"] += 2_000
    elif variant == "gap":
        for i in range(1, len(s)):
            s[i]["monotonic_before_ns"] += 100_000_000 * i
            s[i]["monotonic_after_ns"] += 100_000_000 * i
            s[i]["realtime_ns"] += 100_000_000 * i
    elif variant == "step":
        s = brackets(noise=[0, 0, 50_000, 50_000, 50_000])
    elif variant == "drift":
        s = brackets(noise=[0, 4_000, 8_000, 12_000, 16_000])
    result = build(policy, s)
    assert result["status"] == expected
    assert result["offset_ns"] is None
    assert not result["motor_control_authorized"]


def test_boot_id_change_rejects_bridge_reuse(policy):
    b = build(policy)
    assert convert(policy, b, boot="pi-boot-b")["status"] == "BOOT_ID_MISMATCH"


def test_expiration_rejects_outdated_bridge(policy):
    b = build(policy)
    assert convert(policy, b, now=b["last_sample_monotonic_ns"] + 250_000_001)["status"] == "BRIDGE_EXPIRED"


def test_future_bridge_refused(policy):
    b = build(policy)
    assert convert(policy, b, now=b["last_sample_monotonic_ns"] - 1)["status"] == "BRIDGE_FROM_FUTURE"


def test_future_camera_stamp_refused(policy):
    b = build(policy)
    assert convert(policy, b,
        source=b["last_sample_monotonic_ns"] + 5_000_000,
        now=b["last_sample_monotonic_ns"],
    )["status"] == "STAMP_FROM_FUTURE"


def test_extrapolation_limit_refused(policy):
    b = build(policy)
    assert convert(policy, b,
        source=b["first_sample_monotonic_ns"] - 10_000_001,
    )["status"] == "STAMP_OUTSIDE_BRIDGE_WINDOW"


def test_boundaries_inclusive(policy):
    b = build(policy)
    result = convert(policy, b,
        source=b["first_sample_monotonic_ns"] - 10_000_000,
        now=b["last_sample_monotonic_ns"] + 250_000_000,
    )
    assert result["status"] == "HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY"


def test_prior_window_continuity_and_step(policy):
    first = build(policy)
    good = build(policy, brackets(start=START + 1_000_000_000), previous=first)
    assert good["status"] == "DIAGNOSTIC_BRIDGE_READY"
    bad = build(policy, brackets(start=START + 1_000_000_000, noise=[50_000]*5), previous=first)
    assert bad["status"] == "CLOCK_STEP_SUSPECTED"


def test_out_of_order_windows_and_boot_reset_refused(policy):
    first = build(policy)
    second = build(policy, brackets(start=START + 1_000_000_000), previous=first, boot="pi-boot-b")
    assert second["status"] == "BOOT_ID_CHANGED"
    overlap = build(policy, brackets(), previous=first)
    assert overlap["status"] == "BRIDGE_WINDOWS_OUT_OF_ORDER"


def test_invalid_previous_bridge_refused(policy):
    b = build(policy, previous={"status": "DIAGNOSTIC_BRIDGE_READY"})
    assert b["status"] == "INVALID_PREVIOUS_BRIDGE"


def test_bad_bridge_and_tampered_payload_refused(policy):
    b = build(policy)
    assert convert(policy, {"status": "NOT_READY"}, source=START, now=START + 1)["status"] == "BRIDGE_UNAVAILABLE"
    tampered = deepcopy(b)
    tampered["offset_ns"] = "nan"
    assert convert(policy, tampered)["status"] == "INVALID_BRIDGE_FIELDS"


@pytest.mark.parametrize("field,value", [
    ("max_age_ns", -1),
    ("max_offset_span_ns", True),
    ("max_sample_gap_ns", 1.5),
    ("max_extrapolation_ns", None),
])
def test_policy_limits_reject_invalid_types(field, value, policy):
    from dataclasses import asdict
    args = asdict(policy)
    args[field] = value
    with pytest.raises(ValueError):
        HostBridgeTestPolicy(**args)


def test_reproducibility_no_hidden_now(policy):
    samples = brackets()
    a, b = build(policy, samples), build(policy, deepcopy(samples))
    assert a == b
    result = convert(policy, a)
    assert result == convert(policy, b)
    assert not any("authorized" in x and bool(y) for x, y in result.items())


def test_changed_limits_same_policy_id_are_rejected(policy):
    from dataclasses import replace
    bridge = build(policy)
    changed = replace(policy, max_extrapolation_ns=11_000_000)
    assert convert(changed, bridge)["status"] == "POLICY_LIMITS_MISMATCH"
    later = build(
        changed, brackets(start=START + 1_000_000_000),
        previous=bridge,
    )
    assert later["status"] == "INVALID_PREVIOUS_BRIDGE"


def test_corrupt_previous_bridge_last_stamp_is_refused(policy):
    bridge = build(policy)
    bridge["last_sample_monotonic_ns"] = None
    later = build(
        policy, brackets(start=START + 1_000_000_000),
        previous=bridge,
    )
    assert later["status"] == "INVALID_PREVIOUS_BRIDGE"


def test_missing_boot_id_rejected(policy):
    with pytest.raises(ValueError):
        build(policy, boot="")
    with pytest.raises(ValueError):
        convert(policy, build(policy), boot="")
