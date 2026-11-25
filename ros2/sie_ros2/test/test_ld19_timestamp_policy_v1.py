"""Synthetic tests for LD19 timestamp policy draft: no ROS or hardware."""

from copy import deepcopy
from datetime import datetime, timezone
import json

import pytest

from sie_ros2.ld19_timestamp_policy_v1 import (
    SCHEMA,
    TimestampPolicyV1,
    evaluate_ld19_timestamp,
)


BASE_NS = 1_795_612_952_000_000_000


def _as_utc_iso(ns):
    sec, nsec = divmod(ns, 1_000_000_000)
    dt = datetime.fromtimestamp(sec, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + f".{nsec:09d}Z"


def _received_iso(ns):
    # The existing diagnostic node records Python datetime microseconds.
    sec, nsec = divmod(ns, 1_000_000_000)
    dt = datetime.fromtimestamp(sec, tz=timezone.utc)
    return dt.replace(microsecond=nsec // 1000).isoformat()


def record(*, source_ns=BASE_NS, receive_offset_ns=500_000):
    sec, nanosec = divmod(source_ns, 1_000_000_000)
    digest = "a" * 64
    return {
        "schema_version": "sie.ld19.diagnostic_observation_candidate.v0",
        "status": "DIAGNOSTIC_ONLY_NOT_ADMITTED",
        "reference_frame": "ld19_sensor",
        "source_topic": "/sie/lidar/scan_raw",
        "timestamp": _as_utc_iso(source_ns),
        "received_at_utc": _received_iso(source_ns + receive_offset_ns),
        "evidence": {"raw_scan_sha256": digest},
        "evidence_ids": [f"ld19-raw:{digest}"],
        "admission": {
            "measurement_authorized": False,
            "world_state_update_authorized": False,
            "motor_control_authorized": False,
        },
        "payload": {
            "scan": {
                "header": {
                    "stamp_sec": sec,
                    "stamp_nanosec": nanosec,
                    "frame_id": "ld19_sensor",
                },
                "ranges_m": [0.815, 0.823],
            },
        },
    }


@pytest.fixture
def policy():
    # Test fixtures, NOT approved runtime thresholds.
    return TimestampPolicyV1(
        policy_id="test-only-250ms-host-receipt",
        max_receive_age_ns=250_000_000,
        max_future_skew_ns=5_000_000,
        max_conversion_to_receive_ns=20_000_000,
    )


def check(record_value, policy, *, now_ns=None, last=None, verified=True):
    if now_ns is None:
        now_ns = BASE_NS + 10_000_000
    return evaluate_ld19_timestamp(
        record_value,
        evaluation_time_utc_ns=now_ns,
        policy=policy,
        previous_source_stamp_ns=last,
        same_host_clock_domain_verified=verified,
    )


def test_host_window_pass_does_not_admit_anything(policy):
    result = check(record(), policy)
    assert result["schema_version"] == SCHEMA
    assert result["status"] == "HOST_TIMING_WINDOW_PASS_DIAGNOSTIC_ONLY"
    assert result["diagnostic_time_window_passed"] is True
    assert result["source_stamp_meaning"] == "ROS_SCAN_CONVERSION_TIME_NOT_ACQUISITION"
    assert result["physical_acquisition_time"] == "UNKNOWN"
    assert result["conversion_to_receive_ns"] == 500_000
    assert result["receive_age_ns"] == 9_500_000
    assert result["next_accepted_source_stamp_ns"] == BASE_NS
    assert not result["measurement_authorized"]
    assert not result["world_state_update_authorized"]
    assert not result["motor_control_authorized"]
    assert json.loads(json.dumps(result, allow_nan=False)) == result


def test_unverified_clock_domain_never_passes(policy):
    result = check(record(), policy, verified=False)
    assert result["status"] == "CLOCK_DOMAIN_UNVERIFIED"
    assert result["diagnostic_time_window_passed"] is False
    assert result["conversion_to_receive_ns"] is None
    assert result["next_accepted_source_stamp_ns"] is None


def test_expired_host_receipt_is_stale(policy):
    r = record()
    result = check(r, policy, now_ns=BASE_NS + 500_500_000)
    assert result["status"] == "STALE_RECEIPT"
    assert result["receive_age_ns"] == 500_000_000
    assert result["next_accepted_source_stamp_ns"] is None


def test_future_host_receipt_is_rejected(policy):
    r = record()
    result = check(r, policy, now_ns=BASE_NS - 10_000_000)
    assert result["status"] == "RECEIVE_TIME_IN_FUTURE"
    assert result["diagnostic_time_window_passed"] is False


def test_duplicate_and_out_of_order_are_rejected(policy):
    r = record()
    duplicate = check(r, policy, last=BASE_NS)
    assert duplicate["status"] == "DUPLICATE_SOURCE_STAMP"
    assert duplicate["next_accepted_source_stamp_ns"] == BASE_NS
    previous = BASE_NS + 100_000_000
    reordered = check(r, policy, last=previous)
    assert reordered["status"] == "OUT_OF_ORDER_SOURCE_STAMP"
    assert reordered["next_accepted_source_stamp_ns"] == previous


def test_source_stamp_ahead_of_receipt_is_not_rewritten(policy):
    r = record(receive_offset_ns=-20_000_000)
    result = check(r, policy)
    assert result["status"] == "ROS_STAMP_AHEAD_OF_RECEIPT"
    assert result["conversion_to_receive_ns"] == -20_000_000
    assert result["source_stamp_ns"] == BASE_NS


def test_conversion_to_receive_delay_limit_is_host_only(policy):
    r = record(receive_offset_ns=40_000_000)
    result = check(r, policy, now_ns=BASE_NS + 50_000_000)
    assert result["status"] == "CONVERSION_TO_RECEIVE_TOO_OLD"
    assert result["conversion_to_receive_ns"] == 40_000_000
    assert result["physical_acquisition_time"] == "UNKNOWN"


def test_policy_boundaries_are_inclusive(policy):
    r = record(receive_offset_ns=20_000_000)
    result = check(r, policy, now_ns=BASE_NS + 270_000_000)
    assert result["status"] == "HOST_TIMING_WINDOW_PASS_DIAGNOSTIC_ONLY"
    assert result["receive_age_ns"] == 250_000_000


@pytest.mark.parametrize(
    "change",
    [
        lambda r: r.update(reference_frame="base_link"),
        lambda r: r["payload"]["scan"]["header"].update(frame_id="base_link"),
        lambda r: r.update(source_topic="/unexpected"),
        lambda r: r.update(timestamp="2026-11-25T00:00:00Z"),
        lambda r: r.update(received_at_utc="2026-11-25T00:00:00"),
        lambda r: r.update(schema_version="unknown"),
        lambda r: r["admission"].update(motor_control_authorized=True),
        lambda r: r["evidence"].update(raw_scan_sha256="broken"),
        lambda r: r.update(evidence_ids=["wrong"]),
        lambda r: r["payload"]["scan"]["header"].update(stamp_nanosec=1_000_000_000),
    ],
)
def test_invalid_metadata_fails_closed(policy, change):
    r = record()
    change(r)
    result = check(r, policy)
    assert result["status"] == "INVALID_SOURCE"
    assert result["next_accepted_source_stamp_ns"] is None
    assert result["diagnostic_time_window_passed"] is False


def test_clock_discontinuity_requires_new_review(policy):
    r1 = record()
    first = check(r1, policy)
    next_ns = BASE_NS + 100_000_000
    r2 = record(source_ns=next_ns)
    second = check(
        r2,
        policy,
        last=first["next_accepted_source_stamp_ns"],
        now_ns=next_ns + 10_000_000,
    )
    assert second["status"] == "HOST_TIMING_WINDOW_PASS_DIAGNOSTIC_ONLY"
    rollback = check(
        r1,
        policy,
        last=second["next_accepted_source_stamp_ns"],
        now_ns=next_ns + 20_000_000,
    )
    assert rollback["status"] == "OUT_OF_ORDER_SOURCE_STAMP"
    assert rollback["next_accepted_source_stamp_ns"] == next_ns


def test_reproducible_and_no_implicit_now(policy):
    r = record()
    kwargs = dict(
        evaluation_time_utc_ns=BASE_NS + 50_000_000,
        policy=policy,
        same_host_clock_domain_verified=True,
    )
    a = evaluate_ld19_timestamp(r, **kwargs)
    b = evaluate_ld19_timestamp(deepcopy(r), **kwargs)
    assert a == b
    assert r["timestamp"] == _as_utc_iso(BASE_NS)


@pytest.mark.parametrize(
    "limits",
    [
        dict(max_receive_age_ns=-1),
        dict(max_future_skew_ns=-1),
        dict(max_conversion_to_receive_ns=True),
    ],
)
def test_unapproved_or_invalid_policy_parameters_are_not_accepted(limits):
    params = dict(
        policy_id="test",
        max_receive_age_ns=100_000_000,
        max_future_skew_ns=5_000_000,
        max_conversion_to_receive_ns=20_000_000,
    )
    params.update(limits)
    with pytest.raises(ValueError):
        TimestampPolicyV1(**params)


def test_invalid_evaluation_context_is_rejected(policy):
    with pytest.raises(ValueError):
        check(record(), policy, now_ns=0)
    with pytest.raises(ValueError):
        evaluate_ld19_timestamp(
            record(),
            policy=policy,
            evaluation_time_utc_ns=BASE_NS + 1_000_000,
            same_host_clock_domain_verified="yes",
        )
