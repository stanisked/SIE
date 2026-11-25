"""Diagnostic-only LD19 adapter contract tests (no ROS runtime required)."""

from math import inf, nan
from types import SimpleNamespace

import pytest

from sie_ros2.ld19_diagnostic_evidence import (
    DIAGNOSTIC_SCHEMA,
    make_ld19_diagnostic_record,
)


def scan(**overrides):
    msg = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="ld19_sensor",
            stamp=SimpleNamespace(sec=1795611900, nanosec=123456789),
        ),
        angle_min=0.0,
        angle_max=1.5,
        angle_increment=0.5,
        time_increment=0.0002,
        scan_time=0.1,
        range_min=0.02,
        range_max=25.0,
        ranges=[inf, 0.815, 0.823, nan],
        intensities=[0.0, 1.0, 2.0, 0.0],
    )
    for key, value in overrides.items():
        setattr(msg, key, value)
    return msg


def convert(msg=None, received="2026-11-25T16:00:00+00:00"):
    return make_ld19_diagnostic_record(
        msg if msg is not None else scan(), received_at_utc=received
    )


def test_raw_evidence_is_preserved_and_strict_json_compatible():
    import json

    record = convert()
    assert record["schema_version"] == DIAGNOSTIC_SCHEMA
    assert record["reference_frame"] == "ld19_sensor"
    assert record["timestamp"].endswith(".123456789Z")
    assert record["quality"]["valid_samples"] == 2
    assert record["quality"]["total_samples"] == 4
    assert record["quality"]["valid_fraction"] == 0.5
    assert record["payload"]["scan"]["ranges_m"] == [
        "+Infinity", 0.815, 0.823, "NaN"
    ]
    assert record["payload"]["distance_unit"] == "m"
    assert record["payload"]["angle_unit"] == "rad"
    json.dumps(record, allow_nan=False)


def test_digest_is_deterministic_and_receive_time_is_not_source_time():
    a = convert(received="2026-11-25T16:00:00+00:00")
    b = convert(received="2026-11-25T16:00:03+00:00")
    assert a["evidence"]["raw_scan_sha256"] == b["evidence"]["raw_scan_sha256"]
    assert a["observation_id"] == b["observation_id"]
    assert a["timestamp"] != a["received_at_utc"]


def test_raw_scan_cannot_authorize_measurement_world_state_or_motors():
    record = convert()
    assert record["status"] == "DIAGNOSTIC_ONLY_NOT_ADMITTED"
    assert record["quality"]["assessment"] == "NOT_QUALIFIED"
    assert record["confidence"] == 0.0
    assert record["admission"] == {
        "measurement_authorized": False,
        "world_state_update_authorized": False,
        "motor_control_authorized": False,
    }


def test_rejects_missing_frame_and_unusable_stamps():
    with pytest.raises(ValueError, match="reference_frame"):
        convert(scan(header=SimpleNamespace(
            frame_id="base_link",
            stamp=SimpleNamespace(sec=1795611900, nanosec=0)
        )))
    with pytest.raises(ValueError, match="source stamp"):
        convert(scan(header=SimpleNamespace(
            frame_id="ld19_sensor",
            stamp=SimpleNamespace(sec=0, nanosec=0)
        )))


def test_rejects_invalid_scan_geometry_or_array_lengths():
    with pytest.raises(ValueError, match="angular"):
        convert(scan(angle_increment=0.0))
    with pytest.raises(ValueError, match="sample count"):
        convert(scan(ranges=[]))
    with pytest.raises(ValueError, match="intensity count"):
        convert(scan(intensities=[1.0]))
    with pytest.raises(ValueError, match="timing"):
        convert(scan(range_min=25.0, range_max=0.02))


def test_rejects_unqualified_received_timestamp():
    with pytest.raises(ValueError, match="timezone"):
        convert(received="2026-11-25T16:00:00")
