from __future__ import annotations

from datetime import datetime, timedelta, timezone
import math
from pathlib import Path

from vision_core.person_approach.decision import PersonApproachDecisionEngine


NOW = datetime(2026, 9, 2, 12, 0, 0, tzinfo=timezone.utc)


def record(index, *, status="SUCCESS", x=.02, z=2.0, frame="rectified_left_optical_frame"):
    timestamp = (NOW - timedelta(milliseconds=(5-index)*50)).isoformat()
    measurement = None if status != "SUCCESS" else {
        "status": "SUCCESS", "measurement_id": f"m-{index}", "timestamp": timestamp,
        "reference_frame": frame, "x_m": x, "z_m": z,
        "range_m": math.hypot(x, z),
    }
    return {"status": status, "measurement": measurement, "captured_at_utc": timestamp}


def engine(): return PersonApproachDecisionEngine(now_utc=lambda: NOW)


def ingest(values):
    value = engine()
    for item in values: result = value.ingest(item)
    return result


def test_warmup_and_stable_target_hold_with_explicit_units():
    assert ingest([record(i) for i in range(4)]).status == "BLOCKED_WARMUP"
    result = ingest([record(i, x=.03, z=2.02) for i in range(5)])
    assert result.status == "HOLD_TARGET_REACHED" and result.turn_angle_deg is None and result.forward_step_m is None
    assert result.units == "m" and result.turn_angle_units == "deg" and len(result.source_measurement_ids) == 5


def test_turn_left_right_and_bounded_advance():
    assert ingest([record(i, x=.4) for i in range(5)]).status == "TURN_RIGHT"
    left = ingest([record(i, x=-.4) for i in range(5)])
    assert left.status == "TURN_LEFT" and abs(left.turn_angle_deg) <= 10.
    advance = ingest([record(i, x=.02, z=2.4) for i in range(5)])
    assert advance.status == "ADVANCE" and advance.forward_step_m == .1 and advance.turn_angle_deg is None


def test_multiple_and_latest_lost_are_fail_closed():
    assert ingest([record(i) for i in range(4)] + [record(5, status="MULTIPLE_PERSONS")]).status == "BLOCKED_MULTIPLE_PERSONS"
    assert ingest([record(i) for i in range(4)] + [record(5, status="PERSON_LOST")]).status == "BLOCKED_PERSON_LOST"


def test_stale_unstable_and_too_close_never_propose_motion():
    stale_records = [record(i) for i in range(5)]
    for index, item in enumerate(stale_records): item["measurement"]["timestamp"] = (NOW-timedelta(seconds=2)+timedelta(milliseconds=index)).isoformat()
    assert ingest(stale_records).status == "BLOCKED_STALE"
    unstable = ingest([record(i, x=.2*i, z=2.0) for i in range(5)])
    assert unstable.status == "BLOCKED_UNSTABLE" and unstable.forward_step_m is None
    close = ingest([record(i, z=1.7) for i in range(5)])
    assert close.status == "HOLD_TARGET_REACHED" and close.detail == "TOO_CLOSE_NO_REVERSE" and close.forward_step_m is None


def test_nonfinite_and_frame_mismatch_block_without_actuation():
    bad = record(5); bad["measurement"]["x_m"] = float("nan")
    assert ingest([record(i) for i in range(4)] + [bad]).status == "BLOCKED_UNSTABLE"
    frame = ingest([record(i, frame="other" if i == 3 else "rectified_left_optical_frame") for i in range(5)])
    assert frame.status == "BLOCKED_UNSTABLE" and frame.turn_angle_deg is None and frame.forward_step_m is None
    source = Path("vision_core/tools/run_person_approach_dry_run.py").read_text()
    assert "ESP32" not in source and "motor" not in source.lower() and "ROS" not in source
