from __future__ import annotations

import json
from datetime import datetime, timezone

from vision_core.person_approach.supervised_acquire_range import coordinate_supervised_acquire_range


NOW = datetime(2026, 9, 7, 14, 0, tzinfo=timezone.utc)
STAMP = NOW.isoformat()


def temporal(*, result: str, stage: str, latest: str = "SINGLE_PERSON", frame: str = "ar0234_image_frame") -> dict:
    planned = {"method": "POST", "endpoint": "/turn-left", "angle_deg": 4} if result == "PLANNED_TURN" else None
    return {
        "schema_version": "sie.temporal_yaw_alignment.v1", "result": result, "stage": stage,
        "timestamp": STAMP, "reference_frame": frame, "units": "px", "used_evidence_ids": ["e-1", "e-2", "e-3", "e-4", "e-5"],
        "valid_single_person_count": 5, "person_lost_count": 0, "latest_person_status": latest,
        "robust_median_image_offset_px": -120.0, "mad_image_offset_px": 2.0, "center_tolerance_px": 40.0,
        "planned_command": planned, "reobserve_required": result == "PLANNED_TURN",
    }


def depth() -> dict:
    return {
        "schema_version": "sie.person_depth_live_cycle.v1", "status": "SUCCESS", "captured_at_utc": STAMP,
        "person": {"status": "SINGLE_PERSON"},
        "measurement": {"status": "SUCCESS", "measurement_id": "m-1", "timestamp": STAMP,
                        "reference_frame": "rectified_left_optical_frame", "units": "m",
                        "x_m": 0.0, "y_m": 0.0, "z_m": 2.2, "range_m": 2.2},
    }


def evaluate(value: object) -> dict:
    return coordinate_supervised_acquire_range(value, now_utc=lambda: NOW)


def test_planned_turn_awaits_operator_and_reobservation():
    result = evaluate({"temporal_alignment": temporal(result="PLANNED_TURN", stage="AWAIT_REOBSERVATION")})
    assert result["stage"] == "AWAIT_OPERATOR_TURN_AND_REOBSERVATION"
    assert result["planned_turn"] == {"method": "POST", "endpoint": "/turn-left", "angle_deg": 4}
    assert result["network_performed"] is False and result["motor_command_performed"] is False


def test_centered_without_depth_requires_range_acquisition():
    result = evaluate({"temporal_alignment": temporal(result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED")})
    assert result["stage"] == "RANGE_ACQUISITION_REQUIRED"
    assert result["source_measurement_ids"] == []


def test_centered_with_valid_depth_requires_existing_decision_layer():
    result = evaluate({"temporal_alignment": temporal(result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED"), "person_depth": depth()})
    assert result["stage"] == "DEPTH_APPROACH_DECISION_REQUIRED"
    assert result["source_measurement_ids"] == ["m-1"]
    assert result["depth_reference_frame"] == "rectified_left_optical_frame" and result["depth_units"] == "m"


def test_invalid_multiple_or_mismatched_inputs_block_json_safely():
    cases = [
        {"temporal_alignment": temporal(result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED", latest="MULTIPLE_PERSONS")},
        {"temporal_alignment": temporal(result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED", frame="other")},
        {"temporal_alignment": temporal(result="NO_TURN_CENTERED", stage="NO_TURN_CENTERED"), "person_depth": {"invalid": True}},
    ]
    for value in cases:
        result = evaluate(value)
        assert result["stage"] == result["result"] == "BLOCKED_NO_ACTION"
        assert result["planned_turn"] is None
        json.dumps(result, allow_nan=False)
