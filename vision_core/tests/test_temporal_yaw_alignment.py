from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from vision_core.person_approach.temporal_yaw_alignment import TemporalYawAlignmentPlanner, evaluate_temporal_yaw_alignment


NOW = datetime(2026, 9, 7, 12, 0, tzinfo=timezone.utc)


def window(offsets: list[float], *, status: str = "SINGLE_PERSON") -> list[dict]:
    return [
        {
            "evidence_id": f"evidence-{index}",
            "timestamp": f"2026-09-07T12:00:0{index}+00:00",
            "reference_frame": "ar0234_image_frame",
            "units": "px",
            "person_status": status,
            "image_offset_px": offset,
            "center_tolerance_px": 40,
        }
        for index, offset in enumerate(offsets)
    ]


def evaluate(observations: object) -> dict:
    return evaluate_temporal_yaw_alignment(observations, now_utc=lambda: NOW)


def test_stable_image_right_plans_only_turn_right():
    result = evaluate(window([180, 175, 185, 180, 178]))
    assert result["result"] == "PLANNED_TURN"
    assert result["stage"] == "AWAIT_REOBSERVATION"
    assert result["planned_command"] == {"method": "POST", "endpoint": "/turn-right", "angle_deg": 4}
    assert result["reobserve_required"] is True


def test_stable_image_left_plans_only_turn_left():
    result = evaluate(window([-180, -175, -185, -180]))
    assert result["planned_command"] == {"method": "POST", "endpoint": "/turn-left", "angle_deg": 4}


def test_centered_window_has_no_turn():
    result = evaluate(window([-10, 0, 10, -5, 5]))
    assert result["result"] == result["stage"] == "NO_TURN_CENTERED"
    assert result["planned_command"] is None


def test_far_field_status_and_bbox_fields_are_accepted():
    observations = window([0, 0, 0, 0])
    for observation in observations:
        observation.pop("person_status")
        observation.pop("image_offset_px")
        observation["status"] = "SINGLE_PERSON"
        observation["bbox_xyxy_px"] = [1100, 300, 1200, 700]
        observation["optical_axis_cx_px"] = 960
    result = evaluate(observations)
    assert result["planned_command"] == {"method": "POST", "endpoint": "/turn-right", "angle_deg": 4}


def test_invalid_lost_multiple_and_unstable_windows_block():
    cases = [
        window([100, 100, 100, 100], status="PERSON_LOST"),
        window([100, 100, 100, 100], status="MULTIPLE_PERSONS"),
        window([100, -100, 100, -100]),
    ]
    for observations in cases:
        result = evaluate(observations)
        assert result["result"] == result["stage"] == "BLOCKED_NO_TURN"
        assert result["planned_command"] is None
        assert result["reobserve_required"] is True

    too_short = evaluate(window([100, 100, 100]))
    assert too_short["block_reason"] == "FEWER_THAN_FOUR_VALID_SINGLE_PERSON_OBSERVATIONS"


def test_same_planned_window_requires_new_observation():
    planner = TemporalYawAlignmentPlanner(now_utc=lambda: NOW)
    observations = window([100, 101, 99, 100])
    assert planner.evaluate_window(observations)["result"] == "PLANNED_TURN"
    repeat = planner.evaluate_window(observations)
    assert repeat["stage"] == "AWAIT_REOBSERVATION"
    assert repeat["block_reason"] == "EVIDENCE_WINDOW_ALREADY_PLANNED"


def test_json_safe_and_cli_source_has_no_network_or_hardware_clients():
    json.dumps(evaluate(window([100, 100, 100, 100])), allow_nan=False)
    source = Path("vision_core/tools/run_temporal_yaw_alignment_dry_run.py").read_text().lower()
    forbidden = ("requests", "urllib", "http.client", "socket", "wifi", "serial", "esp32", "motor", "subprocess")
    assert all(token not in source for token in forbidden)
