from __future__ import annotations

from datetime import datetime, timedelta, timezone

from vision_core.person_approach.spatial_navigation import (
    evaluate_person_navigation_window,
    navigation_decision,
)
from vision_core.tools.run_sie_static_target_mvp import fresh_supervised_execution_plan


NOW = datetime(2026, 9, 17, 12, 0, tzinfo=timezone.utc)


def _window(*, x_m: float, z_m: float, yolo: str = "SINGLE_TARGET") -> list[dict]:
    rows = []
    for index in range(5):
        timestamp = (NOW - timedelta(milliseconds=40 - index)).isoformat()
        rows.append({
            "cycle_id": f"cycle-{index}", "captured_at_utc": timestamp,
            "status": "SUCCESS",
            "primary_person_observation": {"target_status": yolo},
            "measurement": {
                "status": "SUCCESS", "measurement_id": f"measurement-{index}",
                "timestamp": timestamp, "reference_frame": "rectified_left_optical_frame",
                "units": "m", "x_m": x_m, "z_m": z_m,
                "range_m": (x_m * x_m + z_m * z_m) ** 0.5,
            },
        })
    return rows


def _evaluate(**values: object) -> dict:
    return evaluate_person_navigation_window(
        now_utc=lambda: NOW, safe_distance_m=1.0, bearing_deadband_deg=2.0,
        **values,
    )


def test_left_bearing_plans_left_turn_then_requires_reobservation() -> None:
    result = _evaluate(cycles=_window(x_m=-0.5, z_m=2.0))
    assert result["result"] == "TURN_REQUIRED"
    assert result["action"] == {"status": "TURN_LEFT", "angle_deg": 10.0}
    assert result["image_center_used_as_gate"] is False


def test_right_bearing_plans_right_turn() -> None:
    result = _evaluate(cycles=_window(x_m=0.5, z_m=2.0))
    assert result["result"] == "TURN_REQUIRED"
    assert result["action"]["status"] == "TURN_RIGHT"


def test_centered_person_plans_fixed_bounded_forward_step() -> None:
    result = _evaluate(cycles=_window(x_m=0.0, z_m=2.0))
    assert result["result"] == "FORWARD_REQUIRED"
    assert result["action"] == {"status": "ADVANCE", "distance_m": 0.10}
    decision = navigation_decision(result, sequence=1, timestamp=NOW.isoformat())
    assert decision["forward_step_m"] == 0.10


def test_far_windows_continue_and_arrive_only_at_safe_distance() -> None:
    far = _evaluate(cycles=_window(x_m=0.0, z_m=3.5))
    nearer = _evaluate(cycles=_window(x_m=0.0, z_m=1.5))
    arrived = _evaluate(cycles=_window(x_m=0.0, z_m=1.0))
    assert far["result"] == nearer["result"] == "FORWARD_REQUIRED"
    assert arrived["result"] == "ARRIVED" and arrived["action"] is None


def test_invalid_post_action_window_blocks_without_action() -> None:
    invalid = _window(x_m=0.0, z_m=2.0)
    invalid[-1]["status"] = "BLOCKED_QUALITY"
    result = _evaluate(cycles=invalid)
    assert result["result"] == "BLOCKED"
    assert result.get("action") is None


def test_each_navigation_action_receives_a_new_execution_command_id() -> None:
    plan = {
        "result": "PLANNED_BOUNDED_COMMAND", "method": "POST", "endpoint": "/turn-left",
        "command_id": "pa-deterministic", "network_performed": False,
        "query": {"boot_session_id": "0123456789ABCDEF", "command_id": "pa-deterministic", "angle_deg": "4"},
    }
    first, second = fresh_supervised_execution_plan(plan), fresh_supervised_execution_plan(plan)
    assert first["command_id"] != second["command_id"]
    assert first["query"]["command_id"] == first["command_id"]
    assert second["query"]["command_id"] == second["command_id"]


def test_no_fixed_start_range_or_image_center_gate() -> None:
    result = evaluate_person_navigation_window(
        _window(x_m=0.01, z_m=8.0), safe_distance_m=1.0,
        bearing_deadband_deg=2.0, now_utc=lambda: NOW,
    )
    assert result["result"] == "FORWARD_REQUIRED"
    assert result["image_center_used_as_gate"] is False
