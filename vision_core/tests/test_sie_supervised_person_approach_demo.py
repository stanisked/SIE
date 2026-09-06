from __future__ import annotations

import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from vision_core.person_approach.decision import PersonApproachDecisionEngine
from vision_core.person_approach.supervised_demo import (
    AWAITING_OPERATOR_CONFIRMATION,
    SupervisedPersonApproachDemo,
)


NOW = datetime(2026, 9, 6, 14, 0, tzinfo=timezone.utc)
FRAME = "rectified_left_optical_frame"
SESSION = "0123456789ABCDEF"


def cycle(index: int, *, status: str = "SUCCESS", x: float = .02, z: float = 2.0, age_s: float = 0.) -> dict:
    stamp = (NOW - timedelta(seconds=age_s, milliseconds=(4-index)*50)).isoformat()
    measurement = None
    if status == "SUCCESS":
        measurement = {
            "status": "SUCCESS", "measurement_id": f"measurement-{index}", "timestamp": stamp,
            "reference_frame": FRAME, "units": "m", "x_m": x, "y_m": 0., "z_m": z,
            "range_m": math.hypot(x, z), "confidence": .9,
        }
    return {
        "schema_version": "sie.person_depth_live_cycle.v1", "cycle_id": f"cycle-{index}",
        "captured_at_utc": stamp, "status": status,
        "person": {"status": "SINGLE_PERSON" if status == "SUCCESS" else status},
        "measurement": measurement,
    }


def demo(**kwargs) -> SupervisedPersonApproachDemo:
    return SupervisedPersonApproachDemo(
        boot_session_id=SESSION,
        decision_engine=PersonApproachDecisionEngine(now_utc=lambda: NOW),
        now_utc=lambda: NOW,
        **kwargs,
    )


def process(runner: SupervisedPersonApproachDemo, records: list[dict]) -> dict:
    for item in records:
        result = runner.process_cycle(item)
    return result


def test_target_reached_produces_no_action_record():
    result = process(demo(), [cycle(index, z=2.02) for index in range(5)])
    assert result["stage"] == "NO_ACTION"
    assert result["decision_status"] == "HOLD_TARGET_REACHED"
    assert result["planned_command"] is None
    assert result["network_performed"] is False and result["motor_command_performed"] is False
    assert len(result["evidence_window_summary"]) == 5
    assert all(set(entry) == {"cycle_id", "cycle_status", "person_status", "measurement_status"} for entry in result["evidence_window_summary"])


def test_advance_produces_one_plan_then_awaits_confirmation():
    runner = demo()
    planned = process(runner, [cycle(index, z=2.4) for index in range(5)])
    assert planned["stage"] == AWAITING_OPERATOR_CONFIRMATION
    assert planned["planned_command"]["result"] == "PLANNED_BOUNDED_COMMAND"
    assert planned["planned_command"]["endpoint"] == "/move-forward"
    assert runner.stage == AWAITING_OPERATOR_CONFIRMATION
    waiting = runner.process_cycle(cycle(5, z=2.4))
    assert waiting["stage"] == AWAITING_OPERATOR_CONFIRMATION
    assert waiting["planned_command"] is None
    assert waiting["block_reason"] == "OPERATOR_CONFIRMATION_REQUIRED"


def test_turn_is_blocked_by_demo_v0_policy_before_planner():
    result = process(demo(), [cycle(index, x=.4, z=2.0) for index in range(5)])
    assert result["stage"] == "NO_ACTION"
    assert result["decision_status"] == "TURN_RIGHT"
    assert result["planned_command"] is None
    assert result["block_reason"] == "TURN_NOT_ENABLED_IN_SUPERVISED_DEMO_V0"


@pytest.mark.parametrize(
    ("records", "status"),
    [
        ([cycle(index, status="PERSON_LOST") for index in range(5)], "BLOCKED_PERSON_LOST"),
        ([cycle(index, status="MULTIPLE_PERSONS") for index in range(5)], "BLOCKED_MULTIPLE_PERSONS"),
        ([cycle(index, z=2.4, age_s=2.) for index in range(5)], "BLOCKED_STALE"),
    ],
)
def test_lost_multiple_and_stale_cases_are_no_action(records: list[dict], status: str):
    result = process(demo(), records)
    assert result["stage"] == "NO_ACTION"
    assert result["decision_status"] == status
    assert result["planned_command"] is None
    assert result["reobservation_required"] is True


def test_partial_progress_requires_reobservation_and_blocks_plan():
    result = process(
        demo(previous_terminal_motion_outcome={"command_state": "PARTIAL_PROGRESS", "reobserve_required": True}),
        [cycle(index, z=2.4) for index in range(5)],
    )
    assert result["stage"] == "NO_ACTION"
    assert result["block_reason"] == "REOBSERVE_REQUIRED_AFTER_PARTIAL_PROGRESS"
    assert result["reobservation_required"] is True


def test_blocked_depth_summary_shows_fewer_than_four_success_cycles():
    records = [cycle(0, status="DEPTH_UNAVAILABLE"), cycle(1, status="PERSON_LOST"),
               cycle(2, z=2.4), cycle(3, z=2.4), cycle(4, z=2.4)]
    result = process(demo(), records)
    assert result["decision_status"] == "BLOCKED_DEPTH_UNAVAILABLE"
    summary = result["evidence_window_summary"]
    assert len(summary) == 5
    assert [entry["cycle_status"] for entry in summary] == [
        "DEPTH_UNAVAILABLE", "PERSON_LOST", "SUCCESS", "SUCCESS", "SUCCESS",
    ]
    assert sum(entry["cycle_status"] == "SUCCESS" for entry in summary) == 3
    assert all(set(entry) == {"cycle_id", "cycle_status", "person_status", "measurement_status"} for entry in summary)


def test_boot_session_is_explicit_and_marked_unverified_and_output_is_json_safe():
    result = process(demo(), [cycle(index, z=2.4) for index in range(5)])
    assert result["boot_session_id"] == SESSION
    assert result["boot_session_freshness_verified"] is False
    assert result["reference_frame"] == FRAME and result["units"] == "m"
    assert len(result["evidence_window_summary"]) == 5
    assert all(set(entry) == {"cycle_id", "cycle_status", "person_status", "measurement_status"} for entry in result["evidence_window_summary"])
    json.dumps(result, allow_nan=False)


def test_demo_source_has_no_network_clients_or_execution_calls():
    paths = [
        Path("vision_core/person_approach/supervised_demo.py"),
        Path("vision_core/tools/run_sie_supervised_person_approach_demo.py"),
    ]
    source = "\n".join(path.read_text() for path in paths).lower()
    forbidden = (
        "requests", "urllib", "http.client", "socket", "wifi", "serial",
        "server.send", "subprocess", "os.system", "http://", "https://",
    )
    assert all(token not in source for token in forbidden)
