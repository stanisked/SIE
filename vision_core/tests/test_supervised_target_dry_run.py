from __future__ import annotations

from datetime import datetime, timedelta, timezone

from vision_core.person_approach.supervised_target_dry_run import SupervisedTargetDryRun


NOW = datetime(2026, 9, 7, 15, 0, tzinfo=timezone.utc)


def cycle(index: int, *, center_x: float = 960.0, x_m: float = 0.02, z_m: float = 2.24, status: str = "SUCCESS", person_status: str = "SINGLE_PERSON") -> dict:
    timestamp = (NOW - timedelta(milliseconds=(5 - index) * 50)).isoformat()
    measurement = None if status != "SUCCESS" else {
        "status": "SUCCESS", "measurement_id": f"m-{index}", "person_evidence_id": f"e-{index}",
        "timestamp": timestamp, "reference_frame": "rectified_left_optical_frame", "units": "m",
        "x_m": x_m, "y_m": 0.0, "z_m": z_m, "range_m": (x_m * x_m + z_m * z_m) ** .5,
    }
    return {
        "schema_version": "sie.person_depth_live_cycle.v1", "cycle_id": f"cycle-{index}",
        "captured_at_utc": timestamp, "status": status,
        "person": {"status": person_status, "bbox_xyxy_px": [center_x - 50, 300, center_x + 50, 700]},
        "measurement": measurement,
    }


class FakeRuntime:
    def __init__(self, values: list[dict]) -> None:
        self.values = values
        self.calls = 0

    def cycle(self, _cycle_id: str) -> dict:
        value = self.values[self.calls]
        self.calls += 1
        return value


def runner(values: list[dict]) -> SupervisedTargetDryRun:
    return SupervisedTargetDryRun(live_runtime=FakeRuntime(values), optical_axis_cx_px=960.0, center_tolerance_px=40.0, now_utc=lambda: NOW)


def test_off_center_shared_window_awaits_operator_turn():
    result = runner([cycle(index, center_x=1250, x_m=.3) for index in range(1, 6)]).run_live_window()
    assert result["stage"] == "AWAIT_OPERATOR_TURN_AND_REOBSERVATION"
    assert result["planned_dry_run"] == {"method": "POST", "endpoint": "/turn-right", "angle_deg": 4}
    assert result["entity_type"] == "person" and result["reobserve_required"] is True
    assert result["alignment_summary"]["planned_turn_endpoint"] == "/turn-right"
    assert result["network_performed"] is False and result["motor_command_performed"] is False


def test_unstable_temporal_window_exposes_summary_and_stays_blocked():
    result = runner([cycle(index, center_x=960.0 + index * 100.0) for index in range(1, 6)]).run_live_window()
    summary = result["alignment_summary"]
    assert result["stage"] == result["result"] == "BLOCKED_NO_ACTION"
    assert summary["temporal_result"] == "BLOCKED_NO_TURN"
    assert summary["temporal_block_reason"] == "UNSTABLE_EVIDENCE_WINDOW"
    assert summary["valid_single_person_count"] == 5
    assert summary["mad_image_offset_px"] == 100.0
    assert summary["planned_turn_endpoint"] is None
    assert result["network_performed"] is False and result["motor_command_performed"] is False


def test_centered_valid_depth_beyond_target_awaits_operator_advance():
    result = runner([cycle(index, z_m=2.23577, x_m=.02854) for index in range(1, 6)]).run_live_window()
    assert result["stage"] == "AWAIT_OPERATOR_ADVANCE_AND_REOBSERVATION"
    assert result["planned_dry_run"] == {"method": "POST", "endpoint": "/move-forward", "distance_m": .1}
    assert result["source_measurement_ids"] == ["m-1", "m-2", "m-3", "m-4", "m-5"]


def test_centered_valid_depth_at_target_holds():
    result = runner([cycle(index, z_m=2.02) for index in range(1, 6)]).run_live_window()
    assert result["stage"] == "HOLD_TARGET_REACHED"
    assert result["planned_dry_run"] is None


def test_lost_multiple_or_invalid_provenance_blocks():
    for values in (
        [cycle(index, person_status="PERSON_LOST", status="PERSON_LOST") for index in range(1, 6)],
        [cycle(index, person_status="MULTIPLE_PERSONS", status="MULTIPLE_PERSONS") for index in range(1, 6)],
        [cycle(index) for index in range(1, 5)] + [{"invalid": True}],
    ):
        result = runner(values).run_live_window()
        assert result["stage"] == result["result"] == "BLOCKED_NO_ACTION"
