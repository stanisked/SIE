from __future__ import annotations

from datetime import datetime, timedelta, timezone

from vision_core.person_approach.metric_first_target_supervisor import MetricFirstTargetSupervisor


NOW = datetime(2026, 9, 7, 16, 0, tzinfo=timezone.utc)


def cycle(index: int, *, center_x: float, z_m: float = 2.25, x_m: float = .02, with_metric: bool = True) -> dict:
    timestamp = (NOW - timedelta(milliseconds=(5 - index) * 50)).isoformat()
    measurement = None if not with_metric else {
        "status": "SUCCESS", "measurement_id": f"m-{index}", "person_evidence_id": f"e-{index}",
        "timestamp": timestamp, "reference_frame": "rectified_left_optical_frame", "units": "m",
        "x_m": x_m, "y_m": 0.0, "z_m": z_m, "range_m": (x_m * x_m + z_m * z_m) ** .5,
    }
    return {
        "schema_version": "sie.person_depth_live_cycle.v1", "cycle_id": f"cycle-{index}",
        "captured_at_utc": timestamp, "status": "SUCCESS",
        "person": {"status": "SINGLE_PERSON", "bbox_xyxy_px": [center_x - 50, 300, center_x + 50, 700]},
        "measurement": measurement,
    }


def supervisor() -> MetricFirstTargetSupervisor:
    return MetricFirstTargetSupervisor(live_runtime=None, optical_axis_cx_px=960.0, center_tolerance_px=40.0, now_utc=lambda: NOW)


def test_current_capability_profile_blocks_metric_advance_but_preserves_decision():
    result = supervisor().process_shared_window([cycle(index, center_x=1250) for index in range(1, 6)])
    assert result["stage"] == result["result"] == "BLOCKED_ACTUATOR_CAPABILITY_NOT_QUALIFIED"
    assert result["winning_evidence_path"] == "metric_depth"
    assert result["planned_dry_run"] == {"method": "POST", "endpoint": "/move-forward", "distance_m": .1}
    assert result["metric_decision"]["status"] == "ADVANCE"
    assert result["actuator_capability_gate"]["capability_record"]["adapter_id"] == "esp32_zk5ad_sgm37_520"
    assert result["actuator_capability_gate"]["capability_record"]["qualification_status"] == "NOT_QUALIFIED"
    assert result["network_performed"] is False and result["motor_command_performed"] is False
    assert result["alignment_summary"]["temporal_result"] == "PLANNED_TURN"


def test_metric_hold_wins_over_off_center_alignment():
    result = supervisor().process_shared_window([cycle(index, center_x=1250, z_m=2.02) for index in range(1, 6)])
    assert result["stage"] == "HOLD_TARGET_REACHED"
    assert result["winning_evidence_path"] == "metric_depth"
    assert result["planned_dry_run"] is None and result["reobserve_required"] is False


def test_no_metric_stable_off_center_alignment_falls_back_to_turn():
    result = supervisor().process_shared_window([cycle(index, center_x=1250, with_metric=False) for index in range(1, 6)])
    assert result["stage"] == "AWAIT_OPERATOR_TURN_AND_REOBSERVATION"
    assert result["winning_evidence_path"] == "image_alignment_fallback"
    assert result["planned_dry_run"] == {"method": "POST", "endpoint": "/turn-right", "angle_deg": 4}


def test_invalid_metric_and_unstable_alignment_blocks():
    result = supervisor().process_shared_window([
        cycle(index, center_x=960.0 + index * 100.0, with_metric=False) for index in range(1, 6)
    ])
    assert result["stage"] == result["result"] == "BLOCKED_NO_ACTION"
    assert result["winning_evidence_path"] is None
    assert result["alignment_summary"]["temporal_block_reason"] == "UNSTABLE_EVIDENCE_WINDOW"
