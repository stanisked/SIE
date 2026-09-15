from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

from vision_core.person_approach.bounded_bridge import plan_bounded_command
from vision_core.person_approach.metric_first_target_supervisor import MetricFirstTargetSupervisor
from vision_core.person_localization.yolo11_person_upper_body import PersonUpperBodyDetection
from vision_core.person_localization.yolo11_person_upper_body_runtime import (
    NO_TARGET,
    build_yolo_primary_observation,
)
from vision_core.tools.run_sie_static_target_mvp import (
    _combined,
    bridge_envelope,
    primary_yolo_alignment_window,
    primary_yolo_evidence_block,
)


NOW = datetime(2026, 9, 15, 12, 0, tzinfo=timezone.utc)


def _cycle(index: int, *, center_x: float = 1450.0, yolo: str = "single") -> dict:
    stamp = (NOW - timedelta(milliseconds=(5 - index) * 50)).isoformat()
    detections = [] if yolo == "none" else [
        PersonUpperBodyDetection((center_x - 120.0, 100.0, center_x + 120.0, 1200.0), 0.91)
    ]
    primary = build_yolo_primary_observation(
        detections=detections,
        model_sha256="d" * 64,
        confidence_threshold=0.40,
        frame_width=1920,
        frame_height=1200,
        captured_at_utc=datetime.fromisoformat(stamp),
        cycle_id=f"cycle-{index}",
    )
    return {
        "schema_version": "sie.person_depth_live_cycle.v1",
        "cycle_id": f"cycle-{index}",
        "captured_at_utc": stamp,
        "status": "SUCCESS",
        "person": {"status": "SINGLE_PERSON", "bbox_xyxy_px": [800, 300, 1100, 900]},
        "measurement": {
            "status": "SUCCESS", "measurement_id": f"m-{index}",
            "person_evidence_id": f"legacy-depth-evidence-{index}", "timestamp": stamp,
            "reference_frame": "rectified_left_optical_frame", "units": "m",
            "x_m": 0.02, "y_m": 0.0, "z_m": 2.25, "range_m": 2.2501,
            "confidence": 0.9,
        },
        "primary_person_observation": primary,
    }


def test_yolo_primary_observation_enters_same_window_sie_evidence() -> None:
    cycles = [_cycle(index) for index in range(1, 6)]
    evidence, observations = primary_yolo_alignment_window(
        cycles, optical_axis_cx_px=941.0, center_tolerance_px=40.0
    )
    result = MetricFirstTargetSupervisor(
        live_runtime=None, optical_axis_cx_px=941.0, center_tolerance_px=40.0,
        now_utc=lambda: NOW,
    ).process_shared_window(cycles, alignment_observations=observations)
    assert primary_yolo_evidence_block(evidence) is None
    assert result["source_evidence_ids"] == [item["evidence_id"] for item in evidence]
    assert result["alignment_summary"]["temporal_result"] == "PLANNED_TURN"
    assert result["metric_decision"]["status"] == "ADVANCE"
    assert evidence[0]["truncated_bottom"] is True


def test_no_yolo_target_blocks_before_any_motor_command() -> None:
    cycles = [_cycle(index, yolo="none") for index in range(1, 6)]
    evidence, _ = primary_yolo_alignment_window(
        cycles, optical_axis_cx_px=941.0, center_tolerance_px=40.0
    )
    assert all(item["target_status"] == NO_TARGET for item in evidence)
    blocked = _combined(
        "BLOCKED_PRIMARY_YOLO_EVIDENCE", primary_yolo_evidence_block(evidence),
        supervision=None, bridge=None, executor=None, yolo_primary_evidence=evidence,
        network=False,
    )
    assert blocked["motor_command_performed"] is False
    assert blocked["bridge_plan"] is None and blocked["command_id"] is None


def test_yolo_evidence_does_not_change_bounded_forward_payload_or_json_safety() -> None:
    cycles = [_cycle(index) for index in range(1, 6)]
    _, observations = primary_yolo_alignment_window(
        cycles, optical_axis_cx_px=941.0, center_tolerance_px=40.0
    )
    result = MetricFirstTargetSupervisor(
        live_runtime=None, optical_axis_cx_px=941.0, center_tolerance_px=40.0,
        now_utc=lambda: NOW,
    ).process_shared_window(cycles, alignment_observations=observations)
    envelope = bridge_envelope(
        supervision=result, cycles=cycles, boot_session_id="0123456789ABCDEF",
        allow_supervised_demo_override=True,
    )
    assert envelope is not None
    assert envelope["decision"]["status"] == "ADVANCE"
    assert envelope["decision"]["forward_step_m"] == 0.1
    plan = plan_bounded_command(envelope, now_utc=lambda: NOW).to_dict()
    assert plan["result"] == "PLANNED_BOUNDED_COMMAND"
    assert plan["method"] == "POST" and plan["endpoint"] == "/move-forward"
    assert plan["query"]["distance_m"] == "0.1"
    json.dumps({"supervision": result, "yolo": cycles[0]["primary_person_observation"]}, allow_nan=False)
