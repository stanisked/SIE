"""Deterministic regressions for the supervised static-target MVP.

All fixtures are scalar contracts or synthetic arrays.  They never open a
camera, load ONNX, or issue a real HTTP request.
"""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

from sie_core.supervised_bounded_executor import (
    authorize_operator_trial,
    execute_one_supervised_command,
)
from vision_core.person_approach.bounded_bridge import plan_bounded_command
from vision_core.person_depth_fusion.live import LivePersonDepthFusion
from vision_core.person_depth_fusion.offline import (
    AR_SHAPE,
    COMBINED_SHAPE,
    FusionCalibration,
    PersonDepthFusionOffline,
)
from vision_core.person_localization.detector import (
    ARTIFACT_SCHEMA_VERSION,
    CONFIDENCE_SEMANTICS,
    OUTPUT_CONTRACT_VERSION,
    DetectorArtifact,
)
from vision_core.person_localization.models import BoundingBox, PersonDetection
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.person_localization.yolo11_person_upper_body import PersonUpperBodyDetection
from vision_core.person_localization.yolo11_person_upper_body_runtime import (
    MULTIPLE_TARGETS,
    NO_TARGET,
    build_yolo_primary_observation,
)
from vision_core.stereo.guarded_runtime_v6_v2 import StereoDepthFrame
from vision_core.stereo.stereo_calibration_guard_v6 import TemperatureGateResult
from vision_core.tools import run_sie_static_target_mvp as runner


NOW = datetime(2026, 9, 17, 12, 0, tzinfo=timezone.utc)
SESSION = "0123456789ABCDEF"


class _LegacyDetector:
    def __init__(self, detections: list[PersonDetection]) -> None:
        self.artifact = DetectorArtifact(
            ARTIFACT_SCHEMA_VERSION, "legacy-test", "1", "model", "1", "0" * 64,
            OUTPUT_CONTRACT_VERSION, "person", CONFIDENCE_SEMANTICS, 0.5,
            {"score_threshold": 0.5},
        )
        self.detections = detections
        self.calls = 0

    def detect(self, _frame: np.ndarray) -> list[PersonDetection]:
        self.calls += 1
        return self.detections


class _Camera:
    def __init__(self, frame: np.ndarray) -> None:
        self.frame = frame

    def open(self, _path: object) -> dict[str, object]:
        return {}

    def read(self, _shape: object) -> np.ndarray:
        return self.frame

    def close(self) -> None:
        pass


class _Kernel:
    def process(self, _frame: np.ndarray) -> StereoDepthFrame:
        return StereoDepthFrame(
            1, "synthetic", "rectified_left_optical_frame", "stereo-v6", "a", "b", "profile",
            TemperatureGateResult("synthetic", 0.0, {}),
            np.empty((800, 1280, 3), np.uint8), np.empty((800, 1280, 3), np.uint8),
            np.ones((800, 1280), np.int16), np.ones((800, 1280), np.float32),
            np.ones((800, 1280), bool), np.ones((800, 1280), np.float32),
        )


def _calibration() -> FusionCalibration:
    return FusionCalibration(
        Path("candidate"), "a" * 64, Path("validation"), "b" * 64, "candidate", np.eye(4),
        np.eye(4), np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 600.0], [0.0, 0.0, 1.0]]),
        np.zeros((1, 5)), np.array([[1000.0, 0.0, 640.0, 0.0], [0.0, 1000.0, 400.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
        Path("stereo"), "c" * 64, Path("policy"), "policy", (0.38, 2.55),
    )


def _live_cycle(primary: dict, legacy: list[PersonDetection]) -> tuple[dict, _LegacyDetector]:
    detector = _LegacyDetector(legacy)
    fusion = PersonDepthFusionOffline(
        PersonLocalizationPipeline(detector, now_utc=lambda: NOW), _Kernel(), _calibration()
    )

    class _Observer:
        def observe(self, _frame: np.ndarray, *, captured_at_utc: datetime, cycle_id: str) -> dict:
            assert captured_at_utc == NOW and cycle_id == "cycle-001"
            return primary

    live = LivePersonDepthFusion(
        fusion,
        ar_camera=_Camera(np.zeros(AR_SHAPE, np.uint8)),
        stereo_camera=_Camera(np.zeros(COMBINED_SHAPE, np.uint8)),
        primary_image_observer=_Observer(),
        now_utc=lambda: NOW,
        monotonic=lambda: 0.0,
    )
    live.started = True
    return live.cycle("cycle-001"), detector


def _primary(cycle_id: str, detections: list[PersonUpperBodyDetection]) -> dict:
    return build_yolo_primary_observation(
        detections=detections,
        model_sha256="d" * 64,
        confidence_threshold=0.70,
        frame_width=1920,
        frame_height=1200,
        captured_at_utc=NOW,
        cycle_id=cycle_id,
    )


def _legacy_one() -> list[PersonDetection]:
    return [PersonDetection(BoundingBox(100, 100, 1800, 1100), 0.9)]


def _plan(command_id: str = "pa-regression-001") -> dict:
    return {
        "schema_version": "sie.person_approach_bounded_bridge.v1",
        "result": "PLANNED_BOUNDED_COMMAND",
        "method": "POST",
        "endpoint": "/move-forward",
        "query": {"boot_session_id": SESSION, "command_id": command_id, "distance_m": "0.1"},
        "network_performed": False,
    }


def _ready(**extra: object) -> dict:
    return {
        "motion_api_version": "v2_bounded_motion_api", "boot_session_id": SESSION,
        "state": "READY", "bounded_fault_latched": False, "active_command_id": None,
        "last_command_id": None, "last_command_state": None, **extra,
    }


def _authorization(plan: dict) -> dict:
    return authorize_operator_trial(
        planned_command=plan,
        confirmation_command_id=plan["query"]["command_id"],
        authorization_mode="SUPERVISED_EXPERIMENTAL_TRIAL",
        experimental_reason="regression fixture",
    )


def test_yolo_single_target_overrides_legacy_multiple_for_metric_roi() -> None:
    primary = _primary("cycle-001", [PersonUpperBodyDetection((500.0, 300.0, 1400.0, 1000.0), 0.924)])
    cycle, legacy = _live_cycle(primary, [*_legacy_one(), *_legacy_one()])

    assert cycle["status"] == "SUCCESS"
    assert cycle["person"]["bbox_xyxy_px"] == [500, 300, 1400, 1000]
    assert cycle["measurement"]["person_evidence_id"] == primary["evidence_id"]
    assert cycle["measurement"]["rgb_roi_xyxy_px"] == [500, 300, 1400, 1000]
    assert cycle["measurement"]["detector_provenance"]["roi_source"] == "YOLO_PRIMARY_PERSON_UPPER_BODY"
    assert legacy.calls == 0
    final = runner._combined("RANGE_ACQUISITION_REQUIRED", None, supervision=None, bridge=None, executor=None, yolo_primary_evidence=[primary], network=False)
    assert final["command_id"] is None and final["network_performed"] is False
    assert final["motor_command_performed"] is False and final["result"] == "RANGE_ACQUISITION_REQUIRED"


def test_ambiguous_yolo_blocks_without_selecting_bbox_or_planning() -> None:
    primary = _primary("cycle-001", [
        PersonUpperBodyDetection((300.0, 200.0, 800.0, 1000.0), 0.93),
        PersonUpperBodyDetection((1000.0, 200.0, 1500.0, 1000.0), 0.90),
    ])
    cycle, legacy = _live_cycle(primary, _legacy_one())

    assert primary["target_status"] == MULTIPLE_TARGETS
    assert cycle["status"] == "MULTIPLE_PERSONS" and cycle["measurement"] is None
    assert cycle["person"]["bbox_xyxy_px"] is None and legacy.calls == 0
    final = runner._combined("BLOCKED_PRIMARY_YOLO_EVIDENCE", "YOLO_PRIMARY_MULTIPLE_TARGETS", supervision=None, bridge=None, executor=None, yolo_primary_evidence=[primary], network=False)
    assert final["command_id"] is None and final["network_performed"] is False
    assert final["motor_command_performed"] is False and final["result"] == "BLOCKED_PRIMARY_YOLO_EVIDENCE"


def test_yolo_no_target_keeps_legacy_persondet_metric_roi_fallback() -> None:
    primary = _primary("cycle-001", [])
    cycle, legacy = _live_cycle(primary, _legacy_one())

    assert primary["target_status"] == NO_TARGET
    assert cycle["status"] == "SUCCESS" and legacy.calls == 1
    assert cycle["measurement"]["person_evidence_id"] == "evidence.ar0234_frame.cycle-001"
    assert cycle["measurement"]["rgb_roi_xyxy_px"] == [100, 100, 1800, 1100]
    assert cycle["measurement"]["detector_provenance"].get("roi_source") is None
    final = runner._combined("RANGE_ACQUISITION_REQUIRED", None, supervision=None, bridge=None, executor=None, yolo_primary_evidence=[primary], network=False)
    assert final["command_id"] is None and final["network_performed"] is False
    assert final["motor_command_performed"] is False and final["result"] == "RANGE_ACQUISITION_REQUIRED"


def test_two_zero_metric_windows_require_range_without_http_or_motor() -> None:
    def invalid_window(attempt: int) -> dict:
        cycles = [
            {
                "cycle_id": f"a{attempt}-{index}", "status": "SUCCESS",
                "measurement": {"status": "BLOCKED_QUALITY"},
            }
            for index in range(5)
        ]
        return {"cycles": cycles, "supervision": {"result": "RANGE_ACQUISITION_REQUIRED", "reason": "NO_CURRENT_VALID_METRIC_DEPTH_DECISION", "metric_decision": None}}

    selected, acquisition = runner.acquire_metric_depth_with_one_retry(invalid_window)
    final = runner._combined(selected["supervision"]["result"], selected["supervision"]["reason"], supervision=selected["supervision"], bridge=None, executor=None, yolo_primary_evidence=None, network=False, metric_acquisition=acquisition)
    assert acquisition["attempt_count"] == 2 and acquisition["final_metric_decision_available"] is False
    assert final["result"] == "RANGE_ACQUISITION_REQUIRED" and final["command_id"] is None
    assert final["network_performed"] is False and final["motor_command_performed"] is False


def test_preflight_retries_get_then_posts_only_once_after_ready() -> None:
    plan = _plan()
    calls: list[str] = []
    responses: list[object] = [
        OSError("temporary preflight timeout"),
        (200, _ready()),
        (202, {"accepted": True, "command_id": plan["query"]["command_id"]}),
        (200, _ready(last_command_id=plan["query"]["command_id"], last_command_state="PARTIAL_PROGRESS")),
    ]

    def request(method: str, _url: str, _timeout_s: float) -> tuple[int, dict]:
        calls.append(method)
        response = responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response  # type: ignore[return-value]

    final = execute_one_supervised_command(planned_command=plan, authorization=_authorization(plan), base_url="http://127.0.0.1", request=request, sleep=lambda _: None, monotonic=lambda: 0.0)
    assert calls == ["GET", "GET", "POST", "GET"] and calls.count("POST") == 1
    assert final["planned_command"]["query"]["command_id"] == plan["query"]["command_id"]
    assert final["network_performed"] is True and final["motor_command_performed"] is True
    assert final["result"] == "AWAIT_REOBSERVATION" and final["terminal_status"]["last_command_state"] == "PARTIAL_PROGRESS"


def test_stale_evidence_blocks_before_any_http_preflight() -> None:
    old = NOW - timedelta(seconds=2)
    cycles = []
    for index in range(5):
        stamp = (old + timedelta(milliseconds=index)).isoformat()
        cycles.append({
            "cycle_id": f"stale-{index}", "captured_at_utc": stamp, "status": "SUCCESS",
            "person": {"status": "SINGLE_PERSON"},
            "measurement": {"status": "SUCCESS", "measurement_id": f"m-{index}", "timestamp": stamp, "reference_frame": "rectified_left_optical_frame", "units": "m", "x_m": 0.0, "y_m": 0.0, "z_m": 2.2, "range_m": 2.2, "confidence": 0.9},
        })
    decision = {"schema_version": "sie.person_approach_decision.v1", "decision_id": "stale-decision", "timestamp": NOW.isoformat(), "status": "ADVANCE", "reference_frame": "rectified_left_optical_frame", "units": "m", "turn_angle_units": "deg", "forward_step_m": 0.1, "turn_angle_deg": None, "source_measurement_ids": [f"m-{index}" for index in range(5)]}
    plan = plan_bounded_command({"decision": decision, "evidence_window": cycles, "boot_session_id": SESSION}, now_utc=lambda: NOW).to_dict()
    final = runner._combined("BLOCKED_NO_EXECUTION_PLAN", plan["block_reason"], supervision=None, bridge=plan, executor=None, yolo_primary_evidence=None, network=False)
    assert plan["block_reason"] == "LATEST_CYCLE_STALE_OR_FROM_FUTURE"
    assert final["command_id"] is None and final["network_performed"] is False
    assert final["motor_command_performed"] is False and final["result"] == "BLOCKED_NO_EXECUTION_PLAN"


def test_each_execute_run_gets_new_command_id_and_one_post_keeps_it() -> None:
    first = runner.fresh_supervised_execution_plan(_plan("pa-bridge-id"))
    second = runner.fresh_supervised_execution_plan(_plan("pa-bridge-id"))
    assert first["command_id"] != second["command_id"]
    calls: list[str] = []

    def request(method: str, _url: str, _timeout_s: float) -> tuple[int, dict]:
        calls.append(method)
        if method == "POST":
            return 202, {"accepted": True, "command_id": first["command_id"]}
        if calls.count("GET") == 1:
            return 200, _ready()
        return 200, _ready(last_command_id=first["command_id"], last_command_state="SUCCESS")

    final = execute_one_supervised_command(planned_command=first, authorization=_authorization(first), base_url="http://127.0.0.1", request=request, sleep=lambda _: None, monotonic=lambda: 0.0)
    assert calls.count("POST") == 1 and final["planned_command"]["command_id"] == first["command_id"]
    assert final["network_performed"] is True and final["motor_command_performed"] is True
    assert final["result"] == "AWAIT_REOBSERVATION" and final["terminal_status"]["last_command_id"] == first["command_id"]
    json.dumps(final, allow_nan=False)
