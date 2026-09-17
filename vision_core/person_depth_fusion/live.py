"""Headless live AR0234-to-Stereo-V6 person-depth fusion MVP.

This module owns no actuation.  It only joins one fresh RGB/stereo pair with
the already validated array fusion core and emits JSON-safe cycle records.
"""
from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV
from vision_core.person_localization.models import (
    BoundingBox,
    PersonLocalizationResult,
    PersonLocalizationStatus,
)
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.observation import Observation
from vision_core.rgb_stereo_extrinsic.capture import (
    AR0234_BY_ID, AR_MODE, STEREO_BY_ID, STEREO_MODE, CheckedCamera,
    ControlRunner, default_control_runner, set_control,
)

from .offline import (
    AR_SHAPE, COMBINED_SHAPE, FusionCalibration, PersonDepthFusionError,
    PersonDepthFusionOffline, PersonMeasurement, PersonMeasurementStatus,
    build_offline_stereo_kernel, load_fusion_calibration,
)

MAX_PAIR_SKEW_S = 0.050
WARMUP_READS = 60
YOLO_PRIMARY_SCHEMA = "sie.ar0234.yolo11_person_upper_body_observation.v1"
YOLO_PRIMARY_SINGLE = "SINGLE_TARGET"
YOLO_PRIMARY_NONE = "NO_TARGET"
YOLO_PRIMARY_MULTIPLE = "MULTIPLE_TARGETS"


class LiveFusionError(RuntimeError):
    pass


def _controls(runner: ControlRunner) -> dict[str, dict[str, int]]:
    return {
        "ar0234_auto_exposure": set_control(AR0234_BY_ID, "auto_exposure", 3, runner),
        "ar0234_white_balance_automatic": set_control(AR0234_BY_ID, "white_balance_automatic", 1, runner),
        "stereo_auto_exposure": set_control(STEREO_BY_ID, "auto_exposure", 3, runner),
    }


def _measurement_payload(measurement: PersonMeasurement) -> dict[str, Any] | None:
    return measurement.to_dict() if measurement.status is PersonMeasurementStatus.SUCCESS else None


def _primary_yolo_localization(
    value: object, *, captured_at_utc: datetime, cycle_id: str, frame_shape: tuple[int, ...],
) -> PersonLocalizationResult | None:
    """Adapt same-cycle YOLO evidence for metric ROI selection.

    ``None`` means the primary detector saw no target, so the existing
    MP-PersonDet path remains the fallback. Ambiguous YOLO evidence is never
    resolved by choosing one bbox: it returns ``MULTIPLE_PERSONS`` instead.
    """
    if type(value) is not dict or value.get("schema_version") != YOLO_PRIMARY_SCHEMA:
        return None
    timestamp = captured_at_utc.astimezone(timezone.utc).isoformat()
    status = value.get("target_status")
    if status == YOLO_PRIMARY_NONE:
        return None
    if status == YOLO_PRIMARY_MULTIPLE:
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MULTIPLE_PERSONS,
            candidate_count=(
                int(value["eligible_detection_count"])
                if type(value.get("eligible_detection_count")) is int
                else 0
            ),
            captured_at_utc=timestamp,
            detail="YOLO primary observation contains multiple eligible targets",
        )
    if status != YOLO_PRIMARY_SINGLE:
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MALFORMED_OUTPUT,
            candidate_count=0,
            captured_at_utc=timestamp,
            detail="YOLO primary observation has an invalid target status",
        )
    if (
        value.get("source_cycle_id") != cycle_id
        or value.get("captured_at_utc") != timestamp
        or value.get("reference_frame") != "ar0234_image_frame"
        or value.get("units") != "px"
        or type(value.get("evidence_id")) is not str
        or not value["evidence_id"]
        or type(value.get("confidence")) not in (int, float)
        or not np.isfinite(float(value["confidence"]))
        or not 0.0 <= float(value["confidence"]) <= 1.0
    ):
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MALFORMED_OUTPUT,
            candidate_count=0,
            captured_at_utc=timestamp,
            detail="YOLO primary observation provenance is invalid",
        )
    bbox_value = value.get("bbox_xyxy_px")
    if (
        type(bbox_value) is not list
        or len(bbox_value) != 4
        or any(type(item) not in (int, float) or not np.isfinite(float(item)) for item in bbox_value)
    ):
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MALFORMED_OUTPUT,
            candidate_count=0,
            captured_at_utc=timestamp,
            detail="YOLO primary observation bbox is invalid",
        )
    height, width = frame_shape[:2]
    x_min, y_min, x_max, y_max = (float(item) for item in bbox_value)
    if not (0.0 <= x_min < x_max <= width and 0.0 <= y_min < y_max <= height):
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MALFORMED_OUTPUT,
            candidate_count=0,
            captured_at_utc=timestamp,
            detail="YOLO primary observation bbox is outside the AR0234 frame",
        )
    try:
        bbox = BoundingBox(
            math.floor(x_min), math.floor(y_min), math.ceil(x_max), math.ceil(y_max),
        )
    except ValueError:
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.MALFORMED_OUTPUT,
            candidate_count=0,
            captured_at_utc=timestamp,
            detail="YOLO primary observation bbox is outside the AR0234 frame",
        )
    detector = {
        "adapter_id": "ar0234_yolo11_person_upper_body_primary",
        "model_sha256": value.get("model_sha256"),
        "confidence_threshold": value.get("confidence_threshold"),
        "roi_source": "YOLO_PRIMARY_PERSON_UPPER_BODY",
    }
    observation = Observation(
        observation_id=(
            value["observation_id"]
            if type(value.get("observation_id")) is str
            else f"observation.person.ar0234.{cycle_id}"
        ),
        source_id="ar0234_yolo11_person_upper_body",
        timestamp=timestamp,
        cycle_id=cycle_id,
        observation_type="single_person_upper_body_bbox",
        payload={
            "reference_frame": "ar0234_image_frame",
            "unit": "px",
            "image_size_px": [width, height],
            "person_count": 1,
            "bounding_box_xyxy_px": bbox.to_xyxy(),
            "detector": detector,
        },
        confidence=float(value["confidence"]),
        quality={
            "status": "PASS",
            "single_person_required": True,
            "candidate_count": 1,
            "detector_confidence": float(value["confidence"]),
            "required_confidence_threshold": value.get("confidence_threshold"),
            "roi_source": "YOLO_PRIMARY_PERSON_UPPER_BODY",
        },
        evidence_ids=(value["evidence_id"],),
    )
    return PersonLocalizationResult(
        status=PersonLocalizationStatus.SINGLE_PERSON,
        candidate_count=1,
        captured_at_utc=timestamp,
        observation=observation,
        bounding_box=bbox,
    )


class LivePersonDepthFusion:
    """Long-lived live adapter; detector, calibration and SGBM are constructed once."""

    def __init__(
        self, fusion: PersonDepthFusionOffline, *, ar_camera: Any, stereo_camera: Any,
        primary_image_observer: Any | None = None,
        control_runner: ControlRunner = default_control_runner,
        now_utc: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.fusion, self.ar_camera, self.stereo_camera = fusion, ar_camera, stereo_camera
        self.primary_image_observer = primary_image_observer
        self.control_runner, self.now_utc, self.monotonic = control_runner, now_utc, monotonic
        self.person_threshold = float(fusion.person_pipeline.detector.artifact.confidence_threshold)
        self.started = False

    def start(self) -> None:
        if self.started:
            raise LiveFusionError("runtime is already started")
        try:
            self.ar_camera.open(AR0234_BY_ID)
            self.stereo_camera.open(STEREO_BY_ID)
            _controls(self.control_runner)
            for _ in range(WARMUP_READS):
                self._read_ar("warm-up")
                self._read_stereo("warm-up")
        except Exception as error:
            self.close()
            raise LiveFusionError(f"live camera startup failed: {error}") from error
        self.started = True

    def close(self) -> None:
        self.ar_camera.close()
        self.stereo_camera.close()
        self.started = False

    def _read_ar(self, phase: str) -> np.ndarray:
        frame = self.ar_camera.read(AR_SHAPE)
        if type(frame) is not np.ndarray or frame.dtype != np.uint8 or frame.shape != AR_SHAPE:
            raise LiveFusionError(f"invalid AR0234 {phase} frame")
        return frame

    def _read_stereo(self, phase: str) -> np.ndarray:
        frame = self.stereo_camera.read(COMBINED_SHAPE)
        if type(frame) is not np.ndarray or frame.dtype != np.uint8 or frame.shape != COMBINED_SHAPE:
            raise LiveFusionError(f"invalid OV9281 {phase} frame")
        return frame

    def cycle(self, cycle_id: str) -> dict[str, Any]:
        if not self.started:
            raise LiveFusionError("runtime is not started")
        total_start = self.monotonic()
        try:
            ar_frame = self._read_ar("live")
            ar_stamp = float(self.monotonic())
            stereo_frame = self._read_stereo("live")
            stereo_stamp = float(self.monotonic())
        except Exception:
            self.close()
            raise
        captured = self.now_utc()
        if not isinstance(captured, datetime) or captured.tzinfo is None:
            self.close()
            raise LiveFusionError("UTC clock must return timezone-aware datetime")
        primary_observation = None
        if self.primary_image_observer is not None:
            try:
                primary_observation = self.primary_image_observer.observe(
                    ar_frame, captured_at_utc=captured, cycle_id=cycle_id
                )
                json.dumps(primary_observation, allow_nan=False, sort_keys=True)
            except Exception as error:
                self.close()
                raise LiveFusionError(f"primary AR0234 image observation failed: {error}") from error
        skew = abs(ar_stamp - stereo_stamp)
        base: dict[str, Any] = {
            "schema_version": "sie.person_depth_live_cycle.v1", "cycle_id": cycle_id,
            "captured_at_utc": captured.astimezone(timezone.utc).isoformat(), "pair_skew_s": skew,
            "temperature_eligibility_evaluated": False, "person_threshold": self.person_threshold,
            "temperature_monitoring": _temperature_monitoring_payload(self.fusion.calibration),
            "primary_person_observation": primary_observation,
        }
        if not np.isfinite(skew) or skew > MAX_PAIR_SKEW_S:
            base.update(status="PAIR_SKEW_TOO_HIGH", person={"status": None, "confidence": None, "bbox_xyxy_px": None}, measurement=None,
                        x_m=None, y_m=None, z_m=None, range_m=None, reference_frame=None, selected_point_count=0, inlier_count=0,
                        measurement_confidence=None, detector_ms=0.0, stereo_fusion_ms=0.0, total_cycle_ms=(self.monotonic()-total_start)*1000.)
            return base
        detector_start = self.monotonic()
        localization = _primary_yolo_localization(
            primary_observation,
            captured_at_utc=captured,
            cycle_id=cycle_id,
            frame_shape=ar_frame.shape,
        )
        if localization is None:
            localization = self.fusion.person_pipeline.process(
                ar_frame, captured_at_utc=captured, cycle_id=cycle_id
            )
        detector_ms = (self.monotonic() - detector_start) * 1000.
        fusion_start = self.monotonic()
        measurement = self.fusion.fuse_localization(localization, stereo_frame, captured_at_utc=captured, cycle_id=cycle_id, measurement_mode="live")
        fusion_ms = (self.monotonic() - fusion_start) * 1000.
        person = {
            "status": localization.status.value,
            "confidence": None if localization.observation is None else localization.observation.confidence,
            "bbox_xyxy_px": None if localization.bounding_box is None else localization.bounding_box.to_xyxy(),
        }
        base.update(status=measurement.status.value, person=person, measurement=_measurement_payload(measurement),
                    x_m=measurement.x_m, y_m=measurement.y_m, z_m=measurement.z_m, range_m=measurement.range_m,
                    reference_frame=measurement.reference_frame, selected_point_count=measurement.selected_point_count,
                    inlier_count=measurement.inlier_count, measurement_confidence=measurement.confidence,
                    detector_ms=detector_ms, stereo_fusion_ms=fusion_ms, total_cycle_ms=(self.monotonic()-total_start)*1000.)
        json.dumps(base, allow_nan=False, sort_keys=True)
        return base


def _temperature_monitoring_payload(calibration: FusionCalibration) -> dict[str, str | None]:
    if calibration.temperature_monitoring_mode == "DISABLED_EXPERIMENTAL_MVP":
        return {
            "status": "DISABLED",
            "reason": "temperature_monitoring_disabled_for_supervised_mvp",
        }
    return {"status": "REQUIRED", "reason": None}


def build_live_runtime(*, model: Path, reference: Path, project_root: Path, person_threshold: float = .5,
                       detector_factory: Callable[[Path, Path, float], Any] = MPPersonDetOpenCV,
                       calibration_loader: Callable[[], FusionCalibration] = load_fusion_calibration,
                       kernel_factory: Callable[[FusionCalibration], Any] | None = None,
                       camera_factory: Callable[[Path, Any], Any] = CheckedCamera,
                       stereo_policy_path: Path | None = None,
                       primary_image_observer: Any | None = None) -> LivePersonDepthFusion:
    if not model.is_absolute() or not reference.is_absolute() or not project_root.is_absolute():
        raise LiveFusionError("model, reference and project-root must be absolute paths")
    if stereo_policy_path is not None:
        if calibration_loader is not load_fusion_calibration:
            raise LiveFusionError("custom calibration_loader cannot be combined with stereo_policy_path")
        if not stereo_policy_path.is_absolute():
            raise LiveFusionError("stereo_policy_path must be an absolute path")
        calibration = load_fusion_calibration(stereo_policy_path=stereo_policy_path)
    else:
        calibration = calibration_loader()
    kernel = (kernel_factory or (lambda value: build_offline_stereo_kernel(value, project_root=project_root)))(calibration)
    detector = detector_factory(model, reference, person_threshold)
    return LivePersonDepthFusion(PersonDepthFusionOffline(PersonLocalizationPipeline(detector), kernel, calibration),
                                 ar_camera=camera_factory(AR0234_BY_ID, AR_MODE),
                                 stereo_camera=camera_factory(STEREO_BY_ID, STEREO_MODE),
                                 primary_image_observer=primary_image_observer)
