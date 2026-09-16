"""ONNX Runtime adapter for visual-only AR0234 person_upper_body observations."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .yolo11_person_upper_body import (
    CLASS_NAME,
    MODEL_INPUT_SIZE,
    MODEL_SHA256,
    REFERENCE_FRAME,
    PersonUpperBodyDetection,
    build_preview_record,
    decode_yolo11_one_class_candidates,
    letterbox_bgr,
    nms_detections,
    verify_model_sha256,
)


YOLO_OBSERVATION_SCHEMA = "sie.ar0234.yolo11_person_upper_body_observation.v1"
YOLO_PRIMARY_KEY = "primary_person_observation"
SINGLE_TARGET = "SINGLE_TARGET"
NO_TARGET = "NO_TARGET"
MULTIPLE_TARGETS = "MULTIPLE_TARGETS"


def _timestamp(value: datetime) -> str:
    if not isinstance(value, datetime) or value.tzinfo is None:
        raise ValueError("captured_at_utc must be timezone-aware")
    return value.astimezone(timezone.utc).isoformat()


class OnnxRuntimeYolo11PersonUpperBodyObserver:
    """Keep one ONNX session and emit an image-space observation per AR frame."""

    def __init__(self, model_path: Path, *, confidence_threshold: float = 0.40) -> None:
        if (
            type(confidence_threshold) not in (int, float)
            or not math.isfinite(confidence_threshold)
            or not 0.0 <= confidence_threshold <= 1.0
        ):
            raise ValueError("confidence_threshold must be finite and in [0, 1]")
        self.model_path = model_path
        self.model_sha256 = verify_model_sha256(model_path, MODEL_SHA256)
        self.confidence_threshold = float(confidence_threshold)
        try:
            import onnxruntime as ort
        except ModuleNotFoundError as error:
            raise RuntimeError(
                "onnxruntime is unavailable; use the isolated companion venv documented in "
                "docs/datasets/ar0234_close_range_person_alignment_v1/ONNX_PREVIEW_SETUP.md"
            ) from error
        self._session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        inputs = self._session.get_inputs()
        if len(inputs) != 1 or tuple(inputs[0].shape) != (1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE):
            raise RuntimeError("YOLO11 model must expose one fixed 1x3x640x640 input")
        self._input_name = str(inputs[0].name)

    def observe(
        self, frame_bgr: np.ndarray, *, captured_at_utc: datetime, cycle_id: str
    ) -> dict[str, Any]:
        timestamp = _timestamp(captured_at_utc)
        if type(cycle_id) is not str or not cycle_id:
            raise ValueError("cycle_id must be a non-empty string")
        tensor, transform = letterbox_bgr(frame_bgr, input_size=MODEL_INPUT_SIZE)
        outputs = self._session.run(None, {self._input_name: tensor})
        if len(outputs) != 1:
            raise RuntimeError(f"YOLO11 model must produce one output, got {len(outputs)}")
        detections = decode_yolo11_one_class_candidates(
            outputs[0], transform=transform
        )
        return build_yolo_primary_observation(
            detections=detections,
            model_sha256=self.model_sha256,
            confidence_threshold=self.confidence_threshold,
            frame_width=frame_bgr.shape[1],
            frame_height=frame_bgr.shape[0],
            captured_at_utc=captured_at_utc,
            cycle_id=cycle_id,
        )


def build_yolo_primary_observation(
    *,
    detections: list[PersonUpperBodyDetection],
    model_sha256: str,
    confidence_threshold: float,
    frame_width: int,
    frame_height: int,
    captured_at_utc: datetime,
    cycle_id: str,
) -> dict[str, Any]:
    """Build a JSON-safe primary image observation without raw frame content."""
    timestamp = _timestamp(captured_at_utc)
    if type(cycle_id) is not str or not cycle_id:
        raise ValueError("cycle_id must be a non-empty string")
    eligible_detections = nms_detections(
        [
            detection
            for detection in detections
            if detection.confidence >= float(confidence_threshold)
        ]
    )
    preview = build_preview_record(
        model_sha256=model_sha256,
        frame_width=frame_width,
        frame_height=frame_height,
        confidence_threshold=confidence_threshold,
        detections=eligible_detections,
    )
    status = (
        NO_TARGET
        if not eligible_detections
        else SINGLE_TARGET if len(eligible_detections) == 1 else MULTIPLE_TARGETS
    )
    observation_id = f"ar0234-yolo11-person-upper-body:{cycle_id}"
    selected_record = preview["detections"][0] if status == SINGLE_TARGET else None
    result = {
        "schema_version": YOLO_OBSERVATION_SCHEMA,
        "observation_id": observation_id,
        "evidence_id": observation_id,
        "source_cycle_id": cycle_id,
        "captured_at_utc": timestamp,
        "entity_type": "person",
        "class_name": CLASS_NAME,
        "target_status": status,
        "reference_frame": REFERENCE_FRAME,
        "units": "px",
        "model_sha256": model_sha256,
        "model_input_size_px": {"width": MODEL_INPUT_SIZE, "height": MODEL_INPUT_SIZE},
        "confidence_threshold": confidence_threshold,
        "frame_size_px": preview["frame_size_px"],
        "raw_detection_count": len(detections),
        "eligible_detection_count": len(eligible_detections),
        "detection_count": preview["detection_count"],
        "detections": preview["detections"],
        "bbox_xyxy_px": None if selected_record is None else selected_record["bbox_xyxy_px"],
        "center_x_px": None if selected_record is None else selected_record["center_x_px"],
        "confidence": None if selected_record is None else selected_record["confidence"],
        "truncated_left": None if selected_record is None else selected_record["truncated_left"],
        "truncated_right": None if selected_record is None else selected_record["truncated_right"],
        "truncated_top": None if selected_record is None else selected_record["truncated_top"],
        "truncated_bottom": None if selected_record is None else selected_record["truncated_bottom"],
    }
    return json.loads(json.dumps(result, allow_nan=False, sort_keys=True))


def temporal_observation_from_yolo_primary(
    value: object, *, optical_axis_cx_px: float, center_tolerance_px: float
) -> dict[str, Any]:
    """Adapt a same-cycle YOLO record to the existing temporal planner contract."""
    if (
        type(value) is not dict
        or type(optical_axis_cx_px) not in (int, float)
        or not math.isfinite(optical_axis_cx_px)
        or type(center_tolerance_px) not in (int, float)
        or not math.isfinite(center_tolerance_px)
        or center_tolerance_px < 0.0
    ):
        raise ValueError("invalid YOLO primary observation or alignment configuration")
    status = value.get("target_status")
    mapping = {
        SINGLE_TARGET: "SINGLE_PERSON",
        NO_TARGET: "PERSON_LOST",
        MULTIPLE_TARGETS: "MULTIPLE_PERSONS",
    }
    person_status = mapping.get(status, "INVALID_PERSON_EVIDENCE")
    if (
        value.get("schema_version") != YOLO_OBSERVATION_SCHEMA
        or value.get("reference_frame") != REFERENCE_FRAME
        or value.get("units") != "px"
        or type(value.get("evidence_id")) is not str
        or not value["evidence_id"]
        or type(value.get("source_cycle_id")) is not str
        or not value["source_cycle_id"]
        or type(value.get("captured_at_utc")) is not str
        or not value["captured_at_utc"]
    ):
        person_status = "INVALID_PERSON_EVIDENCE"
    result: dict[str, Any] = {
        "evidence_id": value.get("evidence_id") if type(value.get("evidence_id")) is str else "invalid-yolo-evidence",
        "source_cycle_id": value.get("source_cycle_id") if type(value.get("source_cycle_id")) is str else "invalid-yolo-cycle",
        "timestamp": value.get("captured_at_utc") if type(value.get("captured_at_utc")) is str else None,
        "reference_frame": REFERENCE_FRAME,
        "units": "px",
        "person_status": person_status,
        "center_tolerance_px": float(center_tolerance_px),
    }
    center_x = value.get("center_x_px")
    bbox = value.get("bbox_xyxy_px")
    if person_status == "SINGLE_PERSON":
        if (
            type(center_x) not in (int, float)
            or not math.isfinite(center_x)
            or type(bbox) is not list
            or len(bbox) != 4
            or any(type(item) not in (int, float) or not math.isfinite(item) for item in bbox)
        ):
            result["person_status"] = "INVALID_PERSON_EVIDENCE"
        else:
            x1, y1, x2, y2 = (float(item) for item in bbox)
            expected_center_x = (x1 + x2) / 2.0
            if x2 <= x1 or y2 <= y1 or not math.isclose(float(center_x), expected_center_x, abs_tol=1e-6):
                result["person_status"] = "INVALID_PERSON_EVIDENCE"
            else:
                result["image_offset_px"] = float(center_x) - float(optical_axis_cx_px)
    return result
