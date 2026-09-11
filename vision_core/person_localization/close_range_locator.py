"""Close-range face-based image locator for AR0234 alignment observation.

This module is intentionally separate from the full-body person/depth pipeline.
It emits image-frame observations only and has no depth, planning, network, or
motor-control dependency.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np


SCHEMA_VERSION = "sie.ar0234_close_range_locator.v1"
REFERENCE_FRAME = "ar0234_image_frame"
UNITS = "px"
DETECTOR_ID = "opencv_haar_frontalface_default"
CONFIDENCE_SEMANTICS = "opencv_haar_level_weight_sigmoid_uncalibrated"
CASCADE_FILENAME = "haarcascade_frontalface_default.xml"


class CloseRangeTargetStatus(str, Enum):
    SINGLE_TARGET = "SINGLE_TARGET"
    NO_TARGET = "NO_TARGET"
    MULTIPLE_TARGETS = "MULTIPLE_TARGETS"


@dataclass(frozen=True)
class CloseRangeDetection:
    """One face candidate in the AR0234 image frame."""

    bbox_xyxy_px: tuple[int, int, int, int]
    confidence: float

    def __post_init__(self) -> None:
        x_min, y_min, x_max, y_max = self.bbox_xyxy_px
        if any(type(value) is not int for value in self.bbox_xyxy_px):
            raise ValueError("bbox coordinates must be integer pixels")
        if x_min < 0 or y_min < 0 or x_max <= x_min or y_max <= y_min:
            raise ValueError("bbox must have positive in-frame area")
        if type(self.confidence) not in (int, float) or not math.isfinite(self.confidence):
            raise ValueError("confidence must be finite")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("confidence must be in [0, 1]")

    @property
    def center_x_px(self) -> float:
        return (self.bbox_xyxy_px[0] + self.bbox_xyxy_px[2]) / 2.0


def default_face_cascade_path() -> Path:
    """Return the bundled OpenCV Haar cascade, never downloading a model."""
    roots: list[Path] = []
    data = getattr(cv2, "data", None)
    if data is not None and isinstance(getattr(data, "haarcascades", None), str):
        roots.append(Path(data.haarcascades))
    cv2_path = getattr(cv2, "__file__", None)
    if isinstance(cv2_path, str):
        roots.append(Path(cv2_path).resolve().parent / "data")
    roots.append(Path("/usr/share/opencv4/haarcascades"))
    for root in roots:
        path = root / CASCADE_FILENAME
        if path.is_file():
            return path
    raise RuntimeError("bundled OpenCV face cascade is unavailable")


class OpenCvHaarFaceLocator:
    """Use the local OpenCV Haar frontal-face cascade without DNN inference."""

    def __init__(self, cascade_path: Path | None = None) -> None:
        self.cascade_path = cascade_path or default_face_cascade_path()
        self._cascade = cv2.CascadeClassifier(str(self.cascade_path))
        if self._cascade.empty():
            raise RuntimeError(f"unable to load OpenCV face cascade: {self.cascade_path}")

    def detect(self, frame_bgr: np.ndarray) -> list[CloseRangeDetection]:
        _validate_frame(frame_bgr)
        height, width = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        rectangles, _reject_levels, level_weights = self._cascade.detectMultiScale3(
            gray,
            scaleFactor=1.1,
            minNeighbors=3,
            outputRejectLevels=True,
        )
        detections: list[CloseRangeDetection] = []
        for rectangle, level_weight in zip(rectangles, level_weights, strict=True):
            x, y, bbox_width, bbox_height = (int(value) for value in rectangle)
            x_min = max(0, x)
            y_min = max(0, y)
            x_max = min(width, x + bbox_width)
            y_max = min(height, y + bbox_height)
            if x_max <= x_min or y_max <= y_min:
                continue
            detections.append(
                CloseRangeDetection(
                    (x_min, y_min, x_max, y_max),
                    _normalized_level_weight(float(level_weight)),
                )
            )
        return detections


def build_close_range_target_record(
    *,
    detections: Sequence[CloseRangeDetection],
    evidence_id: str,
    captured_at_utc: datetime,
    image_width: int,
    image_height: int,
) -> dict[str, Any]:
    """Build a JSON-safe, fail-closed image-frame target observation."""
    if type(evidence_id) is not str or not evidence_id:
        raise ValueError("evidence_id must be a non-empty string")
    timestamp = _timestamp(captured_at_utc)
    if type(image_width) is not int or type(image_height) is not int:
        raise ValueError("image dimensions must be integers")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    validated = list(detections)
    for detection in validated:
        if not isinstance(detection, CloseRangeDetection):
            raise TypeError("detections must contain CloseRangeDetection values")
        x_min, y_min, x_max, y_max = detection.bbox_xyxy_px
        if x_max > image_width or y_max > image_height:
            raise ValueError("detection bbox is outside image dimensions")

    status: CloseRangeTargetStatus
    center_x_px: float | None = None
    bbox_xyxy_px: list[int] | None = None
    confidence: float | None = None
    if len(validated) == 0:
        status = CloseRangeTargetStatus.NO_TARGET
    elif len(validated) == 1:
        status = CloseRangeTargetStatus.SINGLE_TARGET
        detection = validated[0]
        center_x_px = detection.center_x_px
        bbox_xyxy_px = list(detection.bbox_xyxy_px)
        confidence = float(detection.confidence)
    else:
        status = CloseRangeTargetStatus.MULTIPLE_TARGETS

    return _json_safe(
        {
            "schema_version": SCHEMA_VERSION,
            "evidence_id": evidence_id,
            "timestamp": timestamp,
            "reference_frame": REFERENCE_FRAME,
            "units": UNITS,
            "detector_id": DETECTOR_ID,
            "confidence_semantics": CONFIDENCE_SEMANTICS,
            "target_status": status.value,
            "target_count": len(validated),
            "center_x_px": center_x_px,
            "bbox_xyxy_px": bbox_xyxy_px,
            "confidence": confidence,
        }
    )


def _normalized_level_weight(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("OpenCV Haar level weight must be finite")
    bounded = max(-50.0, min(50.0, value))
    return 1.0 / (1.0 + math.exp(-bounded))


def _timestamp(value: datetime) -> str:
    if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("captured_at_utc must be timezone-aware")
    return value.astimezone(timezone.utc).isoformat()


def _validate_frame(frame_bgr: object) -> None:
    if not (
        isinstance(frame_bgr, np.ndarray)
        and frame_bgr.ndim == 3
        and frame_bgr.shape[0] > 0
        and frame_bgr.shape[1] > 0
        and frame_bgr.shape[2] == 3
        and frame_bgr.dtype == np.uint8
    ):
        raise ValueError("frame must be a non-empty BGR uint8 image")


def _json_safe(value: object) -> dict[str, Any]:
    try:
        output = json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
    except (TypeError, ValueError) as error:
        raise ValueError("close-range locator record must be JSON-safe") from error
    if type(output) is not dict:
        raise ValueError("close-range locator record must be an object")
    return output
