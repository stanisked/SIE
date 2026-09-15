"""Pure preprocessing and postprocessing for the visual-only YOLO11 preview.

The module deliberately has no camera or ONNX Runtime dependency.  It turns a
model tensor into image-frame detections but does not make a SIE measurement,
decision, yaw request, or motor action.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


MODEL_SHA256 = "dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c"
MODEL_INPUT_SIZE = 640
CLASS_NAME = "person_upper_body"
REFERENCE_FRAME = "ar0234_image_frame"


@dataclass(frozen=True)
class LetterboxTransform:
    """Mapping from a source image into the square model input."""

    scale: float
    pad_x: int
    pad_y: int
    input_width: int
    input_height: int
    source_width: int
    source_height: int


@dataclass(frozen=True)
class PersonUpperBodyDetection:
    """One visual-only class prediction expressed in the source image frame."""

    bbox_xyxy_px: tuple[float, float, float, float]
    confidence: float

    @property
    def center_x_px(self) -> float:
        return (self.bbox_xyxy_px[0] + self.bbox_xyxy_px[2]) / 2.0


def verify_model_sha256(path: Path, expected_sha256: str = MODEL_SHA256) -> str:
    """Return the verified digest, failing before inference on a mismatch."""
    if not path.is_file():
        raise ValueError(f"ONNX model is not a regular file: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != expected_sha256:
        raise ValueError(
            "ONNX model SHA-256 mismatch: "
            f"expected={expected_sha256}, actual={digest}"
        )
    return digest


def letterbox_bgr(
    frame_bgr: np.ndarray,
    *,
    input_size: int = MODEL_INPUT_SIZE,
) -> tuple[np.ndarray, LetterboxTransform]:
    """Letterbox BGR input and produce a normalized RGB NCHW tensor."""
    if (
        not isinstance(frame_bgr, np.ndarray)
        or frame_bgr.dtype != np.uint8
        or frame_bgr.ndim != 3
        or frame_bgr.shape[2] != 3
    ):
        raise ValueError("frame must be a uint8 HxWx3 BGR image")
    if type(input_size) is not int or input_size <= 0:
        raise ValueError("input_size must be a positive integer")
    source_height, source_width = frame_bgr.shape[:2]
    if source_width <= 0 or source_height <= 0:
        raise ValueError("frame dimensions must be positive")
    scale = min(input_size / source_width, input_size / source_height)
    resized_width = int(round(source_width * scale))
    resized_height = int(round(source_height * scale))
    pad_x = (input_size - resized_width) // 2
    pad_y = (input_size - resized_height) // 2
    resized = cv2.resize(frame_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    tensor = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None, ...]
    transform = LetterboxTransform(
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
        input_width=input_size,
        input_height=input_size,
        source_width=source_width,
        source_height=source_height,
    )
    return tensor, transform


def _prediction_rows(output: np.ndarray) -> np.ndarray:
    """Normalize the one-class YOLO11 output to ``(candidate, 5)`` rows."""
    values = np.asarray(output, dtype=np.float32)
    if values.ndim == 3 and values.shape[0] == 1:
        values = values[0]
    if values.ndim != 2:
        raise ValueError(f"unexpected YOLO output rank: {values.shape}")
    if values.shape[0] == 5:
        values = values.T
    elif values.shape[1] != 5:
        raise ValueError(
            "expected one-class YOLO11 output with five values per candidate, "
            f"got {values.shape}"
        )
    if values.shape[1] != 5 or not np.isfinite(values).all():
        raise ValueError("YOLO output must be finite one-class xywh+score values")
    return values


def _iou(left: PersonUpperBodyDetection, right: PersonUpperBodyDetection) -> float:
    ax1, ay1, ax2, ay2 = left.bbox_xyxy_px
    bx1, by1, bx2, by2 = right.bbox_xyxy_px
    intersection_width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    intersection_height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_width * intersection_height
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
    return 0.0 if union <= 0.0 else intersection / union


def nms_detections(
    detections: list[PersonUpperBodyDetection], *, iou_threshold: float = 0.45
) -> list[PersonUpperBodyDetection]:
    """Deterministic confidence-ordered NMS for one class."""
    if not isinstance(iou_threshold, (int, float)) or not math.isfinite(iou_threshold) or not 0.0 < iou_threshold <= 1.0:
        raise ValueError("iou_threshold must be finite and in (0, 1]")
    selected: list[PersonUpperBodyDetection] = []
    for candidate in sorted(detections, key=lambda item: item.confidence, reverse=True):
        if all(_iou(candidate, kept) <= float(iou_threshold) for kept in selected):
            selected.append(candidate)
    return selected


def decode_yolo11_one_class_output(
    output: np.ndarray,
    *,
    transform: LetterboxTransform,
    confidence_threshold: float,
    nms_iou_threshold: float = 0.45,
) -> list[PersonUpperBodyDetection]:
    """Decode YOLO11 ``xywh + person_upper_body score`` into source pixels."""
    if (
        not isinstance(confidence_threshold, (int, float))
        or not math.isfinite(confidence_threshold)
        or not 0.0 < confidence_threshold <= 1.0
    ):
        raise ValueError("confidence_threshold must be finite and in (0, 1]")
    candidates: list[PersonUpperBodyDetection] = []
    for center_x, center_y, width, height, confidence in _prediction_rows(output):
        if float(confidence) < float(confidence_threshold) or width <= 0.0 or height <= 0.0:
            continue
        x1 = (float(center_x) - float(width) / 2.0 - transform.pad_x) / transform.scale
        y1 = (float(center_y) - float(height) / 2.0 - transform.pad_y) / transform.scale
        x2 = (float(center_x) + float(width) / 2.0 - transform.pad_x) / transform.scale
        y2 = (float(center_y) + float(height) / 2.0 - transform.pad_y) / transform.scale
        x1 = max(0.0, min(float(transform.source_width), x1))
        y1 = max(0.0, min(float(transform.source_height), y1))
        x2 = max(0.0, min(float(transform.source_width), x2))
        y2 = max(0.0, min(float(transform.source_height), y2))
        if x2 <= x1 or y2 <= y1:
            continue
        candidates.append(
            PersonUpperBodyDetection((x1, y1, x2, y2), float(confidence))
        )
    return nms_detections(candidates, iou_threshold=nms_iou_threshold)


def build_preview_record(
    *,
    model_sha256: str,
    frame_width: int,
    frame_height: int,
    confidence_threshold: float,
    detections: list[PersonUpperBodyDetection],
) -> dict[str, object]:
    """Build a JSON-safe visual observation record without source pixels."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")
    serialized = [
        {
            "class_name": CLASS_NAME,
            "bbox_xyxy_px": [round(value, 6) for value in item.bbox_xyxy_px],
            "center_x_px": round(item.center_x_px, 6),
            "confidence": round(item.confidence, 6),
        }
        for item in detections
    ]
    return {
        "schema_version": "sie.ar0234.person_upper_body_onnx_preview.v1",
        "model_sha256": model_sha256,
        "frame_size_px": {"width": frame_width, "height": frame_height},
        "confidence_threshold": confidence_threshold,
        "reference_frame": REFERENCE_FRAME,
        "units": "px",
        "detection_count": len(serialized),
        "detections": serialized,
    }
