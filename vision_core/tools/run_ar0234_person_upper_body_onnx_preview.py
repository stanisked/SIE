#!/usr/bin/env python3
"""Live, visual-only AR0234 preview for the local one-class YOLO11 ONNX model."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_localization import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig  # noqa: E402
from vision_core.person_localization.yolo11_person_upper_body import (  # noqa: E402
    CLASS_NAME,
    MODEL_INPUT_SIZE,
    decode_yolo11_one_class_output,
    build_preview_record,
    letterbox_bgr,
    verify_model_sha256,
)
from vision_core.tools.run_ar0234_alignment_preview import PreviewWindow  # noqa: E402


AR_WIDTH = 1920
AR_HEIGHT = 1200
WINDOW_NAME = "AR0234 person_upper_body YOLO11 ONNX Preview"
DEFAULT_MODEL = Path(
    "/home/stanislav/dev_ws/model_artifacts/"
    "ar0234_person_upper_body_yolo11n_v1/best.onnx"
)


def _positive_probability(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value) or not 0.0 < value <= 1.0:
        raise ValueError(f"{name} must be finite and in (0, 1]")
    return float(value)


def _preview_image(frame: np.ndarray, width: int) -> np.ndarray:
    if type(width) is not int or width <= 0:
        raise ValueError("--preview-width must be a positive integer")
    if width >= frame.shape[1]:
        return frame
    height = int(round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def draw_overlay(frame: np.ndarray, detections: list[object], threshold: float) -> np.ndarray:
    """Draw only on a copy; no raw frame is written to disk or stdout."""
    view = frame.copy()
    for detection in detections:
        x1, y1, x2, y2 = (int(round(value)) for value in detection.bbox_xyxy_px)
        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 0), 3)
        cv2.putText(
            view,
            f"{CLASS_NAME} {detection.confidence:.2f}",
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0),
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        view,
        f"visual-only | {CLASS_NAME}={len(detections)} | threshold={threshold:.2f}",
        (24, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 220, 0),
        2,
        cv2.LINE_AA,
    )
    return view


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--device", type=Path, default=AR0234_BY_ID)
    parser.add_argument("--confidence-threshold", type=float, default=0.40)
    parser.add_argument("--nms-iou-threshold", type=float, default=0.45)
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means until Q or Esc")
    return parser.parse_args(argv)


def _load_onnx_session(model: Path) -> object:
    try:
        import onnxruntime as ort
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "onnxruntime is unavailable; create the dedicated preview venv documented in "
            "docs/datasets/ar0234_close_range_person_alignment_v1/ONNX_PREVIEW_SETUP.md"
        ) from error
    return ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])


def _validate_onnx_input(inputs: list[object]) -> str:
    if len(inputs) != 1:
        raise RuntimeError(f"expected exactly one ONNX input, got {len(inputs)}")
    input_metadata = inputs[0]
    if tuple(input_metadata.shape) != (1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE):
        raise RuntimeError(
            "unexpected ONNX input shape: "
            f"expected=(1, 3, {MODEL_INPUT_SIZE}, {MODEL_INPUT_SIZE}), "
            f"actual={tuple(input_metadata.shape)}"
        )
    return str(input_metadata.name)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.device != AR0234_BY_ID:
        raise ValueError("--device must be the exact approved AR0234 stable by-id path")
    if args.max_frames < 0:
        raise ValueError("--max-frames must be 0 or positive")
    threshold = _positive_probability(args.confidence_threshold, "--confidence-threshold")
    nms_threshold = _positive_probability(args.nms_iou_threshold, "--nms-iou-threshold")
    model_sha256 = verify_model_sha256(args.model)
    session = _load_onnx_session(args.model)
    input_name = _validate_onnx_input(session.get_inputs())
    capture = AR0234Capture(
        AR0234CaptureConfig(
            device=AR0234_BY_ID,
            width=AR_WIDTH,
            height=AR_HEIGHT,
            fps=30.0,
            fourcc="MJPG",
            buffer_size=1,
        )
    )
    preview: PreviewWindow | None = None
    try:
        capture.open()
        for _ in range(60):
            capture.read()
        preview = PreviewWindow(WINDOW_NAME)
        count = 0
        while args.max_frames == 0 or count < args.max_frames:
            frame = capture.read()
            count += 1
            tensor, transform = letterbox_bgr(frame, input_size=MODEL_INPUT_SIZE)
            outputs = session.run(None, {input_name: tensor})
            if len(outputs) != 1:
                raise RuntimeError(f"expected exactly one ONNX output, got {len(outputs)}")
            detections = decode_yolo11_one_class_output(
                outputs[0],
                transform=transform,
                confidence_threshold=threshold,
                nms_iou_threshold=nms_threshold,
            )
            record = build_preview_record(
                model_sha256=model_sha256,
                frame_width=AR_WIDTH,
                frame_height=AR_HEIGHT,
                confidence_threshold=threshold,
                detections=detections,
            )
            print(json.dumps(record, allow_nan=False, sort_keys=True), flush=True)
            if preview.show(_preview_image(draw_overlay(frame, detections, threshold), args.preview_width)):
                break
        return 0
    finally:
        capture.close()
        if preview is not None:
            preview.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
