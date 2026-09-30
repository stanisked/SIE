#!/usr/bin/env python3
"""Inspect raw custom YOLO candidate scores on one saved AR0234 image.

This is a detector diagnostic only. It neither publishes ROS topics nor can
issue any navigation or actuator command.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from vision_core.person_localization.yolo11_person_upper_body import (
    decode_yolo11_one_class_candidates,
    letterbox_bgr,
    verify_model_sha256,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--minimum-score", type=float, default=0.01)
    parser.add_argument("--limit", type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not 0.0 <= args.minimum_score <= 1.0:
        raise ValueError("--minimum-score must be in [0, 1]")
    if args.limit <= 0:
        raise ValueError("--limit must be positive")
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f"Could not decode image: {args.image}")

    import onnxruntime as ort

    model_sha256 = verify_model_sha256(args.model)
    tensor, transform = letterbox_bgr(frame)
    session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    if len(inputs) != 1:
        raise RuntimeError(f"Expected one ONNX input, got {len(inputs)}")
    outputs = session.run(None, {str(inputs[0].name): tensor})
    if len(outputs) != 1:
        raise RuntimeError(f"Expected one ONNX output, got {len(outputs)}")

    candidates = decode_yolo11_one_class_candidates(outputs[0], transform=transform)
    ranked = sorted(candidates, key=lambda item: item.confidence, reverse=True)
    selected = [
        {
            "confidence": round(float(item.confidence), 6),
            "bbox_xyxy_px": [round(float(value), 3) for value in item.bbox_xyxy_px],
        }
        for item in ranked
        if item.confidence >= args.minimum_score
    ][: args.limit]
    result = {
        "schema_version": "sie.ar0234.yolo_raw_score_diagnostic.v1",
        "image": str(args.image),
        "image_shape_hwc": [int(value) for value in frame.shape],
        "model": str(args.model),
        "model_sha256": model_sha256,
        "onnxruntime_version": ort.__version__,
        "output_shape": [int(value) for value in np.asarray(outputs[0]).shape],
        "raw_geometrically_valid_candidate_count": len(candidates),
        "highest_raw_score": (
            None if not ranked else round(float(ranked[0].confidence), 6)
        ),
        "minimum_score": args.minimum_score,
        "candidates_at_or_above_minimum_score": selected,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
