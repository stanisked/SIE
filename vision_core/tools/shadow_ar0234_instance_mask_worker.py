#!/usr/bin/env python3
"""Long-lived, isolated CPU segmentation worker for diagnostic ROS audits.

One JSON request and response per line on stdin/stdout. The worker only reads
saved AR frames and writes mask evidence into the same cycle directory.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
import time
from pathlib import Path

from segment_saved_ar0234_instances import segment_frame, sha256


def emit(value: dict) -> None:
    sys.stdout.write(json.dumps(value, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    if not args.model.is_file():
        emit({"status": "UNAVAILABLE", "reason": "MODEL_MISSING"})
        return
    started = time.monotonic()
    try:
        with contextlib.redirect_stdout(sys.stderr):
            from ultralytics import YOLO
            model = YOLO(str(args.model), task="segment")
        if model.task != "segment" or model.names.get(0) != "person":
            raise ValueError("expected COCO person instance segmentation model")
    except Exception as error:
        emit({"status": "UNAVAILABLE", "reason": type(error).__name__})
        return
    model_sha = sha256(args.model)
    emit({"status": "READY", "model_sha256": model_sha,
          "load_elapsed_s": time.monotonic() - started})
    for line in sys.stdin:
        started = time.monotonic()
        cycle_id = None
        try:
            request = json.loads(line)
            image = Path(request["image"])
            output_dir = Path(request["output_dir"])
            if image.name != "ar0234_source.jpg" or image.parent != output_dir:
                raise ValueError("source image must belong to evidence cycle")
            cycle_id = str(request["cycle_id"])
            if output_dir.name != cycle_id:
                raise ValueError("cycle id and evidence directory disagree")
            with contextlib.redirect_stdout(sys.stderr):
                result = segment_frame(image, cycle_id, request["candidate"], model,
                                       model_sha, output_dir, 0.50, 0.15)
            result["elapsed_s"] = time.monotonic() - started
            emit(result)
        except Exception as error:
            emit({"cycle_id": cycle_id, "status": "MASK_UNAVAILABLE",
                  "reason": type(error).__name__,
                  "elapsed_s": time.monotonic() - started})


if __name__ == "__main__":
    main()
