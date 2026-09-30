#!/usr/bin/env python3
"""Capture an immutable headless AR0234 image session for detector revalidation.

The tool intentionally makes no SIE measurement, navigation decision, or motor
request. It persists raw frames plus a manifest so a later labelled dataset can
be reproduced from the actual Raspberry Pi camera configuration.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0",
        help="AR0234 V4L2 device path",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--frame-count", type=int, required=True)
    parser.add_argument("--interval-s", type=float, default=0.5)
    parser.add_argument("--session-label", required=True)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--warmup-frames", type=int, default=30)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.frame_count <= 0:
        raise ValueError("--frame-count must be positive")
    if args.interval_s < 0.0:
        raise ValueError("--interval-s must be non-negative")
    if args.warmup_frames < 0:
        raise ValueError("--warmup-frames must be non-negative")
    if not args.session_label.strip():
        raise ValueError("--session-label must be non-empty")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open AR0234 device: {args.device}")
    try:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Required proven setting for the AR0234/OV9281 setup.
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

        for _ in range(args.warmup_frames):
            ok, _ = capture.read()
            if not ok:
                raise RuntimeError("AR0234 failed while warming up")

        frames_dir = args.output_dir / "images"
        frames_dir.mkdir()
        records: list[dict[str, object]] = []
        for index in range(1, args.frame_count + 1):
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError(f"AR0234 frame read failed at sample {index}")
            filename = f"ar0234_{index:04d}.jpg"
            path = frames_dir / filename
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"Could not save {path}")
            records.append(
                {
                    "file": f"images/{filename}",
                    "captured_at_utc": _utc_now(),
                    "shape_hwc": [int(value) for value in frame.shape],
                }
            )
            print(f"SAVED {index:03d}/{args.frame_count}: {path.name}", flush=True)
            remaining = args.interval_s - (time.monotonic() - started)
            if index < args.frame_count and remaining > 0.0:
                time.sleep(remaining)

        manifest = {
            "schema_version": "sie.ar0234.pi_detector_capture.v1",
            "created_at_utc": _utc_now(),
            "session_label": args.session_label,
            "camera": {
                "device": args.device,
                "fourcc": "MJPG",
                "requested_width": args.width,
                "requested_height": args.height,
                "requested_fps": args.fps,
                "auto_exposure": 3,
                "buffer_size": 1,
            },
            "frame_count": len(records),
            "frames": records,
        }
        manifest_path = args.output_dir / "capture_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        print(f"Capture status: COMPLETE; frames={len(records)}")
        print(f"Manifest: {manifest_path}")
        return 0
    finally:
        capture.release()


if __name__ == "__main__":
    sys.exit(main())
