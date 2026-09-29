#!/usr/bin/env python3
"""Headless checkerboard capture for fresh AR0234-to-OV9281 extrinsic calibration.

The tool forces auto_exposure=3 on both UVC cameras, writes diagnostic JPEG previews,
and captures raw triples only. It never estimates range, publishes ROS messages, or
activates a calibration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def configure_auto_exposure(device: str, value: int) -> str:
    subprocess.run(
        ["v4l2-ctl", "-d", device, "-c", f"auto_exposure={value}"],
        check=True,
        text=True,
        capture_output=True,
    )
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "-C", "auto_exposure"],
        check=True,
        text=True,
        capture_output=True,
    )
    text = result.stdout.strip()
    if f": {value}" not in text:
        raise RuntimeError(f"auto_exposure verification failed for {device}: {text}")
    return text


def find_corners(image: np.ndarray, board_size: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    found, corners = cv2.findChessboardCornersSB(gray, board_size, flags=flags)
    if not found or corners is None:
        return None
    return corners.reshape(-1, 2).astype(np.float32)


def open_camera(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open camera: {device}")
    return capture


def read_frame(capture: cv2.VideoCapture, name: str) -> tuple[np.ndarray, float]:
    ok, frame = capture.read()
    timestamp_monotonic = time.monotonic()
    if not ok or frame is None:
        raise RuntimeError(f"Camera returned no frame: {name}")
    return frame, timestamp_monotonic


def write_metadata(path: Path, metadata: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    temporary.replace(path)


def preview_tile(image: np.ndarray, label: str, detected: bool) -> np.ndarray:
    tile = cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)
    colour = (40, 200, 40) if detected else (40, 40, 230)
    status = "corners: YES" if detected else "corners: NO"
    cv2.rectangle(tile, (0, 0), (640, 42), (0, 0, 0), thickness=-1)
    cv2.putText(tile, label, (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(tile, status, (12, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1)
    return tile


def save_preview(
    preview_dir: Path,
    ar_frame: np.ndarray,
    captured_first_frame: np.ndarray,
    captured_second_frame: np.ndarray,
    ar_found: bool,
    captured_first_found: bool,
    captured_second_found: bool,
) -> None:
    files = (
        ("latest_ar0234.jpg", ar_frame),
        ("latest_captured_first_physical_right.jpg", captured_first_frame),
        ("latest_captured_second_physical_left.jpg", captured_second_frame),
    )
    for filename, frame in files:
        if not cv2.imwrite(str(preview_dir / filename), frame):
            raise RuntimeError(f"Could not save preview {filename}")
    triplet = np.hstack(
        (
            preview_tile(ar_frame, "AR0234", ar_found),
            preview_tile(captured_first_frame, "OV9281 first / physical_right", captured_first_found),
            preview_tile(captured_second_frame, "OV9281 second / physical_left", captured_second_found),
        )
    )
    if not cv2.imwrite(str(preview_dir / "latest_triplet.jpg"), triplet):
        raise RuntimeError("Could not save preview triplet")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--ar-device",
        default="/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0",
    )
    parser.add_argument(
        "--stereo-device",
        default="/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0",
    )
    parser.add_argument("--auto-exposure", type=int, default=3)
    parser.add_argument("--ar-width", type=int, default=1920)
    parser.add_argument("--ar-height", type=int, default=1200)
    parser.add_argument("--ar-fps", type=float, default=30.0)
    parser.add_argument("--stereo-width", type=int, default=2560)
    parser.add_argument("--stereo-height", type=int, default=800)
    parser.add_argument("--stereo-fps", type=float, default=60.0)
    parser.add_argument("--board-cols", type=int, default=9)
    parser.add_argument("--board-rows", type=int, default=6)
    parser.add_argument("--square-size-mm", type=float, default=24.5)
    parser.add_argument("--target-pairs", type=int, default=50)
    parser.add_argument("--min-interval-s", type=float, default=0.75)
    parser.add_argument("--min-motion-px", type=float, default=30.0)
    parser.add_argument("--max-skew-ms", type=float, default=80.0)
    parser.add_argument("--preview-interval-s", type=float, default=2.0)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    args = parser.parse_args()

    if args.target_pairs < 1:
        raise ValueError("--target-pairs must be positive")
    if args.stereo_width % 2:
        raise ValueError("--stereo-width must be divisible by two")
    if not args.ar_intrinsic.is_file():
        raise FileNotFoundError(f"AR0234 intrinsic file not found: {args.ar_intrinsic}")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")

    board_size = (args.board_cols, args.board_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    ar_dir = args.output_dir / "ar0234"
    left_dir = args.output_dir / "stereo_left"
    right_dir = args.output_dir / "stereo_right"
    preview_dir = args.output_dir / "debug_preview"
    for directory in (ar_dir, left_dir, right_dir, preview_dir):
        directory.mkdir()

    ar_exposure = configure_auto_exposure(args.ar_device, args.auto_exposure)
    stereo_exposure = configure_auto_exposure(args.stereo_device, args.auto_exposure)
    metadata = {
        "schema_version": "sie.ar0234_ov9281.extrinsic_capture.v1",
        "capture_started_at_utc": utc_now(),
        "status": "IN_PROGRESS",
        "ar0234": {
            "device": args.ar_device,
            "image_size_px": {"width": args.ar_width, "height": args.ar_height},
            "requested_fps": args.ar_fps,
            "auto_exposure": ar_exposure,
            "intrinsic_path": str(args.ar_intrinsic),
            "intrinsic_sha256": sha256_file(args.ar_intrinsic),
        },
        "ov9281_stereo": {
            "device": args.stereo_device,
            "combined_image_size_px": {"width": args.stereo_width, "height": args.stereo_height},
            "eye_image_size_px": {"width": args.stereo_width // 2, "height": args.stereo_height},
            "requested_fps": args.stereo_fps,
            "auto_exposure": stereo_exposure,
            "captured_first_half_semantics": "physical_right",
            "captured_second_half_semantics": "physical_left",
            "storage_note": "stereo_left is captured first; stereo_right is captured second",
        },
        "target": {
            "checkerboard_inner_corners": list(board_size),
            "square_size_mm": args.square_size_mm,
        },
        "capture_policy": {
            "target_pairs": args.target_pairs,
            "min_interval_s": args.min_interval_s,
            "min_motion_px": args.min_motion_px,
            "max_sequential_skew_ms": args.max_skew_ms,
            "preview_interval_s": args.preview_interval_s,
        },
        "debug_preview": {
            "directory": str(preview_dir),
            "triplet": str(preview_dir / "latest_triplet.jpg"),
        },
        "pairs": [],
    }
    metadata_path = args.output_dir / "capture_metadata.json"
    write_metadata(metadata_path, metadata)

    ar_capture = open_camera(args.ar_device, args.ar_width, args.ar_height, args.ar_fps)
    stereo_capture = open_camera(
        args.stereo_device, args.stereo_width, args.stereo_height, args.stereo_fps
    )
    last_center: np.ndarray | None = None
    last_saved_at = 0.0
    last_preview_at = 0.0

    print(f"AR0234 {ar_exposure}; OV9281 {stereo_exposure}")
    print("Capture started. Move the checkerboard through different positions, angles, and distances.")
    print(f"Preview updates every {args.preview_interval_s:g} s: {preview_dir / 'latest_triplet.jpg'}")
    try:
        while len(metadata["pairs"]) < args.target_pairs:
            ar_frame, ar_time = read_frame(ar_capture, "AR0234")
            stereo_frame, stereo_time = read_frame(stereo_capture, "OV9281 stereo")
            if ar_frame.shape[:2] != (args.ar_height, args.ar_width):
                raise RuntimeError(f"Unexpected AR0234 size: {ar_frame.shape[1]}x{ar_frame.shape[0]}")
            if stereo_frame.shape[:2] != (args.stereo_height, args.stereo_width):
                raise RuntimeError(
                    f"Unexpected OV9281 combined size: {stereo_frame.shape[1]}x{stereo_frame.shape[0]}"
                )

            split = args.stereo_width // 2
            captured_first_frame, captured_second_frame = stereo_frame[:, :split], stereo_frame[:, split:]
            ar_corners = find_corners(ar_frame, board_size)
            captured_first_corners = find_corners(captured_first_frame, board_size)
            captured_second_corners = find_corners(captured_second_frame, board_size)
            now = time.monotonic()
            if now - last_preview_at >= args.preview_interval_s:
                save_preview(
                    preview_dir, ar_frame, captured_first_frame, captured_second_frame,
                    ar_corners is not None, captured_first_corners is not None, captured_second_corners is not None,
                )
                last_preview_at = now

            if any(corners is None for corners in (ar_corners, captured_first_corners, captured_second_corners)):
                continue

            skew_ms = abs(stereo_time - ar_time) * 1000.0
            if skew_ms > args.max_skew_ms:
                print(f"SKIP sequential skew={skew_ms:.1f} ms")
                continue

            center = np.mean(ar_corners, axis=0)
            motion = float("inf") if last_center is None else float(np.linalg.norm(center - last_center))
            if now - last_saved_at < args.min_interval_s or motion < args.min_motion_px:
                continue

            index = len(metadata["pairs"]) + 1
            filename = f"{index:04d}.png"
            for path, frame in (
                (ar_dir / filename, ar_frame),
                (left_dir / filename, captured_first_frame),
                (right_dir / filename, captured_second_frame),
            ):
                if not cv2.imwrite(str(path), frame):
                    raise RuntimeError(f"Could not save {path}")

            metadata["pairs"].append(
                {
                    "pair_id": f"ar0234-ov9281-extrinsic:{index:04d}",
                    "filename": filename,
                    "captured_at_utc": utc_now(),
                    "sequential_skew_ms": skew_ms,
                    "ar0234_corner_center_px": center.tolist(),
                    "captured_first_corner_center_px": np.mean(captured_first_corners, axis=0).tolist(),
                    "captured_second_corner_center_px": np.mean(captured_second_corners, axis=0).tolist(),
                    "ar0234_sha256": sha256_file(ar_dir / filename),
                    "stereo_left_sha256": sha256_file(left_dir / filename),
                    "stereo_right_sha256": sha256_file(right_dir / filename),
                }
            )
            write_metadata(metadata_path, metadata)
            last_center, last_saved_at = center, now
            print(
                f"SAVED {index:02d}/{args.target_pairs}: {filename}; "
                f"sequential_skew={skew_ms:.1f} ms; motion={motion:.1f} px"
            )
    except KeyboardInterrupt:
        print("\nCapture interrupted by user.")
    finally:
        ar_capture.release()
        stereo_capture.release()

    metadata["capture_finished_at_utc"] = utc_now()
    metadata["status"] = "COMPLETE" if len(metadata["pairs"]) >= args.target_pairs else "INCOMPLETE"
    write_metadata(metadata_path, metadata)
    print(f"Capture status: {metadata['status']}; pairs={len(metadata['pairs'])}")
    return 0 if metadata["status"] == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
