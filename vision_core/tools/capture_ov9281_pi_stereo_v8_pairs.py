#!/usr/bin/env python3
"""Capture raw OV9281 checkerboard pairs for a non-active v8 candidate.

This tool never changes a runtime calibration or publishes a range.  It saves
only the two unmodified UVC halves plus a provenance manifest.  Pi stream
identity is fixed by the established device contract: the first UVC half is
physical_right and the second UVC half is physical_left.
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


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2), encoding="utf-8")
    temporary.replace(path)


def configure_auto_exposure(device: str, value: int) -> str:
    subprocess.run(
        ["v4l2-ctl", "-d", device, "-c", f"auto_exposure={value}"],
        check=True, text=True, capture_output=True,
    )
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "-C", "auto_exposure"],
        check=True, text=True, capture_output=True,
    )
    verified = result.stdout.strip()
    if f": {value}" not in verified:
        raise RuntimeError(f"auto_exposure verification failed: {verified}")
    return verified


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, result = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or result is None else result.reshape(-1, 2).astype(np.float32)


def save_preview(
    preview_dir: Path,
    first: np.ndarray,
    second: np.ndarray,
    first_corners: np.ndarray | None,
    second_corners: np.ndarray | None,
    board: tuple[int, int],
) -> None:
    for name, image in (("latest_captured_first_physical_right.jpg", first),
                        ("latest_captured_second_physical_left.jpg", second)):
        if not cv2.imwrite(str(preview_dir / name), image):
            raise RuntimeError(f"Could not save {preview_dir / name}")
    tiles = []
    for image, label, corners in (
        (first, "first UVC half / physical_right", first_corners),
        (second, "second UVC half / physical_left", second_corners),
    ):
        shown = image.copy()
        if corners is not None:
            cv2.drawChessboardCorners(shown, board, corners.reshape(-1, 1, 2), True)
        tile = cv2.resize(shown, (640, 400), interpolation=cv2.INTER_AREA)
        colour = (50, 220, 50) if corners is not None else (40, 40, 230)
        cv2.rectangle(tile, (0, 0), (640, 40), (0, 0, 0), thickness=-1)
        cv2.putText(tile, label, (12, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(tile, "corners: YES" if corners is not None else "corners: NO",
                    (12, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1)
        tiles.append(tile)
    if not cv2.imwrite(str(preview_dir / "latest_pair.jpg"), np.hstack(tiles)):
        raise RuntimeError("Could not save pair preview")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--device",
        default="/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0",
    )
    parser.add_argument("--width", type=int, default=2560)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--auto-exposure", type=int, default=3)
    parser.add_argument("--board-cols", type=int, default=9)
    parser.add_argument("--board-rows", type=int, default=6)
    parser.add_argument("--square-size-mm", type=float, default=48.0)
    parser.add_argument("--target-pairs", type=int, default=72)
    parser.add_argument("--warmup-frames", type=int, default=180)
    parser.add_argument("--min-interval-s", type=float, default=0.75)
    parser.add_argument("--min-motion-px", type=float, default=30.0)
    parser.add_argument("--preview-interval-s", type=float, default=2.0)
    parser.add_argument("--accepted-center-y-min-px", type=float, default=0.0)
    parser.add_argument("--accepted-center-y-max-px", type=float, default=float("inf"))
    args = parser.parse_args()

    if args.width % 2:
        raise ValueError("--width must be divisible by two")
    if args.target_pairs < 67:
        raise ValueError("--target-pairs must be at least 67 for the solver split")
    if args.accepted_center_y_min_px > args.accepted_center_y_max_px:
        raise ValueError("accepted centre Y minimum exceeds maximum")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {args.output_dir}")
    board = (args.board_cols, args.board_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    left_dir = args.output_dir / "stereo_left"
    right_dir = args.output_dir / "stereo_right"
    preview_dir = args.output_dir / "debug_preview"
    for directory in (left_dir, right_dir, preview_dir):
        directory.mkdir()

    exposure = configure_auto_exposure(args.device, args.auto_exposure)
    metadata = {
        "schema_version": "sie.ov9281.pi_stereo_capture.v8",
        "status": "IN_PROGRESS",
        "capture_started_at_utc": utc_now(),
        "ov9281_stereo": {
            "device": args.device,
            "combined_image_size_px": {"width": args.width, "height": args.height},
            "eye_image_size_px": {"width": args.width // 2, "height": args.height},
            "requested_fps": args.fps,
            "auto_exposure": exposure,
            "captured_first_half_semantics": "physical_right",
            "captured_second_half_semantics": "physical_left",
            "storage_note": "stereo_left stores first/physical_right; stereo_right stores second/physical_left",
        },
        "target": {
            "checkerboard_inner_corners": list(board),
            "square_size_mm": args.square_size_mm,
            "surface": "matte",
        },
        "capture_policy": {
            "target_pairs": args.target_pairs,
            "warmup_frames": args.warmup_frames,
            "min_interval_s": args.min_interval_s,
            "min_motion_px": args.min_motion_px,
            "preview_interval_s": args.preview_interval_s,
            "accepted_center_y_min_px": args.accepted_center_y_min_px,
            "accepted_center_y_max_px": args.accepted_center_y_max_px,
        },
        "debug_preview": {"directory": str(preview_dir), "pair": str(preview_dir / "latest_pair.jpg")},
        "pairs": [],
    }
    metadata_path = args.output_dir / "capture_metadata.json"
    write_json(metadata_path, metadata)

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open camera: {args.device}")
    try:
        for _ in range(args.warmup_frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("Camera returned no frame during warmup")
        print(f"OV9281 {exposure}; warmup complete.")
        print("Move the matte 9x6-inner-corner board across the image, with varied distance and tilt.")
        print(f"Saving up to {args.target_pairs} raw pairs; Ctrl+C marks this session INCOMPLETE.")
        last_center: np.ndarray | None = None
        last_saved_at = 0.0
        last_preview_at = 0.0
        while len(metadata["pairs"]) < args.target_pairs:
            ok, combined = capture.read()
            if not ok or combined is None:
                raise RuntimeError("Camera returned no frame")
            if combined.shape[:2] != (args.height, args.width):
                raise RuntimeError(f"Unexpected OV9281 size: {combined.shape[1]}x{combined.shape[0]}")
            split = args.width // 2
            first, second = combined[:, :split], combined[:, split:]
            first_corners, second_corners = find_corners(first, board), find_corners(second, board)
            now = time.monotonic()
            if now - last_preview_at >= args.preview_interval_s:
                save_preview(preview_dir, first, second, first_corners, second_corners, board)
                last_preview_at = now
            if first_corners is None or second_corners is None:
                continue
            center = np.mean(second_corners, axis=0)
            if not args.accepted_center_y_min_px <= center[1] <= args.accepted_center_y_max_px:
                continue
            motion = float("inf") if last_center is None else float(np.linalg.norm(center - last_center))
            if now - last_saved_at < args.min_interval_s or motion < args.min_motion_px:
                continue
            index = len(metadata["pairs"]) + 1
            filename = f"{index:04d}.png"
            for path, image in ((left_dir / filename, first), (right_dir / filename, second)):
                if not cv2.imwrite(str(path), image):
                    raise RuntimeError(f"Could not save {path}")
            metadata["pairs"].append({
                "pair_id": f"ov9281-pi-v8:{index:04d}",
                "filename": filename,
                "captured_at_utc": utc_now(),
                "captured_first_corner_center_px": np.mean(first_corners, axis=0).tolist(),
                "captured_second_corner_center_px": center.tolist(),
                "stereo_left_sha256": sha256_file(left_dir / filename),
                "stereo_right_sha256": sha256_file(right_dir / filename),
            })
            write_json(metadata_path, metadata)
            last_center, last_saved_at = center, now
            print(
                f"SAVED {index:02d}/{args.target_pairs}: {filename}; "
                f"centre=({center[0]:.1f}, {center[1]:.1f}) px; motion={motion:.1f} px"
            )
    except KeyboardInterrupt:
        print("\nCapture interrupted by user.")
    finally:
        capture.release()

    metadata["capture_finished_at_utc"] = utc_now()
    metadata["status"] = "COMPLETE" if len(metadata["pairs"]) >= args.target_pairs else "INCOMPLETE"
    write_json(metadata_path, metadata)
    print(f"Capture status: {metadata['status']}; pairs={len(metadata['pairs'])}")
    return 0 if metadata["status"] == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
