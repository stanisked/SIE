#!/usr/bin/env python3
"""Private AR0234 face-dataset capture, independent of ROS and ESP32.

Capture full RGB frames for later face <-> body binding; YuNet reports visible
face locations, but never labels or identifies a person. Does NOT enroll a
biometric template or use SFace. The camera's MJPG is decoded by OpenCV then
re-encoded as JPEG. Host timestamps are NOT camera exposure timestamps.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import time

import cv2


DEFAULT_ROOT = Path.home() / ".local/state/sie/private/face_datasets"
MAX_DURATION_S = 360.0
MAX_SAMPLES = 1000


def create_detector(yunet_path: Path):
    factory = getattr(cv2, "FaceDetectorYN_create", None)
    if factory is None and hasattr(cv2, "FaceDetectorYN"):
        factory = cv2.FaceDetectorYN.create
    if factory is None:
        raise RuntimeError("YUNET_OPENCV_API_UNAVAILABLE")
    return factory(str(yunet_path), "", (320, 320), .85, .3, 5000)


def face_detections(detector, bgr):
    h, w = bgr.shape[:2]
    scale = min(1.0, 960.0 / max(h, w))
    if scale < 1.0:
        scaled = cv2.resize(bgr, (round(w*scale), round(h*scale)),
                            interpolation=cv2.INTER_AREA)
    else:
        scaled = bgr
    detector.setInputSize((scaled.shape[1], scaled.shape[0]))
    _, faces = detector.detect(scaled)
    result = []
    if faces is not None:
        for face in faces:
            box = [float(v / scale) for v in face[:4]]
            result.append({"xywh_px": [round(v, 2) for v in box],
                           "score": round(float(face[14]), 4)})
    return result


def capture_session(*, camera, detector, directory: Path, duration_s: float,
                    sample_interval_s: float, monotonic=time.monotonic,
                    utc_now=lambda: datetime.now(timezone.utc)):
    """Write bounded images+manifest. Caller owns camera creation and release."""
    if (not 0 < duration_s <= MAX_DURATION_S
            or not .25 <= sample_interval_s <= 3.0
            or math.ceil(duration_s / sample_interval_s) > MAX_SAMPLES):
        raise ValueError("CAPTURE_BOUNDS_INVALID")
    if directory.exists():
        raise FileExistsError("DATASET_DIRECTORY_EXISTS")
    directory.mkdir(parents=True, mode=0o700)
    os.chmod(directory, 0o700)
    images = directory / "rgb"
    images.mkdir(mode=0o700)
    stats = {"frames_saved": 0, "frames_with_face": 0,
             "frames_without_face": 0, "frames_multi_face": 0,
             "read_failures": 0, "frame_shape_mismatch": 0}
    started = monotonic()
    next_sample = started
    end = started + duration_s
    manifest_path = directory / "manifest.jsonl"
    with manifest_path.open("x", encoding="utf-8") as out:
        os.chmod(manifest_path, 0o600)
        while monotonic() < end and stats["frames_saved"] < MAX_SAMPLES:
            ok, frame = camera.read()
            observed = monotonic()
            if not ok or frame is None:
                stats["read_failures"] += 1
                if stats["read_failures"] >= 10:
                    raise RuntimeError("AR0234_READ_FAILED_10_TIMES")
                continue
            stats["read_failures"] = 0
            if frame.shape != (1200,1920,3) or frame.dtype.name != "uint8":
                stats["frame_shape_mismatch"] += 1
                raise RuntimeError("AR0234_UNEXPECTED_FRAME_SHAPE")
            if observed < next_sample:
                continue
            # Skip missed intervals rather than writing duplicates of one frame.
            next_sample = observed + sample_interval_s
            faces = face_detections(detector, frame)
            ok, jpeg = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 94])
            if not ok:
                raise RuntimeError("RGB_JPEG_ENCODING_FAILED")
            payload = jpeg.tobytes()
            index = stats["frames_saved"] + 1
            filename = f"rgb_{index:05d}.jpg"
            with (images / filename).open("xb") as stream:
                os.chmod(images / filename, 0o600)
                stream.write(payload)
            item = {"sample_index": index, "file": f"rgb/{filename}",
                    "capture_clock": "host_after_cv2_read_not_driver_exposure",
                    "host_monotonic_s": round(observed, 6),
                    "host_utc": utc_now().isoformat(),
                    "frame_width": 1920, "frame_height": 1200,
                    "source_device": "/dev/video0",
                    "source_pixel_format": "MJPG",
                    "saved_jpeg_is_reencoded": True,
                    "jpeg_sha256": hashlib.sha256(payload).hexdigest(),
                    "faces": faces, "face_body_associated": False,
                    "identity_verified": False}
            out.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")
            out.flush()
            stats["frames_saved"] += 1
            if not faces:
                stats["frames_without_face"] += 1
            elif len(faces) == 1:
                stats["frames_with_face"] += 1
            else:
                stats["frames_multi_face"] += 1
    summary = {"scope": "PRIVATE_FACE_DATASET_NO_IDENTIFICATION_OR_MOTOR_CONTROL",
               "directory": str(directory), "duration_requested_s": duration_s,
               "sample_interval_s": sample_interval_s, **stats}
    with (directory / "summary.json").open("x", encoding="utf-8") as out:
        os.chmod(directory / "summary.json", 0o600)
        json.dump(summary, out, ensure_ascii=False, indent=2)
        out.write("\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("pilot", "enrollment", "validation", "dynamic"),
                        required=True)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--sample-interval-s", type=float, default=.5)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--yunet", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    if args.device != "/dev/video0":
        parser.error("Only verified AR0234 /dev/video0 is supported in v1")
    if (not math.isfinite(args.duration_s) or not 0 < args.duration_s <= MAX_DURATION_S
            or not math.isfinite(args.sample_interval_s)
            or not .25 <= args.sample_interval_s <= 3.0
            or math.ceil(args.duration_s / args.sample_interval_s) > MAX_SAMPLES):
        parser.error("Requested duration/sampling outside capture bounds")
    if not args.yunet.is_file():
        parser.error("Pinned YuNet model not found")
    os.umask(0o077)
    root = args.output_root.expanduser()
    root.mkdir(parents=True, mode=0o700, exist_ok=True)
    os.chmod(root, 0o700)
    name = (f"{args.stage}_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_"
            f"{secrets.token_hex(3)}")
    dest = root / name
    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError("AR0234_CAMERA_BUSY_OR_UNAVAILABLE")
    try:
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        camera.set(cv2.CAP_PROP_FPS, 15)
        if (round(camera.get(cv2.CAP_PROP_FRAME_WIDTH)) != 1920
                or round(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)) != 1200
                or int(camera.get(cv2.CAP_PROP_FOURCC)) !=
                    cv2.VideoWriter_fourcc(*"MJPG")):
            raise RuntimeError("AR0234_MJPG_1920X1200_NOT_CONFIRMED")
        detector = create_detector(args.yunet)
        print(f"FACE_CAPTURE_BEGIN stage={args.stage} dir={dest}", flush=True)
        summary = capture_session(
            camera=camera, detector=detector, directory=dest,
            duration_s=args.duration_s,
            sample_interval_s=args.sample_interval_s)
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    finally:
        camera.release()


if __name__ == "__main__":
    main()
