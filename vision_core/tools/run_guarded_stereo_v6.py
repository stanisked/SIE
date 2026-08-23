#!/usr/bin/env python3

"""Fail-closed live SIE Measurement producer for stereo calibration v6."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


RAW_WIDTH = 2560
RAW_HEIGHT = 800
REQUIRED_FPS = 60.0
EXPECTED_FRAME_SHAPE = (RAW_HEIGHT, RAW_WIDTH, 3)
EYE_WIDTH = RAW_WIDTH // 2
EYE_HEIGHT = RAW_HEIGHT
MEASURED_DEPTH_MIN_M = 0.38
MEASURED_DEPTH_MAX_M = 2.55
VALIDATED_GROUND_TRUTH_MIN_M = 0.40
VALIDATED_GROUND_TRUTH_MAX_M = 2.50
MINIMUM_VALID_RATIO = 0.90
MAXIMUM_DISPARITY_MAD_PX = 0.50
MAXIMUM_DEPTH_SPATIAL_MAD_M = 0.020
NUM_DISPARITIES = 192
RUNTIME_PROFILE_ID = "stereo_runtime_v2_sgbm192_roi100"
EXPECTED_CALIBRATION_ID = "stereo_calibration_v6"
EXPECTED_CALIBRATION_SHA256 = (
    "bb8fb665c6e06e2cbb633cf4c3c61aa74933dd253c9c7950a8420591975dd5e7"
)
PREVIEW_WINDOW = "SIE Stereo v6 | Rectified LEFT | Q/ESC to quit"


def json_line(payload: dict) -> None:
    print(json.dumps(payload, sort_keys=True), flush=True)


class MeasurementJsonlWriter:
    """Single-writer append-only sink containing only SIE Measurement objects."""

    def __init__(self, path: Path) -> None:
        self.path = path.resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fd = os.open(
            self.path,
            os.O_WRONLY | os.O_CREAT | os.O_APPEND,
            0o640,
        )
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self.fd)
            raise RuntimeError(
                f"measurement JSONL already has another writer: {self.path}"
            )
        self.written_count = 0

    def write(self, measurement: dict) -> None:
        if measurement.get("schema_version") != "sie_measurement_v1":
            raise RuntimeError("refusing to write an invalid Measurement schema")
        if measurement.get("measurement_type") != "stereo_roi_depth":
            raise RuntimeError("refusing to write an unexpected Measurement type")
        payload = (
            json.dumps(
                measurement,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise RuntimeError(
                f"partial Measurement JSONL write: {written}/{len(payload)} bytes"
            )
        self.written_count += 1

    def close(self) -> None:
        if self.fd is not None:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
            os.close(self.fd)
            self.fd = None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_runtime_policy(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError("runtime policy must be a JSON object")
    return value


def build_sie_measurement(
    *,
    run_id: str,
    device: str,
    depth_frame,
    roi_result: dict,
    runtime_policy_path: Path,
    runtime_policy_sha256: str,
) -> dict:
    sequence = int(depth_frame.sequence)
    return {
        "schema_version": "sie_measurement_v1",
        "measurement_id": f"{run_id}:depth:{sequence}",
        "measurement_type": "stereo_roi_depth",
        "emitted_at": datetime.now(timezone.utc).isoformat(),
        "value": {
            "name": "depth",
            "value": roi_result["depth_median_m"],
            "unit": "m",
        },
        "reference_frame": depth_frame.reference_frame,
        "source_observation": {
            "observation_id": f"{run_id}:stereo_frame:{sequence}",
            "sensor_id": "ov9281_stereo_camera",
            "device": device,
            "frame_sequence": sequence,
            "capture_mode": "MJPG 2560x800 @ 60 FPS",
        },
        "calibration": {
            "calibration_id": depth_frame.calibration_id,
            "calibration_sha256": depth_frame.calibration_sha256,
            "activation_record_sha256": (
                depth_frame.activation_record_sha256
            ),
        },
        "runtime_policy": {
            "runtime_profile_id": RUNTIME_PROFILE_ID,
            "path": str(runtime_policy_path),
            "sha256": runtime_policy_sha256,
        },
        "validated_operating_claim": {
            "ground_truth_distance_range_m": [
                VALIDATED_GROUND_TRUTH_MIN_M,
                VALIDATED_GROUND_TRUTH_MAX_M,
            ],
            "measured_depth_acceptance_range_m": [
                MEASURED_DEPTH_MIN_M,
                MEASURED_DEPTH_MAX_M,
            ],
        },
        "region": {
            "image": "rectified_left",
            "bounds_xyxy": roi_result["bounds_xyxy"],
        },
        "quality": {
            "status": roi_result["quality_status"],
            "valid_pixel_ratio": roi_result["valid_ratio"],
            "disparity_median_px": roi_result["disparity_median_px"],
            "disparity_mad_px": roi_result["disparity_mad_px"],
            "depth_spatial_mad_m": roi_result["depth_mad_m"],
            "within_validated_depth_range": (
                roi_result["within_validated_depth_range"]
            ),
        },
        "conditions": {
            "temperatures_c": depth_frame.temperature_gate.temperatures_c,
            "temperature_state_age_s": depth_frame.temperature_gate.state_age_s,
        },
    }


def draw_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int],
    *,
    scale: float = 0.7,
) -> None:
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


def render_preview(
    depth_frame,
    roi_result: dict,
    *,
    processing_fps: float,
    preview_width: int,
) -> np.ndarray:
    preview = depth_frame.left_rectified_bgr.copy()
    x1, y1, x2, y2 = roi_result["bounds_xyxy"]
    passed = roi_result["quality_status"] == "PASS"
    color = (0, 220, 0) if passed else (0, 0, 255)
    cv2.rectangle(preview, (x1, y1), (x2, y2), color, 3)

    depth_m = roi_result["depth_median_m"]
    if passed:
        headline = f"PASS  depth={depth_m:.3f} m"
    elif depth_m is None:
        headline = "REJECTED  no valid depth"
    elif not roi_result["within_validated_depth_range"]:
        headline = (
            f"REJECTED  depth={depth_m:.3f} m outside "
            f"{MEASURED_DEPTH_MIN_M:.2f}-{MEASURED_DEPTH_MAX_M:.2f} m"
        )
    else:
        headline = f"REJECTED  ROI quality  depth={depth_m:.3f} m"

    disparity_mad = roi_result["disparity_mad_px"]
    disparity_mad_text = (
        "n/a" if disparity_mad is None else f"{disparity_mad:.3f} px"
    )
    temperatures = depth_frame.temperature_gate.temperatures_c
    lines = [
        headline,
        (
            f"valid={roi_result['valid_ratio'] * 100.0:.1f}%  "
            f"disparity MAD={disparity_mad_text}"
        ),
        (
            f"temperature L/R={temperatures['camera_left']:.2f}/"
            f"{temperatures['camera_right']:.2f} C"
        ),
        f"depth compute={processing_fps:.2f} FPS  Q/ESC: quit",
    ]
    for index, line in enumerate(lines):
        draw_text(preview, line, (24, 36 + index * 32), color)

    if preview_width == EYE_WIDTH:
        return preview
    preview_height = int(round(EYE_HEIGHT * preview_width / EYE_WIDTH))
    return cv2.resize(
        preview,
        (preview_width, preview_height),
        interpolation=cv2.INTER_AREA,
    )


def median_and_mad(values: np.ndarray) -> tuple[float, float]:
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    return median, mad


def roi_depth_diagnostic(
    depth_frame,
    *,
    center_x: int,
    center_y: int,
    size: int,
) -> dict:
    half = size // 2
    x1 = center_x - half
    y1 = center_y - half
    x2 = x1 + size
    y2 = y1 + size
    roi = np.s_[y1:y2, x1:x2]
    valid = depth_frame.valid_mask[roi]
    valid_ratio = float(np.mean(valid))

    if not np.any(valid):
        return {
            "bounds_xyxy": [x1, y1, x2, y2],
            "valid_ratio": valid_ratio,
            "disparity_median_px": None,
            "disparity_mad_px": None,
            "depth_median_m": None,
            "depth_mad_m": None,
            "within_validated_depth_range": False,
            "quality_status": "FAIL",
        }

    disparity_values = depth_frame.disparity_px[roi][valid]
    depth_values = depth_frame.depth_m[roi][valid]
    disparity_median, disparity_mad = median_and_mad(disparity_values)
    depth_median, depth_mad = median_and_mad(depth_values)
    within_range = MEASURED_DEPTH_MIN_M <= depth_median <= MEASURED_DEPTH_MAX_M
    quality_ok = (
        valid_ratio >= MINIMUM_VALID_RATIO
        and disparity_mad <= MAXIMUM_DISPARITY_MAD_PX
        and depth_mad <= MAXIMUM_DEPTH_SPATIAL_MAD_M
        and np.isfinite(depth_median)
        and within_range
    )
    return {
        "bounds_xyxy": [x1, y1, x2, y2],
        "valid_ratio": valid_ratio,
        "disparity_median_px": disparity_median,
        "disparity_mad_px": disparity_mad,
        "depth_median_m": depth_median,
        "depth_mad_m": depth_mad,
        "within_validated_depth_range": within_range,
        "quality_status": "PASS" if quality_ok else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--policy", required=True)
    parser.add_argument("--expected-policy-sha256", required=True)
    parser.add_argument("--expected-review-sha256", required=True)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--warmup-frames", type=int, default=180)
    parser.add_argument("--eligibility-timeout-s", type=float, default=1800.0)
    parser.add_argument("--console-interval-s", type=float, default=2.0)
    parser.add_argument("--roi-center-x", type=int, required=True)
    parser.add_argument("--roi-center-y", type=int, required=True)
    parser.add_argument("--roi-size", type=int, required=True)
    parser.add_argument("--preview-width", type=int, default=960)
    parser.add_argument(
        "--measurement-jsonl",
        required=True,
        help=(
            "append-only output containing one pure sie_measurement_v1 JSON "
            "object per line"
        ),
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="disable the OpenCV live window for headless operation",
    )
    parser.add_argument(
        "--max-depth-frames",
        type=int,
        default=0,
        help="0 means run until interrupted",
    )
    args = parser.parse_args()

    project_root = Path(args.project_root).resolve()
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

    from vision_core.stereo.guarded_runtime_v6 import GuardedStereoDepthProcessor
    from vision_core.stereo.stereo_calibration_guard_v6 import (
        CalibrationGuardError,
        StereoCalibrationGuard,
    )

    if args.warmup_frames <= 0:
        raise ValueError("warmup-frames must be positive")
    if args.eligibility_timeout_s <= 0:
        raise ValueError("eligibility-timeout-s must be positive")
    if args.console_interval_s <= 0:
        raise ValueError("console-interval-s must be positive")
    if args.max_depth_frames < 0:
        raise ValueError("max-depth-frames must be non-negative")
    if args.roi_size <= 0:
        raise ValueError("roi-size must be positive")
    if args.preview_width <= 0:
        raise ValueError("preview-width must be positive")
    roi_half = args.roi_size // 2
    roi_x1 = args.roi_center_x - roi_half
    roi_y1 = args.roi_center_y - roi_half
    roi_x2 = roi_x1 + args.roi_size
    roi_y2 = roi_y1 + args.roi_size
    if roi_x1 < 0 or roi_y1 < 0 or roi_x2 > EYE_WIDTH or roi_y2 > EYE_HEIGHT:
        raise ValueError(
            "ROI is outside the rectified LEFT image: "
            f"bounds=({roi_x1}, {roi_y1}, {roi_x2}, {roi_y2})"
        )
    if abs(args.fps - REQUIRED_FPS) > 1e-9:
        raise ValueError(
            "stereo_runtime_v1 is frozen to 60 FPS; "
            "another frame rate requires a separately validated profile"
        )

    policy_path = Path(args.policy)
    if not policy_path.is_absolute():
        policy_path = project_root / policy_path
    policy_path = policy_path.resolve()
    policy = load_runtime_policy(policy_path)
    if policy.get("runtime_profile_id") != RUNTIME_PROFILE_ID:
        raise RuntimeError(
            "runtime policy profile mismatch: "
            f"expected={RUNTIME_PROFILE_ID}, "
            f"actual={policy.get('runtime_profile_id')!r}"
        )
    if policy.get("roi_center_xy") != [args.roi_center_x, args.roi_center_y]:
        raise RuntimeError("CLI ROI center does not match the frozen policy")
    if policy.get("roi_size_px") != [args.roi_size, args.roi_size]:
        raise RuntimeError("CLI ROI size does not match the frozen policy")
    if policy.get("num_disparities") != NUM_DISPARITIES:
        raise RuntimeError("policy num_disparities does not match the runtime")
    if policy.get("validated_ground_truth_distance_range_m") != [
        VALIDATED_GROUND_TRUTH_MIN_M,
        VALIDATED_GROUND_TRUTH_MAX_M,
    ]:
        raise RuntimeError("validated ground-truth range mismatch")
    if policy.get("measured_depth_acceptance_range_m") != [
        MEASURED_DEPTH_MIN_M,
        MEASURED_DEPTH_MAX_M,
    ]:
        raise RuntimeError("measured depth acceptance range mismatch")
    if float(policy.get("minimum_valid_pixel_ratio", -1.0)) != MINIMUM_VALID_RATIO:
        raise RuntimeError("minimum valid pixel ratio mismatch")
    if float(policy.get("maximum_disparity_mad_px", -1.0)) != MAXIMUM_DISPARITY_MAD_PX:
        raise RuntimeError("maximum disparity MAD mismatch")
    if float(policy.get("maximum_depth_spatial_mad_m", -1.0)) != MAXIMUM_DEPTH_SPATIAL_MAD_M:
        raise RuntimeError("maximum spatial depth MAD mismatch")
    expected_policy_sha256 = args.expected_policy_sha256.strip().lower()
    expected_review_sha256 = args.expected_review_sha256.strip().lower()
    if len(expected_policy_sha256) != 64:
        raise ValueError("expected-policy-sha256 must contain 64 hex characters")
    if len(expected_review_sha256) != 64:
        raise ValueError("expected-review-sha256 must contain 64 hex characters")
    try:
        bytes.fromhex(expected_policy_sha256)
        bytes.fromhex(expected_review_sha256)
    except ValueError as error:
        raise ValueError("expected SHA-256 values must be hexadecimal") from error
    if policy.get("calibration_id") != EXPECTED_CALIBRATION_ID:
        raise RuntimeError(
            "runtime policy calibration ID mismatch: "
            f"expected={EXPECTED_CALIBRATION_ID!r}, "
            f"actual={policy.get('calibration_id')!r}"
        )
    if policy.get("calibration_sha256") != EXPECTED_CALIBRATION_SHA256:
        raise RuntimeError("V6 calibration SHA mismatch in policy")
    if policy.get("extended_range_review_sha256") != expected_review_sha256:
        raise RuntimeError("extended-range review SHA mismatch in policy")
    review_path = project_root / policy["extended_range_review_path"]
    if sha256_file(review_path) != expected_review_sha256:
        raise RuntimeError("extended-range review file SHA mismatch")
    policy_sha256 = sha256_file(policy_path)
    if policy_sha256 != expected_policy_sha256:
        raise RuntimeError(
            "runtime policy SHA-256 mismatch: "
            f"expected={expected_policy_sha256}, actual={policy_sha256}"
        )
    run_id = f"stereo-v6-{uuid.uuid4()}"
    measurement_jsonl_path = Path(args.measurement_jsonl).expanduser()
    measurement_writer = MeasurementJsonlWriter(measurement_jsonl_path)

    guard = StereoCalibrationGuard.from_policy(
        policy_path,
        project_root=project_root,
    )

    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            args.device,
            "--set-ctrl=auto_exposure=3",
        ],
        check=True,
    )
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"camera open failed: {args.device}")

    try:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, RAW_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, RAW_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
        actual_fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = "".join(
            chr((actual_fourcc_value >> (8 * index)) & 0xFF)
            for index in range(4)
        )
        if (actual_width, actual_height) != (RAW_WIDTH, RAW_HEIGHT):
            raise RuntimeError(
                f"camera mode mismatch: {actual_width}x{actual_height}"
            )
        if actual_fourcc != "MJPG":
            raise RuntimeError(f"camera pixel format mismatch: {actual_fourcc!r}")
        if abs(actual_fps - REQUIRED_FPS) > 0.1:
            raise RuntimeError(
                f"camera FPS mismatch: expected {REQUIRED_FPS:.3f}, "
                f"got {actual_fps:.3f}"
            )

        json_line(
            {
                "status": "CAMERA_OPEN",
                "mode": f"{actual_fourcc} {actual_width}x{actual_height}",
                "actual_fps": actual_fps,
                "depth_frame_computed": False,
                "sie_measurement_emitted": False,
            }
        )

        successful_frames = 0
        while successful_frames < args.warmup_frames:
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            if frame.shape != EXPECTED_FRAME_SHAPE or frame.dtype != np.uint8:
                raise RuntimeError(
                    "camera frame mismatch during warm-up: "
                    f"shape={frame.shape}, dtype={frame.dtype}"
                )
            successful_frames += 1
        json_line(
            {
                "status": "CAMERA_WARMUP_FRAMES_COMPLETE",
                "successful_frames": successful_frames,
                "depth_frame_computed": False,
                "sie_measurement_emitted": False,
            }
        )

        eligibility_started = time.monotonic()
        last_gate_report = 0.0
        calibration = None
        while calibration is None:
            ok, frame = capture.read()
            if not ok or frame is None:
                json_line(
                    {
                        "status": "BLOCKED_CAMERA_READ",
                        "depth_frame_computed": False,
                        "sie_measurement_emitted": False,
                    }
                )
                return 2
            if frame.shape != EXPECTED_FRAME_SHAPE or frame.dtype != np.uint8:
                raise RuntimeError(
                    "camera frame mismatch while waiting for eligibility: "
                    f"shape={frame.shape}, dtype={frame.dtype}"
                )
            try:
                calibration = guard.startup()
            except CalibrationGuardError as error:
                if error.reason != "TEMPERATURE_OUTSIDE_VALIDATED_ENVELOPE":
                    json_line(
                        {
                            "status": "BLOCKED_FATAL",
                            "reason": error.reason,
                            "detail": error.detail,
                            "depth_frame_computed": False,
                            "sie_measurement_emitted": False,
                        }
                    )
                    return 2
                now = time.monotonic()
                if now - last_gate_report >= args.console_interval_s:
                    json_line(
                        {
                            "status": "BLOCKED_WARMUP",
                            "reason": error.reason,
                            "detail": error.detail,
                            "depth_frame_computed": False,
                            "sie_measurement_emitted": False,
                        }
                    )
                    last_gate_report = now
                if now - eligibility_started > args.eligibility_timeout_s:
                    json_line(
                        {
                            "status": "BLOCKED_TIMEOUT",
                            "reason": error.reason,
                            "detail": error.detail,
                            "depth_frame_computed": False,
                            "sie_measurement_emitted": False,
                        }
                    )
                    return 2
        processor = GuardedStereoDepthProcessor(
            guard=guard,
            calibration=calibration,
            num_disparities=NUM_DISPARITIES,
        )
        if calibration.calibration_id != EXPECTED_CALIBRATION_ID:
            raise RuntimeError(
                "activated calibration ID mismatch: "
                f"expected={EXPECTED_CALIBRATION_ID!r}, "
                f"actual={calibration.calibration_id!r}"
            )
        if calibration.calibration_sha256 != EXPECTED_CALIBRATION_SHA256:
            raise RuntimeError(
                "activated V6 calibration SHA mismatch: "
                f"expected={EXPECTED_CALIBRATION_SHA256}, "
                f"actual={calibration.calibration_sha256}"
            )
        json_line(
            {
                "status": "ACTIVE",
                "runtime_profile_id": RUNTIME_PROFILE_ID,
                "run_id": run_id,
                "runtime_policy_sha256": policy_sha256,
                "measurement_jsonl": str(measurement_writer.path),
                "measurement_stream_schema": "sie_measurement_v1",
                "calibration_id": calibration.calibration_id,
                "calibration_sha256": calibration.calibration_sha256,
                "activation_record_sha256": calibration.activation_record_sha256,
                "reference_frame": "rectified_left_optical_frame",
                "roi_center_xy": [args.roi_center_x, args.roi_center_y],
                "roi_size_px": args.roi_size,
                "num_disparities": NUM_DISPARITIES,
                "validated_ground_truth_distance_range_m": [
                    VALIDATED_GROUND_TRUTH_MIN_M,
                    VALIDATED_GROUND_TRUTH_MAX_M,
                ],
                "measured_depth_acceptance_range_m": [
                    MEASURED_DEPTH_MIN_M,
                    MEASURED_DEPTH_MAX_M,
                ],
                "depth_frame_computed": False,
                "sie_measurement_emitted": False,
            }
        )

        processed_frames = 0
        interval_started = time.monotonic()
        interval_frames = 0
        latest_processing_fps = 0.0
        runtime_temperature_blocked = False
        last_runtime_gate_report = 0.0
        operator_stop = False
        if not args.no_preview:
            cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)
        while args.max_depth_frames == 0 or processed_frames < args.max_depth_frames:
            ok, frame = capture.read()
            if not ok or frame is None:
                json_line(
                    {
                        "status": "BLOCKED_CAMERA_READ",
                        "depth_frame_computed": False,
                        "sie_measurement_emitted": False,
                    }
                )
                return 2
            try:
                depth_frame = processor.process(frame)
            except CalibrationGuardError as error:
                if error.reason == "TEMPERATURE_OUTSIDE_VALIDATED_ENVELOPE":
                    now = time.monotonic()
                    if (
                        not runtime_temperature_blocked
                        or now - last_runtime_gate_report >= args.console_interval_s
                    ):
                        json_line(
                            {
                                "status": "BLOCKED_RUNTIME",
                                "reason": error.reason,
                                "detail": error.detail,
                                "depth_frame_computed": False,
                                "sie_measurement_emitted": False,
                            }
                        )
                        last_runtime_gate_report = now
                    runtime_temperature_blocked = True
                    continue
                json_line(
                    {
                        "status": "BLOCKED_FATAL",
                        "reason": error.reason,
                        "detail": error.detail,
                        "depth_frame_computed": False,
                        "sie_measurement_emitted": False,
                    }
                )
                return 2

            if runtime_temperature_blocked:
                json_line(
                    {
                        "status": "ACTIVE_RESTORED",
                        "temperatures_c": depth_frame.temperature_gate.temperatures_c,
                        "temperature_state_age_s": (
                            depth_frame.temperature_gate.state_age_s
                        ),
                        "depth_frame_computed": True,
                        "sie_measurement_emitted": False,
                    }
                )
                runtime_temperature_blocked = False
                interval_started = time.monotonic()
                interval_frames = 0

            processed_frames += 1
            interval_frames += 1
            now = time.monotonic()
            roi_result = roi_depth_diagnostic(
                depth_frame,
                center_x=args.roi_center_x,
                center_y=args.roi_center_y,
                size=args.roi_size,
            )
            if now - interval_started >= args.console_interval_s:
                processing_fps = interval_frames / (now - interval_started)
                latest_processing_fps = processing_fps
                if roi_result["quality_status"] == "PASS":
                    measurement = build_sie_measurement(
                        run_id=run_id,
                        device=args.device,
                        depth_frame=depth_frame,
                        roi_result=roi_result,
                        runtime_policy_path=policy_path.relative_to(project_root),
                        runtime_policy_sha256=policy_sha256,
                    )
                    try:
                        measurement_writer.write(measurement)
                    except (OSError, RuntimeError) as error:
                        json_line(
                            {
                                "status": "BLOCKED_MEASUREMENT_STREAM",
                                "reason": "MEASUREMENT_JSONL_WRITE_FAILED",
                                "detail": str(error),
                                "depth_frame_computed": True,
                                "sie_measurement_emitted": False,
                            }
                        )
                        return 2
                    json_line(
                        {
                            "status": "SIE_MEASUREMENT",
                            "processing_fps": processing_fps,
                            "measurement": measurement,
                            "measurement_jsonl": str(measurement_writer.path),
                            "measurement_stream_count": (
                                measurement_writer.written_count
                            ),
                            "depth_frame_computed": True,
                            "sie_measurement_emitted": True,
                        }
                    )
                else:
                    json_line(
                        {
                            "status": "ROI_DEPTH_REJECTED",
                            "sequence": depth_frame.sequence,
                            "processing_fps": processing_fps,
                            "roi": roi_result,
                            "temperatures_c": (
                                depth_frame.temperature_gate.temperatures_c
                            ),
                            "temperature_state_age_s": (
                                depth_frame.temperature_gate.state_age_s
                            ),
                            "reference_frame": depth_frame.reference_frame,
                            "depth_frame_computed": True,
                            "sie_measurement_emitted": False,
                        }
                    )
                interval_started = now
                interval_frames = 0
            if not args.no_preview:
                preview = render_preview(
                    depth_frame,
                    roi_result,
                    processing_fps=latest_processing_fps,
                    preview_width=args.preview_width,
                )
                cv2.imshow(PREVIEW_WINDOW, preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    operator_stop = True
                    break
        if operator_stop:
            json_line(
                {
                    "status": "STOPPED_BY_OPERATOR",
                    "processed_depth_frames": processed_frames,
                    "measurement_stream_count": measurement_writer.written_count,
                    "sie_measurement_emitted": False,
                }
            )
            return 0
        json_line(
            {
                "status": "COMPLETED",
                "processed_depth_frames": processed_frames,
                "measurement_stream_count": measurement_writer.written_count,
                "sie_measurement_emitted": False,
            }
        )
        return 0
    except KeyboardInterrupt:
        json_line(
            {
                "status": "STOPPED_BY_OPERATOR",
                "sie_measurement_emitted": False,
            }
        )
        return 0
    finally:
        capture.release()
        measurement_writer.close()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
