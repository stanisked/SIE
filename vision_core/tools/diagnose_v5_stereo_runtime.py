#!/usr/bin/env python3

"""
SIE H2 stereo runtime diagnostic and strict fresh-checkerboard gate.

No WLS, CLAHE, temporal median, smoothing, hole filling, or depth correction.
"""

import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


RAW_W = 2560
RAW_H = 800
EYE_W = 1280
EYE_H = 800
PASS = "PASS"
WARNING = "WARNING"
FAIL = "FAIL"


def read_temperature_snapshot(
    bridge_tool: Path,
    state_file: Path,
    maximum_age_s: float,
) -> dict:
    if not bridge_tool.is_file():
        raise RuntimeError(f"temperature bridge tool not found: {bridge_tool}")
    if not state_file.is_file():
        raise RuntimeError(f"temperature state file not found: {state_file}")

    completed = subprocess.run(
        [
            sys.executable,
            str(bridge_tool),
            "snapshot",
            "--state-file",
            str(state_file),
            "--max-age-s",
            str(maximum_age_s),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("temperature snapshot returned no JSON")
    payload = json.loads(lines[-1])
    temperatures = payload.get("temperatures_c")
    expected_channels = {"ambient", "camera_left", "camera_right"}
    if not isinstance(temperatures, dict) or set(temperatures) != expected_channels:
        raise RuntimeError(f"invalid temperature snapshot: {payload}")
    if not all(np.isfinite(float(temperatures[key])) for key in expected_channels):
        raise RuntimeError(f"non-finite temperature snapshot: {payload}")

    return {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "state_file": str(state_file),
        "maximum_age_s": maximum_age_s,
        "temperatures_c": {
            key: float(temperatures[key])
            for key in sorted(expected_channels)
        },
    }


def draw_horizontal_guides(image, step=80):
    guided = image.copy()
    height = guided.shape[0]

    for y in range(step, height, step):
        cv2.line(
            guided,
            (0, y),
            (guided.shape[1] - 1, y),
            (0, 255, 255),
            1,
        )

    return guided


def create_matcher(num_disparities: int):
    block = 7

    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block,
        P1=8 * block * block,
        P2=32 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=6,
        speckleWindowSize=80,
        speckleRange=4,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def robust_stats(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]

    if values.size == 0:
        return None

    median = float(np.median(values))

    return {
        "count": int(values.size),
        "median": median,
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "mad": float(np.median(np.abs(values - median))),
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def save_json(path: Path, data: dict) -> None:
    path.write_text(
        json.dumps(
            data,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ),
        encoding="utf-8",
    )


def find_checkerboard_corners(
    gray: np.ndarray,
    checkerboard: tuple[int, int],
) -> tuple[bool, np.ndarray | None, str]:
    sb_flags = (
        cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    found, corners = cv2.findChessboardCornersSB(
        gray,
        checkerboard,
        flags=sb_flags,
    )

    if found and corners is not None:
        return True, corners.reshape(-1, 2).astype(np.float32), "SB"

    classic_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    found, corners = cv2.findChessboardCorners(
        gray,
        checkerboard,
        classic_flags,
    )

    if not found or corners is None:
        return False, None, "not_found"

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-5,
    )
    refined = cv2.cornerSubPix(
        gray,
        corners,
        (5, 5),
        (-1, -1),
        criteria,
    )
    return True, refined.reshape(-1, 2).astype(np.float32), "classic"


def checkerboard_metrics(
    left_corners: np.ndarray,
    right_corners: np.ndarray,
) -> dict:
    dy = left_corners[:, 1] - right_corners[:, 1]
    abs_dy = np.abs(dy)
    disparity = left_corners[:, 0] - right_corners[:, 0]

    return {
        "median_signed_dy_px": float(np.median(dy)),
        "mean_abs_dy_px": float(np.mean(abs_dy)),
        "median_abs_dy_px": float(np.median(abs_dy)),
        "p95_abs_dy_px": float(np.percentile(abs_dy, 95)),
        "max_abs_dy_px": float(np.max(abs_dy)),
        "within_0_25_px": float(np.mean(abs_dy <= 0.25)),
        "within_0_50_px": float(np.mean(abs_dy <= 0.50)),
        "within_1_00_px": float(np.mean(abs_dy <= 1.00)),
        "median_disparity_px": float(np.median(disparity)),
        "positive_disparity_ratio": float(np.mean(disparity > 0)),
    }


def choose_right_corner_order(
    left_corners: np.ndarray,
    right_corners: np.ndarray,
) -> tuple[np.ndarray, bool, dict]:
    candidates = []

    for reversed_order, candidate in (
        (False, right_corners),
        (True, right_corners[::-1].copy()),
    ):
        metrics = checkerboard_metrics(left_corners, candidate)
        score = (
            1000.0 * (1.0 - metrics["positive_disparity_ratio"])
            + metrics["median_abs_dy_px"]
        )
        candidates.append((score, reversed_order, candidate, metrics))

    _, reversed_order, selected, metrics = min(
        candidates,
        key=lambda item: item[0],
    )
    return selected, reversed_order, metrics


def threshold_status(
    value: float,
    pass_limit: float,
    warning_limit: float,
) -> str:
    if value <= pass_limit:
        return PASS
    if value <= warning_limit:
        return WARNING
    return FAIL


def checkerboard_decision(metrics: dict) -> dict:
    checks = {
        "median_abs_dy": {
            "value_px": metrics["median_abs_dy_px"],
            "pass_limit_px": 0.25,
            "warning_limit_px": 0.75,
            "status": threshold_status(
                metrics["median_abs_dy_px"],
                0.25,
                0.75,
            ),
        },
        "p95_abs_dy": {
            "value_px": metrics["p95_abs_dy_px"],
            "pass_limit_px": 1.0,
            "warning_limit_px": 2.0,
            "status": threshold_status(
                metrics["p95_abs_dy_px"],
                1.0,
                2.0,
            ),
        },
        "max_abs_dy": {
            "value_px": metrics["max_abs_dy_px"],
            "pass_limit_px": 1.0,
            "warning_limit_px": 2.0,
            "status": threshold_status(
                metrics["max_abs_dy_px"],
                1.0,
                2.0,
            ),
        },
        "positive_disparity_ratio": {
            "value": metrics["positive_disparity_ratio"],
            "pass_limit": 0.95,
            "warning_limit": 0.80,
            "status": (
                PASS
                if metrics["positive_disparity_ratio"] >= 0.95
                else WARNING
                if metrics["positive_disparity_ratio"] >= 0.80
                else FAIL
            ),
        },
    }
    statuses = [check["status"] for check in checks.values()]
    overall = FAIL if FAIL in statuses else WARNING if WARNING in statuses else PASS

    return {
        "overall_status": overall,
        "approved_rectification_gate": overall == PASS,
        "checks": checks,
    }


def save_checkerboard_csv(
    path: Path,
    left_corners: np.ndarray,
    right_corners: np.ndarray,
) -> None:
    fields = [
        "corner_index",
        "left_x_px",
        "left_y_px",
        "right_x_px",
        "right_y_px",
        "signed_dy_px",
        "abs_dy_px",
        "disparity_px",
    ]

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()

        for index, (left, right) in enumerate(
            zip(left_corners, right_corners)
        ):
            signed_dy = float(left[1] - right[1])
            writer.writerow(
                {
                    "corner_index": index,
                    "left_x_px": float(left[0]),
                    "left_y_px": float(left[1]),
                    "right_x_px": float(right[0]),
                    "right_y_px": float(right[1]),
                    "signed_dy_px": signed_dy,
                    "abs_dy_px": abs(signed_dy),
                    "disparity_px": float(left[0] - right[0]),
                }
            )


def save_checkerboard_overlay(
    path: Path,
    left_rect: np.ndarray,
    right_rect: np.ndarray,
    left_corners: np.ndarray,
    right_corners: np.ndarray,
    status: str,
) -> None:
    overlay = draw_horizontal_guides(
        np.hstack([left_rect, right_rect]),
        step=80,
    )
    right_offset = left_rect.shape[1]

    for left, right in zip(left_corners, right_corners):
        abs_dy = abs(float(left[1] - right[1]))
        color = (
            (0, 255, 0)
            if abs_dy <= 0.25
            else (0, 255, 255)
            if abs_dy <= 1.0
            else (0, 0, 255)
        )
        left_point = tuple(np.rint(left).astype(int))
        right_point = (
            int(round(float(right[0]))) + right_offset,
            int(round(float(right[1]))),
        )
        cv2.circle(overlay, left_point, 3, color, -1, cv2.LINE_AA)
        cv2.circle(overlay, right_point, 3, color, -1, cv2.LINE_AA)
        cv2.line(
            overlay,
            left_point,
            right_point,
            color,
            1,
            cv2.LINE_AA,
        )

    status_color = {
        PASS: (0, 255, 0),
        WARNING: (0, 255, 255),
        FAIL: (0, 0, 255),
    }[status]
    cv2.rectangle(overlay, (0, 0), (overlay.shape[1] - 1, 52), (0, 0, 0), -1)
    cv2.putText(
        overlay,
        f"SIE v5 fresh checkerboard gate: {status}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        status_color,
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(str(path), overlay)


def run_checkerboard_gate(
    output_dir: Path,
    left_rect: np.ndarray,
    right_rect: np.ndarray,
    checkerboard: tuple[int, int],
    calibration_path: Path,
    calibration_sha256: str,
) -> tuple[str, dict]:
    left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
    left_found, left_corners, left_detector = find_checkerboard_corners(
        left_gray,
        checkerboard,
    )
    right_found, right_corners, right_detector = find_checkerboard_corners(
        right_gray,
        checkerboard,
    )

    report = {
        "gate": "SIE_stereo_calibration_v5_fresh_checkerboard",
        "calibration": str(calibration_path),
        "calibration_sha256": calibration_sha256,
        "checkerboard_inner_corners": list(checkerboard),
        "expected_corner_count": int(checkerboard[0] * checkerboard[1]),
        "left_detection": {
            "found": left_found,
            "detector": left_detector,
            "corner_count": 0 if left_corners is None else len(left_corners),
        },
        "right_detection": {
            "found": right_found,
            "detector": right_detector,
            "corner_count": 0 if right_corners is None else len(right_corners),
        },
    }

    if not left_found or not right_found:
        report["decision"] = {
            "overall_status": FAIL,
            "approved_rectification_gate": False,
            "reason": "checkerboard_not_found_in_both_rectified_images",
        }
        save_json(output_dir / "checkerboard_gate.json", report)
        return FAIL, report

    assert left_corners is not None
    assert right_corners is not None
    right_corners, order_reversed, metrics = choose_right_corner_order(
        left_corners,
        right_corners,
    )
    decision = checkerboard_decision(metrics)
    report["right_corner_order_reversed"] = order_reversed
    report["metrics"] = metrics
    report["decision"] = decision

    save_checkerboard_csv(
        output_dir / "checkerboard_corners.csv",
        left_corners,
        right_corners,
    )
    save_checkerboard_overlay(
        output_dir / "checkerboard_rectified_overlay.png",
        left_rect,
        right_rect,
        left_corners,
        right_corners,
        decision["overall_status"],
    )
    save_json(output_dir / "checkerboard_gate.json", report)
    return decision["overall_status"], report


def select_roi_center(
    image: np.ndarray,
    roi_size: int,
    initial_center_x: int,
    initial_center_y: int,
) -> tuple[int, int]:
    height, width = image.shape[:2]

    if roi_size <= 0 or roi_size > min(width, height):
        raise ValueError(
            f"invalid roi_size={roi_size} for image {width}x{height}"
        )

    state = {
        "center_x": int(initial_center_x),
        "center_y": int(initial_center_y),
        "dragging": False,
    }

    def clamp_center() -> None:
        half = roi_size // 2
        max_x = width - (roi_size - half)
        max_y = height - (roi_size - half)
        state["center_x"] = int(np.clip(state["center_x"], half, max_x))
        state["center_y"] = int(np.clip(state["center_y"], half, max_y))

    def move_to(x: int, y: int) -> None:
        state["center_x"] = int(x)
        state["center_y"] = int(y)
        clamp_center()

    def mouse_callback(event, x, y, flags, userdata) -> None:
        del flags, userdata

        if event == cv2.EVENT_LBUTTONDOWN:
            state["dragging"] = True
            move_to(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state["dragging"]:
            move_to(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            state["dragging"] = False
            move_to(x, y)

    clamp_center()

    window_name = "Select LEFT-reference ROI"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, mouse_callback)

    while True:
        preview = image.copy()
        half = roi_size // 2
        x1 = state["center_x"] - half
        y1 = state["center_y"] - half
        x2 = x1 + roi_size
        y2 = y1 + roi_size

        cv2.rectangle(
            preview,
            (x1, y1),
            (x2 - 1, y2 - 1),
            (0, 255, 0),
            2,
        )

        cv2.putText(
            preview,
            "Drag/click or W/A/S/D | ENTER/SPACE accept | Q/ESC abort",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            preview,
            f"ROI center=({state['center_x']}, {state['center_y']}) size={roi_size}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(window_name, preview)
        key = cv2.waitKey(20) & 0xFF

        if key in (10, 13, 32):
            break
        if key in (27, ord("q"), ord("Q")):
            cv2.destroyWindow(window_name)
            raise RuntimeError("ROI selection cancelled")

        step = 5
        if key in (ord("a"), ord("A")):
            move_to(state["center_x"] - step, state["center_y"])
        elif key in (ord("d"), ord("D")):
            move_to(state["center_x"] + step, state["center_y"])
        elif key in (ord("w"), ord("W")):
            move_to(state["center_x"], state["center_y"] - step)
        elif key in (ord("s"), ord("S")):
            move_to(state["center_x"], state["center_y"] + step)

    cv2.destroyWindow(window_name)
    return state["center_x"], state["center_y"]


def main():
    parser = argparse.ArgumentParser(
        description="Minimal H2 stereo order and rectification diagnostic"
    )

    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument(
        "--calibration",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v5/solution_all_pairs_joint_refine_v1_1/"
            "stereo_params_v5.npz"
        ),
    )
    parser.add_argument(
        "--expected_calibration_sha256",
        required=True,
        help=(
            "required SHA-256 of the calibration file"
        ),
    )
    parser.add_argument(
        "--expected_calibration_id",
        default="stereo_calibration_v5",
    )
    parser.add_argument(
        "--temperature_bridge_tool",
        default="vision_core/tools/sie_ds18b20_serial_bridge.py",
    )
    parser.add_argument(
        "--temperature_state_file",
        default="/tmp/sie_h05b_temperature_state.json",
    )
    parser.add_argument(
        "--temperature_max_age_s",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--pixel_format",
        choices=["MJPG", "YUYV"],
        default="MJPG",
    )

    parser.add_argument(
        "--fps",
        type=float,
        default=60.0,
    )
    parser.add_argument(
        "--warmup_frames",
        type=int,
        default=180,
    )
    parser.add_argument(
        "--num_disparities",
        type=int,
        default=160,
        help="StereoSGBM search range; must be a positive multiple of 16",
    )
    parser.add_argument(
        "--output_dir",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v5/fresh_checkerboard_gate"
        ),
    )
    parser.add_argument("--roi_size", type=int, default=160)
    parser.add_argument("--roi_center_x", type=int)
    parser.add_argument("--roi_center_y", type=int)
    parser.add_argument(
        "--interactive_roi",
        action="store_true",
        help="move the fixed-size ROI on rectified LEFT with mouse or W/A/S/D",
    )
    parser.add_argument("--ground_truth_mm", type=float)
    parser.add_argument(
        "--distance_label",
        choices=["near", "mid", "far"],
        help="required label for a physical depth-validation run",
    )
    parser.add_argument(
        "--ground_truth_reference_frame",
        default="camera_front_plane_frame",
    )
    parser.add_argument(
        "--checkerboard_gate",
        action="store_true",
        help=(
            "run the fresh 9x6 checkerboard rectification gate "
            "and exit before SGBM/ROI depth processing"
        ),
    )
    parser.add_argument("--board_cols", type=int, default=9)
    parser.add_argument("--board_rows", type=int, default=6)
    parser.add_argument(
        "--offline_left",
        help="physical LEFT raw image for an offline replay",
    )
    parser.add_argument(
        "--offline_right",
        help="physical RIGHT raw image for an offline replay",
    )

    args = parser.parse_args()

    if args.warmup_frames <= 0:
        raise ValueError(
            "warmup_frames must be greater than zero"
        )
    if args.num_disparities <= 0 or args.num_disparities % 16 != 0:
        raise ValueError(
            "num_disparities must be a positive multiple of 16"
        )
    if args.temperature_max_age_s <= 0:
        raise ValueError("temperature_max_age_s must be positive")
    if not args.checkerboard_gate:
        if args.ground_truth_mm is None or args.ground_truth_mm <= 0:
            raise ValueError(
                "positive --ground_truth_mm is required for depth validation"
            )
        if args.distance_label is None:
            raise ValueError(
                "--distance_label near|mid|far is required for depth validation"
            )

    calibration_path = Path(args.calibration)
    if not calibration_path.is_file():
        raise FileNotFoundError(
            f"calibration file not found: {calibration_path}"
        )

    calibration_sha256 = sha256_file(calibration_path)
    expected_sha256 = args.expected_calibration_sha256.strip().lower()

    if (
        expected_sha256
        and calibration_sha256.lower() != expected_sha256
    ):
        raise RuntimeError(
            "calibration SHA-256 mismatch: "
            f"expected={expected_sha256}, "
            f"actual={calibration_sha256}"
        )

    offline_requested = (
        args.offline_left is not None
        or args.offline_right is not None
    )

    if offline_requested and (
        args.offline_left is None
        or args.offline_right is None
    ):
        raise ValueError(
            "--offline_left and --offline_right must be used together"
        )

    checkerboard = (args.board_cols, args.board_rows)
    if checkerboard[0] <= 0 or checkerboard[1] <= 0:
        raise ValueError("checkerboard dimensions must be positive")

    output_dir = Path(args.output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    # SIE calibration semantics:
    #
    # Physical camera order in the combined UVC frame:
    #   left half  -> physical RIGHT camera
    #   right half -> physical LEFT camera
    #
    # Runtime split:
    #   right_raw = combined[:, :1280]
    #   left_raw  = combined[:, 1280:]
    #
    # Calibration semantics:
    #   K1/D1/R1/P1 -> physical LEFT camera
    #   K2/D2/R2/P2 -> physical RIGHT camera
    params = np.load(str(calibration_path), allow_pickle=False)

    required_keys = {
        "K1", "D1", "R1", "P1",
        "K2", "D2", "R2", "P2",
        "baseline_mm", "calibration_id",
        "activation_eligible", "capture_thermal_status",
    }
    missing_keys = sorted(required_keys.difference(params.files))
    if missing_keys:
        raise RuntimeError(
            f"calibration is missing keys: {missing_keys}"
        )

    for key, expected in (
        ("calibration_id", args.expected_calibration_id),
        ("camera_1_semantics", "physical_left"),
        ("camera_2_semantics", "physical_right"),
    ):
        if key in params.files:
            actual = str(params[key].item())
            if actual != expected:
                raise RuntimeError(
                    f"unexpected calibration {key}: "
                    f"expected={expected}, actual={actual}"
                )

    activation_eligible = bool(params["activation_eligible"].item())
    if activation_eligible:
        raise RuntimeError(
            "v5 candidate unexpectedly declares activation_eligible=true"
        )
    capture_thermal_status = str(params["capture_thermal_status"].item())

    K1 = params["K1"]
    D1 = params["D1"]
    R1 = params["R1"]
    P1 = params["P1"]

    K2 = params["K2"]
    D2 = params["D2"]
    R2 = params["R2"]
    P2 = params["P2"]

    baseline_mm = float(params["baseline_mm"])
    baseline_m = baseline_mm / 1000.0
    fx_rectified = float(P1[0, 0])
    image_size = (EYE_W, EYE_H)

    left_map_x, left_map_y = cv2.initUndistortRectifyMap(
        K1,
        D1,
        R1,
        P1,
        image_size,
        cv2.CV_32FC1,
    )

    right_map_x, right_map_y = cv2.initUndistortRectifyMap(
        K2,
        D2,
        R2,
        P2,
        image_size,
        cv2.CV_32FC1,
    )

    capture_details = {
        "source": "offline" if offline_requested else "live_camera",
        "device": args.device,
        "requested_pixel_format": args.pixel_format,
        "requested_fps": args.fps,
        "warmup_successful_frames_required": args.warmup_frames,
    }

    if offline_requested:
        left_raw = cv2.imread(str(args.offline_left), cv2.IMREAD_COLOR)
        right_raw = cv2.imread(str(args.offline_right), cv2.IMREAD_COLOR)

        if left_raw is None:
            raise RuntimeError(
                f"cannot read offline physical LEFT: {args.offline_left}"
            )
        if right_raw is None:
            raise RuntimeError(
                f"cannot read offline physical RIGHT: {args.offline_right}"
            )

        capture_details.update(
            {
                "offline_left": str(args.offline_left),
                "offline_right": str(args.offline_right),
            }
        )
        print("Capture source: offline physical LEFT/RIGHT pair")
    else:
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                args.device,
                "--set-ctrl=auto_exposure=3",
            ],
            check=True,
        )

        time.sleep(0.5)

        cap = cv2.VideoCapture(
            args.device,
            cv2.CAP_V4L2,
        )

        if not cap.isOpened():
            raise RuntimeError(f"camera open failed: {args.device}")

        try:
            fourcc = cv2.VideoWriter_fourcc(*args.pixel_format)
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, RAW_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RAW_H)
            cap.set(cv2.CAP_PROP_FPS, args.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            actual_fourcc_str = "".join(
                chr((actual_fourcc >> (8 * i)) & 0xFF)
                for i in range(4)
            )

            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(cap.get(cv2.CAP_PROP_FPS))

            if actual_width != RAW_W or actual_height != RAW_H:
                raise RuntimeError(
                    "camera did not accept 2560x800: "
                    f"actual={actual_width}x{actual_height}"
                )

            print("Actual FPS:", actual_fps)
            print(
                "Capture mode:",
                f"{actual_fourcc_str} "
                f"{actual_width}x{actual_height} "
                f"@ {actual_fps:.3f} FPS",
            )

            combined = None
            successful_frames = 0
            attempts = 0
            max_attempts = max(
                args.warmup_frames * 3,
                args.warmup_frames + 60,
            )

            while (
                successful_frames < args.warmup_frames
                and attempts < max_attempts
            ):
                attempts += 1
                ok, frame = cap.read()

                if not ok or frame is None:
                    continue

                combined = frame
                successful_frames += 1

            if (
                combined is None
                or successful_frames < args.warmup_frames
            ):
                raise RuntimeError(
                    "camera warm-up failed: "
                    f"successful={successful_frames}, "
                    f"required={args.warmup_frames}, "
                    f"attempts={attempts}"
                )

            print(
                "Warm-up:",
                f"{successful_frames} successful frames",
                f"in {attempts} attempts",
            )

            temperature_before_frame = read_temperature_snapshot(
                Path(args.temperature_bridge_tool),
                Path(args.temperature_state_file),
                args.temperature_max_age_s,
            )

            final_attempts = 0
            while final_attempts < 60:
                final_attempts += 1
                ok, frame = cap.read()
                if ok and frame is not None:
                    combined = frame
                    break
            else:
                raise RuntimeError("failed to capture final gate frame")

            temperature_after_frame = read_temperature_snapshot(
                Path(args.temperature_bridge_tool),
                Path(args.temperature_state_file),
                args.temperature_max_age_s,
            )

            temperature_change = {
                key: round(
                    temperature_after_frame["temperatures_c"][key]
                    - temperature_before_frame["temperatures_c"][key],
                    4,
                )
                for key in ("ambient", "camera_left", "camera_right")
            }
            capture_details["temperature_before_frame"] = (
                temperature_before_frame
            )
            capture_details["temperature_after_frame"] = (
                temperature_after_frame
            )
            capture_details["temperature_change_c"] = temperature_change
            capture_details["final_frame_capture_attempts"] = final_attempts
            print(
                "Temperature before frame:",
                temperature_before_frame["temperatures_c"],
            )
            print(
                "Temperature after frame:",
                temperature_after_frame["temperatures_c"],
            )
            capture_details.update(
                {
                    "actual_pixel_format": actual_fourcc_str,
                    "actual_resolution": [
                        actual_width,
                        actual_height,
                    ],
                    "actual_fps": actual_fps,
                    "warmup_successful_frames": successful_frames,
                    "warmup_attempts": attempts,
                }
            )
        finally:
            cap.release()

        # Combined UVC left half is physical RIGHT.
        # Combined UVC right half is physical LEFT.
        right_raw = combined[:, :EYE_W]
        left_raw = combined[:, EYE_W:]

    for name, image in (
        ("physical LEFT", left_raw),
        ("physical RIGHT", right_raw),
    ):
        if image.shape[:2] != (EYE_H, EYE_W):
            raise RuntimeError(
                f"unexpected {name} image size: "
                f"{image.shape[1]}x{image.shape[0]}"
            )

    left_rect = cv2.remap(
        left_raw,
        left_map_x,
        left_map_y,
        cv2.INTER_LINEAR,
    )

    right_rect = cv2.remap(
        right_raw,
        right_map_x,
        right_map_y,
        cv2.INTER_LINEAR,
    )

    cv2.imwrite(str(output_dir / "raw_left.png"), left_raw)
    cv2.imwrite(str(output_dir / "raw_right.png"), right_raw)
    cv2.imwrite(str(output_dir / "rectified_left.png"), left_rect)
    cv2.imwrite(str(output_dir / "rectified_right.png"), right_rect)

    rectified_pair = np.hstack(
        [
            left_rect,
            right_rect,
        ]
    )
    cv2.imwrite(
        str(output_dir / "rectified_pair.png"),
        draw_horizontal_guides(rectified_pair),
    )

    left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)

    def brightness(gray):
        return {
            "mean": float(np.mean(gray)),
            "median": float(np.median(gray)),
            "std": float(np.std(gray)),
            "p05": float(np.percentile(gray, 5)),
            "p95": float(np.percentile(gray, 95)),
        }

    left_brightness = brightness(left_gray)
    right_brightness = brightness(right_gray)
    print("Left brightness :", left_brightness)
    print("Right brightness:", right_brightness)
    print(f"Calibration: {calibration_path}")
    print(f"Calibration SHA-256: {calibration_sha256}")
    print(f"Calibration ID: {args.expected_calibration_id}")
    print(f"Capture thermal status: {capture_thermal_status}")
    print(f"Activation eligible before this gate: {activation_eligible}")
    print(f"fx_rectified: {fx_rectified:.3f} px")
    print(f"baseline_mm: {baseline_mm:.3f}")

    if "temperature_before_frame" in capture_details:
        save_json(
            output_dir / "temperature_before_frame.json",
            capture_details["temperature_before_frame"],
        )
        save_json(
            output_dir / "temperature_after_frame.json",
            capture_details["temperature_after_frame"],
        )

    if args.checkerboard_gate:
        status, gate_report = run_checkerboard_gate(
            output_dir,
            left_rect,
            right_rect,
            checkerboard,
            calibration_path,
            calibration_sha256,
        )
        gate_report["capture"] = capture_details
        gate_report["runtime_camera_order"] = {
            "combined_left_half": "physical_right",
            "combined_right_half": "physical_left",
            "K1_D1_R1_P1": "physical_left",
            "K2_D2_R2_P2": "physical_right",
        }
        gate_report["calibration_id"] = args.expected_calibration_id
        gate_report["capture_thermal_status"] = capture_thermal_status
        gate_report["candidate_activation_eligible"] = activation_eligible
        gate_report["gate_scope"] = (
            "rectification_only; does_not_activate_calibration"
        )
        save_json(output_dir / "checkerboard_gate.json", gate_report)

        print()
        print("Fresh checkerboard rectification gate")
        print("-------------------------------------")
        print(
            "Detection:",
            f"LEFT={gate_report['left_detection']['corner_count']}/"
            f"{gate_report['expected_corner_count']}",
            f"RIGHT={gate_report['right_detection']['corner_count']}/"
            f"{gate_report['expected_corner_count']}",
        )

        if "metrics" in gate_report:
            metrics = gate_report["metrics"]
            print(
                "Right detector order reversed:",
                gate_report["right_corner_order_reversed"],
            )
            print(
                f"Median signed dy: {metrics['median_signed_dy_px']:+.6f} px"
            )
            print(
                f"Median |dy|:      {metrics['median_abs_dy_px']:.6f} px"
            )
            print(
                f"P95 |dy|:         {metrics['p95_abs_dy_px']:.6f} px"
            )
            print(
                f"Max |dy|:         {metrics['max_abs_dy_px']:.6f} px"
            )
            print(
                f"Within 0.25 px:    {metrics['within_0_25_px']:.2%}"
            )
            print(
                f"Within 0.50 px:    {metrics['within_0_50_px']:.2%}"
            )
            print(
                f"Within 1.00 px:    {metrics['within_1_00_px']:.2%}"
            )
            print(
                "Positive disparity:",
                f"{metrics['positive_disparity_ratio']:.3f}",
            )

        print(f"Overall status: {status}")
        print(f"Artifacts: {output_dir}")

        return {
            PASS: 0,
            WARNING: 1,
            FAIL: 2,
        }[status]

    stereo_matcher = create_matcher(args.num_disparities)
    print(f"StereoSGBM num_disparities: {args.num_disparities}")
    disparity_raw = stereo_matcher.compute(left_gray, right_gray)
    disparity = disparity_raw.astype(np.float32) / 16.0

    finite = np.isfinite(disparity)
    positive_ratio = (
        float(np.mean(disparity[finite] > 0))
        if np.any(finite)
        else 0.0
    )

    valid = finite & (disparity > 0)
    valid_pixel_ratio = float(np.mean(valid))
    median_disparity = (
        float(np.median(disparity[valid]))
        if np.any(valid)
        else float("nan")
    )

    depth_m = np.full(disparity.shape, np.nan, dtype=np.float32)
    depth_m[valid] = fx_rectified * baseline_m / disparity[valid]

    height, width = disparity.shape
    roi_size = int(args.roi_size)

    if roi_size <= 0 or roi_size > min(width, height):
        raise ValueError(
            f"invalid roi_size={roi_size} for disparity {width}x{height}"
        )

    center_x = (
        int(args.roi_center_x)
        if args.roi_center_x is not None
        else width // 2
    )

    center_y = (
        int(args.roi_center_y)
        if args.roi_center_y is not None
        else height // 2
    )

    if args.interactive_roi:
        center_x, center_y = select_roi_center(
            left_rect,
            roi_size,
            center_x,
            center_y,
        )

    half = roi_size // 2
    x1 = int(np.clip(center_x - half, 0, width - roi_size))
    y1 = int(np.clip(center_y - half, 0, height - roi_size))
    x2 = x1 + roi_size
    y2 = y1 + roi_size
    center_x = x1 + half
    center_y = y1 + half

    roi = np.s_[y1:y2, x1:x2]

    print(
        "ROI:",
        f"center=({center_x}, {center_y})",
        f"bounds=({x1}:{x2}, {y1}:{y2})",
        f"size={roi_size}x{roi_size}",
    )

    roi_disparity_stats = robust_stats(disparity[roi][valid[roi]])
    roi_depth_stats = robust_stats(depth_m[roi])
    roi_area = int(roi_size * roi_size)
    roi_valid_count = int(np.count_nonzero(valid[roi]))
    roi_valid_ratio = roi_valid_count / roi_area
    np.save(output_dir / "disparity_raw_q4.npy", disparity_raw)
    np.save(output_dir / "disparity_px.npy", disparity)
    np.save(output_dir / "valid_mask.npy", valid)
    np.save(output_dir / "depth_m.npy", depth_m)

    cv2.imwrite(
        str(output_dir / "valid_mask.png"),
        valid.astype(np.uint8) * 255,
    )

    np.save(
        output_dir / "roi_bounds_xyxy.npy",
        np.asarray(
            [x1, y1, x2, y2],
            dtype=np.int32,
        ),
    )
    # ------------------------------------------------------------
    # ROI visualization
    # ------------------------------------------------------------

    left_roi = left_rect.copy()
    right_roi = right_rect.copy()

    cv2.rectangle(
        left_roi,
        (x1, y1),
        (x2 - 1, y2 - 1),
        (0, 255, 0),
        2,
    )

    cv2.rectangle(
        right_roi,
        (x1, y1),
        (x2 - 1, y2 - 1),
        (0, 255, 0),
        2,
    )

    cv2.imwrite(
        str(output_dir / "rectified_left_roi.png"),
        left_roi,
    )

    cv2.imwrite(
        str(output_dir / "rectified_right_roi.png"),
        right_roi,
    )

    pair_roi = np.hstack(
        [
            left_roi,
            right_roi,
        ]
    )

    cv2.imwrite(
        str(output_dir / "rectified_pair_roi.png"),
        draw_horizontal_guides(pair_roi),
    )

    if args.ground_truth_mm is not None:
        if roi_depth_stats is None:
            raise RuntimeError("ground truth comparison failed: no valid ROI depth")

        median_depth_mm = roi_depth_stats["median"] * 1000.0
        signed_error = median_depth_mm - args.ground_truth_mm
        absolute_error = abs(signed_error)
        relative_error = absolute_error / args.ground_truth_mm * 100.0

    temperature_evidence_present = (
        "temperature_before_frame" in capture_details
        and "temperature_after_frame" in capture_details
    )
    maximum_temperature_change_c = (
        max(
            abs(float(value))
            for value in capture_details["temperature_change_c"].values()
        )
        if temperature_evidence_present
        else None
    )
    quality_checks = {
        "roi_valid_ratio_at_least_0_95": roi_valid_ratio >= 0.95,
        "roi_disparity_mad_at_most_2_px": (
            roi_disparity_stats is not None
            and roi_disparity_stats["mad"] <= 2.0
        ),
        "roi_depth_is_finite": (
            roi_depth_stats is not None
            and bool(np.isfinite(roi_depth_stats["median"]))
        ),
        "temperature_evidence_present": temperature_evidence_present,
        "maximum_temperature_change_at_most_0_125_c": (
            maximum_temperature_change_c is not None
            and maximum_temperature_change_c <= 0.125
        ),
    }
    run_status = PASS if all(quality_checks.values()) else FAIL

    depth_report = {
        "schema_version": "sie_depth_validation_run_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "diagnostic_tool": {
            "path": str(Path(__file__).resolve()),
            "sha256": sha256_file(Path(__file__).resolve()),
        },
        "calibration": {
            "calibration_id": args.expected_calibration_id,
            "path": str(calibration_path),
            "sha256": calibration_sha256,
            "capture_thermal_status": capture_thermal_status,
            "activation_eligible_before_run": activation_eligible,
            "fx_rectified_px": fx_rectified,
            "baseline_mm": baseline_mm,
        },
        "distance_label": args.distance_label,
        "reference_frames": {
            "measurement": "rectified_left_optical_frame",
            "ground_truth": args.ground_truth_reference_frame,
            "ground_truth_measurement": (
                "physical_distance_from_camera_front_plane_to_target_plane"
            ),
            "direct_absolute_comparison": False,
            "reason": "reference_frame_origins_differ",
        },
        "ground_truth_mm": args.ground_truth_mm,
        "measured_optical_depth_mm": median_depth_mm,
        "raw_cross_frame_signed_difference_mm": signed_error,
        "raw_cross_frame_absolute_difference_mm": absolute_error,
        "raw_cross_frame_relative_difference_percent": relative_error,
        "capture": capture_details,
        "brightness": {
            "physical_left": left_brightness,
            "physical_right": right_brightness,
        },
        "stereo_sgbm": {
            "min_disparity": 0,
            "num_disparities": args.num_disparities,
            "block_size": 7,
        },
        "frame_metrics": {
            "median_positive_disparity_px": median_disparity,
            "positive_disparity_ratio": positive_ratio,
            "valid_pixel_ratio": valid_pixel_ratio,
        },
        "roi": {
            "bounds_xyxy": [x1, y1, x2, y2],
            "size_px": [roi_size, roi_size],
            "area_px": roi_area,
            "valid_count": roi_valid_count,
            "valid_ratio": roi_valid_ratio,
            "disparity_px": roi_disparity_stats,
            "depth_m": roi_depth_stats,
        },
        "decision": {
            "measurement_quality_status": run_status,
            "quality_checks": quality_checks,
            "maximum_temperature_change_c": maximum_temperature_change_c,
            "absolute_accuracy_status": (
                "PENDING_CROSS_FRAME_OFFSET_MODEL"
            ),
            "activation_eligible_after_run": False,
        },
    }
    save_json(output_dir / "depth_validation.json", depth_report)

    print(f"Median disparity: {median_disparity:.3f}")
    print(f"Positive disparity ratio: {positive_ratio:.3f}")
    print(f"Valid pixel ratio: {valid_pixel_ratio:.3f}")
    print(f"ROI disparity stats: {roi_disparity_stats}")
    print(f"ROI depth stats, m: {roi_depth_stats}")
    print(f"ROI valid ratio: {roi_valid_ratio:.6f}")

    if args.ground_truth_mm is not None:
        print(f"Ground truth:      {args.ground_truth_mm:.0f} mm")
        print(f"Measured:          {median_depth_mm:.1f} mm")
        print(f"Signed error:      {signed_error:.1f} mm")
        print(f"Absolute error:    {absolute_error:.1f} mm")
        print(f"Relative error:    {relative_error:.2f} %")

    print(f"Artifacts: {output_dir}")
    print(f"Measurement quality status: {run_status}")
    return 0 if run_status == PASS else 2


if __name__ == "__main__":
    sys.exit(main())
