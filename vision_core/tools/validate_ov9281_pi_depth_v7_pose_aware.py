#!/usr/bin/env python3
"""Collect a pose-aware, diagnostic-only planar depth check for active OV9281 v7.

This tool deliberately does not activate a calibration, change a runtime
profile, or calculate a depth correction.  It records a declared physical
reference for the board plane in ``rectified_left_optical_frame`` and rejects
frames whose observed checkerboard pose is not close to the physical-left
optical axis or perpendicular to it.

The PnP pose check uses the same left-camera intrinsics as the stereo
reconstruction.  It is therefore evidence that the captured board satisfies
the declared image-space pose constraints, not an independent replacement for
the mechanical survey of the reference datum.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


DEPTH_FRAME = "rectified_left_optical_frame"
LENS_RIM_FRAME = "physical_left_lens_front_rim_frame"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def set_auto_exposure(device: str) -> None:
    subprocess.run(["v4l2-ctl", "-d", device, "-c", "auto_exposure=3"], check=True)


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    found, corners = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or corners is None else corners.astype(np.float32)


def object_points(board: tuple[int, int], square_size_m: float) -> np.ndarray:
    columns, rows = board
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    return points * square_size_m


def percentile_summary(values: list[float], suffix: str) -> dict[str, float | int]:
    data = np.asarray(values, dtype=np.float64)
    if not data.size:
        raise ValueError("cannot summarise an empty value set")
    median = float(np.median(data))
    return {
        "count": int(data.size),
        f"min_{suffix}": float(np.min(data)),
        f"p05_{suffix}": float(np.percentile(data, 5)),
        f"p25_{suffix}": float(np.percentile(data, 25)),
        f"median_{suffix}": median,
        f"mad_{suffix}": float(np.median(np.abs(data - median))),
        f"p75_{suffix}": float(np.percentile(data, 75)),
        f"p95_{suffix}": float(np.percentile(data, 95)),
        f"max_{suffix}": float(np.max(data)),
    }


def board_pose_in_rectified_left(
    corners: np.ndarray,
    object_template: np.ndarray,
    k_left: np.ndarray,
    d_left: np.ndarray,
    rectification_r1: np.ndarray,
) -> dict[str, Any]:
    """Estimate board pose for capture gating, expressed in the depth frame."""
    solved, rvec, tvec = cv2.solvePnP(
        object_template,
        corners,
        k_left,
        d_left,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        raise RuntimeError("solvePnP could not estimate the physical-left board pose")
    rotation_raw, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(
        object_template, rvec, tvec, k_left, d_left
    )
    reprojection = np.linalg.norm(
        projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1
    )
    board_center_raw = rotation_raw @ object_template.mean(axis=0) + tvec.reshape(3)
    board_normal_raw = rotation_raw[:, 2]
    board_center_rectified = rectification_r1 @ board_center_raw
    board_normal_rectified = rectification_r1 @ board_normal_raw
    board_normal_rectified /= np.linalg.norm(board_normal_rectified)
    optical_axis_alignment = abs(float(board_normal_rectified[2]))
    normal_misalignment_deg = math.degrees(
        math.acos(float(np.clip(optical_axis_alignment, -1.0, 1.0)))
    )
    return {
        "board_center_m": [float(value) for value in board_center_rectified],
        "board_normal_unit": [float(value) for value in board_normal_rectified],
        "lateral_offset_m": float(np.linalg.norm(board_center_rectified[:2])),
        "board_normal_misalignment_deg": normal_misalignment_deg,
        "pnp_reprojection_rms_px": float(np.sqrt(np.mean(np.square(reprojection)))),
    }


def absolute_reference_definition(args: argparse.Namespace) -> dict[str, Any]:
    """Return an explicit board-plane z datum without introducing a correction."""
    if not args.reference_description or not args.reference_description.strip():
        raise ValueError("--reference-description must describe the physical survey")
    if args.reference_method == "optical_axis_survey":
        if args.reference_z_m is None or args.reference_uncertainty_mm is None:
            raise ValueError(
                "optical_axis_survey requires --reference-z-m and "
                "--reference-uncertainty-mm"
            )
        if args.reference_z_m <= 0.0 or args.reference_uncertainty_mm < 0.0:
            raise ValueError("invalid optical-axis survey reference")
        return {
            "method": args.reference_method,
            "reference_frame": DEPTH_FRAME,
            "board_plane_z_m": args.reference_z_m,
            "standard_uncertainty_m": args.reference_uncertainty_mm / 1000.0,
            "description": args.reference_description,
        }

    required = (
        args.front_rim_to_board_plane_m,
        args.front_rim_distance_uncertainty_mm,
        args.front_rim_to_optical_center_z_mm,
        args.front_rim_to_optical_center_uncertainty_mm,
    )
    if any(value is None for value in required):
        raise ValueError(
            "front_rim_transform requires the rim-to-board distance, its uncertainty, "
            "the explicit rim-to-optical-center axial transform, and its uncertainty"
        )
    if (
        args.front_rim_to_board_plane_m <= 0.0
        or args.front_rim_distance_uncertainty_mm < 0.0
        or args.front_rim_to_optical_center_uncertainty_mm < 0.0
    ):
        raise ValueError("invalid front-rim transform reference")
    transform_m = args.front_rim_to_optical_center_z_mm / 1000.0
    uncertainty_m = math.hypot(
        args.front_rim_distance_uncertainty_mm / 1000.0,
        args.front_rim_to_optical_center_uncertainty_mm / 1000.0,
    )
    return {
        "method": args.reference_method,
        "reference_frame": DEPTH_FRAME,
        "board_plane_z_m": args.front_rim_to_board_plane_m + transform_m,
        "standard_uncertainty_m": uncertainty_m,
        "description": args.reference_description,
        "source_measurement": {
            "reference_frame": LENS_RIM_FRAME,
            "rim_to_board_plane_m": args.front_rim_to_board_plane_m,
            "rim_to_board_standard_uncertainty_m": (
                args.front_rim_distance_uncertainty_mm / 1000.0
            ),
            "rim_to_optical_center_axial_transform_m": transform_m,
            "rim_to_optical_center_standard_uncertainty_m": (
                args.front_rim_to_optical_center_uncertainty_mm / 1000.0
            ),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--device",
        default="/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0",
    )
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=180)
    parser.add_argument("--max-capture-attempts", type=int, default=1800)
    parser.add_argument("--board-inner-corners", type=int, nargs=2, metavar=("COLUMNS", "ROWS"))
    parser.add_argument("--square-size-mm", type=float, required=True)
    parser.add_argument("--evidence-dir", type=Path)
    parser.add_argument(
        "--validation-mode",
        choices=("absolute_optical_z", "relative_axis_translation"),
        default="absolute_optical_z",
        help=(
            "absolute_optical_z requires a surveyed optical-frame datum; "
            "relative_axis_translation records one axis-aligned point for a "
            "later delta-z series analysis"
        ),
    )
    parser.add_argument(
        "--reference-method",
        choices=("optical_axis_survey", "front_rim_transform"),
    )
    parser.add_argument("--reference-description")
    parser.add_argument("--reference-z-m", type=float)
    parser.add_argument("--reference-uncertainty-mm", type=float)
    parser.add_argument("--front-rim-to-board-plane-m", type=float)
    parser.add_argument("--front-rim-distance-uncertainty-mm", type=float)
    parser.add_argument("--front-rim-to-optical-center-z-mm", type=float)
    parser.add_argument("--front-rim-to-optical-center-uncertainty-mm", type=float)
    parser.add_argument(
        "--axis-reference-id",
        help=(
            "stable physical datum for all relative points, for example "
            "physical_left_lens_front_rim_axis"
        ),
    )
    parser.add_argument(
        "--axis-position-m",
        type=float,
        help=(
            "surveyed board-plane position along the fixed physical-left optical axis "
            "from --axis-reference-id; its unknown constant offset to the optical "
            "center is intentionally not estimated"
        ),
    )
    parser.add_argument("--axis-position-uncertainty-mm", type=float)
    parser.add_argument("--axis-survey-description")
    parser.add_argument("--max-lateral-offset-mm", type=float, default=30.0)
    parser.add_argument("--max-normal-misalignment-deg", type=float, default=3.0)
    parser.add_argument("--max-pnp-reprojection-rms-px", type=float, default=0.50)
    parser.add_argument(
        "--acceptance-max-relative-error",
        type=float,
        help="optional declared criterion; no runtime policy is changed by this check",
    )
    return parser.parse_args()


def relative_axis_reference_definition(args: argparse.Namespace) -> dict[str, Any]:
    if not args.axis_reference_id or not args.axis_survey_description:
        raise ValueError(
            "relative_axis_translation requires --axis-reference-id and "
            "--axis-survey-description"
        )
    if args.axis_position_m is None or args.axis_position_uncertainty_mm is None:
        raise ValueError(
            "relative_axis_translation requires --axis-position-m and "
            "--axis-position-uncertainty-mm"
        )
    if args.axis_position_m <= 0.0 or args.axis_position_uncertainty_mm < 0.0:
        raise ValueError("invalid relative axis position or uncertainty")
    return {
        "reference_frame": LENS_RIM_FRAME,
        "axis_reference_id": args.axis_reference_id,
        "board_plane_axis_position_m": args.axis_position_m,
        "standard_uncertainty_m": args.axis_position_uncertainty_mm / 1000.0,
        "description": args.axis_survey_description,
        "interpretation_boundary": (
            "This is a repeatable physical-axis position, not optical-frame z. "
            "It may only be compared with other reports sharing this exact axis "
            "reference; the constant rim-to-optical-center offset is not estimated."
        ),
    }


def main() -> int:
    args = parse_args()
    if (
        args.frames < 5
        or args.warmup_frames < 1
        or args.max_capture_attempts < args.frames
        or args.square_size_mm <= 0.0
        or args.max_lateral_offset_mm < 0.0
        or not 0.0 <= args.max_normal_misalignment_deg <= 90.0
        or args.max_pnp_reprojection_rms_px <= 0.0
        or (
            args.acceptance_max_relative_error is not None
            and args.acceptance_max_relative_error < 0.0
        )
    ):
        raise ValueError("invalid capture, board, pose, or acceptance argument")
    if args.validation_mode == "absolute_optical_z":
        if args.reference_method is None:
            raise ValueError("absolute_optical_z requires --reference-method")
        reference = absolute_reference_definition(args)
        relative_axis_reference = None
    else:
        reference = None
        relative_axis_reference = relative_axis_reference_definition(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.evidence_dir is not None:
        args.evidence_dir.mkdir(parents=True, exist_ok=True)

    with np.load(args.stereo_calibration, allow_pickle=False) as values:
        if str(values["camera_1_semantics"].item()) != "physical_left":
            raise RuntimeError("calibration is not bound to physical_left")
        k1, d1, k2, d2 = values["K1"], values["D1"], values["K2"], values["D2"]
        r1, r2, p1, p2 = values["R1"], values["R2"], values["P1"], values["P2"]
        calibration_board = tuple(int(value) for value in values["board_size"])
        size = tuple(int(value) for value in values["size"])
    board = tuple(args.board_inner_corners) if args.board_inner_corners else calibration_board
    if board[0] < 2 or board[1] < 2:
        raise ValueError("checkerboard inner-corner dimensions must be at least 2x2")
    object_template = object_points(board, args.square_size_mm / 1000.0)

    set_auto_exposure(args.device)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0] * 2)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    capture.set(cv2.CAP_PROP_FPS, 60)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    accepted: list[dict[str, Any]] = []
    depth_values: list[float] = []
    disparity_values: list[float] = []
    rejections: dict[str, int] = {}
    attempts = 0
    try:
        for _ in range(args.warmup_frames):
            capture.read()
        while len(accepted) < args.frames and attempts < args.max_capture_attempts:
            attempts += 1
            ok, combined = capture.read()
            if not ok or combined is None or combined.shape[:2] != (size[1], size[0] * 2):
                rejections["CAPTURE_FRAME_INVALID"] = rejections.get("CAPTURE_FRAME_INVALID", 0) + 1
                continue
            physical_right = combined[:, :size[0]]
            physical_left = combined[:, size[0]:]
            left = find_corners(physical_left, board)
            right = find_corners(physical_right, board)
            if left is None or right is None:
                rejections["CHECKERBOARD_NOT_FOUND"] = rejections.get("CHECKERBOARD_NOT_FOUND", 0) + 1
                continue
            pose = board_pose_in_rectified_left(left, object_template, k1, d1, r1)
            if pose["lateral_offset_m"] > args.max_lateral_offset_mm / 1000.0:
                rejections["LATERAL_OFFSET_EXCEEDED"] = rejections.get("LATERAL_OFFSET_EXCEEDED", 0) + 1
                continue
            if pose["board_normal_misalignment_deg"] > args.max_normal_misalignment_deg:
                rejections["BOARD_NORMAL_MISALIGNMENT_EXCEEDED"] = rejections.get("BOARD_NORMAL_MISALIGNMENT_EXCEEDED", 0) + 1
                continue
            if pose["pnp_reprojection_rms_px"] > args.max_pnp_reprojection_rms_px:
                rejections["PNP_REPROJECTION_EXCEEDED"] = rejections.get("PNP_REPROJECTION_EXCEEDED", 0) + 1
                continue
            left_rectified = cv2.undistortPoints(left, k1, d1, R=r1, P=p1).reshape(-1, 2)
            right_rectified = cv2.undistortPoints(right, k2, d2, R=r2, P=p2).reshape(-1, 2)
            disparity = left_rectified[:, 0] - right_rectified[:, 0]
            valid = disparity > 0.5
            if int(valid.sum()) < math.ceil(0.9 * disparity.size):
                rejections["INSUFFICIENT_POSITIVE_DISPARITY"] = rejections.get("INSUFFICIENT_POSITIVE_DISPARITY", 0) + 1
                continue
            depth = abs(float(p2[0, 3])) / disparity[valid] / 1000.0
            frame_index = len(accepted) + 1
            evidence: dict[str, str] | None = None
            if args.evidence_dir is not None:
                frame_name = f"accepted_{frame_index:03d}"
                combined_path = args.evidence_dir / f"{frame_name}_combined_source.jpg"
                arrays_path = args.evidence_dir / f"{frame_name}_corners_pose_and_disparity.npz"
                if not cv2.imwrite(str(combined_path), combined):
                    raise OSError(f"could not write evidence image: {combined_path}")
                np.savez_compressed(
                    arrays_path,
                    physical_left_raw_corners=left,
                    physical_right_raw_corners=right,
                    physical_left_rectified_corners=left_rectified,
                    physical_right_rectified_corners=right_rectified,
                    disparity_px=disparity,
                    valid_disparity_mask=valid,
                    board_center_rectified_left_m=np.asarray(pose["board_center_m"]),
                    board_normal_rectified_left_unit=np.asarray(pose["board_normal_unit"]),
                )
                evidence = {
                    "combined_source": combined_path.name,
                    "corners_pose_and_disparity": arrays_path.name,
                }
            depth_values.extend(float(value) for value in depth)
            disparity_values.extend(float(value) for value in disparity[valid])
            accepted.append(
                {
                    "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                    "valid_corner_count": int(valid.sum()),
                    "median_depth_m": float(np.median(depth)),
                    "median_disparity_px": float(np.median(disparity[valid])),
                    "median_abs_vertical_error_px": float(
                        np.median(np.abs(left_rectified[:, 1] - right_rectified[:, 1]))
                    ),
                    "board_pose_capture_gate": pose,
                    "evidence": evidence,
                }
            )
            print(
                f"ACCEPTED {frame_index:02d}/{args.frames}: "
                f"depth={float(np.median(depth)):.4f} m "
                f"lateral={pose['lateral_offset_m'] * 1000.0:.1f} mm "
                f"normal={pose['board_normal_misalignment_deg']:.2f} deg"
            )
    finally:
        capture.release()

    if len(accepted) != args.frames:
        raise RuntimeError(
            f"only {len(accepted)}/{args.frames} frames passed pose gates after "
            f"{attempts} attempts: {rejections}"
        )

    depth_summary = percentile_summary(depth_values, "m")
    disparity_summary = percentile_summary(disparity_values, "px")
    frame_depth_summary = percentile_summary(
        [float(item["median_depth_m"]) for item in accepted], "m"
    )
    lateral_summary = percentile_summary(
        [float(item["board_pose_capture_gate"]["lateral_offset_m"]) for item in accepted], "m"
    )
    normal_summary = percentile_summary(
        [float(item["board_pose_capture_gate"]["board_normal_misalignment_deg"]) for item in accepted], "deg"
    )
    pnp_summary = percentile_summary(
        [float(item["board_pose_capture_gate"]["pnp_reprojection_rms_px"]) for item in accepted], "px"
    )
    comparison: dict[str, Any]
    if reference is not None:
        difference_m = float(depth_summary["median_m"]) - float(reference["board_plane_z_m"])
        relative_error = abs(difference_m) / float(reference["board_plane_z_m"])
        criterion: dict[str, Any] = {"status": "NOT_DECLARED"}
        if args.acceptance_max_relative_error is not None:
            criterion = {
                "status": "PASS" if relative_error <= args.acceptance_max_relative_error else "FAIL",
                "maximum_relative_error": args.acceptance_max_relative_error,
                "observed_absolute_relative_error": relative_error,
            }
        comparison = {
            "stereo_minus_reference_z_m": difference_m,
            "absolute_relative_error": relative_error,
            "declared_criterion": criterion,
        }
    else:
        if args.acceptance_max_relative_error is not None:
            raise ValueError(
                "--acceptance-max-relative-error applies only to absolute_optical_z; "
                "use the relative-series analyzer for a delta criterion"
            )
        comparison = {
            "status": "AWAITING_RELATIVE_SERIES_ANALYSIS",
            "reason": (
                "A single axis-aligned point cannot estimate or cancel the unknown "
                "constant offset between the front rim and optical center."
            ),
        }
    report = {
        "schema_version": "sie.ov9281.pi_stereo_pose_aware_planar_depth_diagnostic.v1",
        "status": "DIAGNOSTIC_NOT_ACTIVATED",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "calibration": {
            "path": str(args.stereo_calibration),
            "sha256": sha256_file(args.stereo_calibration),
            "camera_1_semantics": "physical_left",
        },
        "depth_reference_frame": DEPTH_FRAME,
        "validation_mode": args.validation_mode,
        "reference": reference,
        "relative_axis_reference": relative_axis_reference,
        "stereo_depth_m": depth_summary,
        "frame_median_depth_m": frame_depth_summary,
        "disparity_px": disparity_summary,
        "comparison": comparison,
        "pose_capture_gate": {
            "reference_frame": DEPTH_FRAME,
            "max_lateral_offset_m": args.max_lateral_offset_mm / 1000.0,
            "max_board_normal_misalignment_deg": args.max_normal_misalignment_deg,
            "max_pnp_reprojection_rms_px": args.max_pnp_reprojection_rms_px,
            "lateral_offset_m": lateral_summary,
            "board_normal_misalignment_deg": normal_summary,
            "pnp_reprojection_rms_px": pnp_summary,
            "note": (
                "This PnP result is a capture-pose gate derived from the active "
                "left intrinsics. It does not replace the declared independent "
                "physical reference survey."
            ),
        },
        "capture": {
            "device": args.device,
            "requested_mode": "V4L2 MJPG 2560x800@60, auto_exposure=3",
            "warmup_frames": args.warmup_frames,
            "attempts": attempts,
            "accepted_frames": len(accepted),
            "rejections": rejections,
            "calibration_board_inner_corners": list(calibration_board),
            "test_board_inner_corners": list(board),
            "test_square_size_mm": args.square_size_mm,
            "evidence_dir": None if args.evidence_dir is None else str(args.evidence_dir),
        },
        "per_frame": accepted,
        "interpretation_boundary": (
            "This report records a physical validation diagnostic. It does not "
            "activate a calibration, change an error envelope, create an SIE "
            "Measurement, or authorize navigation or actuation."
        ),
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "validation_mode": args.validation_mode,
                "reference_z_m": None if reference is None else reference["board_plane_z_m"],
                "axis_position_m": (
                    None
                    if relative_axis_reference is None
                    else relative_axis_reference["board_plane_axis_position_m"]
                ),
                "median_depth_m": depth_summary["median_m"],
                "comparison": comparison,
            },
            indent=2,
        )
    )
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
