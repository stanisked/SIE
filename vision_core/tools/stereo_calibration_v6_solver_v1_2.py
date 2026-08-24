#!/usr/bin/env python3

"""
Stereo Calibration v6 Simple Solver

Надёжная версия на основе ранее рабочего скрипта:
- без переупорядочивания углов шахматной доски;
- findChessboardCornersSB используется как основной детектор;
- отдельная monocular calibration;
- stereoCalibrate с CALIB_FIX_INTRINSIC;
- stereoRectify с CALIB_ZERO_DISPARITY и alpha=0;
- проверка вертикальной ошибки после rectification;
- сохранение NPZ, JSON, CSV, Markdown и preview.
"""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

PASS = "PASS"
WARNING = "WARNING"
FAIL = "FAIL"

CALIBRATION_ID = "stereo_calibration_v6"
EXPERIMENT = "H0.6_Independent_Recalibration_With_V5_Semantics"
SOLVER_NAME = "Stereo Calibration v6 Solver with Explicit Corner-Order Corrections"
SOLVER_VERSION = "1.3"
RAW_DATASET_FREEZE_ID = "stereo_calibration_v6_raw_frozen_2026-08-20_v2"
RAW_DATASET_ARCHIVE_SHA256 = (
    "8f5a78071dc5eb6909ab52982d7a0227265913b32b7b7656b01070008710db12"
)
RAW_DATASET_CHECKSUMS_SHA256 = (
    "9b609b2ead73901c2b2adc6f7e523b959f74e8d1f5d4af8db1dcdc223795d3d0"
)
RAW_DATASET_FREEZE_MANIFEST_SHA256 = (
    "dc74101aa7fd87b293df3c49c2931cf8ce77864711d446412472ca2b4d47be9d"
)
RAW_CAPTURE_METADATA_SHA256 = (
    "be53a9341593a3dcbc590ec9cccc92bd00d94702cea81e7e8757cc91aa8d3b54"
)
EXPECTED_THERMAL_STATUS = "CAPTURE_THERMAL_PROVENANCE_INCOMPLETE"
EXPECTED_SQUARE_SIZE_MM = 24.5
CORNER_ORDER_CORRECTIONS_SHA256 = (
    "a61c2872860b044081c6bec46f336792d0ab0e99ae0fe16318b5071112c28440"
)
PREVIOUS_CALIBRATION_ID = "stereo_calibration_v5"
PREVIOUS_CALIBRATION_STATUS = "ACTIVE_CONDITIONAL_REFERENCE"


def json_default(value: Any):
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(data, indent=2, ensure_ascii=False, default=json_default),
        encoding="utf-8",
    )


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_frozen_dataset(
    frozen_root: Path,
    archive_path: Path,
) -> dict[str, Any]:
    if not frozen_root.is_dir():
        raise RuntimeError(f"Frozen dataset directory not found: {frozen_root}")

    checksums_path = frozen_root / "SHA256SUMS.txt"
    manifest_path = frozen_root / "FREEZE_MANIFEST.json"
    if not checksums_path.is_file() or not manifest_path.is_file():
        raise RuntimeError("Frozen dataset manifest/checksums are missing")

    checksums_sha = file_sha256(checksums_path)
    manifest_sha = file_sha256(manifest_path)
    if checksums_sha != RAW_DATASET_CHECKSUMS_SHA256:
        raise RuntimeError(
            f"SHA256SUMS hash is {checksums_sha}, "
            f"expected {RAW_DATASET_CHECKSUMS_SHA256}"
        )
    if manifest_sha != RAW_DATASET_FREEZE_MANIFEST_SHA256:
        raise RuntimeError(
            f"FREEZE_MANIFEST hash is {manifest_sha}, "
            f"expected {RAW_DATASET_FREEZE_MANIFEST_SHA256}"
        )

    root_resolved = frozen_root.resolve()
    verified_files = []
    for line_number, raw_line in enumerate(
        checksums_path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        if not raw_line.strip():
            continue
        try:
            expected_hash, relative_name = raw_line.split("  ", 1)
        except ValueError as exc:
            raise RuntimeError(
                f"Malformed SHA256SUMS line {line_number}: {raw_line!r}"
            ) from exc
        candidate = (frozen_root / relative_name).resolve()
        if root_resolved != candidate and root_resolved not in candidate.parents:
            raise RuntimeError(f"Unsafe checksum path: {relative_name}")
        if not candidate.is_file():
            raise RuntimeError(f"Frozen file missing: {relative_name}")
        actual_hash = file_sha256(candidate)
        if actual_hash != expected_hash:
            raise RuntimeError(
                f"Frozen file hash mismatch: {relative_name}: "
                f"{actual_hash} != {expected_hash}"
            )
        verified_files.append(relative_name)

    if not archive_path.is_file():
        raise RuntimeError(f"Frozen dataset archive not found: {archive_path}")
    archive_sha = file_sha256(archive_path)
    if archive_sha != RAW_DATASET_ARCHIVE_SHA256:
        raise RuntimeError(
            f"Frozen archive hash is {archive_sha}, "
            f"expected {RAW_DATASET_ARCHIVE_SHA256}"
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_checks = {
        "freeze_id": manifest.get("freeze_id") == RAW_DATASET_FREEZE_ID,
        "calibration_id": manifest.get("calibration_id") == CALIBRATION_ID,
        "pair_count": manifest.get("pair_count") == 74,
        "metadata_sha256":
            manifest.get("capture_metadata_sha256")
            == RAW_CAPTURE_METADATA_SHA256,
        "thermal_status":
            manifest.get("thermal_status") == EXPECTED_THERMAL_STATUS,
        "target_square_size":
            manifest.get("target_square_size_mm")
            == EXPECTED_SQUARE_SIZE_MM,
    }
    failed = [name for name, passed in manifest_checks.items() if not passed]
    if failed:
        raise RuntimeError(
            "Frozen dataset manifest contract failed: " + ", ".join(failed)
        )

    return {
        "frozen_root": str(frozen_root),
        "checksums_sha256": checksums_sha,
        "freeze_manifest_sha256": manifest_sha,
        "archive_path": str(archive_path),
        "archive_sha256": archive_sha,
        "verified_file_count": len(verified_files),
        "manifest_contract_checks": manifest_checks,
    }


def validate_dataset_provenance(
    metadata_path: Path,
    expected_sha256: str,
    expected_pairs: int,
) -> dict[str, Any]:
    if not metadata_path.is_file():
        raise RuntimeError(f"Capture metadata not found: {metadata_path}")

    actual_sha256 = file_sha256(metadata_path)
    if actual_sha256 != expected_sha256:
        raise RuntimeError(
            "Frozen dataset provenance check failed: "
            f"capture_metadata SHA-256 is {actual_sha256}, "
            f"expected {expected_sha256}"
        )

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    thermal = metadata.get("thermal_stabilization", {})
    channels = thermal.get("temperature_channels", {})
    checks = {
        "calibration_id": metadata.get("calibration_id") == CALIBRATION_ID,
        "experiment": metadata.get("experiment") == EXPERIMENT,
        "dataset_name":
            metadata.get("dataset_name") == "stereo_calibration_v6_raw",
        "num_pairs_total": metadata.get("num_pairs_total") == expected_pairs,
        "target_square_size":
            metadata.get("target", {}).get("square_size_mm")
            == EXPECTED_SQUARE_SIZE_MM,
        "saved_left":
            metadata.get("saved_image_semantics", {}).get("left")
            == "physical_left",
        "saved_right":
            metadata.get("saved_image_semantics", {}).get("right")
            == "physical_right",
        "thermal_status": thermal.get("status") == EXPECTED_THERMAL_STATUS,
        "camera_left_sensor":
            channels.get("camera_left", {}).get("sensor_id") == "S03"
            and channels.get("camera_left", {}).get("rom")
            == "28F952990F510A0C",
        "camera_right_sensor":
            channels.get("camera_right", {}).get("sensor_id") == "S02"
            and channels.get("camera_right", {}).get("rom")
            == "28FE6BA3299C9AC4",
        "ambient_sensor":
            channels.get("ambient", {}).get("sensor_id") == "S10"
            and channels.get("ambient", {}).get("rom")
            == "289452A555A32FDA",
    }

    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(
            "Frozen dataset metadata contract failed: "
            + ", ".join(failed)
        )

    return {
        "capture_metadata_path": str(metadata_path),
        "capture_metadata_sha256": actual_sha256,
        "thermal_status": thermal.get("status"),
        "contract_checks": checks,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fields = sorted(set().union(*(row.keys() for row in rows)))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def robust_stats(values: list[float] | np.ndarray) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if len(arr) == 0:
        return {k: math.nan for k in ["mean", "median", "std", "p95", "max"]} | {"count": 0}
    return {
        "count": int(len(arr)),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def find_corners(gray: np.ndarray, checkerboard: tuple[int, int]):
    flags = (
        cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    ok, corners = cv2.findChessboardCornersSB(gray, checkerboard, flags=flags)
    if ok and corners is not None:
        # SB уже возвращает субпиксельные координаты.
        return True, corners.astype(np.float32)

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, checkerboard, classic_flags)
    if not ok or corners is None:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-5,
    )
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return True, corners.astype(np.float32)


def make_object_points(checkerboard: tuple[int, int], square_mm: float) -> np.ndarray:
    cols, rows = checkerboard
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_mm)
    return objp


def load_corner_order_corrections(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}

    if not path.is_file():
        raise RuntimeError(f"Corner-order correction manifest not found: {path}")

    actual_sha256 = file_sha256(path)
    if actual_sha256 != CORNER_ORDER_CORRECTIONS_SHA256:
        raise RuntimeError(
            f"Corner-order correction manifest SHA-256 is {actual_sha256}, "
            f"expected {CORNER_ORDER_CORRECTIONS_SHA256}"
        )

    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise RuntimeError("Unsupported corner-order correction schema")

    corrections = payload.get("corrections")
    if not isinstance(corrections, dict):
        raise RuntimeError("Corner-order corrections must be an object")

    normalized: dict[str, str] = {}
    for filename, action in corrections.items():
        filename = str(filename)
        action = str(action)
        if action != "reverse_right_180":
            raise RuntimeError(
                f"Unsupported corner-order correction: {filename}={action}"
            )
        normalized[filename] = action

    return normalized


def load_pairs(
    left_pattern: str,
    right_pattern: str,
    checkerboard: tuple[int, int],
    corner_order_corrections: dict[str, str],
):
    left_files = sorted(glob.glob(left_pattern))
    right_files = sorted(glob.glob(right_pattern))

    if not left_files:
        raise RuntimeError(f"No images found: {left_pattern}")
    if len(left_files) != len(right_files):
        raise RuntimeError(f"Pairs mismatch: left={len(left_files)}, right={len(right_files)}")

    pairs = []
    image_size = None

    for lf, rf in zip(left_files, right_files):
        lp = Path(lf)
        rp = Path(rf)
        if lp.name != rp.name:
            raise RuntimeError(f"Filename mismatch: {lp.name} != {rp.name}")

        left = cv2.imread(str(lp), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(rp), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            print(f"SKIP unreadable: {lp.name}")
            continue
        if left.shape != right.shape:
            print(f"SKIP size mismatch: {lp.name}")
            continue

        current_size = (left.shape[1], left.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            print(f"SKIP inconsistent resolution: {lp.name}")
            continue

        ok_l, cl = find_corners(left, checkerboard)
        ok_r, cr = find_corners(right, checkerboard)
        if not (ok_l and ok_r and cl is not None and cr is not None):
            print(f"SKIP corners not found: {lp.name}")
            continue

        left_xy = cl.reshape(-1, 2)
        right_xy = cr.reshape(-1, 2)
        reversed_right_xy = right_xy[::-1]

        direct_median_dy = float(
            np.median(np.abs(left_xy[:, 1] - right_xy[:, 1]))
        )
        reversed_median_dy = float(
            np.median(np.abs(left_xy[:, 1] - reversed_right_xy[:, 1]))
        )

        correction = corner_order_corrections.get(lp.name)

        if correction == "reverse_right_180":
            strongly_supported = (
                reversed_median_dy < direct_median_dy * 0.25
                and direct_median_dy - reversed_median_dy > 20.0
            )
            if not strongly_supported:
                raise RuntimeError(
                    f"Corner-order correction not supported by geometry for "
                    f"{lp.name}: direct_median_dy={direct_median_dy:.3f}, "
                    f"reversed_median_dy={reversed_median_dy:.3f}"
                )

            cr = cr[::-1].copy()
            print(
                f"CORNER_ORDER_CORRECTED: {lp.name}: "
                f"right reversed 180 deg; "
                f"direct_median_dy={direct_median_dy:.3f}; "
                f"corrected_median_dy={reversed_median_dy:.3f}"
            )

        elif (
            reversed_median_dy < direct_median_dy * 0.25
            and direct_median_dy - reversed_median_dy > 20.0
        ):
            raise RuntimeError(
                f"Unlisted probable corner-order reversal: {lp.name}: "
                f"direct_median_dy={direct_median_dy:.3f}, "
                f"reversed_median_dy={reversed_median_dy:.3f}"
            )

        pairs.append({
            "filename": lp.name,
            "left_path": lp,
            "right_path": rp,
            "left_image": left,
            "right_image": right,
            "left_corners": cl,
            "right_corners": cr,
            "corner_order_correction": correction or "none",
            "direct_median_dy_px": direct_median_dy,
            "reversed_median_dy_px": reversed_median_dy,
        })
        print(f"Loaded: {lp.name}")

    if image_size is None:
        raise RuntimeError("No readable image pairs")

    return pairs, image_size


def calibrate_camera(objpoints, imgpoints, image_size):
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        200,
        1e-10,
    )
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
        criteria=criteria,
    )

    per_view = []
    for obj, img, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        error = np.linalg.norm(
            img.reshape(-1, 2) - projected.reshape(-1, 2),
            axis=1,
        )
        per_view.append(float(np.sqrt(np.mean(error ** 2))))

    return {
        "rms": float(rms),
        "K": K,
        "D": D,
        "per_view_errors": per_view,
    }


def evaluate_camera_model(objpoints, imgpoints, K, D):
    per_view = []
    squared_error_sum = 0.0
    point_count = 0

    for obj, img in zip(objpoints, imgpoints):
        ok, rvec, tvec = cv2.solvePnP(
            obj,
            img,
            K,
            D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed while evaluating final camera model")

        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        residual = (
            img.reshape(-1, 2).astype(np.float64)
            - projected.reshape(-1, 2).astype(np.float64)
        )
        squared = np.sum(residual ** 2, axis=1)
        per_view.append(float(np.sqrt(np.mean(squared))))
        squared_error_sum += float(np.sum(squared))
        point_count += int(len(squared))

    if point_count == 0:
        raise RuntimeError("No points available for final camera model evaluation")

    return {
        "rms": float(np.sqrt(squared_error_sum / point_count)),
        "per_view_errors": per_view,
    }


def select_mask(left_errors, right_errors, absolute_limit: float, mad_scale: float):
    combined = np.maximum(np.asarray(left_errors), np.asarray(right_errors))
    median = float(np.median(combined))
    mad = float(np.median(np.abs(combined - median)))
    robust_limit = median + mad_scale * max(mad, 1e-6)
    limit = min(absolute_limit, robust_limit)
    return combined <= limit, float(limit)


def stereo_calibrate(
    objpoints,
    left_points,
    right_points,
    left_cal,
    right_cal,
    image_size,
    joint_refine_intrinsics: bool = False,
):
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        300,
        1e-10,
    )
    flags = (
        cv2.CALIB_USE_INTRINSIC_GUESS
        if joint_refine_intrinsics
        else cv2.CALIB_FIX_INTRINSIC
    )

    ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        left_points,
        right_points,
        left_cal["K"].copy(),
        left_cal["D"].copy(),
        right_cal["K"].copy(),
        right_cal["D"].copy(),
        image_size,
        criteria=criteria,
        flags=flags,
    )
    return {
        "stereo_rms": float(ret),
        "K1": K1,
        "D1": D1,
        "K2": K2,
        "D2": D2,
        "R": R,
        "T": T,
        "E": E,
        "F": F,
    }


def build_rectification(stereo, image_size):
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        stereo["K1"],
        stereo["D1"],
        stereo["K2"],
        stereo["D2"],
        image_size,
        stereo["R"],
        stereo["T"],
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
    )

    lmx, lmy = cv2.initUndistortRectifyMap(
        stereo["K1"], stereo["D1"], R1, P1, image_size, cv2.CV_32FC1
    )
    rmx, rmy = cv2.initUndistortRectifyMap(
        stereo["K2"], stereo["D2"], R2, P2, image_size, cv2.CV_32FC1
    )

    return {
        "R1": R1,
        "R2": R2,
        "P1": P1,
        "P2": P2,
        "Q": Q,
        "roi1": tuple(int(v) for v in roi1),
        "roi2": tuple(int(v) for v in roi2),
        "lmx": lmx,
        "lmy": lmy,
        "rmx": rmx,
        "rmy": rmy,
    }


def rectify_points(corners, K, D, R, P):
    return cv2.undistortPoints(corners, K, D, R=R, P=P).reshape(-1, 2)


def validate_rectification(pairs, stereo, rectification):
    all_dy = []
    all_abs_dy = []
    all_disp = []
    rows = []

    for pair in pairs:
        left = rectify_points(
            pair["left_corners"], stereo["K1"], stereo["D1"],
            rectification["R1"], rectification["P1"]
        )
        right = rectify_points(
            pair["right_corners"], stereo["K2"], stereo["D2"],
            rectification["R2"], rectification["P2"]
        )

        dy = left[:, 1] - right[:, 1]
        abs_dy = np.abs(dy)
        disp = left[:, 0] - right[:, 0]

        all_dy.extend(dy.tolist())
        all_abs_dy.extend(abs_dy.tolist())
        all_disp.extend(disp.tolist())

        rows.append({
            "filename": pair["filename"],
            "median_signed_dy_px": float(np.median(dy)),
            "median_abs_dy_px": float(np.median(abs_dy)),
            "p95_abs_dy_px": float(np.percentile(abs_dy, 95)),
            "max_abs_dy_px": float(np.max(abs_dy)),
            "median_disparity_px": float(np.median(disp)),
            "positive_disparity_ratio": float(np.mean(disp > 0)),
        })

    abs_arr = np.asarray(all_abs_dy, dtype=np.float64)
    disp_arr = np.asarray(all_disp, dtype=np.float64)

    summary = {
        "absolute_vertical_error_px": robust_stats(abs_arr),
        "signed_vertical_error_px": robust_stats(all_dy),
        "disparity_px": robust_stats(disp_arr),
        "within_0_25_px": float(np.mean(abs_arr <= 0.25)),
        "within_0_50_px": float(np.mean(abs_arr <= 0.50)),
        "within_1_00_px": float(np.mean(abs_arr <= 1.00)),
        "positive_disparity_ratio": float(np.mean(disp_arr > 0)),
    }
    return summary, rows


def save_previews(pairs, rectification, output_dir: Path, count: int):
    output_dir.mkdir(parents=True, exist_ok=True)
    indices = np.linspace(0, len(pairs) - 1, min(count, len(pairs)), dtype=int)

    for preview_index, pair_index in enumerate(indices):
        pair = pairs[int(pair_index)]
        left = cv2.remap(
            pair["left_image"], rectification["lmx"], rectification["lmy"], cv2.INTER_LINEAR
        )
        right = cv2.remap(
            pair["right_image"], rectification["rmx"], rectification["rmy"], cv2.INTER_LINEAR
        )
        combined = cv2.cvtColor(np.hstack([left, right]), cv2.COLOR_GRAY2BGR)
        for y in range(0, combined.shape[0], 40):
            cv2.line(combined, (0, y), (combined.shape[1] - 1, y), (0, 255, 0), 1)
        cv2.putText(
            combined, pair["filename"], (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA
        )
        cv2.imwrite(
            str(output_dir / f"rectified_{preview_index:02d}_{pair['filename']}"),
            combined,
        )


def metric_status(value: float, pass_limit: float, warning_limit: float):
    if value <= pass_limit:
        return PASS
    if value <= warning_limit:
        return WARNING
    return FAIL


def decision(left_rms, right_rms, stereo_rms, baseline_mm, physical_baseline_mm, rect):
    median_dy = rect["absolute_vertical_error_px"]["median"]
    p95_dy = rect["absolute_vertical_error_px"]["p95"]
    positive_ratio = rect["positive_disparity_ratio"]

    checks = {
        "left_rms": {"value": left_rms, "status": metric_status(left_rms, 0.60, 0.90)},
        "right_rms": {"value": right_rms, "status": metric_status(right_rms, 0.60, 0.90)},
        "stereo_rms": {"value": stereo_rms, "status": metric_status(stereo_rms, 0.80, 1.20)},
        "baseline": {
            "value_mm": baseline_mm,
            "status": PASS if 63 <= baseline_mm <= 67 else WARNING if 60 <= baseline_mm <= 70 else FAIL,
        },
        "median_abs_dy": {"value_px": median_dy, "status": metric_status(median_dy, 0.25, 0.75)},
        "p95_abs_dy": {"value_px": p95_dy, "status": metric_status(p95_dy, 1.0, 2.0)},
        "positive_disparity_ratio": {
            "value": positive_ratio,
            "status": PASS if positive_ratio >= 0.95 else WARNING if positive_ratio >= 0.80 else FAIL,
        },
    }

    if physical_baseline_mm is not None:
        diff = abs(baseline_mm - physical_baseline_mm)
        checks["physical_baseline_agreement"] = {
            "difference_mm": diff,
            "status": PASS if diff <= 1.0 else WARNING if diff <= 3.0 else FAIL,
        }

    statuses = [check["status"] for check in checks.values()]
    overall = FAIL if FAIL in statuses else WARNING if WARNING in statuses else PASS
    return {
        "overall_status": overall,
        "solver_quality_pass": overall == PASS,
        "approved_candidate": False,
        "activation_eligible": False,
        "activation_blockers": [
            "capture_thermal_provenance_incomplete",
            "fresh_v6_checkerboard_rectification_gate_pending",
            "near_mid_far_physical_depth_validation_pending",
        ],
        "checks": checks,
    }


def build_markdown(summary: dict[str, Any]) -> str:
    rect = summary["rectification"]
    lines = [
        "# Stereo Calibration v6 Simple Solver",
        "",
        f"- Status: **{summary['decision']['overall_status']}**",
        f"- Solver quality pass: `{summary['decision']['solver_quality_pass']}`",
        f"- Approved candidate: `{summary['decision']['approved_candidate']}`",
        f"- Activation eligible: `{summary['decision']['activation_eligible']}`",
        f"- Input pairs: `{summary['num_input_pairs']}`",
        f"- Used pairs: `{summary['num_used_pairs']}`",
        f"- Outliers: `{summary['num_outliers']}`",
        f"- Stereo intrinsics mode: `{summary['stereo_intrinsics_mode']}`",
        "",
        "## Calibration",
        "",
        f"- Left final-model RMS: `{summary['left_rms_px']:.6f}` px",
        f"- Right final-model RMS: `{summary['right_rms_px']:.6f}` px",
        f"- Left initial monocular RMS: `{summary['left_initial_rms_px']:.6f}` px",
        f"- Right initial monocular RMS: `{summary['right_initial_rms_px']:.6f}` px",
        f"- Stereo RMS: `{summary['stereo_rms_px']:.6f}` px",
        f"- Baseline: `{summary['baseline_mm']:.6f}` mm",
        f"- Physical baseline difference: `{summary['physical_difference_mm']}` mm",
        f"- Rectified fx: `{summary['rectified_fx_px']:.6f}` px",
        f"- fx × B: `{summary['fx_times_baseline_m_px']:.9f}` m·px",
        "",
        "## Rectification",
        "",
        f"- Median |dy|: `{rect['absolute_vertical_error_px']['median']:.6f}` px",
        f"- P95 |dy|: `{rect['absolute_vertical_error_px']['p95']:.6f}` px",
        f"- Signed median dy: `{rect['signed_vertical_error_px']['median']:.6f}` px",
        f"- Within 0.25 px: `{rect['within_0_25_px']:.2%}`",
        f"- Within 0.50 px: `{rect['within_0_50_px']:.2%}`",
        f"- Within 1.00 px: `{rect['within_1_00_px']:.2%}`",
        f"- Positive disparity: `{rect['positive_disparity_ratio']:.2%}`",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--frozen_dataset_root",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6_raw_frozen_2026-08-20_v2"
        ),
    )
    parser.add_argument(
        "--dataset_archive",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6_raw_frozen_2026-08-20_v2.tar.gz"
        ),
    )
    parser.add_argument(
        "--left",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6_raw_frozen_2026-08-20_v2/"
            "raw_pairs/left/*.png"
        ),
    )
    parser.add_argument(
        "--right",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6_raw_frozen_2026-08-20_v2/"
            "raw_pairs/right/*.png"
        ),
    )
    parser.add_argument(
        "--output_dir",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6/solution_all_pairs_fix_intrinsic"
        ),
    )
    parser.add_argument(
        "--capture_metadata",
        default=(
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v6_raw_frozen_2026-08-20_v2/"
            "raw_pairs/capture_metadata.json"
        ),
    )
    parser.add_argument("--board_cols", type=int, default=9)
    parser.add_argument("--board_rows", type=int, default=6)
    parser.add_argument(
        "--corner_order_corrections",
        type=Path,
        default=None,
    )
    parser.add_argument("--square_size_mm", type=float, default=24.5)
    parser.add_argument("--physical_baseline_mm", type=float, default=65.10)
    parser.add_argument("--maximum_pair_error_px", type=float, default=1.20)
    parser.add_argument("--outlier_mad_scale", type=float, default=3.5)
    parser.add_argument("--minimum_pairs", type=int, default=20)
    parser.add_argument("--expected_pairs", type=int, default=27)
    parser.add_argument("--preview_count", type=int, default=8)
    parser.add_argument("--joint_refine_intrinsics", action="store_true")
    filter_group = parser.add_mutually_exclusive_group()
    filter_group.add_argument("--enable_outlier_filter", action="store_true")
    filter_group.add_argument("--disable_outlier_filter", action="store_true")
    args = parser.parse_args()

    checkerboard = (args.board_cols, args.board_rows)
    output_dir = Path(args.output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    frozen_provenance = validate_frozen_dataset(
        Path(args.frozen_dataset_root),
        Path(args.dataset_archive),
    )
    provenance = validate_dataset_provenance(
        Path(args.capture_metadata),
        RAW_CAPTURE_METADATA_SHA256,
        args.expected_pairs,
    )

    corner_order_corrections = load_corner_order_corrections(
        args.corner_order_corrections
    )
    pairs, image_size = load_pairs(
        args.left,
        args.right,
        checkerboard,
        corner_order_corrections,
    )
    corner_order_corrections_applied = {
        pair["filename"]: pair["corner_order_correction"]
        for pair in pairs
        if pair["corner_order_correction"] != "none"
    }
    if len(pairs) != args.expected_pairs:
        raise RuntimeError(
            f"Frozen dataset pair count mismatch: "
            f"detected={len(pairs)}, expected={args.expected_pairs}"
        )
    if len(pairs) < args.minimum_pairs:
        raise RuntimeError(f"Too few usable pairs: {len(pairs)}")

    if not math.isclose(
        args.square_size_mm,
        EXPECTED_SQUARE_SIZE_MM,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            f"Square size is {args.square_size_mm}, "
            f"expected {EXPECTED_SQUARE_SIZE_MM}"
        )

    obj_template = make_object_points(checkerboard, args.square_size_mm)
    objpoints_initial = [obj_template.copy() for _ in pairs]
    left_initial_points = [pair["left_corners"] for pair in pairs]
    right_initial_points = [pair["right_corners"] for pair in pairs]

    print("\nStereo Calibration v6 Simple Solver")
    print("-----------------------------------")
    print("Image size:", image_size)
    print("Detected pairs:", len(pairs))
    print("Checkerboard:", checkerboard)
    print("Square size mm:", args.square_size_mm)
    print("Dataset provenance: PASS")
    print("Frozen files verified:", frozen_provenance["verified_file_count"])
    print("Archive SHA-256:", frozen_provenance["archive_sha256"])
    print("Capture metadata SHA-256:", provenance["capture_metadata_sha256"])

    print("\nInitial left calibration...")
    left_initial = calibrate_camera(objpoints_initial, left_initial_points, image_size)
    print("Initial right calibration...")
    right_initial = calibrate_camera(objpoints_initial, right_initial_points, image_size)

    if args.enable_outlier_filter:
        mask, pair_limit = select_mask(
            left_initial["per_view_errors"],
            right_initial["per_view_errors"],
            args.maximum_pair_error_px,
            args.outlier_mad_scale,
        )
    else:
        mask = np.ones(len(pairs), dtype=bool)
        pair_limit = None

    used_pairs = [pair for pair, keep in zip(pairs, mask) if bool(keep)]
    outliers = [pair for pair, keep in zip(pairs, mask) if not bool(keep)]

    if len(used_pairs) < args.minimum_pairs:
        raise RuntimeError(f"Too few pairs after filtering: {len(used_pairs)}")

    objpoints = [obj_template.copy() for _ in used_pairs]
    left_points = [pair["left_corners"] for pair in used_pairs]
    right_points = [pair["right_corners"] for pair in used_pairs]

    print("Pairs used:", len(used_pairs))
    print("Outliers:", len(outliers))
    print("Pair error limit:", pair_limit)

    print("\nFinal left calibration...")
    left_final = calibrate_camera(objpoints, left_points, image_size)
    print("Final right calibration...")
    right_final = calibrate_camera(objpoints, right_points, image_size)

    print("Stereo calibration...")
    stereo = stereo_calibrate(
        objpoints,
        left_points,
        right_points,
        left_final,
        right_final,
        image_size,
        joint_refine_intrinsics=args.joint_refine_intrinsics,
    )

    left_model_evaluation = evaluate_camera_model(
        objpoints,
        left_points,
        stereo["K1"],
        stereo["D1"],
    )
    right_model_evaluation = evaluate_camera_model(
        objpoints,
        right_points,
        stereo["K2"],
        stereo["D2"],
    )

    baseline_mm = float(np.linalg.norm(stereo["T"]))

    if (
        not np.isfinite(stereo["stereo_rms"])
        or stereo["stereo_rms"] > 100.0
        or baseline_mm > 200.0
    ):
        raise RuntimeError(
            "Stereo calibration is physically invalid: "
            f"stereo_rms={stereo['stereo_rms']}, baseline_mm={baseline_mm}. "
            "Check stereo pairing and checkerboard correspondence."
        )

    rectification = build_rectification(stereo, image_size)
    rect_summary, rect_rows = validate_rectification(used_pairs, stereo, rectification)

    median_dy = float(rect_summary["absolute_vertical_error_px"]["median"])
    p95_dy = float(rect_summary["absolute_vertical_error_px"]["p95"])
    fx = float(rectification["P1"][0, 0])
    fx_b = fx * baseline_mm / 1000.0
    physical_diff = (
        abs(baseline_mm - args.physical_baseline_mm)
        if args.physical_baseline_mm is not None
        else None
    )

    final_decision = decision(
        left_model_evaluation["rms"],
        right_model_evaluation["rms"],
        stereo["stereo_rms"],
        baseline_mm,
        args.physical_baseline_mm,
        rect_summary,
    )

    calibration_path = output_dir / "stereo_params_v6.npz"
    np.savez(
        calibration_path,
        calibration_id=np.asarray(
            CALIBRATION_ID
        ),
        experiment=np.asarray(
            EXPERIMENT
        ),
        camera_1_semantics=np.asarray(
            "physical_left"
        ),
        camera_2_semantics=np.asarray(
            "physical_right"
        ),
        raw_dataset_manifest_sha256=np.asarray(
            RAW_DATASET_CHECKSUMS_SHA256
        ),
        raw_dataset_freeze_manifest_sha256=np.asarray(
            RAW_DATASET_FREEZE_MANIFEST_SHA256
        ),
        raw_dataset_freeze_id=np.asarray(
            RAW_DATASET_FREEZE_ID
        ),
        raw_dataset_archive_sha256=np.asarray(
            RAW_DATASET_ARCHIVE_SHA256
        ),
        raw_capture_metadata_sha256=np.asarray(
            RAW_CAPTURE_METADATA_SHA256
        ),
        previous_calibration_id=np.asarray(
            PREVIOUS_CALIBRATION_ID
        ),
        previous_calibration_status=np.asarray(
            PREVIOUS_CALIBRATION_STATUS
        ),
        solver_version=np.asarray(
            SOLVER_VERSION
        ),
        capture_thermal_status=np.asarray(
            EXPECTED_THERMAL_STATUS
        ),
        activation_eligible=np.asarray(
            False
        ),
        stereo_intrinsics_mode=np.asarray(
            "joint_refine"
            if args.joint_refine_intrinsics
            else "fix_intrinsic"
        ),
        K1=stereo["K1"], D1=stereo["D1"],
        K2=stereo["K2"], D2=stereo["D2"],
        R=stereo["R"], T=stereo["T"], E=stereo["E"], F=stereo["F"],
        R1=rectification["R1"], R2=rectification["R2"],
        P1=rectification["P1"], P2=rectification["P2"], Q=rectification["Q"],
        roi1=np.asarray(rectification["roi1"], dtype=np.int32),
        roi2=np.asarray(rectification["roi2"], dtype=np.int32),
        size=np.asarray(image_size, dtype=np.int32),
        board_size=np.asarray(checkerboard, dtype=np.int32),
        square_size_mm=np.asarray(args.square_size_mm, dtype=np.float64),
        left_rms=np.asarray(
            left_model_evaluation["rms"], dtype=np.float64
        ),
        right_rms=np.asarray(
            right_model_evaluation["rms"], dtype=np.float64
        ),
        left_initial_rms=np.asarray(left_final["rms"], dtype=np.float64),
        right_initial_rms=np.asarray(right_final["rms"], dtype=np.float64),
        stereo_rms=np.asarray(stereo["stereo_rms"], dtype=np.float64),
        baseline_mm=np.asarray(baseline_mm, dtype=np.float64),
        rectification_median_abs_dy_px=np.asarray(median_dy, dtype=np.float64),
        rectification_p95_abs_dy_px=np.asarray(p95_dy, dtype=np.float64),
    )

    final_left_by_name = {
        pair["filename"]: error
        for pair, error in zip(
            used_pairs,
            left_model_evaluation["per_view_errors"],
        )
    }
    final_right_by_name = {
        pair["filename"]: error
        for pair, error in zip(
            used_pairs,
            right_model_evaluation["per_view_errors"],
        )
    }

    pair_rows = []
    for index, pair in enumerate(pairs):
        pair_rows.append({
            "filename": pair["filename"],
            "left_initial_error_px": left_initial["per_view_errors"][index],
            "right_initial_error_px": right_initial["per_view_errors"][index],
            "left_final_model_error_px":
                final_left_by_name.get(pair["filename"]),
            "right_final_model_error_px":
                final_right_by_name.get(pair["filename"]),
            "kept_after_filter": bool(mask[index]),
        })

    write_csv(output_dir / "pair_errors.csv", pair_rows)
    write_csv(output_dir / "rectification_pair_errors.csv", rect_rows)
    save_previews(
        used_pairs,
        rectification,
        output_dir / "rectification_previews",
        args.preview_count,
    )

    summary = {
        "calibration_id": CALIBRATION_ID,
        "experiment": EXPERIMENT,
        "camera_1_semantics": "physical_left",
        "camera_2_semantics": "physical_right",
        "raw_dataset_freeze_id": RAW_DATASET_FREEZE_ID,
        "raw_dataset_archive_sha256": RAW_DATASET_ARCHIVE_SHA256,
        "raw_dataset_manifest_sha256": RAW_DATASET_CHECKSUMS_SHA256,
        "raw_dataset_freeze_manifest_sha256":
            RAW_DATASET_FREEZE_MANIFEST_SHA256,
        "raw_capture_metadata_sha256": RAW_CAPTURE_METADATA_SHA256,
        "previous_calibration_id": PREVIOUS_CALIBRATION_ID,
        "previous_calibration_status": PREVIOUS_CALIBRATION_STATUS,
        "dataset_provenance": provenance,
        "frozen_dataset_provenance": frozen_provenance,
        "capture_thermal_status": EXPECTED_THERMAL_STATUS,
        "solver": SOLVER_NAME,
        "solver_version": SOLVER_VERSION,
        "outlier_filter_enabled": bool(args.enable_outlier_filter),
        "stereo_intrinsics_mode": (
            "joint_refine"
            if args.joint_refine_intrinsics
            else "fix_intrinsic"
        ),
        "checkerboard_inner_corners": list(checkerboard),
        "square_size_mm": args.square_size_mm,
        "image_size": list(image_size),
        "num_input_pairs": len(pairs),
        "num_used_pairs": len(used_pairs),
        "num_outliers": len(outliers),
        "outlier_pair_names": [pair["filename"] for pair in outliers],
        "pair_error_limit_px": pair_limit,
        "left_rms_px": left_model_evaluation["rms"],
        "right_rms_px": right_model_evaluation["rms"],
        "left_initial_rms_px": left_final["rms"],
        "right_initial_rms_px": right_final["rms"],
        "stereo_rms_px": stereo["stereo_rms"],
        "baseline_mm": baseline_mm,
        "physical_baseline_mm": args.physical_baseline_mm,
        "physical_difference_mm": physical_diff,
        "rectified_fx_px": fx,
        "fx_times_baseline_m_px": fx_b,
        "translation_vector_mm": stereo["T"].reshape(-1).tolist(),
        "rectification": rect_summary,
        "decision": final_decision,
        "calibration_file": str(calibration_path),
        "corner_reordering_applied": bool(
            corner_order_corrections_applied
        ),
        "corner_order_corrections": corner_order_corrections_applied,
        "corner_order_correction_manifest":
            str(args.corner_order_corrections)
            if args.corner_order_corrections is not None
            else None,
        "corner_order_correction_manifest_sha256":
            file_sha256(args.corner_order_corrections)
            if args.corner_order_corrections is not None
            else None,
    }

    save_json(
        output_dir / "calibration_report.json",
        {
            "summary": summary,
            "pair_errors": pair_rows,
            "rectification_pair_errors": rect_rows,
        },
    )
    (output_dir / "calibration_report.md").write_text(
        build_markdown(summary),
        encoding="utf-8",
    )

    print("\nStereo Calibration v6")
    print("---------------------")
    print("Input pairs:", len(pairs))
    print("Used pairs:", len(used_pairs))
    print("Outliers:", len(outliers))
    print("\nLeft final-model RMS:", left_model_evaluation["rms"])
    print("Right final-model RMS:", right_model_evaluation["rms"])
    print("Left initial monocular RMS:", left_final["rms"])
    print("Right initial monocular RMS:", right_final["rms"])
    print("Stereo RMS:", stereo["stereo_rms"])
    print("Baseline mm:", baseline_mm)
    print("Physical difference mm:", physical_diff)
    print("\nMedian |dy| px:", median_dy)
    print("P95 |dy| px:", p95_dy)
    print("Within 1 px:", rect_summary["within_1_00_px"])
    print("Positive disparity:", rect_summary["positive_disparity_ratio"])
    print("\nOverall status:", final_decision["overall_status"])
    print("Approved candidate:", final_decision["approved_candidate"])
    print("\nSaved:")
    print(calibration_path)
    print(output_dir / "calibration_report.json")
    print(output_dir / "calibration_report.md")
    print(output_dir / "rectification_previews")

    if final_decision["overall_status"] == FAIL:
        raise SystemExit(2)
    if final_decision["overall_status"] == WARNING:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
