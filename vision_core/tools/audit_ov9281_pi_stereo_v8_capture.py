#!/usr/bin/env python3
"""Audit raw OV9281 v8 candidate captures before calibration solving."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, result = cv2.findChessboardCornersSB(gray, board, flags=flags)
    return None if not found or result is None else result.reshape(-1, 2).astype(np.float32)


def sharpness(image: np.ndarray) -> float:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def stats(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size), "min": float(np.min(array)),
        "p10": float(np.percentile(array, 10)), "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)), "max": float(np.max(array)),
    }


def load_image(path: Path, expected: tuple[int, int]) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Unreadable image: {path}")
    width, height = expected
    if image.shape[:2] != (height, width):
        raise RuntimeError(f"Unexpected size {path}: {image.shape[1]}x{image.shape[0]}; expected {width}x{height}")
    return image


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--report-path", type=Path, required=True)
    parser.add_argument("--min-pairs", type=int, default=72)
    parser.add_argument("--minimum-sharpness", type=float, default=25.0)
    parser.add_argument("--minimum-center-x-span-px", type=float, default=400.0)
    parser.add_argument("--minimum-center-y-span-px", type=float, default=200.0)
    parser.add_argument(
        "--maximum-center-y-min-px", type=float, default=250.0,
        help="at least one board centre must reach this close to the top edge",
    )
    parser.add_argument(
        "--minimum-center-y-max-px", type=float, default=550.0,
        help="at least one board centre must reach this far toward the bottom edge",
    )
    parser.add_argument("--minimum-area-ratio", type=float, default=2.0)
    args = parser.parse_args()

    sessions: list[tuple[Path, Path, dict[str, Any]]] = []
    for supplied in args.dataset_root:
        root = supplied.resolve()
        metadata_path = root / "capture_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("schema_version") != "sie.ov9281.pi_stereo_capture.v8":
            raise RuntimeError(f"Unexpected capture schema: {root}")
        if metadata.get("status") != "COMPLETE":
            raise RuntimeError(f"Capture is not complete: {root}: {metadata.get('status')}")
        sessions.append((root, metadata_path, metadata))

    reference = sessions[0][2]
    board = tuple(int(value) for value in reference["target"]["checkerboard_inner_corners"])
    square_mm = float(reference["target"]["square_size_mm"])
    eye = reference["ov9281_stereo"]["eye_image_size_px"]
    expected = (int(eye["width"]), int(eye["height"]))
    pairs: list[tuple[Path, dict[str, Any]]] = []
    metadata_hashes: dict[str, str] = {}
    for root, metadata_path, metadata in sessions:
        if tuple(metadata["target"]["checkerboard_inner_corners"]) != board:
            raise RuntimeError(f"Checkerboard contract mismatch: {root}")
        if float(metadata["target"]["square_size_mm"]) != square_mm:
            raise RuntimeError(f"Square-size contract mismatch: {root}")
        if metadata["ov9281_stereo"]["eye_image_size_px"] != eye:
            raise RuntimeError(f"Image-size contract mismatch: {root}")
        metadata_hashes[str(root)] = sha256_file(metadata_path)
        pairs.extend((root, pair) for pair in metadata["pairs"])
    if len(pairs) < args.min_pairs:
        raise RuntimeError(f"Too few pairs: {len(pairs)} < {args.min_pairs}")

    problems: list[str] = []
    centers: dict[str, list[list[float]]] = {"physical_left": [], "physical_right": []}
    areas: list[float] = []
    sharpness_values: dict[str, list[float]] = {"physical_left": [], "physical_right": []}
    for root, pair in pairs:
        filename = str(pair["filename"])
        label = f"{root.name}/{filename}"
        sources = {
            "physical_left": (root / "stereo_right" / filename, pair.get("stereo_right_sha256")),
            "physical_right": (root / "stereo_left" / filename, pair.get("stereo_left_sha256")),
        }
        pair_corners: dict[str, np.ndarray] = {}
        for name, (path, expected_hash) in sources.items():
            if not path.is_file():
                problems.append(f"{label}: missing {name}")
                continue
            if sha256_file(path) != expected_hash:
                problems.append(f"{label}: SHA-256 mismatch {name}")
                continue
            image = load_image(path, expected)
            corners = find_corners(image, board)
            if corners is None:
                problems.append(f"{label}: checkerboard not found in {name}")
                continue
            pair_corners[name] = corners
            centers[name].append(np.mean(corners, axis=0).tolist())
            sharpness_values[name].append(sharpness(image))
        if "physical_left" in pair_corners:
            _, (width, height), _ = cv2.minAreaRect(pair_corners["physical_left"].reshape(-1, 2))
            areas.append(float(width * height))

    coverage: dict[str, dict[str, float]] = {}
    for name, values in centers.items():
        if len(values) != len(pairs):
            problems.append(f"{name}: only {len(values)}/{len(pairs)} valid corner sets")
            continue
        array = np.asarray(values, dtype=np.float64)
        coverage[name] = {
            "x_span_px": float(np.ptp(array[:, 0])), "y_span_px": float(np.ptp(array[:, 1])),
            "x_min_px": float(np.min(array[:, 0])), "x_max_px": float(np.max(array[:, 0])),
            "y_min_px": float(np.min(array[:, 1])), "y_max_px": float(np.max(array[:, 1])),
        }
        if coverage[name]["x_span_px"] < args.minimum_center_x_span_px:
            problems.append(f"{name}: weak horizontal coverage")
        if coverage[name]["y_span_px"] < args.minimum_center_y_span_px:
            problems.append(f"{name}: weak vertical coverage")
        if coverage[name]["y_min_px"] > args.maximum_center_y_min_px:
            problems.append(f"{name}: no board centre sufficiently high in image")
        if coverage[name]["y_max_px"] < args.minimum_center_y_max_px:
            problems.append(f"{name}: no board centre sufficiently low in image")

    sharpness_summary = {name: stats(values) for name, values in sharpness_values.items()}
    for name, summary in sharpness_summary.items():
        if summary["p10"] < args.minimum_sharpness:
            problems.append(f"{name}: p10 sharpness below {args.minimum_sharpness}")
    area_summary = stats(areas)
    area_ratio = area_summary["max"] / area_summary["min"]
    if area_ratio < args.minimum_area_ratio:
        problems.append(f"weak distance/scale diversity: area ratio {area_ratio:.2f} < {args.minimum_area_ratio:.2f}")

    report = {
        "schema_version": "sie.ov9281.pi_stereo_capture_audit.v8",
        "status": "CANDIDATE_EVIDENCE_ONLY",
        "dataset_roots": [str(root) for root, _, _ in sessions],
        "capture_metadata_sha256": metadata_hashes,
        "pair_count": len(pairs),
        "board_size": list(board), "square_size_mm": square_mm,
        "corner_center_coverage_px": coverage,
        "required_center_coverage_px": {
            "minimum_x_span_px": args.minimum_center_x_span_px,
            "minimum_y_span_px": args.minimum_center_y_span_px,
            "maximum_y_min_px": args.maximum_center_y_min_px,
            "minimum_y_max_px": args.minimum_center_y_max_px,
        },
        "physical_left_board_area_px2": {**area_summary, "max_over_min": area_ratio},
        "sharpness_laplacian_variance": sharpness_summary,
        "result": "PASS" if not problems else "REJECT",
        "problems": problems,
    }
    args.report_path.parent.mkdir(parents=True, exist_ok=True)
    args.report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Saved: {args.report_path}")
    return 0 if report["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
