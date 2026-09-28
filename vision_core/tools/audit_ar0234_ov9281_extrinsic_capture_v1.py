#!/usr/bin/env python3
"""Audit one or more raw AR0234 + OV9281 checkerboard capture sessions."""

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


def find_corners(image: np.ndarray, board_size: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, board_size, flags=flags)
    return None if not found or corners is None else corners.reshape(-1, 2).astype(np.float32)


def sharpness(image: np.ndarray) -> float:
    return float(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())


def stats(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(array)),
        "p10": float(np.percentile(array, 10)),
        "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)),
        "max": float(np.max(array)),
    }


def load_image(path: Path, expected_size: tuple[int, int]) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Unreadable image: {path}")
    width, height = expected_size
    if image.shape[:2] != (height, width):
        raise RuntimeError(
            f"Unexpected size {path}: {image.shape[1]}x{image.shape[0]}, "
            f"expected {width}x{height}"
        )
    return image


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--report-path", type=Path, required=True)
    parser.add_argument("--min-pairs", type=int, default=40)
    parser.add_argument("--max-skew-ms", type=float, default=80.0)
    parser.add_argument("--minimum-sharpness", type=float, default=25.0)
    args = parser.parse_args()

    roots = [path.resolve() for path in args.dataset_root]
    sessions: list[tuple[Path, Path, dict[str, Any]]] = []
    for root in roots:
        metadata_path = root / "capture_metadata.json"
        if not metadata_path.is_file():
            raise FileNotFoundError(metadata_path)
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("schema_version") != "sie.ar0234_ov9281.extrinsic_capture.v1":
            raise RuntimeError(f"Unexpected capture schema: {root}")
        if metadata.get("status") != "COMPLETE":
            raise RuntimeError(f"Capture is not complete: {root}: {metadata.get('status')}")
        sessions.append((root, metadata_path, metadata))

    reference = sessions[0][2]
    board_size = tuple(reference["target"]["checkerboard_inner_corners"])
    ar_size = reference["ar0234"]["image_size_px"]
    eye_size = reference["ov9281_stereo"]["eye_image_size_px"]
    ar_expected = (int(ar_size["width"]), int(ar_size["height"]))
    eye_expected = (int(eye_size["width"]), int(eye_size["height"]))
    intrinsic_sha = reference["ar0234"]["intrinsic_sha256"]

    pairs: list[tuple[Path, dict[str, Any]]] = []
    metadata_hashes: dict[str, str] = {}
    for root, metadata_path, metadata in sessions:
        if tuple(metadata["target"]["checkerboard_inner_corners"]) != board_size:
            raise RuntimeError(f"Checkerboard mismatch: {root}")
        if metadata["ar0234"]["image_size_px"] != ar_size:
            raise RuntimeError(f"AR0234 image size mismatch: {root}")
        if metadata["ov9281_stereo"]["eye_image_size_px"] != eye_size:
            raise RuntimeError(f"OV9281 eye image size mismatch: {root}")
        if metadata["ar0234"]["intrinsic_sha256"] != intrinsic_sha:
            raise RuntimeError(f"AR0234 intrinsic mismatch: {root}")
        metadata_hashes[str(root)] = sha256_file(metadata_path)
        pairs.extend((root, pair) for pair in metadata.get("pairs", []))

    if len(pairs) < args.min_pairs:
        raise RuntimeError(f"Too few combined pairs: {len(pairs)} < {args.min_pairs}")

    problems: list[str] = []
    skew_values: list[float] = []
    centers: dict[str, list[list[float]]] = {
        "ar0234": [], "stereo_left": [], "stereo_right": []
    }
    sharpness_values: dict[str, list[float]] = {
        "ar0234": [], "stereo_left": [], "stereo_right": []
    }

    for root, pair in pairs:
        filename = str(pair["filename"])
        label = f"{root.name}/{filename}"
        paths = {
            "ar0234": root / "ar0234" / filename,
            "stereo_left": root / "stereo_left" / filename,
            "stereo_right": root / "stereo_right" / filename,
        }
        hashes = {
            "ar0234": pair.get("ar0234_sha256"),
            "stereo_left": pair.get("stereo_left_sha256"),
            "stereo_right": pair.get("stereo_right_sha256"),
        }
        images: dict[str, np.ndarray] = {}
        for name, path in paths.items():
            if not path.is_file():
                problems.append(f"{label}: missing {name}")
                continue
            if sha256_file(path) != hashes[name]:
                problems.append(f"{label}: sha256 mismatch {name}")
                continue
            images[name] = load_image(path, ar_expected if name == "ar0234" else eye_expected)

        for name, image in images.items():
            corners = find_corners(image, board_size)
            if corners is None:
                problems.append(f"{label}: corners not found in {name}")
                continue
            centers[name].append(np.mean(corners, axis=0).tolist())
            sharpness_values[name].append(sharpness(image))

        skew = float(pair.get("sequential_skew_ms", float("inf")))
        skew_values.append(skew)
        if skew > args.max_skew_ms:
            problems.append(f"{label}: skew {skew:.1f} ms exceeds limit")

    coverage: dict[str, dict[str, float]] = {}
    for name, values in centers.items():
        if len(values) != len(pairs):
            problems.append(f"{name}: only {len(values)}/{len(pairs)} valid corner sets")
            continue
        array = np.asarray(values, dtype=np.float64)
        coverage[name] = {
            "x_min_px": float(np.min(array[:, 0])),
            "x_max_px": float(np.max(array[:, 0])),
            "x_span_px": float(np.ptp(array[:, 0])),
            "y_min_px": float(np.min(array[:, 1])),
            "y_max_px": float(np.max(array[:, 1])),
            "y_span_px": float(np.ptp(array[:, 1])),
        }
        if coverage[name]["x_span_px"] < 200 or coverage[name]["y_span_px"] < 120:
            problems.append(f"{name}: weak image coverage")

    sharpness_summary = {name: stats(values) for name, values in sharpness_values.items()}
    for name, result in sharpness_summary.items():
        if result["p10"] < args.minimum_sharpness:
            problems.append(f"{name}: p10 sharpness below {args.minimum_sharpness}")

    report = {
        "schema_version": "sie.ar0234_ov9281.extrinsic_capture_audit.v1",
        "dataset_roots": [str(root) for root in roots],
        "capture_metadata_sha256": metadata_hashes,
        "pair_count": len(pairs),
        "board_size": list(board_size),
        "sequential_skew_ms": stats(skew_values),
        "corner_center_coverage_px": coverage,
        "sharpness_laplacian_variance": sharpness_summary,
        "result": "PASS" if not problems else "REJECT",
        "problems": problems,
    }
    args.report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Saved: {args.report_path}")
    return 0 if report["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
