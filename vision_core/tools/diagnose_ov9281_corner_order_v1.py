#!/usr/bin/env python3
"""Diagnose OV9281 stream-half identity, transform direction, and corner order."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def find_corners(image: np.ndarray, board: tuple[int, int]) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, board, flags=flags)
    if not found or corners is None:
        raise RuntimeError("checkerboard not found")
    return corners.astype(np.float32)


def object_points(board: tuple[int, int], square_mm: float) -> np.ndarray:
    cols, rows = board
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return points * square_mm


def rms(observed: np.ndarray, predicted: np.ndarray) -> float:
    delta = observed.reshape(-1, 2).astype(float) - predicted.reshape(-1, 2).astype(float)
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def describe(values: list[float]) -> dict[str, float]:
    array = np.asarray(values)
    return {
        "median_px": float(np.median(array)),
        "p95_px": float(np.percentile(array, 95)),
        "max_px": float(np.max(array)),
    }


def hypotheses(
    first: np.ndarray,
    second: np.ndarray,
    K1: np.ndarray, D1: np.ndarray, K2: np.ndarray, D2: np.ndarray,
    R_forward: np.ndarray, T_forward: np.ndarray,
    obj: np.ndarray,
) -> dict[str, float]:
    streams = {
        "captured_first_is_physical_left": (first, second),
        "captured_second_is_physical_left": (second, first),
    }
    directions = {
        "forward_R_T": (R_forward, T_forward),
        "inverse_R_T": (R_forward.T, -R_forward.T @ T_forward),
    }
    result = {}
    for stream_name, (left_image, right_image) in streams.items():
        left_corners, right_corners = find_corners(left_image, BOARD), find_corners(right_image, BOARD)
        ok, rvec, tvec = cv2.solvePnP(obj, left_corners, K1, D1, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError("solvePnP failed")
        R_left_board, _ = cv2.Rodrigues(rvec)
        for direction_name, (R, T) in directions.items():
            R_right_board = R @ R_left_board
            T_right_board = R @ tvec + T
            rvec_right, _ = cv2.Rodrigues(R_right_board)
            predicted, _ = cv2.projectPoints(obj, rvec_right, T_right_board, K2, D2)
            for order, observed in (("direct", right_corners), ("reverse_180", right_corners[::-1])):
                result[f"{stream_name}:{direction_name}:{order}"] = rms(observed, predicted)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, action="append", required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    global BOARD
    with np.load(args.stereo_calibration, allow_pickle=False) as values:
        K1, D1, K2, D2 = values["K1"], values["D1"], values["K2"], values["D2"]
        R_forward, T_forward = values["R"], values["T"]
        BOARD = tuple(int(value) for value in values["board_size"])
        square_mm = float(values["square_size_mm"])
        size = tuple(int(value) for value in values["size"])

    values_by_hypothesis: dict[str, list[float]] = {}
    per_pair = []
    obj = object_points(BOARD, square_mm)
    for root in (path.resolve() for path in args.dataset_root):
        metadata = json.loads((root / "capture_metadata.json").read_text())
        for pair in metadata["pairs"]:
            filename = pair["filename"]
            first = cv2.imread(str(root / "stereo_left" / filename), cv2.IMREAD_COLOR)
            second = cv2.imread(str(root / "stereo_right" / filename), cv2.IMREAD_COLOR)
            if first is None or second is None or first.shape[:2] != (size[1], size[0]) or second.shape[:2] != (size[1], size[0]):
                raise RuntimeError(f"invalid stereo pair: {root.name}/{filename}")
            current = hypotheses(first, second, K1, D1, K2, D2, R_forward, T_forward, obj)
            for key, value in current.items():
                values_by_hypothesis.setdefault(key, []).append(value)
            per_pair.append({"session": root.name, "filename": filename, **current})

    summary = {name: describe(values) for name, values in values_by_hypothesis.items()}
    best = min(summary, key=lambda name: summary[name]["median_px"])
    report = {
        "schema_version": "sie.ov9281.stream_identity_direction_diagnostic.v2",
        "pair_count": len(per_pair),
        "summary": summary,
        "best_hypothesis": best,
        "per_pair": per_pair,
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"pair_count": len(per_pair), "summary": summary, "best_hypothesis": best}, indent=2))
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
