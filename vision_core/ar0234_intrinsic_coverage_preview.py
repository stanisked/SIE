"""Pure overlay helpers for live-only AR0234 checkerboard positioning preview."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


CHECKERBOARD_CORNER_COUNT = 54


@dataclass(frozen=True)
class CoverageOverlayState:
    status: str
    color: str
    target_row: int
    target_column: int
    current_row: int | None
    current_column: int | None
    center_px: tuple[float, float] | None

    def as_record(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "color": self.color,
            "target": {"row": self.target_row, "column": self.target_column},
            "current": None if self.current_row is None else {"row": self.current_row, "column": self.current_column},
            "center_px": None if self.center_px is None else [self.center_px[0], self.center_px[1]],
        }


def validate_target(*, row: int, column: int) -> None:
    if type(row) is not int or type(column) is not int or row not in range(3) or column not in range(3):
        raise ValueError("target row and column must be integers in 0..2")


def coverage_bin(*, center_x_px: float, center_y_px: float, width: int, height: int) -> tuple[int, int]:
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive integers")
    if not math.isfinite(center_x_px) or not math.isfinite(center_y_px) or not 0 <= center_x_px < width or not 0 <= center_y_px < height:
        raise ValueError("checkerboard center must be finite and inside the source image")
    return min(2, int(center_y_px / (height / 3))), min(2, int(center_x_px / (width / 3)))


def overlay_state(*, corners: np.ndarray | None, width: int, height: int, target_row: int, target_column: int) -> CoverageOverlayState:
    validate_target(row=target_row, column=target_column)
    if corners is None:
        return CoverageOverlayState("CHECKERBOARD NOT DETECTED", "red", target_row, target_column, None, None, None)
    if not isinstance(corners, np.ndarray) or corners.shape != (CHECKERBOARD_CORNER_COUNT, 1, 2) or not np.isfinite(corners).all():
        raise ValueError("detected checkerboard corners must be finite 54x1x2")
    center = np.mean(corners.reshape(-1, 2), axis=0)
    row, column = coverage_bin(center_x_px=float(center[0]), center_y_px=float(center[1]), width=width, height=height)
    matched = (row, column) == (target_row, target_column)
    return CoverageOverlayState("54/54 DETECTED", "green" if matched else "yellow", target_row, target_column, row, column, (float(center[0]), float(center[1])))


def _target_name(row: int, column: int) -> str:
    return ("top", "middle", "bottom")[row] + "-" + ("left", "center", "right")[column]


def draw_preview_overlay(frame: np.ndarray, *, corners: np.ndarray | None, target_row: int, target_column: int, preview_width: int) -> tuple[np.ndarray, CoverageOverlayState]:
    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError("preview frame must be uint8 BGR")
    if type(preview_width) is not int or preview_width < 320:
        raise ValueError("preview_width must be an integer of at least 320")
    height, width = frame.shape[:2]
    state = overlay_state(corners=corners, width=width, height=height, target_row=target_row, target_column=target_column)
    view = frame.copy()
    color = {"green": (0, 220, 0), "yellow": (0, 220, 220), "red": (0, 0, 230)}[state.color]
    for column in (1, 2):
        x = int(round(width * column / 3))
        cv2.line(view, (x, 0), (x, height - 1), (255, 255, 255), 2)
    for row in (1, 2):
        y = int(round(height * row / 3))
        cv2.line(view, (0, y), (width - 1, y), (255, 255, 255), 2)
    if corners is not None:
        cv2.drawChessboardCorners(view, (9, 6), corners.astype(np.float32), True)
    target = f"target: {_target_name(target_row, target_column)} (row={target_row}, col={target_column})"
    cv2.putText(view, target, (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(view, target, (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
    if state.center_px is None:
        label = state.status
    else:
        label = f"{state.status} | center=({state.center_px[0]:.1f}, {state.center_px[1]:.1f}) px | row={state.current_row}, column={state.current_column}"
        center = (int(round(state.center_px[0])), int(round(state.center_px[1])))
        cv2.circle(view, center, 12, color, 3)
    cv2.putText(view, label, (24, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    scale = preview_width / width
    preview = cv2.resize(view, (preview_width, max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)
    return preview, state


def json_safe_state(state: CoverageOverlayState) -> dict[str, Any]:
    return json.loads(json.dumps(state.as_record(), allow_nan=False, sort_keys=True))
