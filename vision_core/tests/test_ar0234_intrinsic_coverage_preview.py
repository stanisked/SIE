from __future__ import annotations

import json

import numpy as np

from vision_core.ar0234_intrinsic_coverage_preview import (
    CHECKERBOARD_CORNER_COUNT,
    coverage_bin,
    json_safe_state,
    overlay_state,
)


def _corners(center_x: float, center_y: float) -> np.ndarray:
    return np.tile(np.asarray([[[center_x, center_y]]], dtype=np.float64), (CHECKERBOARD_CORNER_COUNT, 1, 1))


def test_coverage_bin_boundaries_are_deterministic():
    assert coverage_bin(center_x_px=0.0, center_y_px=0.0, width=1920, height=1200) == (0, 0)
    assert coverage_bin(center_x_px=639.999, center_y_px=399.999, width=1920, height=1200) == (0, 0)
    assert coverage_bin(center_x_px=640.0, center_y_px=400.0, width=1920, height=1200) == (1, 1)
    assert coverage_bin(center_x_px=1280.0, center_y_px=800.0, width=1920, height=1200) == (2, 2)
    assert coverage_bin(center_x_px=1919.999, center_y_px=1199.999, width=1920, height=1200) == (2, 2)


def test_overlay_target_states_are_explicit_and_json_safe():
    matched = overlay_state(corners=_corners(960.0, 200.0), width=1920, height=1200, target_row=0, target_column=1)
    mismatched = overlay_state(corners=_corners(100.0, 100.0), width=1920, height=1200, target_row=0, target_column=1)
    missing = overlay_state(corners=None, width=1920, height=1200, target_row=0, target_column=1)

    assert (matched.status, matched.color, matched.current_row, matched.current_column) == ("54/54 DETECTED", "green", 0, 1)
    assert (mismatched.status, mismatched.color, mismatched.current_row, mismatched.current_column) == ("54/54 DETECTED", "yellow", 0, 0)
    assert (missing.status, missing.color, missing.current_row, missing.current_column) == ("CHECKERBOARD NOT DETECTED", "red", None, None)
    json.dumps(json_safe_state(matched), allow_nan=False)
