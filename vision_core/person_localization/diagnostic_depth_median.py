"""Non-emitting median-depth experiment; never fills invalid or unowned pixels."""

from __future__ import annotations

import numpy as np


def masked_median3x3(depth_m: np.ndarray, eligible: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    """Replace a center only when all nine original pixels are valid and eligible.

    Eligibility must be established by the caller from original LR-consistent
    points, their original RGB projections and the eroded instance mask. This
    is a spatial filter on full-resolution depth, not on the stride-4 grid.
    No padding, hole filling, temporal averaging or iterative filtering occurs.
    Internal depth edges and thin surfaces can still be smoothed.
    """
    depth = np.asarray(depth_m)
    support = np.asarray(eligible)
    if depth.ndim != 2 or support.shape != depth.shape or support.dtype != np.bool_:
        raise ValueError("depth and explicit boolean eligibility must be matching 2D maps")
    if not np.issubdtype(depth.dtype, np.floating):
        raise ValueError("depth must be a floating-point map in meters")
    valid = support & np.isfinite(depth) & (depth > 0)
    filtered = depth.copy()
    changed = np.zeros(depth.shape, dtype=bool)
    h, w = depth.shape
    evaluated = 0
    if h >= 3 and w >= 3:
        complete = np.ones((h - 2, w - 2), dtype=bool)
        for dy in range(3):
            for dx in range(3):
                complete &= valid[dy:dy + h - 2, dx:dx + w - 2]
        y, x = np.nonzero(complete)
        y, x = y + 1, x + 1
        evaluated = int(y.size)
        if evaluated:
            neighborhood = np.stack([depth[y + dy, x + dx]
                                     for dy in (-1, 0, 1) for dx in (-1, 0, 1)])
            median = np.median(neighborhood, axis=0)
            changed[y, x] = median != depth[y, x]
            filtered[y, x] = median
    delta = np.abs(filtered[changed] - depth[changed])
    return filtered, changed, {
        "window_px": [3, 3], "required_valid_owned_pixels": 9,
        "eligible_pixel_count": int(valid.sum()),
        "evaluated_center_count": evaluated,
        "skipped_eligible_center_count": int(valid.sum()) - evaluated,
        "changed_pixel_count": int(changed.sum()),
        "absolute_change_median_m": float(np.median(delta)) if delta.size else 0.0,
        "absolute_change_p95_m": float(np.percentile(delta, 95)) if delta.size else 0.0,
        "absolute_change_max_m": float(delta.max()) if delta.size else 0.0,
        "filled_invalid_pixel_count": 0, "removed_pixel_count": 0,
    }
