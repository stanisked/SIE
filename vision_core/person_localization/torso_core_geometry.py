"""Current RGB-instance torso-core selection for motor-off diagnostics only."""
from __future__ import annotations

import math

import numpy as np


# The fractions select the middle half of the detected person width and the
# upper-middle part of its height. This is a geometric body-core heuristic,
# not a keypoint/body-part classifier or an admitted measurement policy.
TORSO_CORE_POLICY = {
    "bbox_x_fraction": [0.25, 0.75],
    "bbox_y_fraction": [0.28, 0.50],
    "minimum_stable_points": 100,
    "range_statistic": "median",
    "status": "MOTOR_OFF_DIAGNOSTIC_ONLY",
}


def torso_core_roi(bbox_xyxy_px, *, image_size=(1920, 1200)):
    """Return the fixed-fraction torso-core ROI for one current RGB box."""
    if (not isinstance(bbox_xyxy_px, (list, tuple)) or
            len(bbox_xyxy_px) != 4 or
            not all(type(v) in (int, float) and math.isfinite(v)
                    for v in bbox_xyxy_px)):
        raise ValueError("invalid current RGB instance bbox")
    x1, y1, x2, y2 = map(float, bbox_xyxy_px)
    width, height = image_size
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError("current RGB instance bbox is outside the image")
    bw, bh = x2 - x1, y2 - y1
    fx = TORSO_CORE_POLICY["bbox_x_fraction"]
    fy = TORSO_CORE_POLICY["bbox_y_fraction"]
    return [x1 + fx[0] * bw, y1 + fy[0] * bh,
            x1 + fx[1] * bw, y1 + fy[1] * bh]


def projected_in_torso_core(projected_xy_px, bbox_xyxy_px):
    """Select finite projected points inside the current torso-core ROI."""
    xy = np.asarray(projected_xy_px, dtype=np.float64)
    if xy.ndim != 2 or xy.shape[1] != 2:
        raise ValueError("projected RGB points must have shape Nx2")
    x1, y1, x2, y2 = torso_core_roi(bbox_xyxy_px)
    selected = (np.isfinite(xy).all(axis=1) &
                (xy[:, 0] >= x1) & (xy[:, 0] <= x2) &
                (xy[:, 1] >= y1) & (xy[:, 1] <= y2))
    return selected, [x1, y1, x2, y2]
