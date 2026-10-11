"""Pure offline RGB instance-mask to stereo-point association diagnostic.

The caller supplies a mask for one detected instance and calibrated projections
of current-frame LR-consistent stereo points. No box fallback, camera access,
ROS publication, calibration changes, or actuator output is provided here.
"""

from __future__ import annotations

import math

import numpy as np


def masked_depth_hypothesis(
    projected_ar_xy_px: np.ndarray,
    depth_m: np.ndarray,
    instance_mask: np.ndarray,
    *,
    minimum_samples: int = 100,
    depth_gap_m: float = 0.15,
    relative_depth_gap: float = 0.04,
    minimum_layer_support_fraction: float = 0.25,
    maximum_depth_mad_m: float = 0.05,
    mask_interior_margin_px: int = 2,
) -> dict:
    """Evaluate visible mask-interior points without requiring 3D connectivity.

    A box is not an instance mask. The caller must bind the mask to one RGB
    observation in the same camera frame and verify its calibration and timing.
    This function cannot establish that a depth layer is the physical target.
    """
    xy = np.asarray(projected_ar_xy_px, dtype=np.float64)
    depth = np.asarray(depth_m, dtype=np.float64)
    mask = np.asarray(instance_mask)
    if xy.ndim != 2 or xy.shape[1] != 2 or depth.shape != (len(xy),):
        raise ValueError("projected points/depth shape mismatch")
    if mask.ndim != 2 or mask.dtype != np.bool_:
        raise ValueError("an explicit boolean instance mask is required")
    if (minimum_samples < 1 or depth_gap_m <= 0 or relative_depth_gap <= 0
            or not 0 < minimum_layer_support_fraction <= 1
            or maximum_depth_mad_m <= 0 or mask_interior_margin_px < 0):
        raise ValueError("invalid diagnostic parameters")
    h, w = mask.shape
    # A conservative square erosion rejects pixels near an uncertain silhouette.
    # Cropping prevents wraparound or implicit padding of the image border.
    interior = mask.copy()
    m = mask_interior_margin_px
    if m:
        interior[:] = False
        if h > 2 * m and w > 2 * m:
            core = mask[m:h - m, m:w - m].copy()
            for dy in range(-m, m + 1):
                for dx in range(-m, m + 1):
                    core &= mask[m + dy:h - m + dy, m + dx:w - m + dx]
            interior[m:h - m, m:w - m] = core
    finite = np.isfinite(xy).all(axis=1) & np.isfinite(depth) & (depth > 0)
    safe_xy = np.where(np.isfinite(xy), xy, -1)
    px = np.rint(safe_xy[:, 0]).astype(np.int64)
    py = np.rint(safe_xy[:, 1]).astype(np.int64)
    on_image = finite & (px >= 0) & (px < w) & (py >= 0) & (py < h)
    selected = np.zeros(len(depth), dtype=bool)
    selected[on_image] = interior[py[on_image], px[on_image]]
    sample = depth[selected]
    report = {
        "status": "INSUFFICIENT_MASKED_DEPTH",
        "selected_sample_count": int(sample.size),
        "mask_pixel_count": int(mask.sum()),
        "interior_pixel_count": int(interior.sum()),
        "depth_layers": [],
        "diagnostic_only": True,
        "semantic_ownership_verified": False,
    }
    if sample.size < minimum_samples:
        return report
    order = np.argsort(sample)
    ordered = sample[order]
    gap = max(depth_gap_m, relative_depth_gap * float(np.median(ordered)))
    groups = np.split(order, np.flatnonzero(np.diff(ordered) > gap) + 1)
    layers = []
    for indices in groups:
        if len(indices) < minimum_samples:
            continue
        values = sample[indices]
        median = float(np.median(values))
        layers.append({"sample_count": int(len(indices)), "median_depth_m": median,
                       "mad_m": float(np.median(np.abs(values - median)))})
    report["depth_layers"] = layers
    if not layers:
        report["status"] = "NO_SUPPORTED_DEPTH_LAYER"
        return report
    support_limit = max(minimum_samples, math.ceil(
        max(layer["sample_count"] for layer in layers) * minimum_layer_support_fraction))
    plausible = [layer for layer in layers if layer["sample_count"] >= support_limit]
    report["plausible_layer_count"] = len(plausible)
    if len(plausible) != 1:
        report["status"] = "MULTIPLE_OR_NO_PLAUSIBLE_DEPTH_LAYERS"
        return report
    report["candidate_depth_m"] = plausible[0]["median_depth_m"]
    report["candidate_mad_m"] = plausible[0]["mad_m"]
    report["candidate_sample_count"] = plausible[0]["sample_count"]
    if plausible[0]["mad_m"] > maximum_depth_mad_m:
        report["status"] = "DEPTH_MAD_EXCEEDED"
        return report
    report["status"] = "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"
    return report
