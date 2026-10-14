"""Local shape-aware diagnostics on original masked stereo samples. No gates."""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


def dispersion(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    if not values.size:
        return {"sample_count": 0}
    median = float(np.median(values))
    return {"sample_count": int(values.size), "median_m": median,
            "mad_m": float(np.median(np.abs(values - median))),
            "absolute_p90_m": float(np.percentile(np.abs(values), 90)),
            "absolute_max_m": float(np.max(np.abs(values)))}


def analyze_local_surfaces(u: np.ndarray, v: np.ndarray, disparity_px: np.ndarray,
                           focal_baseline_m_px: float, *, stride: int = 4,
                           tile_px: int = 32, minimum_points: int = 24) -> tuple[dict, dict]:
    """Fit d=a*u+b*v+c locally, which describes a plane in rectified stereo.

    NumPy least-squares/SVD initializes a SciPy Huber fit. Alternate grid cells
    train and evaluate the model. No sample is trimmed for reported residuals;
    failed and sparse tiles remain explicit. Neighboring held-out samples may
    be correlated, so this is not independent physical accuracy validation.
    """
    u, v, d = (np.asarray(a, dtype=np.float64) for a in (u, v, disparity_px))
    if (u.ndim != 1 or v.shape != u.shape or d.shape != u.shape
            or not np.isfinite(np.r_[u, v, d]).all() or (d <= 0).any()
            or not np.isfinite(focal_baseline_m_px) or focal_baseline_m_px <= 0
            or stride < 1 or tile_px < stride or tile_px % stride or minimum_points < 12):
        raise ValueError("invalid original stereo sample grid or model parameters")
    if not np.allclose(u % stride, 0) or not np.allclose(v % stride, 0):
        raise ValueError("expected original global stride-aligned stereo grid")
    residual = np.full(len(d), np.nan)
    held_out = np.zeros(len(d), dtype=bool)
    patch_ids = np.full(len(d), -1, dtype=np.int32)
    patches = []
    keys = np.column_stack((u.astype(np.int64) // tile_px, v.astype(np.int64) // tile_px))
    cells = np.unique(keys, axis=0)
    raw_z = focal_baseline_m_px / d
    for patch_id, (tx, ty) in enumerate(cells):
        idx = np.flatnonzero((keys[:, 0] == tx) & (keys[:, 1] == ty))
        patch_ids[idx] = patch_id
        count = len(idx)
        patch = {"patch_id": patch_id, "grid_xy": [int(tx), int(ty)],
                 "bbox_stereo_xyxy_px": [int(tx * tile_px), int(ty * tile_px),
                                         int((tx + 1) * tile_px), int((ty + 1) * tile_px)],
                 "sample_count": count, "grid_capacity": (tile_px // stride) ** 2,
                 "raw_depth": dispersion(raw_z[idx]), "status": "TOO_FEW_POINTS"}
        patches.append(patch)
        if count < minimum_points:
            continue
        xy = np.column_stack((u[idx], v[idx]))
        center = xy.mean(axis=0)
        scale = xy.std(axis=0)
        if (scale < 1e-9).any():
            patch["status"] = "DEGENERATE_IMAGE_SUPPORT"
            continue
        a = np.column_stack(((xy - center) / scale, np.ones(count)))
        test = ((u[idx].astype(np.int64) // stride + v[idx].astype(np.int64) // stride) % 2) == 1
        train = ~test
        patch.update({"training_sample_count": int(train.sum()),
                      "held_out_sample_count": int(test.sum())})
        if train.sum() < 6 or test.sum() < 6:
            patch["status"] = "INSUFFICIENT_TRAIN_TEST_SUPPORT"
            continue
        initial, _, rank, singular = np.linalg.lstsq(a[train], d[idx][train], rcond=None)
        if rank != 3:
            patch["status"] = "DEGENERATE_TRAINING_SUPPORT"
            continue
        training_residual = a[train] @ initial - d[idx][train]
        training_mad = np.median(np.abs(training_residual - np.median(training_residual)))
        # An algorithm scale, not a quality/accuracy threshold. Floor reflects
        # the stored SGBM 1/16-pixel disparity step; no per-frame tuning or cap.
        huber_scale = max(1 / 16, 1.345 * 1.4826 * float(training_mad))
        fit = least_squares(lambda p: a[train] @ p - d[idx][train], initial,
                            jac=lambda p: a[train], loss="huber", f_scale=huber_scale,
                            max_nfev=100, method="trf")
        prediction = a @ fit.x
        if not fit.success or not np.isfinite(prediction).all() or (prediction <= 0).any():
            patch["status"] = "FIT_FAILED_OR_NONPOSITIVE_PREDICTION"
            continue
        e = raw_z[idx] - focal_baseline_m_px / prediction
        residual[idx] = e
        held_out[idx] = test
        coeff_u, coeff_v = fit.x[:2] / scale
        intercept = fit.x[2] - coeff_u * center[0] - coeff_v * center[1]
        patch.update({"status": "FIT_DIAGNOSTIC_ONLY",
                      "disparity_plane_abc": [float(coeff_u), float(coeff_v), float(intercept)],
                      "huber_scale_px": huber_scale,
                      "scaled_training_condition": float(singular[0] / singular[-1]),
                      "all_point_axial_residual": dispersion(e),
                      "held_out_axial_residual": dispersion(e[test])})
    fitted = [p for p in patches if p["status"] == "FIT_DIAGNOSTIC_ONLY"]
    tested = held_out & np.isfinite(residual)
    stats = {"status": "LOCAL_SURFACE_DIAGNOSTIC_ONLY", "sample_count": len(d),
             "tile_size_px": tile_px, "minimum_points_per_tile": minimum_points,
             "patch_count": len(patches), "fitted_patch_count": len(fitted),
             "fitted_sample_fraction": float(np.isfinite(residual).mean()) if len(d) else 0.0,
             "raw_depth": dispersion(raw_z),
             "pooled_held_out_axial_residual": dispersion(residual[tested]),
             "patch_status_counts": {s: sum(p["status"] == s for p in patches)
                                      for s in sorted({p["status"] for p in patches})},
             "patches": patches}
    return stats, {"axial_residual_m": residual, "held_out": held_out, "patch_id": patch_ids}
