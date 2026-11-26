"""Pure Step 66 stereo-plane and finite-region comparison, without sensor I/O."""

from collections import Counter
from dataclasses import dataclass
import math

import cv2
import numpy as np

from sie_core.geometry.plane import PlaneFitter

MAX_STEREO_PLANE_P95_M = 0.040
MIN_STEREO_SUPPORT = 48
MIN_STEREO_COVERAGE = 0.15


def residual_stats(values):
    if not len(values):
        return {"count": 0, "median_mm": None, "mad_mm": None, "p95_absolute_mm": None}
    mm = np.asarray(values) * 1000
    median = float(np.median(mm))
    return {"count": len(mm), "median_mm": median,
            "mad_mm": float(np.median(np.abs(mm - median))),
            "p95_absolute_mm": float(np.percentile(np.abs(mm), 95))}


def target_mask(shape, vertices):
    """Rasterize a fixed, independently marked convex quadrilateral."""
    vertices = np.asarray(vertices)
    if (vertices.shape != (4, 2) or vertices.dtype.kind not in "iu"
            or (vertices < 0).any() or (vertices >= shape[::-1]).any()):
        raise ValueError("target needs four in-image integer vertices in perimeter order")
    contour = vertices.astype(np.int32)
    if not cv2.isContourConvex(contour) or cv2.contourArea(contour) < 100:
        raise ValueError("target must be a non-degenerate convex quadrilateral")
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, contour, 1)
    return mask.astype(bool)


@dataclass
class StereoSurface:
    region: np.ndarray
    supported_pixels: np.ndarray
    centroid_m: np.ndarray
    normal: np.ndarray
    support_count: int
    fit_stats: dict


def fit_stereo_surface(disparity_px, lr_consistent, q_mm, r1, region):
    """Use LR-supported pixels inside the operator region, without LD19 input."""
    if disparity_px.shape != region.shape or lr_consistent.shape != region.shape:
        raise ValueError("disparity, LR mask and target region must share the full rectified grid")
    interior = cv2.erode(region.astype(np.uint8), np.ones((5, 5), np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    valid = interior & lr_consistent & np.isfinite(disparity_px) & (disparity_px > .5)
    y, x = np.nonzero(valid)
    uv_d = np.column_stack((x, y, disparity_px[y, x], np.ones(x.size)))
    homogeneous = uv_d @ q_mm.T
    with np.errstate(divide="ignore", invalid="ignore"):
        rectified_m = homogeneous[:, :3] / homogeneous[:, 3:] / 1000
    usable = np.isfinite(rectified_m).all(axis=1) & (rectified_m[:, 2] > 0)
    support_count = int(usable.sum())
    if support_count < max(MIN_STEREO_SUPPORT, math.ceil(MIN_STEREO_COVERAGE * interior.sum())):
        raise ValueError("INSUFFICIENT_STEREO_TARGET_SUPPORT")
    # Row-vector form of p_physical = R1.T @ p_rectified.
    points = rectified_m[usable] @ r1
    sampled = points[np.linspace(0, len(points) - 1, min(len(points), 512), dtype=int)]
    if np.linalg.matrix_rank(sampled - sampled.mean(axis=0)) < 2:
        raise ValueError("DEGENERATE_STEREO_TARGET_SUPPORT")
    plane = PlaneFitter().fit_least_squares(sampled)
    normal = np.array(plane["normal"])
    if abs(normal[2]) < 0.25:
        raise ValueError("STEREO_PLANE_TOO_GRAZING")
    if normal[2] < 0:
        normal = -normal
    centroid = np.array(plane["centroid"])
    stats = residual_stats((points - centroid) @ normal)
    if stats["p95_absolute_mm"] > MAX_STEREO_PLANE_P95_M * 1000:
        raise ValueError("STEREO_TARGET_NOT_PLANAR_UNDER_TRIAL_LIMIT")
    support = np.zeros(region.shape, np.uint8)
    support[y[usable], x[usable]] = 1
    nearby_support = cv2.dilate(support, np.ones((5, 5), np.uint8)).astype(bool)
    return StereoSurface(region, nearby_support & region, centroid, normal, support_count, stats)


def _projection(point, r1, p1, surface):
    rectified = r1 @ point
    image = p1 @ np.r_[rectified, 1]
    if not np.isfinite(image).all() or rectified[2] <= 0 or image[2] <= 0:
        return None, False, False
    uv = image[:2] / image[2]
    h, w = surface.region.shape
    if not (-0.5 <= uv[0] < w - 0.5 and -0.5 <= uv[1] < h - 0.5):
        return uv.tolist(), False, False
    u, v = np.rint(uv).astype(int)
    return uv.tolist(), bool(surface.region[v, u]), bool(surface.supported_pixels[v, u])


def compare_scan(scan, surface, r1, p1, ld_rotation, ld_origin_m, sector_deg):
    """Report all ray decisions. Neither sector nor membership depends on residual size."""
    lo, hi = sector_deg
    if not (math.isfinite(lo) and math.isfinite(hi) and -180 <= lo < hi <= 180):
        raise ValueError("sector must have increasing endpoints in [-180, 180] degrees")
    rows, infinite, finite = [], [], []
    for index, distance in enumerate(scan["ranges_m"]):
        angle = scan["angle_min_rad"] + index * scan["angle_increment_rad"]
        angle_deg = (math.degrees(angle) + 180) % 360 - 180
        row = {"beam_index": index, "angle_deg": angle_deg, "range_m": distance}
        rows.append(row)
        if not lo <= angle_deg <= hi:
            row["status"] = "OUTSIDE_DECLARED_SECTOR"
            continue
        ray = ld_rotation @ np.array([math.cos(angle), math.sin(angle), 0.0])
        denom = float(surface.normal @ ray)
        along = float(surface.normal @ (surface.centroid_m - ld_origin_m)) / denom if abs(denom) > 1e-9 else -1
        plane_uv, plane_inside, expected = (
            _projection(ld_origin_m + along * ray, r1, p1, surface)
            if along > 0 else (None, False, False)
        )
        row.update(ray_plane_uv_px=plane_uv, ray_plane_inside_polygon=plane_inside,
                   ray_plane_stereo_supported=expected)
        if not isinstance(distance, (float, int)) or not math.isfinite(distance) or not (
            0 < distance and scan["range_min_m"] <= distance <= scan["range_max_m"]
        ):
            row["status"] = ("MISSING_RETURN_IN_STEREO_SUPPORTED_REGION" if expected
                             else "INVALID_OR_MISSING_RANGE")
            continue
        hit = ld_origin_m + distance * ray
        signed = float((hit - surface.centroid_m) @ surface.normal)
        hit_uv, hit_inside, hit_support = _projection(hit, r1, p1, surface)
        row.update(hit_estimated_optical_m=hit.tolist(), hit_uv_px=hit_uv,
                   hit_inside_polygon=hit_inside, hit_stereo_supported=hit_support,
                   signed_residual_mm=signed * 1000)
        infinite.append(signed)
        if expected and hit_support:
            row["status"] = "FINITE_SURFACE_CANDIDATE"
            finite.append(signed)
        elif plane_inside or hit_inside:
            row["status"] = "AMBIGUOUS_FINITE_CORRESPONDENCE"
        else:
            row["status"] = "OUTSIDE_OBSERVED_TARGET"
    return {
        "stereo_support_count": surface.support_count,
        "stereo_plane_fit": surface.fit_stats,
        "stereo_plane_centroid_physical_left_optical_m": surface.centroid_m.tolist(),
        "stereo_plane_normal_toward_positive_optical_z": surface.normal.tolist(),
        "infinite_plane_in_declared_sector": residual_stats(infinite),
        "finite_surface_candidates": residual_stats(finite),
        "beam_status_counts": dict(Counter(row["status"] for row in rows)),
        "beams": rows,
    }
