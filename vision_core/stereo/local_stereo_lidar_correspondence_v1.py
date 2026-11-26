"""Local image/depth connectivity on rectified stereo samples, diagnostic only."""
from collections import Counter, deque
from dataclasses import dataclass
import math

import numpy as np

from sie_core.geometry.plane import PlaneFitter
from vision_core.stereo.stereo_lidar_static_v1 import residual_stats

CANDIDATE = "COHERENT_LOCAL_STEREO_CANDIDATE"
AMBIGUOUS = "AMBIGUOUS_LOCAL_STEREO_SURFACES"
NEIGHBORS = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]


@dataclass(frozen=True)
class LocalTrialPolicy:
    neighborhood_radius_px: int
    anchor_radius_px: int
    minimum_component_points: int
    minimum_component_coverage: float
    minimum_competing_component_points: int
    neighbor_depth_step_floor_m: float
    neighbor_depth_step_fraction: float
    maximum_optical_depth_mad_m: float
    maximum_optical_depth_p05_p95_span_m: float


def stereo_grid(disparity_px, lr_consistent, q_mm, r1, region):
    """Full-grid physical-left XYZ in metres. Unsupported pixels remain NaN."""
    if (disparity_px.shape != region.shape or lr_consistent.shape != region.shape
            or lr_consistent.dtype != np.bool_ or region.dtype != np.bool_):
        raise ValueError("disparity and boolean LR/region masks must share the rectified grid")
    lr_support = region & lr_consistent
    y, x = np.nonzero(lr_support & np.isfinite(disparity_px) & (disparity_px > .5))
    samples = np.column_stack((x, y, disparity_px[y, x], np.ones(x.size)))
    homogeneous = samples @ q_mm.T
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        rectified = homogeneous[:, :3] / homogeneous[:, 3:] / 1000
        physical = rectified @ r1  # Row-vector form of R1.T @ p_rectified.
    usable = (np.isfinite(physical).all(axis=1) & (rectified[:, 2] > 0) & (physical[:, 2] > 0))
    xyz = np.full((*region.shape, 3), np.nan)
    xyz[y[usable], x[usable]] = physical[usable]
    return xyz, lr_support


def depth_stats(samples):
    p05, p25, median, p75, p95 = np.percentile(samples, (5, 25, 50, 75, 95))
    return {
        "p05_m": float(p05), "p25_m": float(p25), "median_m": float(median),
        "p75_m": float(p75), "p95_m": float(p95),
        "mad_m": float(np.median(np.abs(samples - median))),
    }


def _depth_components(depth, policy):
    """8-connected occupancy with an extra stereo-only optical-Z edge check."""
    height, width = depth.shape
    valid = np.isfinite(depth)
    visited = np.zeros_like(valid)
    components = []
    discontinuities = 0
    for seed_y, seed_x in np.argwhere(valid):
        if visited[seed_y, seed_x]:
            continue
        pending = deque([(int(seed_y), int(seed_x))])
        visited[seed_y, seed_x] = True
        component = []
        while pending:
            y, x = pending.popleft()
            component.append((y, x))
            z = depth[y, x]
            for dy, dx in NEIGHBORS:
                ny, nx = y + dy, x + dx
                if not (0 <= ny < height and 0 <= nx < width and valid[ny, nx]):
                    continue
                neighbor_z = depth[ny, nx]
                max_step = max(policy.neighbor_depth_step_floor_m,
                               policy.neighbor_depth_step_fraction * min(z, neighbor_z))
                if abs(z - neighbor_z) > max_step:
                    discontinuities += (ny, nx) > (y, x)
                elif not visited[ny, nx]:
                    visited[ny, nx] = True
                    pending.append((ny, nx))
        components.append(np.asarray(component))
    return components, discontinuities


def local_stereo_support(uv_px, xyz, lr_support, region, forward_axis, rim_forward_m, policy):
    """Select a local component using the pixel, stereo XYZ and mask alone."""
    u, v = (math.floor(value + .5) for value in uv_px)
    height, width = region.shape
    radius = policy.neighborhood_radius_px
    x0, x1 = max(0, u - radius), min(width, u + radius + 1)
    y0, y1 = max(0, v - radius), min(height, v + radius + 1)
    patch = xyz[y0:y1, x0:x1]
    depth = patch[:, :, 2]
    roi_pixels = int(region[y0:y1, x0:x1].sum())
    raw_lr = int(lr_support[y0:y1, x0:x1].sum())
    usable = int(np.isfinite(depth).sum())
    summary = {
        "bounds_xyxy_exclusive_px": [x0, y0, x1, y1], "roi_pixel_count": roi_pixels,
        "lr_supported_count": raw_lr, "usable_xyz_count": usable,
        "lr_pixels_without_valid_xyz": raw_lr - usable, "usable_coverage": usable / roi_pixels,
        "components": [], "depth_discontinuity_edges": 0, "support_sufficient": False,
        "coherent": None,
    }
    if not usable:
        status = "NO_VALID_STEREO_XYZ" if raw_lr else "NO_LOCAL_LR_SUPPORT"
        return summary | {"status": status}
    components, discontinuities = _depth_components(depth, policy)
    summary["depth_discontinuity_edges"] = discontinuities
    anchored = []
    for index, pixels in enumerate(components):
        yy, xx = pixels.T
        distances = np.hypot(xx + x0 - uv_px[0], yy + y0 - uv_px[1])
        closest = float(distances.min())
        coverage = len(pixels) / roi_pixels
        plausible = (closest <= policy.anchor_radius_px
                     and len(pixels) >= policy.minimum_competing_component_points)
        sufficient = (plausible and len(pixels) >= policy.minimum_component_points
                      and coverage >= policy.minimum_component_coverage)
        summary["components"].append({
            "index": index, "points": len(pixels), "closest_pixel_distance_px": closest,
            "coverage": coverage, "plausible_near_projection": plausible, "support_sufficient": sufficient,
            "median_optical_z_m": float(np.median(depth[yy, xx])),
        })
        summary["support_sufficient"] |= sufficient
        if plausible:
            anchored.append(index)
    if len(anchored) > 1:
        return summary | {"status": AMBIGUOUS, "plausible_component_indices": anchored}
    if not anchored:
        return summary | {"status": "NO_SUPPORTED_COMPONENT_NEAR_PROJECTION"}
    index, = anchored
    yy, xx = components[index].T
    points = patch[yy, xx]
    optical = depth_stats(points[:, 2])
    longitudinal = depth_stats(points @ forward_axis - rim_forward_m)
    coverage = len(points) / roi_pixels
    summary.update({
        "selected_component_index": index, "selected_count": len(points), "selected_coverage": coverage,
        "selected_pixels_xy_px": np.column_stack((xx + x0, yy + y0)).tolist(),
        "optical_z": optical, "longitudinal_from_rim": longitudinal,
    })
    if not summary["support_sufficient"]:
        return summary | {"status": "INSUFFICIENT_LOCAL_COMPONENT_SUPPORT"}
    if (optical["mad_m"] > policy.maximum_optical_depth_mad_m
            or optical["p95_m"] - optical["p05_m"] > policy.maximum_optical_depth_p05_p95_span_m):
        return summary | {"status": "INCOHERENT_LOCAL_STEREO_DEPTH", "coherent": False}

    # A plane is descriptive geometry only. The candidate gate ends above.
    plane_info = None
    try:
        if np.linalg.matrix_rank(points - points.mean(axis=0)) >= 2:
            plane = PlaneFitter().fit_least_squares(points)
            normal = np.asarray(plane["normal"])
            if normal[2] < 0:
                normal = -normal
            centroid = np.asarray(plane["centroid"])
            plane_info = {
                "centroid_physical_left_optical_m": centroid.tolist(), "normal": normal.tolist(),
                "stereo_point_to_plane": residual_stats((points - centroid) @ normal),
            }
    except np.linalg.LinAlgError:
        plane_info = {"reason": "OPTIONAL_LOCAL_PLANE_SVD_FAILED"}
    return summary | {"status": CANDIDATE, "coherent": True, "optional_local_plane": plane_info}


def compare_local_scan(scan, grid, region, r1, p1, rotation, translation, axial_m, sector_deg, policy):
    """Report every paired beam, including projections with no coherent support."""
    forward_axis = rotation[:, 0]  # Robot forward expressed along optical axes.
    rim_forward_m = axial_m * forward_axis[2]
    height, width = region.shape
    lo, hi = sector_deg
    rows = []
    for index, distance in enumerate(scan["ranges_m"]):
        angle = scan["angle_min_rad"] + index * scan["angle_increment_rad"]
        angle_deg = (math.degrees(angle) + 180) % 360 - 180
        row = {"beam_index": index, "angle_rad": angle, "angle_deg": angle_deg, "range_m": distance,
               "projectable": False, "inside_image": False, "inside_target_polygon": False,
               "physical_target_membership": "UNVERIFIED"}
        rows.append(row)
        if not lo <= angle_deg <= hi:
            row["status"] = "OUTSIDE_DECLARED_SECTOR"
            continue
        if type(distance) not in (int, float) or not math.isfinite(distance) or not (
            0 < distance and scan["range_min_m"] <= distance <= scan["range_max_m"]
        ):
            row["status"] = "INVALID_OR_MISSING_LD19_RANGE"
            continue
        hit = translation + distance * (rotation @ [math.cos(angle), math.sin(angle), 0.0])
        rectified = r1 @ hit
        row.update(hit_estimated_optical_m=hit.tolist(), hit_rectified_left_m=rectified.tolist())
        projected = p1 @ np.r_[rectified, 1]
        if hit[2] <= 0 or rectified[2] <= 0 or projected[2] <= 0:
            row["status"] = "NONPOSITIVE_OPTICAL_DEPTH"
            continue
        uv = projected[:2] / projected[2]
        if not np.isfinite(uv).all():
            row["status"] = "INVALID_PIXEL_PROJECTION"
            continue
        row.update(projectable=True, projected_rectified_left_px=uv.tolist())
        if not (-.5 <= uv[0] < width - .5 and -.5 <= uv[1] < height - .5):
            row["status"] = "PROJECTED_OUTSIDE_IMAGE"
            continue
        row["inside_image"] = True
        u, v = np.floor(uv + .5).astype(int)
        if not region[v, u]:
            row["status"] = "PROJECTED_OUTSIDE_TARGET_POLYGON"
            continue
        row["inside_target_polygon"] = True
        if grid is None:
            row["status"] = "STEREO_FRAME_UNAVAILABLE"
            continue
        xyz, lr_support = grid
        local = local_stereo_support(uv, xyz, lr_support, region, forward_axis, rim_forward_m, policy)
        row.update(status=local["status"], local_stereo=local)
        if local["status"] == CANDIDATE:
            longitudinal = float(hit @ forward_axis - rim_forward_m)
            row.update({
                "hit_longitudinal_from_rim_m": longitudinal,
                "ld19_minus_stereo_optical_z_m": float(hit[2] - local["optical_z"]["median_m"]),
                "ld19_minus_stereo_longitudinal_m": longitudinal - local["longitudinal_from_rim"]["median_m"],
                "cross_sensor_agreement": "RESIDUAL_REPORTED_NO_ACCEPTANCE_LIMIT",
            })
    counts = Counter(row["status"] for row in rows)
    return rows, {
        "beam_status_counts": dict(counts), "total_beams": len(rows),
        "projectable_beams": sum(row["projectable"] for row in rows),
        "inside_image_beams": sum(row["inside_image"] for row in rows),
        "inside_target_polygon_beams": sum(row["inside_target_polygon"] for row in rows),
        "beams_with_usable_local_stereo": sum((row.get("local_stereo") or {}).get("usable_xyz_count", 0) > 0
                                             for row in rows),
        "beams_with_sufficient_local_stereo_support": sum((row.get("local_stereo") or {}).get("support_sufficient", False)
                                                         for row in rows),
        "coherent_local_surface_candidates": counts[CANDIDATE], "ambiguous_beams": counts[AMBIGUOUS],
    }
