#!/usr/bin/env python3
"""Show original near/far masked stereo points on their source RGB frame.

Diagnostic only: reproduce the baseline and candidate layer before rendering.
No capture, filtering, depth completion, ROS publication or motion.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from replay_shadow_mask_depth_range import replay
from compare_shadow_mask_depth_median import project, owned, verify_baseline


def residual_colors(residual_m: np.ndarray, scale_m: float = .2) -> np.ndarray:
    """BGR: blue closer, white median, red farther; shared saturated scale."""
    if not np.isfinite(scale_m) or scale_m <= 0:
        raise ValueError("color scale must be positive and finite")
    t = np.clip(np.asarray(residual_m, dtype=np.float64) / scale_m, -1, 1)
    colors = np.empty((len(t), 3), dtype=np.uint8)
    colors[:, 0] = np.rint(255 * (1 - np.maximum(t, 0))).astype(np.uint8)
    colors[:, 1] = np.rint(255 * (1 - np.abs(t))).astype(np.uint8)
    colors[:, 2] = np.rint(255 * (1 + np.minimum(t, 0))).astype(np.uint8)
    return colors


def candidate_indices(depth: np.ndarray, hypothesis: dict) -> np.ndarray:
    """Recover exactly the layer used by the unchanged diagnostic MAD rule."""
    depth = np.asarray(depth, dtype=np.float64)
    order = np.argsort(depth)
    gap = max(.15, .04 * float(np.median(depth)))
    groups = np.split(order, np.flatnonzero(np.diff(depth[order]) > gap) + 1)
    supported = [g for g in groups if len(g) >= 100]
    limit = max(100, int(np.ceil(max(map(len, supported), default=0) * .25)))
    plausible = [g for g in supported if len(g) >= limit]
    if len(plausible) != 1:
        raise ValueError("no unique candidate layer to visualize")
    indices = plausible[0]
    values = depth[indices]
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if (len(values) != hypothesis.get("candidate_sample_count")
            or median != hypothesis.get("candidate_depth_m")
            or mad != hypothesis.get("candidate_mad_m")):
        raise ValueError("candidate layer does not reproduce the baseline")
    return indices


def extract_points(path: Path, cal: dict, lower: float, upper: float,
                   raw: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    audit = json.loads(path.read_text(encoding="utf-8"))
    g = audit["stereo_geometry"]
    disparity = np.load(path.parent / "disparity_lr_consistent_px.npy", allow_pickle=False)
    stride = int(g["sample_stride_px"])
    v, u = np.mgrid[0:disparity.shape[0]:stride, 0:disparity.shape[1]:stride]
    d = disparity[::stride, ::stride]
    valid = np.isfinite(d) & (d > .5)
    u, v, d = u[valid], v[valid], d[valid]
    z = abs(float(cal["P2"][0, 3])) / d / 1000.0
    valid_z = (z >= lower) & (z <= upper)
    u, v, z = u[valid_z], v[valid_z], z[valid_z]
    xy = project(u.astype(np.float64), v.astype(np.float64), z, cal)
    mask = cv2.imread(str(path.parent / f"{audit['cycle_id']}.png"), cv2.IMREAD_GRAYSCALE)
    interior = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
    selected = owned(xy, interior, g["ar_inner_roi_xyxy_px"])
    xy, z = xy[selected], z[selected].astype(np.float64)
    if len(z) != raw["mask_depth"]["selected_sample_count"]:
        raise ValueError("reconstructed mask support differs from baseline")
    indices = candidate_indices(z, raw["mask_depth"])
    return xy, z, indices, mask


def band_statistics(xy: np.ndarray, z: np.ndarray, roi: list, median: float) -> list[dict]:
    """Image-height thirds are spatial regions, not inferred body-part labels."""
    results = []
    edges = np.linspace(float(roi[1]), float(roi[3]), 4)
    for i, label in enumerate(("upper", "middle", "lower")):
        selected = ((xy[:, 1] >= edges[i]) &
                    ((xy[:, 1] <= edges[i + 1]) if i == 2 else (xy[:, 1] < edges[i + 1])))
        values = z[selected]
        row = {"image_band": label, "ar_y_bounds_px": edges[i:i + 2].tolist(),
               "sample_count": int(values.size)}
        if values.size:
            local = float(np.median(values))
            row.update({"median_depth_m": local,
                        "local_mad_m": float(np.median(np.abs(values - local))),
                        "median_minus_global_m": local - median,
                        "nearer_than_global_by_50mm_fraction": float(np.mean(values < median - .05)),
                        "farther_than_global_by_50mm_fraction": float(np.mean(values > median + .05))})
        results.append(row)
    return results


def fitted(image: np.ndarray, width: int = 520, height: int = 650) -> np.ndarray:
    h, w = image.shape[:2]
    scale = min(width / w, height / h)
    resized = cv2.resize(image, (max(1, round(w * scale)), max(1, round(h * scale))))
    canvas = np.full((height, width, 3), 25, dtype=np.uint8)
    rh, rw = resized.shape[:2]
    y, x = (height - rh) // 2, (width - rw) // 2
    canvas[y:y + rh, x:x + rw] = resized
    return canvas


def render(path: Path, raw: dict, cal: dict, lower: float, upper: float) -> tuple[np.ndarray, dict]:
    xy, z, indices, mask = extract_points(path, cal, lower, upper, raw)
    rgb = cv2.imread(str(path.parent / "ar0234_source.jpg"))
    if rgb is None or rgb.shape[:2] != mask.shape:
        raise ValueError("source RGB image missing or shape mismatch")
    audit = json.loads(path.read_text(encoding="utf-8"))
    roi = audit["stereo_geometry"]["ar_inner_roi_xyxy_px"]
    candidate_xy, candidate_z = xy[indices], z[indices]
    h = raw["mask_depth"]
    median = h["candidate_depth_m"]
    residual = candidate_z - median
    overlay = rgb.copy()
    candidate = np.zeros(len(z), dtype=bool)
    candidate[indices] = True
    for x, y in np.rint(xy[~candidate]).astype(int):
        cv2.circle(overlay, (int(x), int(y)), 2, (100, 100, 100), -1)
    colors = residual_colors(residual)
    # Draw small residuals first, large residuals last so overlapping samples
    # are visible. These are projected stereo samples, not a completed image.
    for i in np.argsort(np.abs(residual)):
        x, y = np.rint(candidate_xy[i]).astype(int)
        cv2.circle(overlay, (int(x), int(y)), 2, tuple(map(int, colors[i])), -1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    plain = rgb.copy()
    for image in (plain, overlay):
        cv2.drawContours(image, contours, -1, (0, 220, 0), 2)
        x1, y1, x2, y2 = map(round, roi)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 220), 2)
        for y in np.linspace(roi[1], roi[3], 4)[1:-1]:
            cv2.line(image, (x1, round(y)), (x2, round(y)), (0, 220, 220), 1)
    yy, xx = np.nonzero(mask)
    x1, x2 = max(0, int(xx.min()) - 40), min(rgb.shape[1], int(xx.max()) + 41)
    y1, y2 = max(0, int(yy.min()) - 40), min(rgb.shape[0], int(yy.max()) + 41)
    panels = np.hstack((fitted(plain[y1:y2, x1:x2]), fitted(overlay[y1:y2, x1:x2])))
    canvas = np.full((810, 1040, 3), 25, dtype=np.uint8)
    canvas[80:730] = panels
    cid = raw["cycle_id"][-8:]
    state = "MAD_REFUSED" if h["status"] == "DEPTH_MAD_EXCEEDED" else "COHERENT_DIAGNOSTIC"
    title = f"{cid} {state} | z={median:.4f}m MAD={h['candidate_mad_m'] * 1000:.2f}mm n={len(indices)}"
    cv2.putText(canvas, title, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, .57, (255, 255, 255), 1)
    cv2.putText(canvas, "Source RGB + mask/ROI", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    cv2.putText(canvas, "Original stereo depth projected into RGB", (530, 60), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
    gradient = residual_colors(np.linspace(-.2, .2, 940))
    canvas[746:762, 50:990] = gradient[np.newaxis, :, :]
    for label, x in (("closer: -200mm", 50), ("frame median: 0", 440), ("farther: +200mm", 815)):
        cv2.putText(canvas, label, (x, 786), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    rows = band_statistics(candidate_xy, candidate_z, roi, median)
    return canvas, {"cycle_id": raw["cycle_id"], "baseline_hypothesis": h,
                    "candidate_point_count": len(indices), "other_masked_point_count": int((~candidate).sum()),
                    "color_scale_residual_m": [-.2, .2],
                    "saturated_color_point_count": int((np.abs(residual) > .2).sum()),
                    "depth_p05_p50_p95_m": np.percentile(candidate_z, [5, 50, 95]).tolist(),
                    "image_height_bands": rows,
                    "source_audit_sha256": sha256(path),
                    "source_disparity_sha256": sha256(path.parent / "disparity_lr_consistent_px.npy"),
                    "source_mask_sha256": sha256(path.parent / f"{raw['cycle_id']}.png"),
                    "source_ar0234_sha256": sha256(path.parent / "ar0234_source.jpg")}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-report", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--cycles", nargs=2, default=["00000003", "00000004"])
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("output directory already exists")
    baseline = json.loads(args.baseline_report.read_text(encoding="utf-8"))
    lower, upper = baseline["diagnostic_depth_range_m"]
    if not np.isfinite([lower, upper]).all() or not 0 < lower < upper <= 4.5:
        raise ValueError("invalid diagnostic range in baseline")
    activation = json.loads(args.activation.read_text(encoding="utf-8"))
    extrinsic = activation["calibration_profile"]["ar0234_to_ov9281_physical_left_extrinsic"]
    source = activation["calibration_profile"]["source_sha256"]
    if (sha256(args.ar_intrinsic) != source["ar0234_intrinsic"]
            or sha256(args.stereo_calibration) != source["ov9281_stereo_calibration"]
            or sha256(Path(extrinsic["path"]).expanduser()) != extrinsic["sha256"]):
        raise ValueError("activation/calibration SHA mismatch")
    cal = load_calibrations(args.ar_intrinsic, args.stereo_calibration,
                            Path(extrinsic["path"]).expanduser())
    if cal["sha"] != baseline["calibration_sha256"]:
        raise ValueError("baseline/calibration SHA mismatch")
    source_root = Path(baseline["source_root"])
    saved = {r["cycle_id"]: r for r in baseline["cycles"]}
    images, rows = [], []
    if len(set(args.cycles)) != 2:
        raise ValueError("choose two different cycles")
    for suffix in args.cycles:
        matches = [cid for cid in saved if cid.endswith(suffix)]
        if len(matches) != 1:
            raise ValueError(f"ambiguous or absent cycle {suffix}")
        cid = matches[0]
        matches_paths = list(source_root.rglob(f"{cid}/audit.json"))
        if len(matches_paths) != 1:
            raise ValueError(f"ambiguous or absent source audit {cid}")
        path = matches_paths[0]
        raw = replay(path, cal, lower, upper)
        verify_baseline(raw, saved[cid])
        image, row = render(path, raw, cal, lower, upper)
        images.append(image)
        rows.append(row)
    states = {r["baseline_hypothesis"]["status"] for r in rows}
    if states != {"DEPTH_MAD_EXCEEDED", "COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY"}:
        raise ValueError("choose one MAD-refused cycle and one coherent cycle")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for image, row in zip(images, rows):
        name = f"{row['cycle_id'][-8:]}_depth_regions.jpg"
        if not cv2.imwrite(str(args.output_dir / name), image):
            raise OSError(f"cannot save {name}")
        row["overlay"] = name
        print(row["cycle_id"][-8:], row["baseline_hypothesis"]["status"], flush=True)
        for band in row["image_height_bands"]:
            print(" ", band["image_band"], "n=", band["sample_count"],
                  "median_m=", round(band["median_depth_m"], 4) if "median_depth_m" in band else None,
                  "MAD_mm=", round(band["local_mad_m"] * 1000, 1) if "local_mad_m" in band else None)
    pair_path = args.output_dir / "depth_regions_pair.jpg"
    if not cv2.imwrite(str(pair_path), np.vstack(images)):
        raise OSError(f"cannot save {pair_path}")
    report = {"status": "ORIGINAL_DEPTH_REGIONS_DIAGNOSTIC_ONLY", "cycles": rows,
              "metric_measurement_authorized": False, "actuator_access": False,
              "reference_frame": "rectified_left_optical_frame", "unit": "m",
              "overlay_reference_frame": "ar0234_image_frame", "confidence": None,
              "confidence_status": "NOT_VALIDATED", "calibration_sha256": cal["sha"],
              "baseline_report_sha256": sha256(args.baseline_report),
              "diagnostic_depth_range_m": [lower, upper], "pair_image": pair_path.name,
              "code_sha256": sha256(Path(__file__)),
              "limitations": ["Color represents stereo z minus each frame's own candidate median, not physical error.",
                              "Shared display scale clips at +/-200 mm; gray points belong to other unsupported depth layers.",
                              "Image-height thirds are not semantic body parts.",
                              "Projection overlap can hide points; no interpolation or filtering is applied.",
                              "Two frames cannot establish a unique noise, motion or geometry cause."]}
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("PAIR:", pair_path)
    print("REPORT:", args.output_dir / "report.json")


if __name__ == "__main__":
    main()
