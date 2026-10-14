#!/usr/bin/env python3
"""Shape-aware offline replay with Huber planes and optional Open3D-ready clouds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import time

import cv2
import numpy as np
import scipy

from analyze_ar0234_stereo_component_ambiguity import load_calibrations, sha256
from replay_shadow_mask_depth_range import replay
from compare_shadow_mask_depth_median import project, owned, verify_baseline
from vision_core.person_localization.local_surface_diagnostic import analyze_local_surfaces


def coarse_coverage(interior: np.ndarray, roi: list, xy: np.ndarray,
                    fitted: np.ndarray, cell_px: int = 16) -> dict:
    """Mask-area coverage by occupied RGB bins; explicitly not per-pixel depth."""
    h, w = interior.shape
    y, x = np.nonzero(interior)
    x1, y1, x2, y2 = roi
    inside = (x >= x1) & (x <= x2) & (y >= y1) & (y <= y2)
    x, y = x[inside], y[inside]
    columns = (w + cell_px - 1) // cell_px
    count = columns * ((h + cell_px - 1) // cell_px)
    mass = np.bincount((y // cell_px) * columns + x // cell_px, minlength=count)
    points = np.rint(xy).astype(np.int64)
    ids = (points[:, 1] // cell_px) * columns + points[:, 0] // cell_px
    all_ids, fit_ids = np.unique(ids), np.unique(ids[fitted])
    total = int(mass.sum())
    return {"rgb_cell_size_px": cell_px, "mask_in_roi_pixel_count": total,
            "mask_area_in_observed_bins_fraction": float(mass[all_ids].sum() / total) if total else 0.0,
            "mask_area_in_fitted_bins_fraction": float(mass[fit_ids].sum() / total) if total else 0.0,
            "definition": "A bin is occupied by at least one original point; this is coarse coverage only."}


def write_cloud(path: Path, xyz: np.ndarray, colors: np.ndarray) -> None:
    if xyz.shape != colors.shape or xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("point/color shape mismatch")
    with path.open("x", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\ncomment frame rectified_left_optical_frame\n"
                     "comment unit m; original stereo points; diagnostic only\n"
                     f"element vertex {len(xyz)}\nproperty double x\nproperty double y\n"
                     "property double z\nproperty uchar red\nproperty uchar green\n"
                     "property uchar blue\nend_header\n")
        for p, c in zip(xyz, colors):
            stream.write(f"{p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {int(c[0])} {int(c[1])} {int(c[2])}\n")


def analyze_cycle(path: Path, raw: dict, cal: dict, lower: float, upper: float,
                  output_dir: Path, export: bool) -> dict:
    if "mask_sha256" not in raw:
        return {**raw, "local_surface": {"status": "NO_VERIFIED_MASKED_STEREO"}}
    started = time.monotonic()
    audit = json.loads(path.read_text())
    g = audit["stereo_geometry"]
    stride = int(g["sample_stride_px"])
    cid = raw["cycle_id"]
    disparity_path = path.parent / "disparity_lr_consistent_px.npy"
    disparity = np.load(disparity_path, allow_pickle=False)
    v, u = np.mgrid[0:disparity.shape[0]:stride, 0:disparity.shape[1]:stride]
    d = disparity[::stride, ::stride]
    good = np.isfinite(d) & (d > .5)
    u, v, d = u[good], v[good], d[good]
    fb = abs(float(cal["P2"][0, 3])) / 1000.
    # Reproduce the source replay's float arithmetic before mask assignment.
    z = abs(float(cal["P2"][0, 3])) / d / 1000.
    inside_range = (z >= lower) & (z <= upper)
    u, v, d, z = u[inside_range], v[inside_range], d[inside_range], z[inside_range]
    xy = project(u.astype(float), v.astype(float), z, cal)
    mask_path = path.parent / f"{cid}.png"
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    interior = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0)
    chosen = owned(xy, interior, g["ar_inner_roi_xyxy_px"])
    u, v, d, z, xy = u[chosen], v[chosen], d[chosen], z[chosen], xy[chosen]
    if len(z) != raw["mask_depth"]["selected_sample_count"]:
        raise ValueError(f"original masked sample count changed: {cid}")
    local, data = analyze_local_surfaces(u, v, d, fb, stride=stride)
    fitted = np.isfinite(data["axial_residual_m"])
    local["rgb_coverage"] = coarse_coverage(interior, g["ar_inner_roi_xyxy_px"], xy, fitted)
    sample_path = output_dir / f"{cid}_samples.npz"
    np.savez_compressed(sample_path, stereo_u=u, stereo_v=v, disparity_px=d,
                        depth_m=z, ar_xy_px=xy, **data)
    artifacts = {"samples": sample_path.name, "samples_sha256": sha256(sample_path)}
    if export:
        p1 = cal["P1"]
        xyz = np.column_stack(((u - p1[0, 2]) * z / p1[0, 0],
                                (v - p1[1, 2]) * z / p1[1, 1], z))
        rgb = cv2.imread(str(path.parent / "ar0234_source.jpg"))
        if rgb is None or rgb.shape[:2] != mask.shape:
            raise ValueError(f"source RGB missing or invalid: {cid}")
        pixels = np.rint(xy).astype(int)
        colors = rgb[pixels[:, 1], pixels[:, 0], ::-1]
        rgb_path = output_dir / f"{cid}_rgb.ply"
        residual_path = output_dir / f"{cid}_residual.ply"
        write_cloud(rgb_path, xyz, colors)
        # A display scale, never a local-quality gate. Gray = no fitted model.
        quality_colors = np.full((len(z), 3), 100, dtype=np.uint8)
        intensity = np.clip(np.abs(data["axial_residual_m"][fitted]) / .05, 0, 1)
        quality_colors[fitted] = np.column_stack((255 * intensity, 255 * (1 - intensity),
                                                  np.zeros(len(intensity)))).astype(np.uint8)
        write_cloud(residual_path, xyz, quality_colors)
        artifacts.update({"rgb_cloud": rgb_path.name, "residual_cloud": residual_path.name,
                          "rgb_cloud_sha256": sha256(rgb_path),
                          "residual_cloud_sha256": sha256(residual_path)})
    return {**raw, "local_surface": local, "artifacts": artifacts,
            "source_audit_sha256": sha256(path), "source_mask_sha256": sha256(mask_path),
            "source_disparity_sha256": sha256(disparity_path),
            "source_rgb_sha256": sha256(path.parent / "ar0234_source.jpg"),
            "analysis_elapsed_s": time.monotonic() - started}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-report", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--stereo-calibration", type=Path, required=True)
    parser.add_argument("--activation", type=Path, required=True)
    parser.add_argument("--export-cycles", nargs="*", default=["00000003", "00000004"])
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("output directory already exists; use a new path")
    baseline = json.loads(args.baseline_report.read_text())
    lower, upper = baseline["diagnostic_depth_range_m"]
    if not np.isfinite([lower, upper]).all() or not 0 < lower < upper <= 4.5:
        raise ValueError("invalid diagnostic range")
    activation = json.loads(args.activation.read_text())
    cp = activation["calibration_profile"]
    extrinsic = cp["ar0234_to_ov9281_physical_left_extrinsic"]
    extrinsic_path = Path(extrinsic["path"]).expanduser()
    if (sha256(args.ar_intrinsic) != cp["source_sha256"]["ar0234_intrinsic"]
            or sha256(args.stereo_calibration) != cp["source_sha256"]["ov9281_stereo_calibration"]
            or sha256(extrinsic_path) != extrinsic["sha256"]):
        raise ValueError("activation/calibration SHA mismatch")
    cal = load_calibrations(args.ar_intrinsic, args.stereo_calibration, extrinsic_path)
    if cal["sha"] != baseline["calibration_sha256"]:
        raise ValueError("baseline/calibration SHA mismatch")
    files = sorted(Path(baseline["source_root"]).rglob("audit.json"))
    saved = {r["cycle_id"]: r for r in baseline["cycles"]}
    if not files or len(files) != len(saved) or len(saved) != len(baseline["cycles"]):
        raise ValueError("baseline cycle count mismatch")
    raw_rows = [replay(path, cal, lower, upper) for path in files]
    if set(saved) != {r["cycle_id"] for r in raw_rows}:
        raise ValueError("baseline cycle identities mismatch")
    for r in raw_rows:
        verify_baseline(r, saved[r["cycle_id"]])
    exports = set()
    for suffix in args.export_cycles:
        matches = [cid for cid in saved if cid.endswith(suffix)]
        if len(matches) != 1:
            raise ValueError(f"ambiguous or absent export cycle: {suffix}")
        exports.add(matches[0])
    print(f"BASELINE_REPLAY_VERIFIED {len(raw_rows)}/{len(raw_rows)}", flush=True)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    for path, raw in zip(files, raw_rows):
        row = analyze_cycle(path, raw, cal, lower, upper, args.output_dir, raw["cycle_id"] in exports)
        rows.append(row)
        s = row["local_surface"]
        held = s.get("pooled_held_out_axial_residual", {})
        def mm(key: str):
            return round(held[key] * 1000, 2) if key in held else None
        print(row["cycle_id"][-8:], raw["mask_depth"]["status"],
              "global_MAD_mm=", round(raw["mask_depth"]["candidate_mad_m"] * 1000, 2)
              if "candidate_mad_m" in raw["mask_depth"] else None,
              "patches=", s.get("fitted_patch_count"), "/", s.get("patch_count"),
              "support=", round(s.get("fitted_sample_fraction", 0), 3),
              "rgb_bins=", round(s.get("rgb_coverage", {}).get("mask_area_in_fitted_bins_fraction", 0), 3),
              "heldout_MAD_mm=", mm("mad_m"), "p90_abs_mm=", mm("absolute_p90_m"),
              "bias_mm=", mm("median_m"), flush=True)
    report = {"status": "SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY", "cycles": rows,
              "reference_frame": "rectified_left_optical_frame", "unit": "m",
              "confidence": None, "confidence_status": "NOT_VALIDATED",
              "metric_measurement_authorized": False, "navigation_eligible": False,
              "actuator_access": False, "calibration_sha256": cal["sha"],
              "diagnostic_depth_range_m": [lower, upper], "baseline_replay_verified": True,
              "activation_sha256": sha256(args.activation),
              "activated_depth_range_m": activation.get("authorization", {}).get("metric_measurement_range_m"),
              "baseline_report_sha256": sha256(args.baseline_report),
              "method": {"model": "local affine disparity plane", "loss": "Huber",
                         "tile_px": 32, "minimum_points": 24,
                         "test_split": "alternating original stereo-grid cells",
                         "point_selection": "all original mask-interior points, no layer trimming"},
              "environment": {"python": platform.python_version(), "numpy": np.__version__,
                              "scipy": scipy.__version__, "opencv": cv2.__version__},
              "code_sha256": {"tool": sha256(Path(__file__)),
                              "projection_helpers": sha256(Path(__file__).with_name("compare_shadow_mask_depth_median.py")),
                              "baseline_replay": sha256(Path(__file__).with_name("replay_shadow_mask_depth_range.py")),
                              "model": sha256(Path(__file__).resolve().parents[1] /
                                              "person_localization/local_surface_diagnostic.py")},
              "limitations": ["Local residuals measure agreement with a shape model, not physical accuracy.",
                              "Coherent wrong correspondences or calibration bias can have small residuals.",
                              "Nearby held-out samples may have correlated errors.",
                              "Curvature and depth discontinuities may legitimately increase residuals.",
                              "Sparse/unfitted patches remain visible; no new acceptance rule is introduced.",
                              "No temporal tracking, motion validation, smoothing or hole filling."]}
    destination = args.output_dir / "report.json"
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("Saved:", destination)
    print("PLY clouds:", args.output_dir)


if __name__ == "__main__":
    main()
