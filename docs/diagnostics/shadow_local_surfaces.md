# Local masked stereo surfaces: NumPy, SciPy and Open3D

Date: 2026-10-14. Status: offline diagnostic implementation tested;
full 25-cycle Pi results pending. No live quality gate or runtime change.

## Reason for the change

The frozen shadow replay contains 22 target cycles (16 coherent and six global
MAD refusals) plus three no-target cycles. Median 3x3 changed hundreds of grid
points without changing any refusal. Original-depth visualization of cycles 03
and 04 showed structured depth differences inside the person mask. In cycle 03,
upper/middle/lower ROI medians were 2.3552/2.2463/2.3060 m with local raw MAD
57.9/40.8/40.1 mm. In cycle 04 they were 2.3281/2.2973/2.3281 m with MAD
39.5/51.0/31.6 mm. These observations do not isolate physical shape, motion,
calibration effects or correspondence errors. They motivate local shape-aware
analysis rather than increasing a global 50 mm cutoff.

## Method

- Reproduce every frozen baseline hypothesis and calibration/image/mask binding.
  Read all original LR-consistent, in-range, mask-interior points on the same
  stride-4 rectified-left grid. Do not trim to a favorable supported depth layer.
- Group points in the same 32x32-pixel stereo tiles used by the earlier patch
  analysis. Tiles with fewer than 24 points or degenerate support remain explicit.
- Fit `disparity = a*u + b*v + c`, a planar-surface model for rectified stereo.
  NumPy least squares initializes the fit; SciPy `least_squares(loss='huber')`
  limits the influence of training outliers. Coordinates are centered/scaled.
- Alternating stereo-grid cells train and evaluate the model. Huber scale is
  `max(1/16 px, 1.345 * 1.4826 * initial_training_residual_MAD_px)`. This is a
  model-fitting parameter, not an absolute-error or safety tolerance.
- Evaluate signed axial depth residuals `observed_z - model_z` on all held-out
  points without trimming: count, median bias, MAD, absolute p90 and maximum.
  Also retain all-point residuals, raw local-depth dispersion, coefficients,
  conditioning, sparse/failed tiles and fitted-sample coverage.
- Report coarse RGB coverage in fixed 16x16-pixel bins weighted by eroded-mask
  area within the original inner ROI. One sample can occupy a bin, so this is
  explicitly coarse coverage, not per-pixel measured depth completeness.
- Export unchanged original XYZ samples for cycles 03 and 04 as RGB PLY and
  residual-colored PLY. Colors are green at zero residual to red at >=50 mm;
  unfitted points are gray. The color scale is not an acceptance gate.

No new acceptance rule is introduced. Original masked-depth statuses remain in
the report. A small local residual can coexist with coherent wrong matching or
calibration bias. Curvature and genuine discontinuities can increase residuals.
Nearby held-out points may have correlated errors; this is not independent
physical validation. There is no temporal tracking, smoothing or hole filling.

## Run on Pi

Open3D is optional and is not needed for the headless analysis. Install the
tested SciPy version only in the existing isolated segmentation companion venv:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python -m pip install --only-binary=:all: scipy==1.17.0
/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python \
  vision_core/tools/analyze_shadow_local_surfaces.py \
  --baseline-report /home/elwis/.local/state/sie/evidence/person_shadow_mask_20261011/replay_0p5_to_4p5m.json \
  --ar-intrinsic /home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  --stereo-calibration /home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  --activation /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  --output-dir /home/elwis/.local/state/sie/evidence/person_shadow_mask_20261011/local_surfaces_v1
```

New output directory is required. `report.json` records the baseline, original
statuses, source/code hashes, environment versions and local metrics. Per-cycle
NPZ files preserve u/v, disparity, depth, RGB projection, patch identity, train/test
membership and signed residuals. PLY files use meters in
`rectified_left_optical_frame`. No source evidence is overwritten.

## Optional desktop Open3D viewer

Copy the PLY files from the Pi to the laptop, pull the same code, then use an
isolated desktop environment (do not use a headless Pi session for the window):

```bash
python3 -m venv ~/dev_ws/runtime_envs/sie_geometry_viewer
~/dev_ws/runtime_envs/sie_geometry_viewer/bin/python -m pip install open3d==0.19.0
~/dev_ws/runtime_envs/sie_geometry_viewer/bin/python \
  vision_core/tools/view_shadow_cloud_open3d.py /absolute/local/path/to/exported_cloud.ply
```

The viewer reads the original geometry and supports rotation/zoom. It does not
filter, register or replace points. Open3D installation/GUI on the user's laptop
has not been performed or tested in this change. Analysis and export do not need it.

## Validation and continuation

Seven tests pass for a sloped plane, held-out and training outliers, sparse and
degenerate support, invalid disparity, coherent scale bias, mask-grid replay,
PLY source-geometry preservation and a full synthetic CLI with calibration hashes.
Python syntax checks pass. Full 25-cycle results on the Pi are pending.

Review local residuals together with bias, absolute p90, coverage and failures
across all 22 target frames. A favorable subset or lower residual is not a motion
permission. Decide whether the model provides useful quality evidence before any
live shadow integration. No new board or human capture is needed for this step.
