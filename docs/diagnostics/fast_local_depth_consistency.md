# Fast current-frame depth-consistency candidate

2026-10-16. Offline implementation first; no live configuration, threshold,
Measurement, navigation or actuator change. Legacy global MAD is retained.

## Why this step

The reviewed 192x320 live run
`control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_134441`
had all 93 steady cycles within 250ms by both driver timestamps; 17/94 completed
cycles passed global MAD. The 17 coherent cycles had RGB ages median/max
220.46/235.39ms and stereo 197.53/207.47ms. Those are diagnostic ages at result
readiness, not independently established exposure-to-command timing. Scene and
person changes prevent assigning different MAD pass rates to input-size alone.

A whole person's depth spread includes shape. The existing robust SciPy local
fit described shape better but was too expensive for every control-preview
cycle. This candidate measures local consistency using batched small fits.

## Algorithm and interpretation

Only current-frame mask-owned LR-consistent stereo samples are used. Match-grid
32-original-pixel tiles use the same 24-point minimum as the prior diagnostic.
The grid checkerboard parity splits each tile into two folds. A disparity plane
is fitted by ordinary least squares to one fold and evaluated on the other,
then swapped. Every reported residual is out of its own training fold. All
training points, including outliers, remain; no smoothing, trimming, replacement,
model-corrected depth or median-depth correction is performed.

Tile-local coordinates keep 3x3 normal equations scaled. Batched moment
accumulation, eigenvalue checks and solves replace per-tile iterative SciPy
Huber fits. Minimum six points per fold and a 1e-10 relative normal-eigenvalue
numerical stability test prevent unsupported/degenerate solves. These are model
applicability checks, not physical quality/error thresholds. Both folds must be
valid; sparse, degenerate, nonpositive-prediction and unfitted support remain
explicit. Patch reason counts can overlap if different folds fail differently.

Axial residual is `fb/d_observed - fb/d_predicted`; raw depth MAD and held-out
residual MAD/p90/max are recorded separately. Fitted sample fraction and coarse
RGB bin coverage are exposed, not converted into automatic admission. A smooth
biased/wrongly associated surface can have zero residual. Local fit agreement
cannot establish physical distance accuracy, target identity or a bound on the
median depth estimate. Existing support, layer ambiguity, calibration, target
ownership and current-frame age checks remain necessary.

This is deliberately a distinct algorithm. Prior robust one-fold Huber and
new ordinary two-fold fits need not produce equal residual statistics.

## Offline comparison

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/run_fast_local_depth_benchmark.sh
```

No camera/person or ROS build needed. The same 22 saved full-evidence cycles
are used, with source/calibration/mask provenance verified. Two repetitions
alternate algorithm order. Both algorithms receive exactly the same original
owned samples. Legacy depth status and old/new residuals, fitted support and
coarse RGB coverage are preserved. Total timing includes fit and coverage;
fast fit/coverage components are also recorded. Mask hashes and source cycle IDs
bind the output to this offline source; no fit is reusable for another frame.
The output is new and all inputs are hash-checked for mutation.

This benchmark uses stored full-resolution disparity and stored masks, not new
live 192x320 frames, and does not establish live cost or fix the latest 77 MAD
refusals. If the measured cost and diagnostic behavior justify integration,
add a separate current-frame shadow path with exact frame/mask binding. It must
fit the measured remaining age budget; expensive occasional diagnostics cannot
authorize later frames. Turning residuals into a physical control-error bound
requires separately validated evidence, not a renamed MAD or arbitrary threshold.

## Validation

16 tests passed across new fast local diagnostics, original local-surface
analysis and control-depth quality shadow. New tests check a sloping noise-free
plane, retained outliers, both-fold residual equivalence to independent per-tile
NumPy SVD least squares, unchanged inputs, empty/sparse/degenerate support and
invalid disparity. A deliberately biased plane test demonstrates the physical
accuracy limitation. Three Python files compiled and the launcher passed shell
syntax checks. Local OpenCV test dependency was 4.11.0; Pi benchmark records its
own versions. Real 22-cycle benchmark/timing remains pending on Pi.
