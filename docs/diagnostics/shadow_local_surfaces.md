# Local masked stereo surfaces: NumPy, SciPy and Open3D

Date: 2026-10-14. Status: offline results reviewed; optional live shadow
implementation tested synthetically. Pi live timing remains pending. No new gate.

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
~/dev_ws/runtime_envs/sie_geometry_viewer/bin/python -m pip install open3d==0.20.0
~/dev_ws/runtime_envs/sie_geometry_viewer/bin/python \
  vision_core/tools/view_shadow_cloud_open3d.py /absolute/local/path/to/exported_cloud.ply
```

The viewer reads the original geometry and supports rotation/zoom. It does not
filter, register or replace points. The operator installed Open3D 0.20.0 and viewed both clouds on the laptop.
Analysis and export do not need it.

## Validation and continuation

Seven tests pass for a sloped plane, held-out and training outliers, sparse and
degenerate support, invalid disparity, coherent scale bias, mask-grid replay,
PLY source-geometry preservation and a full synthetic CLI with calibration hashes.
Python syntax checks pass. The Pi report reproduced all 25 baseline cycles.

Review local residuals together with bias, absolute p90, coverage and failures
across all 22 target frames. A favorable subset or lower residual is not a motion
permission. Decide whether the model provides useful quality evidence before any
live admission. No motion is authorized by this analysis.

## Reviewed offline result

In 22 target cycles, median held-out local MAD was 15.85 mm, compared with
44.93 mm global raw MAD. Median fitted sample fraction was 96.29%; median
coarse RGB coverage was 95.63%. Refused cycle 03 had held-out MAD/p90 absolute
residual 16.66/62.07 mm; accepted cycle 04 had 16.11/61.35 mm. Their overlap
supports recording shape-aware evidence, not replacing the existing gate.
These are residuals relative to a fitted model, not physical accuracy.

## Optional live shadow, separate diagnostic range

`shadow_local_surfaces_enabled` defaults to false and requires evidence capture
and `shadow_instance_mask_enabled`. The two `shadow_diagnostic_range_*_m`
parameters default to 0.5 and 4.5 m, with an upper limit of 4.5 m. They never
change `range_min_m`, `range_max_m`, activation or original association gates.

After the original association computes its result, the existing long-lived
segmentation companion receives a second request. It verifies calibration,
activation, same-cycle source/mask and full LR disparity hashes; then runs the
same whole-frame offline replay and local analysis. No additional stereo matcher
or point filtering is introduced. The diagnostic mask-depth cutoff remains the
offline 50 mm/100-sample setting, recorded as analysis configuration, independent
of the parent's configurable gate. Original association and tracking confirmation
still follow the original result. Worker errors and timeouts produce an explicit
shadow-unavailable field; they cannot turn a refusal into an association.

Each cycle records `shadow_local_surfaces`, with masked depth, local held-out
MAD, absolute p90/max, fitted sample fraction, coarse RGB coverage, unfitted
patch fraction, environment and code hashes. Full patch records and original
samples are in `shadow_local_surfaces/report.json` and its NPZ. A separate
immutable `shadow_surface_input.json` preserves the original outcome before
diagnostics; raw evidence and final `audit.json` are not used as scratch files.

`processing_timing.host_capture_start_to_result_ready_s` includes both camera
reads, detector, stereo, workers and intermediate file writes. Its endpoint is
before final audit serialization/write and ROS publication. This is host
processing latency, not exposure-to-result age; camera buffering is unknown.
The mask worker and surface worker round trip have separate timings. Cold imports
are included in the first request. Early refusal/no-target cycles have timing
and explicit `NOT_RUN`, with no previous mask/depth reused.

On Pi, with a person at the earlier approximately 2.3 m position and the same
camera geometry and documented warm-up, run once for 60 seconds:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_shadow_local_surfaces_probe.sh
```

The script uses the existing `sie_mvp` interpreter for ROS/ONNX and
`sie_seg_offline` for segmentation/SciPy. Each cycle prints `AUDIT`, original
status, shadow status and host latency. Evidence is written to a new timestamped
`person_live_local_surfaces_*` root. The active 2 m refusal is expected for this
scene; evaluate the independent shadow result and timing. Measurement,
navigation and actuator access remain disabled. Do not change hardware or collect
another multi-distance series for this check.

Six additional tests exercise independent diagnostic range beyond the original
2 m gate, unchanged original range refusal, source preservation, corrupt binding,
invalid range/cycle, worker timeout, disabled mode, and early-refusal timing.
Tests use the actual node methods with a ROS-free harness; live cameras, YOLO
performance and ROS publication on Pi are not tested locally.

## Host stage profiling

The same 60-second runner now adds per-cycle `TIMING` lines and a final
`TIMING_SUMMARY`. Separate stages cover AR/stereo reads (including decoding),
RGB detector, mask worker round trip, left/right rectification, left/right SGBM,
LR validation, association geometry, evidence image/array writes, stereo debug
render/write and local surface worker round trip. File stages include image
encoding or array preparation where applicable. Surface-worker file writes are
included in its round trip, not charged a second time as parent file I/O.

`processing_timing.stages` stores call count, inclusive and exclusive seconds.
Nested children are subtracted from exclusive parent time, so sum only
`exclusive_s`. `unattributed_s` exposes control flow and other uninstrumented
work. Timing records survive raised stage errors; each cycle starts a fresh
collector. Final audit write and ROS publication remain outside the stated
latency endpoint. The final summary excludes the first saved cycle per run,
reports median/max and per-stage sample count, and does not classify it as a
validated warmed operating regime. Missing stages are absent, not zero cost.

The 2026-10-14 live run completed 12 cycles, with warm host latency about
3.90–4.00 s. Mask inference was 0.54–0.57 s and surface round trip 0.34–0.38 s.
The remaining approximately 3 s has not yet been attributed. Profiling does not
change capture mode, matcher settings, evidence contents, thresholds or gates.
Optimization is pending measured stage costs from Pi, not an inferred bottleneck.

## Offline SGBM scheduling comparison

The next Pi profile attributed median 1.2891 s to right SGBM and 1.2857 s to
left SGBM, approximately 65% of the 3.9857 s total. Mask/surface round trips
were 0.5045/0.3178 s; RGB detection 0.2864 s. Stereo matching is the measured
priority. This does not imply that a second thread will accelerate it.

`benchmark_stereo_sgbm_parallel.py` reads two saved rectified JPEG pairs and
compares sequential and concurrent calls on two independent SGBM instances.
It extracts all constructor parameters from the actual parent and audit-node
source using a restricted constant AST reader, without importing ROS. Resolution,
disparity count, penalties, filters and bilateral tolerance (1.5 px) are unchanged.
It tests initial OpenCV thread count, one thread and two threads. Duplicate thread
counts are skipped. A persistent two-worker pool excludes startup from measured
calls; one warmup per mode is excluded. Sequential/parallel order alternates.
Both signed raw int16 disparity maps and LR-consistency masks must match exactly,
including warmups. Input/source hashes are recorded and checked after computation.
No masks, depth estimates, source files or live node are changed.

Run on Pi using the same ROS/ONNX Python as the live node, with the node stopped:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/benchmark_stereo_sgbm_parallel.py \
  --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_192836 \
  --output /home/elwis/.local/state/sie/evidence/sgbm_parallel_comparison_20261014.json
```

No person, cameras, ROS launch or colcon build is needed. Output is one summary
per mode/thread count plus `ALL_EXACT` and `INPUTS_UNCHANGED`. Existing output is
refused. The report records every repetition, source/code hashes, constructor
parameters, OpenCV build and thread environment. This small benchmark compares
matcher wall time only, excluding reads, conversions, LR comparison and model
creation. JPEG-decoded inputs are not bit-identical to the original live arrays;
parity is between modes on the same decoded evidence, not with old saved maps.
Load and heating can affect timing. Runtime integration remains pending Pi
parity and speed results; numerical equality is not physical validation.

Four tests cover configuration extraction, real sequential/concurrent SGBM at
one/two threads, detection of a one-pixel disparity/mask change, and a complete
synthetic saved-pair CLI with source preservation and overwrite refusal.

## Optional parallel execution in the live diagnostic node

The Pi offline report `sgbm_parallel_comparison_20261014.json` tested two pairs
with two measured repeats per mode. Both int16 maps and the LR mask matched
exactly in every mode and warmup; source inputs were unchanged. Median pair
times were: sequential/parallel with four OpenCV threads 1.9637/1.7131 s;
with one thread 0.8863/0.6552 s; with two threads 0.7011/1.4769 s. These are
offline JPEG-input matcher timings, not the earlier live 2.57 s pair cost.

The audit node now exposes `stereo_match_execution` (`sequential` by default,
or `parallel`) and `stereo_opencv_threads` (0 by default, preserving the initial
OpenCV setting). The existing 60-second probe explicitly selects `parallel`
and 1. Node defaults preserve the prior scheduling. Both modes use the same
matcher instances/configuration, original resolution and LR validation. No
gate, range, disparity correction, mask ownership or Measurement behavior changes.

Parallel mode owns a persistent two-worker pool and two distinct matcher objects.
The callback waits for BOTH tasks before reusing matchers or frames, including
on a matcher error, which becomes processing refusal. Shutdown drains the pool
before restoring the process's initial OpenCV thread count. There is no silent
fallback or concurrent reconfiguration of OpenCV. One callback owns the pool.

The thread setting affects the entire main ROS process, including OpenCV image
operations, but not the isolated segmentation companion process. Each audit
records requested/effective/initial thread counts and execution mode in
`stereo_execution`. Worker left/right durations are recorded separately with a
warning that they overlap. The stage profiler counts only
`stereo_match_pair_parallel` wall time; sequential mode retains its separate
left/right stage timings. Each new cycle clears previous worker timings.

After pulling and rebuilding, run the same probe once with a person and preserve
the previous camera geometry/warm-up. Inspect `TIMING_SUMMARY`, including total
latency and mask/surface costs. Actual full-cycle speed on Pi remains pending;
offline speedup is not claimed as live performance or motion authorization.

To explicitly run the prior scheduling instead:

```bash
SIE_STEREO_EXECUTION=sequential SIE_OPENCV_THREADS=0 \
  bash vision_core/tools/run_shadow_local_surfaces_probe.sh
```

Four additional tests exercise the actual node LR method with real SGBM in
both modes, timing without double counting, object independence, completion of
both tasks after an error, repeat calls and idempotent pool shutdown. Local
tests do not substitute for the complete camera/ROS cycle on Pi.

## Shape-aware quality criterion: research stage

The optimized live run `person_live_local_surfaces_20261014_201809` recorded
22 target cycles, three accepted masked hypotheses and 19 global-MAD refusals.
Local MAD ranged 16.6–22.1 mm, absolute p90 57.0–77.6 mm and coarse coverage
97.8–100%. A same-series offline check on all 22 JPEG pairs found exact
sequential/parallel disparity and LR-mask equality; input files were unchanged.
This rules out scheduling-induced numeric changes on those decoded inputs,
not changes in live camera timing or physical scene between captures.

Draft criterion structure, with NO new numeric cutoffs or live activation:

1. Require traceable current RGB mask, disparity, calibration and cycle binding.
   Missing/corrupt evidence remains an explicit refusal/incomplete profile.
2. Treat supported point count and plausible depth-layer count as support and
   ambiguity evidence. One plausible layer does not prove physical ownership.
3. Describe local held-out MAD, absolute p90 AND maximum residual together;
   retain raw global MAD as surface dispersion rather than a pure noise estimate.
4. Describe fitted-sample fraction, coarse RGB-bin coverage and sparse/failed
   patch fraction together. High coverage does not repair high residuals.
5. Keep activated range, freshness, exposure synchronization, physical range
   uncertainty and semantic ownership as separate applicability requirements.
   A local fit cannot satisfy them by itself.

The numeric limits for items 3–4 require an explicit error/availability budget
and evidence discriminating correct from incorrect target depths. These saved
unlabelled runs do not establish false-accept rates or physical accuracy. No
20/50/60 mm cutoff or weighted score is silently substituted for global MAD.

`analyze_shadow_quality_profiles.py` checks the report SHA, same-cycle and
calibration linkage, original statuses, compact/full metrics, and frozen input
hashes. It produces complete/incomplete/error profiles without changing any
original decision. Within a single run and matching calibration/method/code it
compares six axes: local MAD, p90, maximum, coarse RGB coverage, fitted-sample
fraction and unfitted-patch fraction. A refused frame dominates an accepted one
only if it is no worse on EVERY axis and strictly better on at least one. This
partial ordering introduces no weights or numeric admission thresholds. No
dominance does not validate global MAD; tradeoffs may simply be inconclusive.
Profiles are relative to model agreement, not ranked physical accuracy.

Run once on Pi, without cameras or a rebuild:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
python3 vision_core/tools/analyze_shadow_quality_profiles.py \
  --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_192836 \
  --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_201809 \
  --output /home/elwis/.local/state/sie/evidence/shadow_quality_profiles_20261014.json
```

Each run is summarized separately; frames from different scenes/runs are not
ordered against each other. Review grouped residual/coverage distributions and
`refused_dominates_accepted` before proposing numeric thresholds. No new capture
or threshold sweep is required for this step. Five tests cover bound profiles,
tampered evidence, absent targets, conservative dominance and a full offline CLI
that preserves inputs and refuses an existing output file. Original runtime and
motor/Measurement restrictions remain unchanged.
