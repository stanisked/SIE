# Foreground control-input preview

Date: 2026-10-14. Latest implementation status: single-segmentation 640x400
mode exercised on Pi; 25 completed masked-depth cycles, host median 0.8674 s.
Physical accuracy, exposure age and motion admission remain unvalidated. Working targets: 0.15 m/s, 1.0 m stand-off from robot front,
100 mm distance error at 1–2 m, 5% above 2 m within a validated range, and
250 ms exposure-to-command age. These targets are not assumed measured abilities.

## Concrete change and reason

The optimized audit measured 2.275 s host-cycle median: parallel stereo 0.885 s,
mask round trip 0.500 s, local surface round trip 0.325 s, RGB detector 0.256 s.
Thus moving image dumps alone cannot reach 250 ms.

`ar0234_ov9281_control_preview_node` is a separate prototype alongside the
unchanged full audit. It acquires a current AR/stereo pair, starts full-resolution
stereo in a worker and runs the current RGB detector concurrently. It associates
that same current RGB observation with those current disparity maps once; no
second SGBM compute is performed by association. It keeps existing ambiguity,
support, MAD and LR gates as diagnostic checks, without claiming them as an
approved shape-aware admission policy. It does not use old masks or a previous
depth as a current observation.

The control-input result is published BEFORE compact journal persistence. The
journal worker holds at most one pending record and replaces it with the latest
when overloaded. It counts drops/errors and has no model/image/array work.
No segmentation, local fits, full image/array dumps or stereo debug rendering
run in this foreground path. Full diagnostics remain available through the
existing audit and offline tools, in a separate run; this change does not
silently start another heavy camera consumer or a background YOLO model.

The new topic is `/sie/diagnostics/ar0234_ov9281/control_preview`, depth 1. It
is not `/cmd_vel`, a metric Measurement, World State or an ESP32 request. Each
record retains current-cycle/calibration provenance, status/refusal, source
geometry, diagnostic range and the unchanged activation's range separately.
The explicit prototype diagnostic frustum defaults to 0.5–4.5 m; it changes no
activation file and authorizes no metric depth outside the original profile.

`approach_control_preview` implements the current task budgets as a pure
non-actuating decision preview. Given an admitted measurement, robot-front
clearance/error bounds, exposure age, motion bounds, alignment and measured
drive stopping bounds, it returns APPROACH at a requested 0.15 m/s or STOP.
It includes both robot stopping travel and target motion during stopping.
The actual diagnostic node supplies none of those missing physical assurances
from raw camera-frame z. Its decision preview therefore remains STOP with named
unknowns; no diagnostic association is silently promoted into a Measurement.
Synthetic tests exercise the APPROACH/STOP path without motors or network.

## Limits and exactly what would remove them

| Observation or gap | Consequence | Required implementation/evidence |
| --- | --- | --- |
| Full-resolution stereo alone measured 0.885 s | Concurrency alone is not expected to meet 250 ms | Measure the new complete path once, then reduce the dominant perception computation rather than repeat the quality series |
| RGB and stereo were serial | Their compute times previously added | They now overlap on identical current-cycle inputs; actual CPU-contention effects remain to be measured |
| Mask/fit/image dumps occupied the live callback | Diagnostic work delayed every control-input result | They are excluded from this path; compact journaling is bounded/background |
| Host read timestamps are not exposure timestamps | A 250 ms host result cannot by itself claim 250 ms exposure age | Establish exposure-age/buffering and inter-camera timing bounds |
| Existing output is camera-frame diagnostic depth | It is not robot-front clearance with a physical error bound | Explicit camera/base/footprint geometry and admitted measurement uncertainty |
| Existing ESP32 bounded POST takes distance, not a commanded velocity | Requested 0.15 m/s is not an implemented/verified drive setpoint | A separately implemented speed interface/profile using encoder feedback and previous traction evidence |

No added arbitrary speed reduction or generic safety margin is used to hide
these gaps. The requested working speed remains 0.15 m/s. The motor boundary
stays disabled because the actual source is still diagnostic and the current
physical/timing/drive admission inputs are missing, not because a new minimum
speed has been invented.

The prior drive evidence is relevant: 17 September target 0.12 m/s had saved
per-wheel speed values 0.1243/0.1237 m/s and 0.0933 m encoder travel for 0.10 m.
18 September used target 0.10 m/s, with three command terminal encoder travels
0.0938/0.0923/0.0928 m; one saved per-wheel pair was 0.0497/0.0990 m/s. These
are historical snapshots, not command-average speed or an established stable
range. Sources: `Вставленный текст(20260917-080341).txt` and
`Вставленный текст(20260918-182945).txt`. Existing distance POST and old crawl
targets must not be described as already supporting 0.15 m/s velocity control.

## Single launch check on Pi

Stop the prior camera node, use the same camera geometry and person position.
This is a 30-second check of the new implementation and latency, not another
multi-distance or quality-threshold experiment:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview.sh
```

The Python module launcher requires no new ROS console-script registration.
Read `CONTROL_PREVIEW` lines, final journal error/drop counters and
`CONTROL_PREVIEW_SUMMARY`. The script reports post-first-cycle median/max host
time and counts within 250 ms. These are processing timings, not exposure-age
guarantees. The new compact `.json` files deliberately are not named
`audit.json`, so older full-audit replay tools will not misread them as a frozen
full image/mask evidence set.

Five tests cover a blocked disk writer with replacement of pending records,
source-copy isolation, journal exceptions, unknown-input STOP, synthetic valid
APPROACH at working speed, stale/too-close/dynamic-target STOP, actual callback
overlap using synchronization events, cached-map cycle isolation and publish
before journal submission. No camera or actuator was exercised locally.

## 2026-10-14 correction after the first preview run

Measured first preview: 21 records, 19 global-MAD refusals (52–65 mm), one
association and one unfinished/default refusal. Post-first host median 1.3291 s,
max 1.4365 s, 0/20 within 250 ms. RGB 0.627–1.058 s and stereo worker
1.194–1.396 s were slower under concurrent CPU load than their prior separate
stage timings. CPU contention is consistent with this result, not independently
proven as the sole cause. The default last refusal does not identify an error.

Removing masks and local diagnostics in the first preview reinstated the old
shape-blind global-MAD bottleneck. The correction supersedes the earlier
statement that this path excludes mask/local computation:

- `preview_mask_surface_enabled=true` starts a dedicated companion with the
  existing isolated segmentation/SciPy interpreter and explicit model path.
- After current RGB/stereo association, `original_association` preserves its
  exact result, including global MAD and ambiguity refusals. `shape_diagnostic`
  separately computes same-cycle instance-mask depth and local shape metrics,
  even after that refusal. No arbitrary new quality gate or accepted Measurement
  is introduced. Top-level status continues to describe original association;
  inspect both fields, rather than counting it as the shape result.
- Temporary uncompressed arrays live in `/dev/shm`, one randomized directory per
  request, then are removed. No JPEG/PNG/full array evidence is persisted by this
  path. The journal records cycle, array/model hashes, timings, mask match,
  depth layers, global MAD, local residuals including maxima and coarse coverage.
  These hashes bind a current request but do not make discarded arrays replayable.
- Worker requests bind cycle ID, input SHA and model SHA. Timeout, wrong cycle or
  invalid provenance closes the worker, so an old mask cannot enter a later cycle.
  Missing/ambiguous masks remain unavailable, with no bbox or old-mask fallback.
- Surface fits use all original mask-owned samples, without layer trimming or
  smoothing. Residuals describe model agreement, never physical depth accuracy.
- Operator interruption/ROS shutdown is explicitly `CANCELLED_SHUTDOWN` and
  excluded from script timing counts. Other processing exceptions retain type
  and bounded detail. The uninformative previous last record is not retroactively
  classified as a proven shutdown event.

Restoring this computation can increase total latency; it is not a claim that
250 ms is reached. The existing compact journal remains background/bounded;
shape result computation must finish before it can be published as current.
No need to repeat the person run yet.

### Next action: one offline resolution comparison

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/run_stereo_resolution_replay.sh
```

No ROS rebuild, cameras or person are required. The script reads all frozen
cycles from `person_live_local_surfaces_20261014_201809` in the companion
interpreter and writes a new report. It does not use the compact preview run,
which contains no images. Every saved mask/calibration/source binding is checked
with the existing replay before comparison. Missing masks are explicitly skipped.

Recompute the full JPEG-based reference and candidates at scales 1, 1/2, 1/4,
with two matchers, OpenCV one thread, scaled disparity search and LR tolerance.
Disparity/depth units and pixel-centre geometry are transformed explicitly.
Local tiles/stride scale to preserve original-image footprint/density; no depth
upsampling or duplicated points enter the mask metrics. The 7px SGBM window and
speckle parameters remain unchanged, increasing their original-image support at
smaller scales; this is a changed estimator, not bitwise equivalence.

Report mask median change, global MAD, depth layers, local residuals/coverage,
LR-valid support, common-point depth differences and match+LR time. Resize,
segmentation, capture and rectification are outside the stereo timer; it cannot
be described as complete exposure-to-command age. Full-resolution depth is a
comparison reference, not physical truth. No scale is automatically selected or
activated. All source hashes are rechecked after processing.

Local verification covers synthetic plane depth at all scales, shape residuals
remaining available after global-MAD refusal, empty-depth refusal, actual SGBM
with a known shift at all scales, wrong-cycle worker closure/temp cleanup, and
the actual callback calling shape analysis despite the old refusal. Pi timings
and frozen-data numerical comparisons remain pending operator execution.

## Separate 640x400 live diagnostic mode

The saved resolution challenge `stereo_resolution_comparison_20261014_224621.json`
contains 22 cycles per scale and confirms source hashes unchanged. Median
match+LR times: full 0.85151 s, half 0.07349 s, quarter 0.01269 s. Half-resolution
mask median differences from the recomputed full-resolution JPEG reference span
-21.28 to +10.51 mm. Half global MAD median 26.14 mm, all 22 pass the old MAD
comparison; local held-out MAD median 9.59 mm. These results establish a useful
candidate on this saved scene, not physical accuracy or dynamic readiness.
Quarter mask median difference has median +64.92 mm and reaches +97.81 mm;
quarter mode is not exposed in the live node.

The challenge also reports whole-frame common-point absolute depth delta p90
with a median 257 mm for half resolution. This includes the background and
boundaries, but the report does not establish which pixels cause that difference.
Small target-median change must not be generalized to all scene points. Coarse
coverage near 100% means occupied RGB bins, not dense correct depth everywhere.

`preview_stereo_mode=half_640x400` is an explicit diagnostic option; the node
still defaults to `full_1280x800`. A dedicated launcher selects half mode:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_640x400.sh
```

Stop any other camera consumer first. Keep the existing rigid camera geometry
and use a person in view for this one 30-second live implementation/latency check.
No new distance sweep or checkerboard run. The script selects the existing
isolated model interpreter and explicit segmentation model. No new dependency
installation is required. It writes a new mode-labelled compact journal root.

The camera still captures combined 2560x800, physical RIGHT first and LEFT
second. Full calibration/remap stays unchanged, then gray rectified images are
resized with INTER_AREA to 640x400. Two independent persistent matchers use the
same challenged SGBM parameters, search 64 instead of 128, right min -64, LR
0.75 small-image px. OpenCV threads=1 is required to match the challenged setup.
The 7px block/speckle settings retain the support change documented offline.

Small maps go directly to current-frame mask/shape analysis with scale=0.5,
scaled focal-baseline product and pixel-centre coordinate conversion. No
upsampling, duplicate points, depth fill, hidden scale/offset correction or
second full-resolution SGBM pass occurs. Local stride 2/tile 16 small px preserve
the original sampling density/footprint. Disparity units, sizes, scale, LR
threshold and half matcher parameters are explicit in every processed journal.

The legacy full-resolution component association is deliberately not called
with a small map: `original_association.status=NOT_EVALUATED`, reason
`NOT_EVALUATED_IN_HALF_RESOLUTION_MODE`. It is not described as passing or as
an observed refusal. Half-mode top-level status describes shape computation;
`SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY` means diagnostics available, NOT quality
admission. Inspect nested `shape_diagnostic.mask_depth.status` separately,
including MAD/layer/support refusals, and local coverage/residuals.
Missing/ambiguous masks remain unavailable. No old mask/depth is used.

The script summarizes total host time, RGB, stereo, segmentation, local surface
and complete companion round-trip time, mask-depth statuses and candidate depth
span. It excludes interrupted cycles from latency statistics. All heavy current
mask/local work is still included in host timing; 250 ms remains an unachieved
complete-path target until measured. `Measurement`, navigation and motors stay
disabled; no activation, calibration or operating envelope is changed.

Regression checks execute the actual live half-stereo method with synthetic
physical camera halves and compare its maps/mask bitwise to the challenged
offline configuration. Callback checks verify that no small map reaches legacy
association, scale reaches the companion, results remain diagnostic and the
next no-target cycle cannot reuse the preceding maps. Existing geometry and
worker-provenance tests also apply. Live Pi timings remain pending this launch.

### First half-mode live check and shutdown correction

Run `control_input_preview_half_640x400_20261014_230140`: 17 completed shape
cycles, every nested mask-depth status coherent; one cancelled shutdown cycle.
Post-first completed host median 1.2852 s, max 1.3324 s, 0/16 within 250 ms.
RGB median 0.4515 s, stereo worker 0.3073 s (includes full rectify/resize and
concurrent CPU load), segmentation 0.5456 s, surfaces 0.2113 s, companion round
trip 0.8025 s. RGB and stereo overlap: do not sum their durations. The restored
mask/surface computation explains why reducing stereo alone did not bring the
complete path close to 250 ms. Constant candidate median 2.71265 m is not proof
of zero physical error, zero drift or source-frame freshness; disparity
quantization can preserve a median across different frames.

A shutdown-only `NameError` was caused by stray references to callback-local
`audit` in `destroy_node`. Remove those references, drain the perception worker
before closing its stereo resources, and send final log lines to stdout after
ROS context shutdown rather than publishing to invalid rosout. A regression
executes the actual cleanup method with no cycle audit and a false ROS context,
checks resource-close order and confirms parent cleanup runs. No new camera
trial is required to check this fix. Six callback/journal/control tests and
py_compile pass; completed records of the previous run remain useful.

## Single segmentation model for current bbox and mask

The dedicated 640x400 launcher now selects `preview_rgb_backend=single_segmentation`.
The general launcher retains `two_model_reference` by default for explicit
comparison. Both continue to be non-actuating diagnostics.

One worker model invocation on the current AR frame supplies COCO person boxes,
confidence and masks with exact model-result indices. Stereo runs concurrently
with this inference. The node selects the current target, then requests local
fusion of that exact index and box. The worker holds only its last inference
result and consumes the selected mask once. It checks cycle ID and original
RGB-input SHA, and rejects a changed box/index. A new inference or any protocol
error clears older masks. No second segment inference, bbox-IoU rematching,
previous-frame depth, nearest-person fallback or old-mask reuse occurs.

The previous ONNX observer is never called in this backend and its session is
released after parent startup. Existing parent artifact checks still load/check
that historical model at construction, so its configured path/dependencies
remain required until a separate startup refactor. This is startup overhead,
not an additional per-frame inference.

Selection preserves the configured target confidence threshold (default 0.70),
current-frame top/left/right edge exclusion, and the existing geometry checks
including width/height and horizontal edge margin. Inference returns raw COCO
person instances down to 0.25, but that does not lower target admission confidence.
One eligible current instance may proceed; multiple eligible instances require
an unambiguous match to a fresh confirmed diagnostic bbox track, with the
existing IoU, normalized centre distance, score margin and optional constant
velocity prediction. No current candidate means unavailable even if a past track
exists. Track confirmation follows a coherent current masked stereo diagnostic;
it stores identity/bbox only, never depth for later substitution.

COCO `person` boxes differ semantically from the prior custom `person_upper_body`
boxes. The node records a new internal diagnostic schema
`sie.ar0234.instance_segmentation_preview.v1`, model SHA, timestamp, current
instances and selection. It does not relabel COCO detections as upper-body or
publish them through that frozen raw-observation contract. The existing central
ROI fractions are applied to the selected COCO person bbox; changed support and
accuracy must not be presumed equivalent to the two-model reference. This
backend changes no canonical contract, activation or Measurement authority.

The journal reports `rgb_detector_s=0`, `rgb_segmentation_round_trip_s`, the
worker's one-inference `mask_s`, and the subsequent fusion-only round trip with
`round_trip_scope=FUSION_ONLY_AFTER_SEGMENTATION`. Stereo overlaps the inference;
these two durations must not be added as sequential time. Complete host timing
still includes capture, requests, selection, mask fusion and local fits.
Removing the earlier measured 452 ms detector stage is the purpose, not a claim
of a guaranteed 452 ms saving or 250 ms total. CPU competition may change costs.

Use the same 30-second launch command above, with a person in view. No new
calibration or distance sweep. Test coverage checks one invocation for boxes
and masks, exact-index ownership, one-time consumption, clearing on inference
failure/wrong cycle/wrong box, zero/one/multiple targets, stale/tied temporal
matches and the actual callback never calling the old observer/adapter or
second segmentation path. Existing geometry and shutdown checks remain relevant.

Validation of this change: 27 targeted tests passed, including the production
worker/client protocol with only inference substituted by a deterministic fixture;
the fixture records exactly one predict call and refuses second use of the same
cached result. py_compile for changed Python files and bash -n for both launchers
passed. Real COCO model outputs and Pi latency remain pending the operator run.


## Latest Pi result: single segmentation, 23:12 on 2026-10-14

Journal: `/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_20261014_231200`.
Implementation `45007ad5d9850d1126a52903db75a3d3bcb8d714`.

- 26 records: 25 completed shape diagnostics, all 25 nested masked-depth statuses
  coherent, plus one cancelled shutdown. Journal dropped=0, errors=0; clean exit.
- Post-first completed timing: 24 cycles, median host 0.8674 s, max 0.9060 s,
  within 250 ms 0/24. Compared with the preceding two-model live run's 1.2852 s
  median, the reduction is approximately 32.5%. This is a run-to-run timing
  comparison, not a controlled physical-depth accuracy comparison.
- Separate detector median/max 0 s; segmentation round-trip median 0.6318 s,
  max 0.6639 s, including inference median 0.5925 s / max 0.6270 s. Stereo worker
  median 0.2116 s / max 0.2212 s overlaps this inference.
- Subsequent fusion round-trip median 0.2024 s / max 0.2148 s, including local
  surfaces median 0.1933 s / max 0.2056 s. Do not sum overlapping stereo and
  segmentation or count nested inference/surface times twice.
- Candidate median depth 2.7126 m; between-cycle candidate-median span 0.0121 m.
  This is not physical error, proof of frame freshness or dynamic-scene validation.

The one-model change is functioning on Pi. Segmentation is now the dominant
measured bottleneck and alone exceeds the complete 250 ms target. Next prepare
an ONNX export of the same checkpoint and compare with PyTorch on frozen AR
frames: inference/transport time, bbox/instance selection and mask overlap.
Record model/export/input hashes and configuration, retain disagreement/refusal
cases and avoid threshold tuning to force agreement. Use the existing saved
full-evidence captures rather than the compact latest journals, whose images
were intentionally not retained. No new person or distance test is required
for this offline step. Live replacement remains pending those results.


### 2026-10-15 offline ONNX comparison implementation

Prepared `benchmark_segmentation_onnx.py`, a separate dependency-overlay setup
and frozen-frame launcher. See [segmentation_onnx_comparison.md](segmentation_onnx_comparison.md).
It discovers and verifies the actual live PyTorch input tensor shape before
export, uses identical tensor hashes for fair ONNX comparison, checks CPU thread
variants 1/2/4, bbox/mask matching and selection/geometry outcomes. It does not
switch the live backend or modify the existing runtime environments. Real model
export/timing remains pending Pi execution. No new person run is requested.

### 2026-10-15 live ONNX rollout prepared

Offline comparison completed with identical input tensors and matched masks,
no selection/geometry-status changes; at two CPU threads, local ONNX median
0.2636 s versus PyTorch 0.4314 s. A separate explicit ONNX launcher now binds
that reviewed export/report, versions, shape and CPU-session settings:
`vision_core/tools/run_control_input_preview_640x400_onnx.sh`.
The existing 640x400 launcher remains PyTorch. Same one-inference target/index/
mask ownership and diagnostic-only motor boundary. See the ONNX runbook for the
exact paths, checks, results and live command. Concurrent Pi latency is pending;
no new 250 ms or physical accuracy claim follows from the offline result.

## 2026-10-15: current masked depth and deferred local fits

### Evidence motivating the change

First ONNX live journal:
`control_input_preview_half_640x400_single_segmentation_onnxruntime_20261015_111247`.
30 completed cycles, one cancelled; 24 global-MAD refusals, six coherent masked
results. Post-first host median 0.6758 s / max 0.7006 s, 0/29 within 250 ms.
Segmentation round-trip median 0.4167 s (inference 0.3760 s); overlapping stereo
worker 0.2637 s. Subsequent fusion round trip 0.2225 s, including local surface
fits 0.2135 s. No journal drops/errors. Candidate-median span 0.1157 m, with no
same-frame basis to attribute the difference from the previous run to ONNX.

The operator's journal extraction found global MAD 36.5–81.1 mm, local held-out
MAD 12.5–20.0 mm, p90 local residual 44.7–74.1 mm and coarse fitted-bin coverage
95.9–99.7%. Refused cycle 17 had local MAD 14.1 mm/p90 44.7 mm; passed cycle 14
had 15.6/65.6 mm. Global MAD alone does not separate local fit quality here.
This does not establish physical accuracy or explain the 115.7 mm depth span.
No mask/depth/layer/MAD/geometry/confidence threshold is changed.

### Implementation

`preview_defer_local_surfaces=true` splits current mask-depth preparation from
local fitting. The explicit ONNX launcher selects it; the general node and
PyTorch reference still default to synchronous local diagnostics.

1. Same segmentation result index, RGB/cycle/model binding and current LR maps.
2. Same projection, range, ROI, mask erosion, supported depth layers and 50 mm
   global-MAD check. A global-MAD refusal remains a refusal in `mask_depth`.
3. Return mask-depth result immediately, plus sparse original samples and
   coverage input through a SHA/cycle-checked temporary RAM transfer. There is
   no local SciPy fitting in this foreground operation.
4. Publish primary preview and submit its compact journal FIRST. Only then
   submit local work to an independent process, which loads no neural model.
5. At most one active task and one latest pending task. A new pending task
   replaces an older pending task; replacement count is explicit. Inputs are
   copied before queue ownership changes, so later frames cannot mutate them.

Foreground status `MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY` means the current masked
hypothesis is available; nested depth status may still refuse it. Local status
is `DEFERRED_NOT_YET_COMPUTED`, never a fabricated pass or old-frame result.
The primary journal's pending marker is a pre-submit snapshot, not evidence that
local computation has completed. A cancelled cycle does not submit new fits.

Each completed local result is independent and carries the EXACT original
`cycle_id`, frame timestamp, calibration/scale/range, RGB/mask/disparity input
hashes, prepared-sample transfer hash, engine/method hashes and original masked
result. It includes local-fit statistics, coarse coverage, queue wait, compute
and submit-to-result times, completion timestamp and replacement counter.
Late completion never updates a primary record or masquerades as the current
frame. Local results remain diagnostic, not Measurement or movement inputs.

Results are published on
`/sie/diagnostics/ar0234_ov9281/local_surface_async` (depth 1) and independently
written under the main journal's `local-surfaces/` directory with filenames
`local-surface-<original-cycle-id>.json`. These filenames cannot be mistaken for
primary records by the existing `ar0234-ov9281-control-preview-*.json` search.
Foreground journals contain no array payload. Large transfer files are deleted;
array hashes do not make these compact journals a replayable image dataset.

The local process sets BLAS/OMP/MKL thread counts to one before numerical imports
to avoid accidental internal oversubscription. This does not change neural or
stereo threads. One foreground mask RPC is never multiplexed with a local-fit
RPC on the same pipe. Worker failure/wrong cycle/hash produces a specific local
refusal and counters; it cannot substitute a previous result. Shutdown drains
at most the active and latest pending tasks before closing the worker. ROS
publication failure is recorded separately while local disk recording proceeds.

### One live check

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_640x400_onnx.sh
```

Stop another camera consumer, person in view, same short implementation check.
Read primary host timing and mask-depth status counts, then `LOCAL_SURFACE_ASYNC`
completed/replaced/error counters and its separate latency summary. No model
export, dependency install, calibration or distance sweep is required.

The change removes the sequential local-fit wait, not its CPU cost. Concurrent
background fitting can slow segmentation/stereo, so do not subtract 213 ms from
the previous result as an established gain. The complete 250 ms target remains
unmet until measured; no applicability or motion gate is bypassed.

### Verification

Tests check exact mask-depth parity and identical synchronous/deferred local
statistics on synthetic inputs, absence of foreground fitting, one-pending-task
replacement under a blocked worker, source-array ownership, cycle/provenance,
real separate-process fitting and wrong-cycle closure/temp cleanup. The actual
mask RPC transports deferred sparse inputs without fitting; the actual callback
publishes and journals before local submission, including MAD-refused frames.
Existing ONNX, selection, stereo, protocol and shutdown regressions remain in
scope. Numerical checks are not physical-depth validation; new Pi concurrent
timing remains pending the command above.

## 2026-10-15: sample local diagnostics every five capture cycles

The every-cycle asynchronous Pi run
`control_input_preview_half_640x400_single_segmentation_onnxruntime_20261015_114555`
completed 33 primary mask-depth and 33 local results, with no replacements,
compute/write errors or journal loss. Post-first host median 0.6335 s / max
0.6694 s, 0/32 within 250 ms. Segmentation round trip 0.4727 s, overlapping
stereo 0.3015 s, masked-depth preparation 0.1064 s, fusion round trip 0.1240 s.
Local fit median 0.4058 s, submit-to-result 0.4215 s; queue wait 0.0003 s.
Only 2/33 passed the existing global-MAD comparison; the other 31 retained
`DEPTH_MAD_EXCEEDED`. Different scene frames do not establish that the async
change worsened physical depth. Source-frame depth parity is checked in tests.

Host latency improved only about 6% over the previous synchronous 0.6758 s run.
Segmentation, stereo and fits all took longer; this is consistent with CPU
competition, not proof that it is the only cause. A background queue removed a
wait but did not remove the fit's CPU cost.

The accepted next implementation sets
`preview_local_surface_every_n_cycles=5` in the ONNX launcher. General node
and synchronous/PyTorch reference defaults stay at 1. The phase is explicit:
cycles 1, 6, 11, ... based on captured callback sequence, independent of depth
pass/fail. If a scheduled cycle lacks a valid current target, no old frame is
substituted and there is no unscheduled catch-up fit. A cancelled cycle does not
submit a fit. Interval 5 is a diagnostic load-sampling choice, not a camera,
accuracy, freshness or motion-admission threshold.

Every frame still runs current segmentation, target selection, stereo and the
unchanged masked-depth/layer/MAD checks. On the four unscheduled cycles the
worker returns that depth immediately with local status
`SKIPPED_BY_DIAGNOSTIC_SAMPLING`; it prepares/transfers no sparse local job and
performs no local fit. On scheduled cycles the existing deferred process/queue
remains unchanged, with at most one active plus one latest pending job.

Primary journals declare interval, phase, whether that cycle was scheduled and
its actual local status. Auxiliary results carry the original cycle ID and
interval in provenance. There is no interpolation, cached local result or
implicit quality pass for unsampled frames. These sparse diagnostics cannot be
claimed as per-frame physical quality certification or movement authority.
The launcher summary adds `LOCAL_SAMPLING` counts beside actual local completed/
replaced/errors and latency. A 30-cycle all-target run would request six fits
instead of 30; real counts depend on current target availability and shutdown.

Use the same ONNX 30-second command above after pull/build. Compare complete
host timing, segmentation/stereo and mask preparation against 0.6335 s, not just
local task count. About 80% fewer fit calls is intended; a specific latency gain
or attainment of 250 ms is not assumed. Model/export, thread counts, calibration,
range, all depth/selection thresholds and disabled actuator state are unchanged.

Tests verify exact masked-depth parity on an unsampled cycle, no local-input
preparation on that path, real worker RPC returning no local payload, and the
actual callback performing six current-depth calls but only two fit submissions
for cycles 1 and 6. Skips are explicit and no previous local result enters a new
frame. Existing bounded-queue, local-fit, ONNX and shutdown tests remain relevant.

## Per-frame control-depth quality shadow (2026-10-15)

The 256x384 live run `control_input_preview_half_640x400_single_segmentation_onnxruntime_20261015_124340`
completed 57 cycles: one legacy coherent masked depth, 56 global-MAD refusals;
host median/max 0.3562/0.4378 s. Twelve same-frame local results had held-out
MAD 15.6–22.3 mm, absolute residual P90 55.6–78.3 mm, coarse fitted-bin
coverage 98.4–99.8%, unfitted patch fraction 4.3–13.2%. High coverage counts
mask area in occupied bins, not dense correct depth. No physical-error bound
or cause of 144 mm candidate-depth span was established.

`control_depth_quality_shadow` is an additional diagnostic record, not a
Measurement contract or decision input. Legacy masked-depth statuses and all
numeric gates remain unchanged. It records independently:

- existing candidate sample support and plausible depth-layer count;
- legacy global MAD as context, without treating it as an independent local
  surface-quality metric;
- source-frame held-out residuals, coarse coverage and unfitted fraction;
- declared age target, primary host processing time and unknown exposure age.

`SHADOW_REFUSED` records known failures such as missing current candidate,
ambiguous layers, cancelled/failed cycle, mismatched local binding or host
processing above the declared age target. `SHADOW_UNDETERMINED` means no such
known failure was found, but evidence is insufficient for admission. Neither
status authorizes control. Local residuals have no validated conversion to
physical distance error, so they are `OBSERVED_NOT_ADMITTED`, not a pass.
No local residual/coverage threshold was invented. Exposure-to-command age
remains unknown even when primary processing is below 250 ms.

Primary assessment is published/journaled alongside the existing result. Local
background results carry a second assessment of the exact source cycle, bound
by cycle ID, mask SHA and identical source depth evidence, on the existing local
diagnostic topic/file. They never update the primary record, become current
for a later camera frame, or approve motion. Delayed assessment preserves the
source primary timing endpoint; it does not claim to measure local-completion
exposure age. No older local fit is substituted on unsampled frames.

Offline replay needs only standard Python and existing compact journals:

```bash
python3 vision_core/tools/analyze_control_depth_quality_shadow.py \
  --journal-root /home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261015_124340 \
  --output /home/elwis/.local/state/sie/evidence/control_depth_quality_shadow_20261015_124340.json
```

Replay rejects duplicate/orphan identities and records source/code SHA hashes.
A local binding mismatch remains a refusal in the output. Sampling means local
assessment is available only on the original twelve cycles, not all 57.
Subsequent live runs use the same assessment after ROS rebuild; no new capture
is needed to check this first implementation on already recorded evidence.

## Equivalent mask ownership preparation (2026-10-15)

Inspection found one stereo-to-RGB projection per frame, not a duplicated
projection. The repeated work was mask ownership: the reference implementation
built the two-pixel interior with 25 full-mask NumPy AND passes; scheduled local
fits then performed another 5x5 OpenCV erosion and ownership selection.

The foreground evaluator now uses a single constant-zero 5x5 OpenCV erosion and
one ROI/mask ownership selection. Both masked-depth statistics and scheduled
local inputs reuse exactly those indices. Layer/support/MAD calculations are
shared with the original NumPy implementation through `depth_from_owned_samples`.
The public reference `masked_depth_hypothesis` retains its original NumPy mask
selector and unchanged defaults. No projection arithmetic, sample density,
matching, mask margins, depth gates, calibration or policy is changed.

`ownership_backend=reference_numpy` exists for offline comparison only; live
calls use the optimized default. Benchmark `run_mask_ownership_benchmark.sh`
uses existing verified masks and stored full-resolution LR disparity, comparing
all result dictionaries and all prepared local sample arrays exactly. It warms
both paths and alternates order twice for each local-preparation setting.
Inputs are hashed before/after. No NN inference, cameras, motion or new scene.
This isolates mask preparation, not whole live latency or physical accuracy.
Pi timing is pending; do not attribute the entire previous 86 ms fusion round
trip to erosion or predict a gain before this comparison.

Local tests cover random holes/borders, empty/full masks, nonfinite projection
coordinates, pixel rounding and every supported stereo scale; complete depth
reports and prepared local arrays match exactly. Existing reference APIs remain
NumPy-only and diagnostic-only. Rebuild ROS before the next live use, after
reviewing the offline exact comparison; the immediate next action is offline.

## Local-worker shutdown isolation (2026-10-15)

Live `...onnxruntime_20261015_131005` completed 56 primary cycles; host median
0.3592 s, fusion round trip 0.0670 s vs prior 0.0861 s, segmentation round trip
0.2543 s vs 0.2331 s. Whole-cycle speedup was not demonstrated. Original masked
statuses: 27 coherent / 29 MAD refusals; these are different frames, not evidence
of quality improvement caused by optimization. Twelve local jobs yielded eleven
results and one `local worker closed` on final scheduled source cycle 56 near
shutdown. The old report contains no process return code; SIGINT from the shared
foreground group is a hypothesis, not a proven historical cause.

LocalSurfaceClient now starts its subprocess in a separate session. Parent-group
terminal/timeout SIGINT does not interrupt it. Queue close still drains the
active and latest pending accepted jobs, then closes worker stdin for normal
EOF exit. Bounded two-second EOF/terminate waits retain kill fallback for a
stalled worker. Request timeout is unchanged; failures remain unavailable.

Unexpected stdout EOF records pre-cleanup return code and signal name (or null
if not yet known), independently of cleanup exit status/method. Async refusal
files retain `worker_failure` and `worker_cleanup_exit`. No stale-success
substitution or change to frame binding, calculations, policy or motion.

A real subprocess-group test sends SIGINT to the isolated parent's entire group,
checks the local request completes and worker exits normally after EOF. Another
real subprocess exits with code 7; the saved unavailable record preserves it.
Queue-drain/replacement, exact fit RPC and preview lifecycle tests also pass.
No camera rerun is required specifically for this fix. Rebuild ROS before the
next planned live run; inspect local counters then. This fix addresses shutdown
integrity, not the remaining 250 ms latency gap.

## Concurrent CPU helper-thread challenge (2026-10-15)

Remaining live latency is about 359 ms. Segmentation companion has explicit
ORT intra=2/inter=1/spinning-off, but its PyTorch/OpenCV helper pools were not
explicitly limited. Actual default counts and any oversubscription are pending
measurement, not established causes of the remaining delay.

`run_concurrent_thread_benchmark.sh` compares unchanged helper defaults against
PyTorch intra-op=1/OpenCV=1 in the companion only, retaining ORT=2. Parent stereo
OpenCV=1 and the two independent half-resolution matchers remain unchanged.
No runtime launcher switches helper settings. The optional worker/client setting
exists solely for this explicit comparison until reviewed. Ready/result metadata
records requested and actual helper counts, including current defaults.

For each of 22 frozen AR/stereo cycles, two alternating-order repetitions run
one active RGB inference RPC concurrently with the two SGBM tasks. Both resident
companion models are initialized/warmed outside timing; they do not infer
simultaneously. Input images are predecoded; this is not exposure-to-command
latency. Benchmark-only mask SHA hashes compare every instance mask and exact
instance dictionaries; both signed stereo maps must match exactly as well.
Fingerprint overhead is present in both variants and excluded from worker
inference-only timer but included in RGB RPC/joint wall time. No projection,
mask-depth calculation, local fits, capture or physical error evaluation.

Source files are hashed before/after; output records code/model/report metadata
and timing. Any numerical difference aborts without activating a setting. Only
an exact and materially faster candidate would justify a separate live change.
Local unit checks exercise optional helper configuration, missing/different-mask
refusal and existing worker/client integration. Real inference timing is pending
execution on Pi. This challenge needs no person or camera capture.

### Corrected helper-thread initialization order

First Pi challenge reported `current torch=3/OpenCV=1` and requested helper1
also `torch=3/OpenCV=1`: joint medians 0.1943/0.1956 s, exact 44/44. This did
not compare different helper thread settings. No performance conclusion or live
switch follows from that run. Initialization reset the earlier request; the
configuration had been applied before Ultralytics/backend initialization.

The optional limit is now applied after model/backend warm-up, verified at
startup, and rechecked after every inference. Parent validates the startup
requested/actual counts as well. A mismatch refuses the experiment rather than
printing misleading comparison statistics. Default unspecified helper behavior
is unchanged. Rerun the same offline command; expect helper1 torch=1/OpenCV=1.
Tests include configuration/reset rejection and initialization/inference order;
19 targeted tests passed. Actual corrected Pi performance remains pending.

## Existing compact-journal timing analysis

`analyze_control_preview_timing.py` reads primary JSONs without captures or
code-path changes. It reports per-cycle raw timers and their direct differences:
RGB RPC minus inference/extraction, fusion RPC minus depth/local preparation,
and stereo worker minus longest matcher task. These differences include
serialization/loading/hashing/scheduling or non-matcher geometry; they are not
isolated pure IPC or rectification measurements.

RGB and stereo overlap. `host - max(RGB RPC, stereo worker) - fusion RPC` is
explicitly an unallocated proxy, not exact exclusive time. Capture durations,
branch start offsets and exact future wait are absent from older journals.
The analyzer cannot reconstruct those timers and never labels the proxy capture
or waiting. Negative differences are retained for inspection. First recorded
cycle per run and cancelled shutdowns are excluded explicitly; missing timers
retain their identity/count instead of becoming zeros. Schedule-group summaries
are saved to compare background-fit contention without claiming causation.
Source SHA hashes and unchanged checks preserve provenance; output is a new
file. Standard-library tests exercise parallel accounting and missing/negative
values. Immediate next step: apply to existing 131005 run, no new live trial.

## RGB mmap transport candidate (2026-10-15)

Existing 131005 timers: host median 359.24 ms; RGB RPC 254.30 ms, inference /
mask extraction 207.24 ms, per-cycle RPC-outside-inference median 47.95 ms.
Fusion RPC 66.99 ms, preparation 57.90 ms; difference 9.09 ms. Stereo worker
213.39 ms overlaps RGB. Host unallocated proxy 36.36 ms cannot be labelled
capture because capture/wait/start-offset timers were not recorded.

A separate optional `rgb_transport=mmap` client path replaces only RGB inference
NPZ transfer. Default live launchers remain NPZ. A private per-request RAM file
`/dev/shm/sie-rgb-*/current.rgb` is memory-mapped; fixed uint8 1200x1920x3,
cycle, random token and payload SHA are bound by a descriptor SHA. Parent owns
mapping/file until validated reply, and stops the worker before cleanup on RPC
failure. Receiver opens a read-only regular file with exact size/no symlink,
validates descriptor/cycle/payload, and copies into owned storage before model
use because Ultralytics may retain `orig_img`. Thus this removes NPZ packing
and loading, not every copy. No reusable buffer, stale mapping or mmap view
survives into model-owned results. Linux file mappings need no resource-tracker
registration. Temporary directory cleanup follows success/failure.

The mask cache uses the current descriptor binding SHA in the same exact
cycle/index protocol; subsequent disparity/calibration fusion still uses the
original NPZ path. Worker records input transport and transport-module code SHA.
No calibration, inference, thresholds, selection or motor behavior changes.

`run_rgb_transport_benchmark.sh` compares NPZ vs mmap with the same reviewed
256x384 ONNX model/ORT=2/helper defaults, concurrently with two half-resolution
stereo tasks. All 22 saved cycles, two alternating repetitions, warm-ups excluded.
Benchmark-only raw RGB SHA, every instance mask SHA, exact bbox/confidence
records and signed stereo maps must match. Independent raw input SHA verifies
received bytes against the predecoded source. Fingerprinting overhead is enabled
for both transports; no live latency reduction is claimed before measurement.
Helper-thread comparison remains available separately. Its corrected result
was 196.1 vs 188.6 ms joint, 44/44 exact, but no live setting was switched.

28 local tests passed, including both real worker RPC transports with mocked
model inference and actual projection/fitting, one-use cache, exact raw bytes,
wrong cycle/hash, corrupt/truncated input, strided source, ownership lifetime,
cleanup on timeout and prior preview/backend tests. Actual ML timings are
pending Pi execution. This step needs no camera or person and no ROS rebuild.

## Segmentation stage profile on frozen frames

RGB transport challenge: NPZ joint median 0.2018 s vs mmap 0.1998 s; RGB RPC
0.2004/0.1980 s, stereo 0.1147/0.1312 s. Exact 44/44, sources unchanged.
No robust/material improvement established; mmap remains optional/offline and
is not selected by live launchers. The former 48 ms live RPC difference cannot
be attributed wholesale to NPZ packing.

`run_segmentation_stage_profile.sh` profiles the reviewed 256x384 adapter on the
same 22 saved RGB frames, twice with alternating reference/profile order.
Actual predictor preprocess/inference/postprocess, ORT session.run,
ops.process_mask_native and nested ops.scale_masks are timed without replacing
calculation implementations. Required call coverage is checked; changed library
paths refuse the run rather than reporting missing mask work as zero. Original
installed predictor/ops source paths and SHA hashes are recorded.

Nested durations must not be summed as exclusive stages. Derived per-cycle
outside-predictor, inference-outside-ORT, postprocess-outside-mask-native and
mask-native-outside-scale values are explicitly separate. Scale includes resize
and unpadding; native mask processing also covers prototype reconstruction,
cropping and thresholding. Final original-size masks, exact instances and target
selection are compared to the uninstrumented path. Reference timings record
profiling overhead; no code/math/threshold change is intended.

This is isolated CPU inference, not the 207 ms live concurrent condition.
No camera/IPC/stereo/local fitting is timed here. Current helper defaults and
ORT=2 remain unchanged and actual counts are recorded. Two wrapper/accounting
unit tests and Python/shell syntax checks passed; actual Pi timings pending.

### Delegated ORT session profiling correction

Pi rejected the first profile because no `ort_run` call was observed. Operator
inspection of installed AutoBackend.forward shows it delegates to
`self.backend.forward`, while an outer `session` attribute also exists. The
profiler had patched that outer session, not necessarily the executed one.

Profiler now follows backend delegation to the session-owning implementation,
patches its session.run and records that implementation's source SHA. New
`ACTIVE_ORT` output reports actual provider, input shape, session options and
whether outer session and active session are identical. Zero ORT thread option
means default/autoselection, not no worker threads. Coverage checks remain.
A regression test includes both an inactive outer session and an active inner
session; only the inner execution is timed. Three profile tests pass.

Important open finding: previous adapter/session-replacement benchmarks assigned
an outer AutoBackend.session. Operator source inspection now raises doubt that
those assignments controlled actual execution in this installed version. Prior
observed times/exact-mask comparisons remain observations, but declared ORT
thread counts must not be treated as verified active thread counts. Profiling
records actual settings without changing them or silently altering the live
path. Review ACTIVE_ORT before any session-binding/runtime correction. This is
separate from the confirmed PyTorch helper-count checks.

## Actual ORT session thread challenge

Corrected profiler on Pi confirmed executed owner
`ultralytics.nn.backends.onnx.ONNXBackend`, delegated=true,
outer_session_is_active=false, CPU-only input [1,3,256,384], active intra/inter
options 0/0 (automatic), sequential. Offline stage medians: total 85.72 ms,
preprocess 2.85, ORT 51.60, inference 51.78, postprocess 27.15, native masks
21.87 including scale_masks 5.36 ms. Exact 44/44; no modified inputs. These are
isolated timings, not the concurrent live 207 ms inference duration.

`active_ort_session.py` resolves the actual delegated owner. Optional worker
`--active-ort-threads 1|2` replaces that owner's session with a verified export,
CPU-only static shape, intra=1|2/inter=1/sequential/spinning-off configuration.
It warms the replacement before READY and verifies active settings at startup
and after each inference. Parent independently checks actual active metadata.
The existing adapter's outer/session behavior and all live launchers remain
unchanged; this optional path is an offline candidate until explicitly reviewed.
Model SHA and package-version review remain prerequisites. Helper defaults,
NPZ transport and all model/quality parameters stay fixed in this challenge.

`run_active_ort_benchmark.sh` compares automatic/current behavior against actual
intra=1 and intra=2 concurrently with half-resolution stereo, using all 22
frozen cycles and two repetitions. Variant order rotates/reverses; only one NN
inference runs at a time. One worker/model stays resident per variant, a stated
benchmark limitation. Every received RGB payload, full mask, exact bbox/confidence
record and both signed stereo maps must match across all three variants.
A difference refuses the experiment; no automatic rollout. Actual executed
session options are printed as ACTIVE_ORT_BUDGET and retained in the report.
No camera/exposure/physical accuracy validation or full live-latency claim.

19 targeted tests passed, including replacement of inner session while keeping
an outer shadow untouched, active-setting mismatch/SHA refusal, profiling
coverage, helper settings, worker protocol, mmap ownership and reviewed adapter.
Actual Pi comparison is pending. Immediate action is offline, no ROS rebuild.

### Preserve active-session numerical differences

First actual active-session challenge stopped on a strict equality difference
without retaining its size/type. This is a benchmark reporting defect; the
error alone does not establish a mask/depth change or merely float rounding.
The comparison now saves every row and reports exact instance records separately
from exact mask hashes, detection counts, per-index bbox/confidence deltas,
selection disposition and geometry changes. Per-index deltas are explicitly
model result order, not asserted object identity. Full instances/selections and
mask hashes remain in the report for review. Exact stereo comparison continues.
`EXACT` now prints the true count; `all_exact` and runtime authorization=false
are explicit. Numerical differences do not abort collection; invalid source
bytes/provenance or worker failures still refuse. No tolerance or acceptance
threshold is introduced, and no candidate is rolled out automatically.
Eight targeted tests pass, including a confidence-only tiny difference with
unchanged mask vs a changed mask. Rerun the same offline benchmark to obtain the
previously missing evidence; no camera or live run is needed.

## Targeted live capture/RPC/wait timers

Actual ORT challenge completed: automatic joint 203.8 ms, intra=1 241.5 ms,
intra=2 228.9 ms. Both candidates had identical masks/counts and unchanged
selection/geometry dispositions, but bbox differences up to 0.000244140625 px
and confidence differences 1.7881393432617188e-7 prevented strict instance
identity (EXACT 0/44 across all variants). Do not replace the live auto session
with a slower capped candidate. No new numerical tolerance/admission rule.
Source inputs unchanged; helper/mmap experiments remain non-default.

Next live run adds measurements only. Primary foreground exclusive timers:
AR capture, stereo capture, stereo submission, RGB call, target selection /
geometry, explicit future wait after target selection, fusion foreground,
final future drain, and remaining foreground time. Per-cycle sum equals the
existing host-to-result endpoint; median sums need not equal median host time.
Actual stereo worker start/end and submit/RGB interval offsets additionally
record queue time and wall-interval overlap. Wall overlap is not simultaneous
CPU use or exposure synchronization.

NPZ RPC adds parent setup, serialization, file hash, request prepare/send,
response wait/decode, and response verification/local-input loading. These
nested timings apply to current RGB and fusion calls, not to mmap candidates.
Worker records input-open/provenance verification, RGB array lazy loading and
prediction/extraction separately. Legacy mask_s still includes RGB lazy load;
new prediction_extract_s excludes it, so comparisons preserve old semantics.
RPC response wait includes remote work and scheduling, not just idle waiting.

The standard launcher prints FOREGROUND, RGB_RPC, FUSION_RPC, RGB_WORKER,
TIMELINE, ACTIVE_ORT_SETTINGS and the usual summary. Groups are nested or
parallel and must not be summed across groups. Active actual automatic ORT,
256x384, half stereo, existing NPZ, every-five local fits and all quality gates
remain unchanged. No new image persistence, Measurement or actuator path.

28 tests passed, including per-cycle exclusive accounting, worker timing
identity and real worker/transport numerical checks; Python and shell syntax
checks passed. Rebuild ROS and run the existing 256x384 diagnostic once with a
person. This is the first measurement of previously absent capture/wait timers,
not a new depth/calibration trial.

## Early geometry preparation candidate

Targeted live run `...onnxruntime_20261015_141126`: 59/59 legacy coherent mask
depths, median candidate 2.7126 m / span 0.0122 m, 12/12 local jobs succeeded,
no write/processing errors. Host median 345 ms, RGB foreground 247.12 ms,
captures AR/stereo 18.10/11.58 ms, fusion foreground 65.29 ms, explicit stereo
wait median 0.01 ms/max 4.52 ms. Stereo queue 0.12 ms, overlap 222.09 ms.
No physical error bound follows from low temporal span; this run establishes
current scene repeatability and timer breakdown, not a new accuracy test.

`prepare_stereo_points` now separates existing disparity sampling/range checks,
metric z and calibrated RGB projection from mask/ROI selection.
`evaluate_prepared` applies the unchanged ownership/layer/MAD/local-input stages.
The usual evaluate wrapper executes the same stages in the original order.
No formulas, masks, sample density, interpolation, quality thresholds or current
live scheduling are changed by default.

Optional companion/client prepared-geometry requests carry points in the
existing SHA-verified temporary NPZ protocol, with source cycle/scale/range
checks before mask consumption. Parent also rejects a mismatched source cycle.
Exact current RGB model-result-index binding and single-use mask cache remain.
No previous-frame geometry or local result can substitute for a new request.

`run_geometry_overlap_benchmark.sh` compares current geometry-after-RGB with
geometry prepared in the stereo job immediately after matching/LR, concurrently
with remaining RGB inference. Same process/model/session/helper defaults and
NPZ transport in both variants; 22 frozen RGB/rectified-stereo cycles with two
alternating repetitions. Compare exact signed maps, all RGB bytes/instances/
mask hashes and complete masked-depth reports. Worker local fitting is skipped
in both; local-input equality is covered in tests. Packet transfer can add bytes,
so a gain is a hypothesis until measured. No camera/rectification/local fitting
in this benchmark timer; diagnostic mask hashing and ROI helper overhead are
common to both. No automatic switch or live latency certification.

31 relevant tests passed: scaled/masked/ROI split equivalence, no repeated
projection, local-array identity, real worker prepared RPC and wrong-cycle
refusal, original geometry/backend lifecycle and quality diagnostics. Python
compile and launcher syntax pass. Next action is the offline Pi comparison,
not another live person capture. Existing live launcher remains unchanged.

## Target-specific stereo crop area/support estimate

Early geometry comparison closed without rollout: current joint median 319.5 ms
vs early geometry 318.9 ms, RGB 220.4/262.9 ms and final fusion 57.9/18.2 ms.
44/44 exact; sources unchanged. Work moved between stages but no material
whole-path gain was demonstrated. Stop rescheduling this work between threads.

Next step is an offline area/support estimate, not cropped matching:
`run_target_crop_analysis.sh`. For all 22 frozen cycles, replay verifies existing
mask/model/RGB/calibration bindings. Unchanged full-frame half-resolution SGBM
and LR checks reconstruct baseline point support from saved rectified JPEGs.
No cropped matcher, new detector, camera or live pipeline is activated.

Two RGB regions are assessed: saved inner ROI and saved segmentation-model bbox.
Their dense <=1-pixel sampled rays are transformed through the existing
AR-to-left extrinsic and rectification for rectified-left optical z=0.5..4.5 m.
Nonforward/singular geometry falls back to a full-frame plan. Sampled ray bounds
are not a formal continuous distortion/calibration/motion uncertainty bound.
The existing diagnostic margin is read from the node parameter declaration;
additional horizontal support follows both signed disparity-search intervals,
block radius and LR rounding. Both image crops use a common origin. The search
range is not narrowed and camera halves are not swapped.

Report records core/common crop, area fraction and getValidDisparityROI support
in each direction. Every observed mask-owned point is checked at its left pixel
and actual rounded right lookup. Counts distinguish points outside the crop,
already outside full-frame local support, and additional local-support loss
caused by cropping. Empty support does not become success. Full bbox and inner
ROI counts are separate and are not summed as independent target evidence.

Even zero observed loss does not prove cropped disparity equality: SGBM paths
and speckle components can change with cropped context. Also, a current-frame
RGB crop is available after selection, whereas current full-frame stereo starts
concurrently with RGB. Area savings alone cannot predict total latency. If the
area estimate is useful, only a subsequent same-input cropped-matcher comparison
can assess changed disparities/quality and the sequential scheduling cost.
No previous-frame ROI/track fallback is introduced here.

11 targeted tests pass: identity/translated frustum, bidirectional support,
clipping/empty/no-overlap behavior and nonforward fallback, plus existing
projection/prepared geometry tests. Python/shell checks pass. Actual recorded
scene areas and point-loss counts await this one offline Pi command.

## Cropped matcher comparison on frozen target ROI

Area/support estimate result: saved inner ROI median/max fraction
0.289791/0.291855, 88,468 observed target points; saved model bbox
0.511418/0.520160, 173,209 points. Across 22 cycles neither region had observed
points outside crop or additional local-support loss; none already fell outside
full-frame support. These counts overlap between regions and are not independent
samples. Frozen source inputs unchanged. Area alone is not speed/equality proof.

`run_target_crop_benchmark.sh` now executes actual full and common-origin cropped
half-resolution SGBM, using saved inner RGB ROI, identical decoded pairs/masks,
same matcher configuration and bilateral LR tolerance. Two warmed repetitions
per 22 cycles alternate full/crop order. Crop plans/search/block/rounding padding
are exactly the preceding area's method, with no narrowed disparity interval.

Raw cropped signed maps are embedded in the original 640x400 coordinate system
with each matcher's invalid sentinel before LR lookup/rounding. Disparity values
are not offset because both image crops share the same origin. Original sampling
grid is retained, with no resize, fill, smoothing, correction or extra gate.
A regression test covers odd crop origin and half-pixel ties: LR rounding must
use global coordinates, not crop-local rounding with different parity.

The report preserves both complete masked-depth dispositions, median-depth
changes, LR support loss, raw left changes at reference target pixels and raw
right changes at their reference lookups. Owned-point sets are compared by
original pixel identity: lost/gained coordinates, common-point disparity/depth
changes and largest changed depths are retained even if median depth/status
happen to agree. No numerical tolerance or automatic crop rollout is introduced.

Timing separates matcher pair, restore/LR and projection/ownership; dense ROI
planning and crop copying are recorded separately. Matching timing assumes a
known saved ROI and excludes RGB inference/capture/rectification/local fitting.
Current-frame crop scheduling after RGB still needs a whole-path assessment.
Sources/model-mask/calibration bindings are checked and inputs hashed before /
after. No saved evidence is altered. 14 targeted tests and Python/shell syntax
checks passed. Actual Pi cropped-match timing/quality differences are pending.

Cropped-match Pi result (2026-10-15, `target_crop_match_20261015_165636.json`):
44 comparisons retained coherent masked-depth disposition; only 10 complete
reports were exact. Reference owned samples totaled 176,936 across repetitions,
with 68 lost / 52 gained and 54,636 changed common disparities. Maximum median
depth difference from full matching was 10.51 mm, not an absolute accuracy claim.
Crop matcher pair / restore-LR / geometry-ownership medians were 16.60 / 9.53 /
14.42 ms; combined match-LR-depth was 40.66 ms. Dense ROI planning separately
cost 83.72 ms. These results do not authorize a live crop switch.

`run_frustum_lookup_benchmark.sh` compares dense planning with a calibration-bound
startup lookup on these same 22 cycles, two alternating repetitions. Near/far
rectified-z ray projections are precomputed in float64 for the integer RGB grid
(including image boundaries), about 74 MB. Startup time/memory are reported
separately; per-ROI operations reduce the relevant table region to bounds.
Calibration, scale and diagnostic range must match; invalid rays fall back to
full frame. Existing search/block/LR padding is retained.

The lookup covers outward-rounded integer lattice points. It is not a certified
continuous distortion or calibration-uncertainty bound. Dense reference samples
can have fractional coordinates, so bounds are compared explicitly, including
containment, largest coordinate difference, area and observed target/support
loss. This benchmark does not rerun cropped matching: if bounds differ, the
previous cropped-match numerical results cannot automatically be attributed to
the lookup. Frozen inputs are hashed before/after. No live launch, quality
threshold, calibration or motion admission changes. Ten targeted tests and
Python/shell syntax checks passed; actual Pi lookup performance is pending.


Lookup-only Pi result: startup 0.9933 s / 73,827,872 bytes; dense plan median
79.30 ms versus lookup 19.50 ms. Of 44 comparisons, 30 bounds were identical;
lookup contained dense crop in all 44, with maximum coordinate difference one
pixel. No observed additional support loss among 176,936 repeated target samples.
Inputs unchanged. This is observed lattice/support evidence, not a continuous bound.

`run_target_crop_lookup_benchmark.sh` now runs the existing full-versus-cropped
matcher comparison using this lookup, selected explicitly by `--frustum-planner
lookup`. Dense remains the benchmark default and live launches remain unchanged.
Both full and crop depth reports and point-wise differences are retained.
`CROP_TOTAL` adds each cycle's measured plan and crop-copy cost to its repeated
match/LR/depth timing; startup is separate. It excludes RGB, capture, rectification
and local fitting and is not exposure-to-command latency. Actual Pi results pending.


Lookup cropped-match replay (2026-10-15, `target_crop_match_lookup_20261015_171244.json`):
full match/LR/depth median 117.95 ms; crop 39.42 ms; plan 20.24 ms; total
plan+copy+crop processing median/max 59.59/61.58 ms. All 44 dispositions remained
coherent, complete reports exact 10/44, lost/gained points 66/52 of 176,936
reference samples across repetitions; 53,584 common disparities changed, maximum
median-depth difference 10.51 mm. Inputs unchanged. This is not absolute accuracy.

Live opt-in `run_control_input_preview_target_crop.sh` sets `preview_target_crop`
(default false). Requires half resolution, single segmentation, diagnostic .5–4.5 m.
Startup calibration lookup is prepared once. Each cycle first selects its current
RGB instance using unchanged selection gates; only then submits the captured stereo
pair with that cycle's inner ROI. No target means no stereo job; no previous ROI
is reused. Both signed matchers use the same crop origin/configuration; maps are
restored to global half-frame coordinates before LR checking and mask fusion.
`crop_target_binding` records cycle/instance/ROI; worker timings include crop plan,
copy, restore and full plan. Host latency includes serial RGB-to-stereo dependency,
capture and fusion. A smaller stereo timing alone does not establish 250 ms latency.

Previous launcher remains unchanged by default. Existing mask MAD, support, layers,
quality shadow and local sampling remain unchanged; neither Measurement nor motor
commands are enabled. Sixteen targeted tests passed, including current-target crop
scheduling, missing-target no-submit, global LR rounding and lookup binding. Pi live
latency/quality are pending.


Live target-crop NPZ result (`20261015_171831`): host median/max 287.0/377.9 ms,
0/52 steady cycles within 250 ms. Current masked depth coherent 47/53, MAD refused
6/53; local results 11 without errors and journal without loss/errors. RGB RPC
154.7 ms, stereo worker 68.9 ms, fusion RPC 27.4 ms. Foreground RGB serialization
12.45 ms plus input file hash 10.66 ms; worker hash/open 10.82 ms and RGB load
6.71 ms are nested within RGB RPC, not extra host time.

`preview_rgb_transport` defaults to `npz`. Explicit opt-in launcher
`run_control_input_preview_target_crop_mmap.sh` reuses the previously tested mmap
transport in the new current-target serial schedule. Parent-owned /dev/shm input
is retained until validated response or worker shutdown. Cycle/token/descriptor/
payload SHA/model checks and receiver-owned RGB copy remain. No integrity checks,
selection or depth thresholds are removed. `shared_publish_s` includes allocation,
copy, payload hashing and descriptor binding; response wait and validation are
recorded separately. Parent timing excludes teardown as the existing RPC timing
did; full foreground RGB and host timers include it. Prior concurrent benchmark
showed only ~2 ms improvement; current scheduling benefit is unmeasured pending
this run. Previous NPZ launcher remains available.


Target-crop mmap live result (`20261015_172330`): host median/max 249.2/338.5 ms,
41/76 steady cycles within host 250 ms; 77/77 masked-depth coherent, candidate
median 2.7126 m, span 12.1 mm. Sixteen local results, no worker/write errors or
journal loss. RGB RPC median 116.8 ms, stereo 68.2 ms, fusion 25.2 ms. These are
different live captures from NPZ; observed ~37.8 ms host improvement does not
isolate transport causation. The host target is not established exposure age.

`analyze_local_rgb_overlap.py` reads existing primary and local JSON journals.
It reconstructs local request intervals from UTC completion, submit-to-result
and queue-wait values; intersects them with timestamp plus RGB timeline offsets
in the same run. Reports per-cycle source IDs and per-group host/RGB/prediction
medians/maxima, excluding first cycle per run and cancellation. Duplicate IDs,
orphan/binding errors and invalid durations refuse analysis. Source files are
hashed before/after. UTC reconstruction is approximate, not a CPU execution trace;
clock changes and unrecorded work remain limitations. Association alone does not
prove interference. No capture, runtime settings, thresholds or motion changes.


Saved overlap analysis: 16 RGB/local-request overlap cycles had host median
300.10 ms and prediction median 147.00 ms, none within 250 ms. Sixty cycles with
no recorded overlap had host median/max 246.38/277.37 ms, prediction median
95.21 ms, 41 within 250 ms. UTC reconstruction is observational, not causal proof.

Explicit opt-in `run_control_input_preview_target_crop_mmap_no_local.sh` sets
`preview_local_surfaces_enabled=false` (default true). Requires deferred single
segmentation path, requests current masked depth with prepare_local=false on every
cycle; no local process starts, no samples/job are prepared or submitted. Reports
local absence as DISABLED_BY_EXPLICIT_PREVIEW_CONFIGURATION. Quality shadow still
cannot admit without local evidence; neither skipped nor older local data becomes
permission. Mask support/layer/global-MAD/selection gates are unchanged. Existing
local-diagnostic launcher remains available. Live no-local latency pending.


No-local live result (`20261015_173449`): host median/max 245.7/260.6 ms,
77/83 steady cycles within 250 ms; 84/84 current masked depths coherent, median
2.7126 m, span 24.2 mm. No journal errors/loss. RGB RPC 116.4 ms, stereo 68.5 ms,
fusion 24.8 ms. Local process explicitly disabled. This does not establish exposure
to command freshness or authorize motors. Prior ESP32 supervised bounded evidence
remains a 0.10 m request / ~0.0938 m actual step, not a qualified .15 m/s setpoint.

`run_frustum_block_benchmark.sh` compares exact block reductions with the previous
whole-region lookup on the same frozen pairs/ROIs. Immutable 32x32 tile extrema
and finite flags index the same immutable float64 table. Each query uses only
fully enclosed tiles and the exact remaining edge strips; no outward tile
rounding, interpolation, dtype change, previous ROI or threshold change. Invalid
cells retain full-frame fallback. Startup/index memory and per-plan timings are
separate. Full plan equality, crop-bound equality and observed support are recorded.
No live switch or automatic acceptance is introduced. Four lookup tests include
320 arbitrary rectangle/index comparisons with interior invalid cells, tile borders,
small rectangles and fractional ROI bounds. Pi performance/equality pending.


Block-index Pi result (`frustum_block_comparison_20261015_175340.json`): reference
plan median/max 19.766/20.644 ms, blocked 4.397/4.938 ms. Complete plans and bounds
exact 44/44, zero coordinate delta or additional observed support loss among
176,936 repeated target samples. Index construction 0.270916 s, index 76,494 bytes;
base table remains 73,827,872 bytes. Frozen inputs unchanged.

`run_control_input_preview_target_crop_blocked.sh` selects explicit
`preview_crop_lookup_backend=blocked` with target crop, mmap, single 256x384 ONNX
and local surfaces disabled. Default backend remains reference; invalid backend
or blocked without target crop refuses startup. Lookup backend is recorded in
startup logs and per-cycle stereo_resolution. Both backends use identical lattice
extrema, calibration binding, padding and invalid-ray fallback. All selection,
mask depth and quality-shadow gates remain unchanged; no motion/Measurement.
Live total latency is pending, not inferred from the sum of median savings.

Blocked live result (`20261015_175659`): host median/max 228.1/238.2 ms,
88/88 steady non-cancelled cycles within host 250 ms. Current masked depth coherent
89/89, median 2.7248 m, span 36.8 mm. Journal no errors/loss. First cycle and
cancelled shutdown excluded from latency summary. This closes this run's host
processing target, not exposure-to-command latency, physical error or motion admission.

Next engineering stage: inspect frame origin/freshness. Current OpenCV `_read`
stamps monotonic time AFTER read; CAP_PROP_BUFFERSIZE=1 does not prove frame age.
`probe_camera_frame_clocks.py` compiles a small native V4L2 metadata probe against
the Pi's own Linux headers (no manually packed ABI), opens cameras sequentially,
verifies current MJPEG dimensions, requests one MMAP buffer and records actual
buffer allocation, sequence, flags, driver timestamp and host dequeue monotonic.
It collects 8 warmup / 16 recorded frames, including one deliberate 250 ms read
pause. No image is persisted or decoded; no exposure/gain/FPS/format is written.
Existing camera applications must be stopped. Native resources are released on
normal/error exits; Python enforces a timeout. Probe failure and stderr remain
in the saved report, not presented as successful timing evidence.

Linux source: https://www.kernel.org/doc/html/v6.0/userspace-api/media/v4l/buffer.html
MONOTONIC permits clock-domain comparison. Timestamp source EOF describes end
of frame; SOE describes start of exposure. Unknown/copy clocks are not compared
to host monotonic. Flags are driver declarations, not independent sensor timing
validation. Queue-pause observations apply to this probe, not automatically to
OpenCV or paired cameras. `exposure_age_established=false` remains explicit.
No motion interface, Measurement, confidence policy or quality gates changed.
Native compilation with -Wall -Wextra -Werror and two summary tests passed;
actual driver metadata on Pi pending.

Pi frame-clock probe (`frame_clocks_20261015_181454.json`): both cameras declared
MONOTONIC START_OF_EXPOSURE for all 16 recorded frames, no buffer errors. Median
stamp-to-dequeue AR0234 32.06 ms / OV9281 16.03 ms; after a 250 ms read pause,
218.17 / 234.12 ms respectively. Normal sequence increments were two. This
establishes probe queue behavior, not physical clock validation or OpenCV age.

`run_control_input_preview_latest_capture.sh` is an opt-in live native-capture
challenge on the existing blocked/mmap/no-local preview. Launcher compiles
`stream_v4l2_mjpeg.c` into a temporary executable using the Pi Linux headers.
After the usual OpenCV startup configuration, both original captures are released
and two persistent native streams open simultaneously. Current format must match
MJPEG 1920x1200 / 2560x800; native helper does not write format/FPS/controls.
One driver buffer is requested; actual allocation/FPS are recorded. The helper
copies payload and metadata, requeues the driver buffer before pipe output, and
emits each header plus exact JPEG payload. Eight startup frames are drained.
Parent-death SIGTERM prevents orphan streaming; stop releases driver resources.

Two reader threads continuously drain compressed packets and retain one immutable
latest packet each; only selected packets are JPEG-decoded in the foreground.
No list/queue of past frames exists. Each selection requires a new driver sequence;
invalid payload, dimensions, clock flags, buffer errors, nonmonotonic sequence,
truncated stream or unavailable new frame refuses current capture, never falls
back to the last valid frame. Both selected snapshots keep their exact metadata.
A buffer retained after selection cannot be mutated by later captures. Latest
replacement count is a cache replacement count, not a claim of hardware drops.

The existing pair-skew gate now compares driver-declared monotonic SOE stamps in
this opt-in backend. Per-cycle `frame_freshness` records stamps/sequences/flags,
dequeue/receipt time, format/buffer metadata, executable SHA, each frame's age at
result readiness, maximum age, pair skew and the driver-based 250 ms comparison.
Observation UTC remains explicitly HOST_CYCLE_START, not a relabeled exposure
stamp. `physical_exposure_timing_independently_verified=false`, no command issued;
quality shadow and original admission blockers remain. No driver-based age is
silently promoted to admitted Measurement or actual exposure-to-command age.

Summary adds FRAME_AGE / FRAME_PAIR / DRIVER_FRESHNESS. The 228.1 ms baseline is
preserved by the earlier OpenCV launcher; native live speed, frame-age and paired
stream quality are pending. Sixteen targeted tests passed (latest-slot ownership,
new-sequence wait, failure invalidation, flags, truncated protocol, driver ages,
cycle binding, unchanged STOP and existing scheduling). Native strict compiler
checks passed; no camera hardware is available in the development container.


First native live run (`20261015_182412`): 82 coherent masked-depth outputs, six
pair-skew refusals and one cancellation. Whole steady summary mixed 81 depth
outputs with six quick refusals: host median/max 232.0/267.4 ms. Driver age across
that mixed set: AR median/max 293.07/335.52 ms, OV 250.66/285.06 ms; pair skew
52.20/104.24 ms. Reported 6/87 driver-age successes include fast refusals and are
not evidence of fresh usable depth. Actual coherent-only counts were not supplied.

Native selection order now freezes BOTH latest compressed snapshots before
foreground JPEG decoding. Per-camera previous sequence requirements remain; no
history search, wider skew gate, changed camera settings or prior-pair fallback.
Only a complete selected pair is returned; selection failure cannot reuse a
previous payload. Decoding consumes frozen packets once, not another camera read.
Selection timestamp and source-stamp-to-selection / selection-to-result ages are
recorded separately. Selection is software pairing, not hardware synchronization.

Summary now separates COHERENT_DEPTH and OTHER_RESULTS in RESULT_FRESHNESS, with
host medians/maxima and driver-age success counts per group. FRAME_AGE_COHERENT and
FRAME_PAIR_COHERENT exclude quick refusals; first cycle/cancellation remain
excluded. Coherent means existing diagnostic status, never admitted Measurement.
Sixteen targeted tests passed, including complete-pair snapshot order, retained
compressed payloads, no partial-pair return and fast-refusal count separation.
Pi corrected pairing/age results pending; no 250 ms exposure-age success claimed.
