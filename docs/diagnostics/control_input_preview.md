# Foreground control-input preview

Date: 2026-10-14. Implementation status: tested with synthetic inputs; live Pi
timing pending. Working targets: 0.15 m/s, 1.0 m stand-off from robot front,
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
