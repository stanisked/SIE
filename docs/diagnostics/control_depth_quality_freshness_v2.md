# Control-depth shadow freshness assessment revision 2

2026-10-16. Diagnostic only. No Measurement, navigation or actuator admission.
The existing diagnostic schema keeps its name and adds `assessment_revision=2`.

## Reason for the change

The original-JPEG live run at
`/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_123558`
had 71 completed cycles. Recorded quality: 67 SHADOW_REFUSED and four
SHADOW_UNDETERMINED. All 67 refusals were
HOST_PROCESSING_EXCEEDS_DECLARED_AGE_TARGET. All 71 had candidate point support
and one plausible depth layer. All lacked current-frame local consistency,
independently established exposure-to-command timing and a physical error bound.

The log separately reported three MAD-coherent cycles with both driver-declared
frame ages within 250 ms: RGB median/max 245.54/248.70 ms, stereo
240.63/241.46 ms. Host duration for those three exceeded 250 ms because a cycle
can wait for a new exposure. Host duration cannot be substituted for frame age.
The steady-state group excludes the first cycle, unlike the 71-cycle quality
counts. Global MAD passed three of 71 and remains unchanged.

## Revised behavior

Host cycle duration remains under `freshness.source_primary_host_processing_s`.
Its comparison against the numeric age target is a performance observation under
`freshness.performance`, not a refusal. There is no new host-performance limit.

`freshness.driver_metadata` separately assesses the original primary report's
two driver-declared SOE-to-result ages. It requires the native capture backend,
the known result-ready endpoint, exactly both sensor keys and finite nonnegative
numeric ages (booleans excluded). The maximum is recomputed from both values,
not copied from a cached success flag or maximum. The existing declared age
target is used; no threshold is added or relaxed. An exceeded age adds
DRIVER_DECLARED_FRAME_AGE_EXCEEDS_TARGET. Missing/invalid ages or target remain
missing evidence, never a successful freshness observation.

Driver ages describe result readiness before publication. They do not establish
physical exposure-to-command age. That missing evidence stays explicit, as do
physical depth-error and current-frame local-consistency gaps. Both SHADOW_REFUSED
and SHADOW_UNDETERMINED remain non-actuating. Deferred local results cannot be
reused for another frame or reset camera age at their completion time.

## Offline replay on Pi

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
python3 vision_core/tools/analyze_control_depth_quality_shadow.py \
  --journal-root /home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_123558 \
  --output /home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_123558/quality_freshness_v2.json
```

The output must be new. The replay preserves original recorded assessments
beside revised ones, counts refusal reasons, missing evidence, driver-age states,
performance observations and recorded-to-replayed status transitions. It includes
the first completed cycle, excludes cancellation and verifies source hashes after
reading. Reports are not rewritten. ROS build, cameras and person are unnecessary.

## Validation and limits

20 tests passed: quality assessment, control scheduling and native capture.
They include slow-cycle/fresh-frame and fast-cycle/stale-frame cases, equality at
the target, missing/partial/negative/NaN/boolean ages, endpoint mismatch, exact
local binding and subprocess replay preserving original files and assessments.
All three modified Python files compiled. Actual 71-cycle replay remains pending
on the Pi; only its terminal log and aggregate counts were available here.
