# Current-frame fast local-consistency shadow

2026-10-16. Explicit opt-in live diagnostic. No Measurement, navigation or motor
command. Defaults preserve the prior 192x320 path without local fitting.

## Evidence and integration choice

Completed Pi comparison `/home/elwis/.local/state/sie/evidence/fast_local_depth_20261016_135612.json`:
reference local diagnostics median/max 126.688/132.344ms, fast including coarse
coverage 25.876/26.780ms, inputs unchanged. Fast fit alone median/max
6.715/6.857ms; coarse coverage 19.143/19.986ms. These are isolated saved
full-resolution measurements, not a guaranteed live cost at half resolution.

The live candidate computes only batched two-fold disparity-plane residuals and
fitted-sample fraction. Coarse RGB coverage is explicitly NOT_COMPUTED; neither
an old frame's coverage nor its local fit can authorize the current frame.
The previous live coherent-frame RGB maximum was 235.39ms, leaving only 14.61ms
against the 250ms diagnostic driver-age target. This motivates measuring fit-only
integration rather than adding the entire 25.9ms diagnostic serially.

## Current-frame ownership and behavior

`preview_fast_local_consistency=true` requires single segmentation, deferred slow
local diagnostics and disabled slow local execution. The dedicated launcher uses
the reviewed 192x320 model, original JPEG transport, new RGB/fresh stereo arrival
and current-target crop. Slow local jobs remain disabled.

The mask worker reuses its exact current selected-instance mask and original
LR-consistent owned sample selection. A fit runs before the normal primary result
returns, including when the old global MAD refuses. `mask_depth` remains exactly
the legacy result; no depth is smoothed, corrected, substituted or admitted.

`shape_diagnostic.fast_local_consistency` records residual MAD/p90/max, support,
failed patch reasons and `fit_s`. Its provenance binds source cycle, raw mask SHA,
instance index, model SHA, source RGB request SHA and fusion input SHA (including
current stereo geometry/calibration arrays). The existing worker protocol checks
input hashes/cycle/model and single-use current mask ownership. The client
requires this exact provenance when fast local is requested. Missing or changed
provenance refuses the diagnostic rather than allowing a stale result.

Quality-shadow assessment revision 3 independently checks those bindings. Valid
residuals become OBSERVED_NOT_ADMITTED with the algorithm named and missing RGB
coverage explicit. Physical error bound and exposure-to-command timing remain
unestablished, and suitable_for_control remains false. Zero/sparse failed fits
cannot establish local consistency. Deferred slow results keep their existing
same-source binding and cannot be reused for another frame.

## Run on Pi with a person in view

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_fast_local.sh
```

The launcher prints FAST_LOCAL counts, fit time, residual MAD/p90 and fitted
sample fraction alongside unchanged depth statuses and frame freshness groups.
These residuals are errors relative to fitted local planes, not ground-truth
range errors or confidence bounds. Driver-age timing remains diagnostic.
Compare actual age/fit cost before deciding whether to retain this trial. Do not
infer a new physical quality threshold from the observed residual distribution.

Rollback: fresh shell with `run_control_input_preview_192x320.sh`, or explicitly
set `SIE_PREVIEW_FAST_LOCAL_CONSISTENCY=false` before that launcher.

## Validation

30 tests passed across single-segmentation worker/client, live mask geometry,
quality shadow, fast fitting and control scheduling. Production subprocess tests
use both NPZ and mmap transport, one inference/current-mask consumption, exact
legacy mask-depth equality with fitting enabled, no slow job, valid provenance
and coverage omission. Separate checks reject all provenance-field mismatches
and demonstrate non-admission. Seven changed Python files compiled and both
shell scripts passed syntax checks. Pi live cost and results remain pending.
