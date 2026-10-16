# Explicit depth-admission draft v1

2026-10-16. DRAFT_NOT_ACTIVATED. Offline shadow assessment only. No admission,
Measurement, navigation, motor command, calibration or runtime-policy change.
PROJECT_CONTEXT is unchanged. This draft does not override frozen specifications.

## Input evidence and motivation

Current source run:
`/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_141536`.
91 completed frames passed legacy MAD. All 90 steady frames had both
driver-declared SOE-to-result ages within 250ms. Fast current-frame fits cost
median/max 6.802/7.320ms; residual MAD median 8.669mm, absolute p90 median
43.139mm; fitted-sample fraction median/min 0.983/0.9233. These observations
establish neither physical accuracy nor a qualified worst-case time bound.

The next step is an executable admission checklist rather than more unlabelled
capture or threshold tuning. It distinguishes data prerequisites from conditions
that allow a Measurement/control decision. It never converts a plausible
current estimate into an admitted Measurement or motion command.

## Explicit draft policy

`vision_core/config/diagnostics/control_depth_admission_draft_v1.json` records
parameter values, unset fields and their sources. Its SHA is attached per result.

| Check | Draft condition | Meaning/source |
|---|---|---|
| Completed cycle | Current completed mask-depth diagnostic | A refused/cancelled cycle cannot supply control input |
| Current target/geometry | OBSERVED and geometry accepted | Existing target rules, no confidence/truncation relaxation |
| Diagnostic units/frame | Metres in rectified_left_optical_frame | A lens-rim datum/robot-front clearance is not substituted |
| Candidate support | Positive finite median z and >=100 samples | Existing algorithm support minimum, not proof of accuracy |
| Layer ambiguity | Exactly one plausible layer | Existing layer rule, not proof of semantic ownership |
| Local ownership | Exact cycle/mask/model/instance/RGB/fusion binding | Uses existing independently checked fast-fit provenance |
| Residual applicability | >=100 evaluated points and finite nonnegative MAD/p90/max | Reuses existing support count; does not set a physical accuracy threshold |
| Driver age | Both known ages <=0.25s | R5 diagnostic screen at result readiness only |
| Local MAD/tail/fitted fraction/coverage limits | UNSET, reported UNKNOWN | No validated map from these features to physical range error |
| Activated metric range | Inside existing activated range | Diagnostic 0.5–4.5m does not expand Measurement activation |
| Physical error bound | UNKNOWN pending applicable validated evidence | R3/R4: 0.10m up to 2m, 5% above 2m; not a fit-residual limit |
| Command timing, robot-front geometry, activation applicability | UNKNOWN | Physical timing/transform/profile evidence required |
| Motion/stop/path observation | UNKNOWN | Target approach data cannot replace obstacle sensing or measured drive bounds |

The existing 100-sample minimum is reused for residual applicability because an
error statistic computed on only a handful of supported points must remain
explicitly insufficient in this draft. It is an engineering prerequisite, not
a derived physical accuracy or statistical confidence guarantee. No local MAD,
p90 or coverage threshold is chosen from the successful recorded frames.

The numeric error requirement displayed from median diagnostic z is only a
camera-frame task comparison. The actual distance requirement must be evaluated
on target-owned robot-front clearance after the geometry is established. It is
not an error estimate, and residual p90 cannot be plugged in as that bound.

R6 timing/association error allocation (20mm) remains UNKNOWN without an
applicable relative motion bound and exposure-pair timing uncertainty. The
current allowed diagnostic timestamp skew is not silently promoted to dynamic
scene accuracy. New code does not change that existing capture gate.

## Results and refusal semantics

Every check is PASS, FAIL or UNKNOWN with observed value, requirement and source.
`sensor_prerequisites=PRESENT_NOT_ADMITTED` means only the enumerated current
sensor checks passed; it does not pass local limits, physical error, activated
range or motion applicability. Global MAD remains alongside the result, without
being treated as a shape-aware physical error bound.

An explicit failed check produces DRAFT_REFUSED. Otherwise the result remains
DRAFT_BLOCKED_MISSING_EVIDENCE. Unset criteria never implicitly pass. Missing
physical fields cannot be supplied by unverified caller booleans or stale local
results. Every result has decision_preview=STOP and actuator_access=false.

The current activated Measurement range is still 0.5–2.0m, while the observed
person is around 2.69m. Range refusal is therefore expected independently of
local residual/driver-age success. Qualifying a new range requires a separate
versioned applicability review; diagnostics do not activate it automatically.

## Offline replay on the already recorded run

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/run_depth_admission_draft_replay.sh
```

The existing analyzer retains recorded quality assessments, recomputes quality
shadow and attaches draft admission checks per cycle. Summary counts separate
sensor prerequisites, failed checks and missing checks. First completed cycle is
included; cancellation is excluded. Source journals/policy are hash-checked and
never rewritten. A new timestamped report is saved. No camera, person or ROS
build is required. The real 91-cycle replay is pending on Pi.

## Concrete remaining evidence, not new arbitrary limits

1. A physical depth-error profile must state its method, scope and applicability
   to calibration, target association, local statistics and capture state. Existing
   relative board results/local residuals do not alone supply it; no additional
   unlabelled board sweep is requested by this change.
2. Robot-front clearance needs the camera/base transform, footprint and surface
   definition. A constant correction to median z is not invented.
3. Timing needs a physical exposure/command-delivery interpretation; motion and
   stop limits must use applicable measured drivetrain evidence, including prior
   work where valid. Driver stamps/mean motor performance are not bounds.
4. Target recognition alone does not observe every obstacle in the swept path.

These conditions remain separate so the next engineering action can target the
actual missing record rather than repeat sensor tests that already answered
support, timing cost and local consistency.

## Verification

10 tests passed across new admission draft and existing quality-shadow/replay.
Checks cover slow cycle with fresh driver ages, 99/bool sample support, multiple
layers, stale frames, wrong mask cycle, wrong units, invalid residual statistics,
unset limits, activated-range failure, missing evidence and ignored fabricated
permission booleans. A subprocess replay test verifies the saved draft stays STOP
and preserves input bytes. Four Python files compiled and launcher syntax passed.
No live control path or movement bridge was connected by this implementation.
