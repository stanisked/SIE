# Vision-derived motion requirements, draft v1

Date: 2026-10-14. Revised to operator-selected working targets.
Status: DRAFT IMPLEMENTATION REQUIREMENTS; physical admission remains unvalidated.
This is a proposed experimental operating profile, not a frozen ES, approved
Measurement policy, activation/range expansion or authorization to move.
All new numeric targets below are engineering proposals, not camera specifications,
standards or already measured performance. PROJECT_CONTEXT.md is unchanged.

## Purpose and measurement meaning

Support approach to an RGB-identified object, static or moving. The geometry and
quality contract must be independent of the semantic class and sensor vendor.
The present RGB implementation detects people; generic recognition is not yet
demonstrated. The initial speed limit does not replace the broader project goal.

Keep three quantities distinct:

1. Current diagnostic `candidate_depth_m`: median axial z of one plausible
   masked stereo layer in `rectified_left_optical_frame`, metres.
2. Target-owned observed surface geometry with an explicit uncertainty bound.
   A median is neither the nearest target point nor the target centre. A single
   plausible layer does not prove ownership or reveal occluded protrusions.
3. Decision distance `d`: clearance from the robot's forward swept footprint
   to the relevant target surface along the intended path, metres. Deriving it
   requires a surveyed camera-to-base transform and robot footprint. Sensor
   mounting offsets and unresolved target geometry belong in its error budget.

The proposed stand-off is expressed in quantity 3, not from a lens front rim.
Other objects in the swept path require obstacle observation independent of
which semantic target was selected. Target approach depth alone is insufficient.

## Proposed measurable requirements

| ID | Proposed requirement | Present evidence/limit |
| --- | --- | --- |
| R1 | Working base speed <=0.15 m/s, enforced using measured speed/odometry | Limit proposed; actual speed and braking bound must be established |
| R2 | Planned stand-off clearance >=1.00 m from forward footprint to target surface | Proposed stand-off, not established current geometry |
| R3 | Approach depth error bound <=5% of declared target-surface distance over a proposed >2.0–4.5 m approach interval | 5% was an earlier engineering goal, not a certified envelope; current activation remains 0.5–2.0 m |
| R4 | Clearance error bound <=0.10 m when estimated clearance is 1.0–2.0 m | Proposed tighter terminal requirement; not established by present diagnostics |
| R5 | Oldest contributing exposure-to-command age <=0.25 s AND the dynamic clearance inequality below must hold | Measured host processing median 2.275 s, max 2.366 s over 21 post-first cycles; buffering/exposure age is unknown |
| R6 | Timing/association error allocation <=0.02 m; at relative surface speed 1.0 m/s this requires exposure-pair timing uncertainty <=0.020 s | Host read timestamps do not prove RGB/stereo or stereo-half exposure timing |
| R7 | Robot travel from stop trigger, including watchdog/controller delay and braking, <=0.025 m at R1 speed; time to rest <=0.50 s | Allocation proposed; must be measured under relevant load, floor, supply and direction |
| R8 | Additional clearance reserve 0.10 m | Proposed reserve, not a bound on unobserved target motion |
| R9 | No motion command or renewal on missing ownership, invalid calibration/range, unknown uncertainty, stale input, ambiguity or violated stopping inequality | Fail-closed architectural requirement; implementation/verification remains separate |

R3/R4 concern physical distance error in the declared decision geometry. They
are not residual-MAD thresholds and do not assert a probability of coverage.
A future error-bound record must state its validation method, applicability and
coverage/confidence semantics explicitly; no numeric confidence is fabricated.

The 1.0 m/s relative speed used below is an illustrative design case, not a bound
on all people or objects. For a moving scene, an independently defensible upper
bound on closing speed is required. Faster/unknown motion must shorten allowed
age, increase clearance or block approach. A static-target assumption cannot
silently cover moving targets. Robot rotation also affects apparent motion and
the validity of the path-distance calculation.

## Derive the admission boundary from clearance

At the instant a command would be issued, define:

- `d_hat`: estimated decision clearance from a target-owned surface observation;
- `d_stop`: required stand-off, selected 1.00 m;
- `E_d`: applicable physical clearance-error bound, including calibration,
  matching, ownership/geometry, mounting transform and association effects;
- `A`: age of the oldest contributing exposure, including computation,
  camera buffering, transport, scheduling and command delivery;
- `v_close_bound`: upper bound on closing speed during that age interval;
- `D_stop`: additional robot travel from stop demand until rest, including
  controller/watchdog delay, proposed bound 0.025 m;
- `T_stop`: time from stop demand until rest including the longest detection/
  watchdog/controller delay, proposed bound 0.50 s;
- `v_target_close_bound`: upper bound on target-surface motion towards the robot
  during stopping; it is separate from robot travel;
- `M`: independent remaining reserve, proposed 0.10 m.

The minimum clearance consistent with the observation and its age is:

`d_lower_now = d_hat - E_d - v_close_bound * A`

The proposed age cap allocates at most 0.25 m of stale-observation closure
at relative closing speed 1.0 m/s: 0.25 m / 1.0 m/s = 0.25 s. It does not
make an arbitrary 250 ms universally safe. Geometry and motion may require a
shorter age. Permit continued approach only if BOTH:

`A <= 0.25 s`

`d_hat - d_stop > E_d + v_close_bound * A + D_stop + v_target_close_bound * T_stop + M`

The second inequality must be checked continuously by a controller/watchdog
using validated inputs, not only once per slow vision cycle. It must account
for the next command interval; `D_stop` includes the longest permitted delay
until a stop takes effect. Target motion after the last observation cannot be
bounded without the stated closing-motion assumption. Stopping the robot does
not prevent a person or object from continuing towards it.

Example, conditional on all bounds being independently established:

`d_hat=2.50 m, E_d=0.100 m, v_close_bound=1.0 m/s, A=0.25 s,
v_target_close_bound=0.85 m/s, T_stop=0.50 s`

`required additional clearance = 0.100 + 0.250 + 0.025 + 0.425 + 0.100 = 0.900 m`

For these assumed bounds, the corresponding estimated stopping boundary is
1.900 m, not 1.00 m. With a stationary target, robot closing speed 0.15 m/s,
0.10 m terminal error, 0.25 s age, 0.025 m robot stopping travel and 0.10 m
reserve, the boundary would be 1.2625 m. These are calculated examples, not
commanded stopping distances or measured new runtime performance. Error
allocations already included in `E_d` must not be charged again. R6 consumes
part of `E_d`; it is not extra slack.

For illustration, if exposure age were 2.275 s, the 1.0 m/s case would have
2.275 m of age uncertainty. The measured 2.275 s host processing median is
not exactly exposure age: reads, buffering and exposure timestamps must be
accounted for separately. The present multi-second diagnostic processing path
does not demonstrate compliance with R5. Measured cycle latency is not a proven maximum
or exposure age. A faster control perception path would need independent timing
and freshness evidence; repeating this same diagnostic run cannot establish it.

## What can and cannot be derived for the quality gate

The actual gate should require an applicable bound:

`E_d(Q, distance, pose, motion, capture state) <= min(error_requirement, available_clearance_budget)`

where:

`available_clearance_budget = d_hat - d_stop - v_close_bound*A - D_stop - v_target_close_bound*T_stop - M`

`Q` includes original sample support, plausible layers, local held-out MAD,
absolute residual p90/max, fitted support, spatial/coarse coverage, ownership
evidence and capture/timing state. The current global MAD remains an observed
surface-dispersion feature; no sole global 50 mm cutoff is justified as a full
shape-aware error model.

An error budget DOES NOT algebraically turn local p90 into physical distance
error. Coherent stereo bias can have near-zero local residual. Therefore:

- Numeric age and exposure-pair thresholds above follow directly from declared
  motion budgets, once their exposure timestamps/uncertainty are established.
- A physical clearance-error requirement follows from the task and stopping
  geometry; it is distinct from residual diagnostics.
- Numeric local-MAD, residual-tail and coverage cutoffs remain UNSET pending a
  validated link between Q and bounded target-clearance error, including wrong
  correspondences and background/ownership failures. They must not be selected
  to make the existing unlabelled frames pass.
- Neither 100 samples, one depth layer, 99% bin coverage nor 20 mm residual MAD
  individually establishes physical accuracy or admission for motion.
- Average/p95 processing time is not a freshness deadline guarantee. Any missing
  age bound, error bound or applicable profile results in refusal.

No new acceptance code is activated by this draft. It defines the target
requirements and identifies the missing evidence, rather than creating an
unverified conversion from a fit residual to a physical safety margin.

## Evidence reviewed and finite next step

The current diagnostic optimization is retained: exact sequential/parallel
maps and LR masks on all 22 saved decoded pairs, with unchanged input files;
optimized live host-cycle median 2.275 s instead of 3.986 s. The quality-profile
report contains 34 complete profiles across two runs and five refused/accepted
pairs where the refused frame is no worse on all six declared local axes.
This demonstrates disagreement with that local profile, not physical correctness.
Individual model residuals reach about 1.2 m and cannot be hidden by a small MAD.

The selected working targets now guide implementation rather than an invented
minimum-speed regime. The separate [control-input preview](control_input_preview.md)
implements overlapping RGB/stereo, deferred compact journaling and a pure
task-budget decision preview. Current unknown exposure age/physical error/transform
bounds remain explicit STOP reasons, not invented measurements. Do not start
another unlabelled multi-distance or stationary-person series to tune thresholds.
