# SIEE cart + Shadow: verified working baseline

This is a short navigation map for the next engineering session. It records
demonstrated results and their limits, with links to the detailed evidence.
Read it before proposing another latency optimization, straight-line motion
probe, steering-from-scratch task, or calibration change. The detailed reports
and raw journals remain the evidence; this page does not promote diagnostics
into an approved Measurement.

## Already demonstrated on the Raspberry Pi 5

| Result | Actual evidence | Boundary |
| --- | --- | --- |
| Fast, same-cycle AR0234 + OV9281 mask-depth processing | The new-RGB preview `20261015_190656` produced 75 coherent masked-depth outputs. AR driver-declared SOE to result median/max was 266.59/275.70 ms, OV 266.79/287.59 ms; host cycle including new-frame wait was 263.54/299.43 ms. [Control-input preview](../diagnostics/control_input_preview.md) | The driver timestamp was not independently checked against actual exposure. Result time is before ROS publication and a motor command. This is **already below 0.5 s to the result**, not a certified exposure-to-command age. |
| Another fast live preview | The 99-cycle v1 preview had 98/98 steady coherent driver ages within 250 ms and host median/max 195.3/242.2 ms. The static-v2 run had host median/max 195.9/242.2 ms, with 97/98 coherent driver ages within 250 ms (max 255.4 ms). [Current work](CURRENT_WORK.md) | All control previews emitted STOP; no motor decision or admitted Measurement follows from these timing numbers. |
| Experimental freshness budget | The operator selected a **500 ms experimental limit**; the preview labels it `experimental_freshness_500ms_v1`. [Current work](CURRENT_WORK.md) | Do not replace that with the older 250 ms target or infer command age from host duration. |
| Live steering on the floor toward a camera target | In the v2 run, a 0.20 m live-steered command accepted two steering updates at −7 and −4 PWM, and the operator saw a left turn toward the target. Encoder center travel was 0.1910 m, heading about 5.5°. [V2 floor report](../diagnostics/live_steering_envelope_v3_20261028.md) | This **did** demonstrate steering during one active forward command. It ended in `BOUNDED_DISTANCE_LIMIT` fault; v3 then completed 0.20 m with READY/SUCCESS but turned left while the target was right. The v4 steering correction was compiled, without a recorded v4 motor trial. [V3/v4 review](../diagnostics/live_steering_direction_v4_20261028.md) |
| Shadow UART drivetrain on the floor | The earlier bounded-forward trial reached roughly 0.10 m. On 2026-11-11 the separate Shadow UART candidate made one approximately 2 cm supervised floor step and terminal STOP. [Shadow UART handoff](../diagnostics/shadow_uart_candidate_20261029.md) | This validates the separate UART Shadow transport's short floor step, not the camera-driven live-steering result above. Do not repeat a straight segment solely to prove that wheels move. |

## Current Shadow integration, not a fresh latency benchmark

`vision_core/tools/run_shadow_live_follow.sh` currently starts the *full
resolution dynamic association audit*, not the optimized control preview.
On 2026-11-11 its four-cycle diagnostic-range run found four current RGB
targets and one `ASSOCIATED_DIAGNOSTIC_ONLY` depth; three were refused at the
existing 0.05 m MAD threshold. This path took **1.29–1.37 s** from capture
start to bridge receive. [Four-cycle report](../diagnostics/shadow_uart_candidate_20261029.md#first-05-45-m-pi-preview-2026-11-11)

That slow audit run does **not** overturn the sub-0.5 s fast-path results. Its
single association also does not establish continuous person-owned depth.
The optimized half-resolution preview uses current-frame instance-mask depth
and intentionally skips the full-resolution frustum association. Its output
cannot be passed unchanged to the current Shadow bridge, which expects
current target confirmation and 3D association. The COCO segmentation model
and the Shadow RGB v3 detector also have different provenance.

## Next implementation boundary

The local candidate now uses the optimized capture, matching and mask-depth
path for the Shadow stream. It binds a same-pixel v3 detection to one COCO
instance without treating a box as a mask; the mask, stereo, track, front
profile and both driver stamps must match the same cycle. The bridge checks
age at its decision and journals the original driver stamps. The separate
`run_shadow_fast_preview.sh` launcher cannot execute motors. See the
[candidate detail](../diagnostics/shadow_uart_candidate_20261029.md#fast-shadow-input-local-candidate-2026-11-11).

Local contract tests passed before the first Pi run. The combined v3 + mask
age, reacquisition and two-person cases still require live evidence before
changing the motor boundary. An old full-resolution audit speed benchmark
and another straight-wheel proof are unnecessary.

**First combined Pi run, 2026-11-12:** 117 camera cycles, all PAUSE. The
selected one-person COCO target appeared in three intervals separated by
two 15-cycle losses. In all 87 selected cycles v3 reported two boxes, so
there was no accepted v3-to-mask match or Shadow track. Both driver-to-decision
ages had about 668 ms median and exceeded 500 ms on every cycle. This is a
new combined-path blocker, not a contradiction of the older mask-only
sub-500 ms results. [Per-cycle findings](../diagnostics/shadow_uart_candidate_20261029.md#first-fast-shadow-pi-camera-only-journal-2026-11-12)

**Second combined Pi run, 2026-11-12:** The 43 camera cycles from
`shadow_fast_20261112_124510/bridge.jsonl` were all PAUSE. In 26 cycles the
segmentation model found no person; in 17 it found one. The v3 detector
reported a left-edge box in all 43 cycles, including all 26 no-person cycles,
and an additional box over the segmented person in 15 cycles. The left-edge
box cannot be declared a false positive without inspecting the RGB image.
Its presence makes the strict v3-to-mask binding unavailable. v3 inference
alone had a 490.44 ms median; driver-to-bridge-decision median was 551.13 ms
for AR and 566.01 ms for OV. Every cycle exceeded 500 ms for at least one
camera. No two-person scene was tested.

**Next isolated test:** `vision_core/tools/run_shadow_fast_mask_preview.sh`
uses the already reviewed COCO instance mask as the sole semantic source,
with its own explicit source marker. It retains exact RGB/mask/stereo hashes,
one-instance and confirmed-track requirements, front geometry, driver stamps
and the 500 ms decision-age gate. It cannot execute motors. The v3-plus-mask
launcher stays strict and separate. The operator currently has one person
available. The next live scene can check loss and reacquisition with that
person; a two-person live scene remains pending until it can be staged.
Existing reviewed offline two-person evidence is recorded in
`PROJECT_CONTEXT.md` and does not substitute for this live test. Record one
RGB still of the left camera edge to explain the persistent v3 box. A fast
mask-only age remains a diagnostic
result, not a certified exposure-to-command age.

**First isolated mask-only Pi run, 2026-11-12:** The bridge journal
`shadow_fast_20261112_130853/bridge.jsonl` has 115 camera cycles. Both
driver-declared ages to the bridge decision stayed below 500 ms in every
cycle: medians 257.80 ms (AR) and 262.27 ms (OV), maxima 289.03/284.86 ms.
The host processing median/max was 261.77/309.38 ms. The segmentation model
found exactly one eligible instance throughout, so this run did not test loss,
reacquisition or two people. Mask depth was coherent in 26 cycles, exceeded
the unchanged MAD gate in 88 and had multiple/no plausible layers in one.
Every decision remained PAUSE because the new mode failed to request RGB
payload and individual mask hashes from the worker; this is an integration
error, not a failed latency measurement. The next commit enables those hashes
for both Shadow semantic modes. Recheck the complete binding on Pi before
drawing any conclusion about FOLLOW_ELIGIBLE or motion.

**Second isolated mask-only Pi run, 2026-11-12:** The bridge journal
`shadow_fast_20261112_133000/bridge.jsonl` has 122 camera cycles. All 122
driver-declared ages to the bridge decision were within 500 ms (AR median/max
252.20/287.93 ms; OV 249.84/282.68 ms). The first 19 cycles had one eligible
person, cycles 20–71 had no eligible non-edge target, and cycles 72–122 again
had one. This supports an observed disappearance and return, without asserting
the operator's exact movements from the journal alone. The bridge emitted 21
`FOLLOW_ELIGIBLE` *diagnostic decisions* and 101 PAUSE. Of 64 cycles with a
current eligible target and masked-depth audit, 21 passed the unchanged MAD
gate and 43 did not. The 21 eligible decisions were split across 13 track IDs;
the longest uninterrupted eligible run was five cycles. Nine diagnostic ranges
were over the separate 2.0 m activated Measurement limit. Two same-track
range jumps were 2.041→1.032 m in 0.272 s and 1.571→2.289 m in 0.267 s.
The journal does not explain their physical cause. No motion was executed,
and these results do not establish stable target-owned range or safe floor
following. Inspect source frames and full per-cycle diagnostics before
considering a continuity policy; do not adjust MAD or activate motors from
these results.
In this saved bridge journal, the no-target rows were incorrectly labeled
`V3_MASK_BINDING` by a preview fallback even though v3 inference was absent.
The old summary also counted missing v3 data as zero boxes. These bookkeeping
fields have been corrected for future journals; the decisions and driver
timestamps above are unaffected.

**Full-audit follow-up:** The 124 saved upstream JSON records show that the
fast front `p05` is computed over every mask-owned depth point, whereas the
MAD gate checks only the one selected plausible depth layer. Ten of the 21
bridge-eligible cycles contain another supported layer, and 14 use more front
points than candidate-layer points. Cycle 19 has 321 near-layer samples at
1.282 m and 2529 far-layer samples at 2.383 m; the all-point front p05 is
1.045 m. Cycle 96 shows a large p05-versus-median gap even with one reported
layer. All 124 upstream previews remained STOP. The full evidence, source
hashes, examples and limits are in the [front-layer audit](../diagnostics/shadow_fast_front_layer_audit_20261112.md).
The front range does not yet satisfy selected-layer ownership, so fast-mask
motor execution remains disabled. No fresh Pi run is needed to establish this
particular mismatch.
The adapter now refuses the current all-mask front point set with
`FRONT_LAYER_BINDING_UNVERIFIED`; replaying the 122 archived bridge cycles
produced 122 PAUSE. This is an offline fail-closed check, not a new Pi run.

**Next camera-only evidence run:** Enable `SHADOW_CAPTURE_SNAPSHOTS=1` on the
mask-only launcher for one one-person enter/leave/reappear scene. Selected
cycles with a front-point-set disagreement, first coherent reacquisition, or
periodic sample retain the original RGB and stereo JPEG packets, exact
instance mask, disparity/LR arrays, calibration arrays and driver timestamps.
The launcher stops the camera process and drains the background writer before
verifying those bundles against the Bridge journal. This is new opt-in
instrumentation, not a repeat of the earlier speed proof. Compare its age at
decision separately, review the saved frames to explain the front layer and
range jumps, and keep all motor actions disabled. Two-person live evidence
remains pending because the operator currently has one person available.
See [source snapshot design](../diagnostics/shadow_fast_front_layer_audit_20261112.md#opt-in-source-snapshots-for-the-next-camera-only-run).

**Source snapshot Pi result, 2026-11-12:** The 75-second motor-off run saved
9/9 exact source bundles. All 296 camera cycles had driver-declared age to
Bridge decision below 500 ms; the AR0234/OV9281 medians were 176.76/175.51
ms and maxima 359.34/355.10 ms. One segmented person was present on cycles
1–48 and absent on 49–296; there was no post-exit reappearance. Thirteen
coherent mask-depth cycles formed five intervals with new track IDs after
MAD refusals; the same person did not become one uninterrupted track. All
decisions remained PAUSE. The all-point and numerical candidate front p05
differed by at most 29.2 mm in this scene, so the older two-layer mismatch
was not reproduced here. The archived source arrays exactly replay the Pi
depth/front calculations, but physical person ownership and optical datum
remain unverified. [Full run report](../diagnostics/shadow_fast_source_snapshot_run_20261112.md).

**Candidate-front adapter, 2026-11-13 (local candidate):** The Fast Shadow
adapter now reads `candidate_layer_front_geometry` instead of the all-mask
front p05. It requires matching current-cycle provenance, profile, selected
sample count, mask/track confirmation, unchanged MAD and driver-age checks.
The all-mask geometry remains in the audit and bridge journal for comparison.
This restores a *diagnostic* range path; numerical layer membership does not
establish physical person ownership or distance accuracy. Motor execution,
calibration and Measurement policy are unchanged. Eight focused adapter and
bridge tests passed. A contract smoke assembled from the saved cycle 44
candidate-front manifest and the adapter fixture produced 1.454037 m from
the candidate p05 of 1.466537 m and the existing 0.0125 m operator offset.
This is not a full replay of the original Pi audit or a physical ground truth
check. The producer code is unchanged; its focused test could not run here
because SciPy is missing from the local test environment. Next: one motor-off
Pi preview with one person, checking eligible/PAUSE decisions, layer range,
track continuity and driver age. Eligible rows do not admit motor motion.

**Fast Shadow Pi run, 2026-11-13 10:46:34:** The motor-off Bridge journal
`shadow_fast_20261113_104634/bridge.jsonl` (uploaded SHA-256
`76c19f3abdbd7bdbc677adbcdbd502a2c4689523142c175d8d4782ba314d702e`)
has 120 camera cycles in 31.83 s. One RGB instance was eligible in every
cycle. Thirty-five cycles produced diagnostic `FOLLOW_ELIGIBLE`; 85 stayed
PAUSE because mask depth exceeded the unchanged 0.05 m MAD gate. The 35
eligible cycles were split over 14 track IDs, longest consecutive run nine.
Adjacent RGB boxes had IoU at least 0.677, while the code discarded its
confirmed track after each MAD refusal. Candidate-layer and all-mask front
p05 differed by at most 0.000088 m; both camera driver-declared ages to
decision stayed below 0.5 s. Selected front ranges were 2.605–3.420 m,
within the 0.5–4.5 m Shadow diagnostic interval. The existing conditional
0.5–4.5 m static v2 profile must not be confused with the v1 activation
file still named by the Fast Shadow launcher. Neither profile authorizes
motion from this run.

**RGB continuity implementation, 2026-11-13 (GitHub commit
`76ce60a68caf803c9413534c7c7f196ede9ca141`):** In the mask-only Fast
Shadow path, a confirmed ID may continue across a MAD-only PAUSE when the
next unique current RGB box matches the prior box within 0.5 s using the
existing 0.20 IoU and 0.75 relative-centre criteria. The previous depth is
never reused. Target loss, ambiguity, geometry refusal or a processing error
clears the carried ID. The audit and compact Bridge journal distinguish the
RGB continuity ID from a stereo-confirmed ID. Offline traversal of the 120
recorded boxes, with their driver timestamp intervals, kept one RGB ID
through all 85 MAD refusals and assigned all 35 eligible cycles to it. This
is a logic replay, not a new Pi measurement or proof of personal identity.
The MAD gate, range source, calibration and motor-off launcher are unchanged.

**RGB continuity Pi result, 2026-11-13 11:58:49:** The motor-off Bridge
journal `shadow_fast_20261113_115849/bridge.jsonl` (uploaded SHA-256
`e34b62a172669cec8715d4833a5f93edec82a5cbfb274b7f327359a53098ff09`)
contains 97 camera cycles over 25.334 s and 656 polls without a new cycle.
Every camera cycle reported exactly one eligible RGB mask. The first two
cycles had MAD refusals before a track was confirmed. Cycles 3–97 used one
RGB continuity ID: 46 had fresh coherent masked depth and diagnostic
`FOLLOW_ELIGIBLE`; 49 had `DEPTH_MAD_EXCEEDED`, retained that RGB ID only,
and stayed PAUSE with no confirmed `track_id` or published `distance_m`.
The longest MAD-only PAUSE interval was cycles 49–80 (8.233 s from first to
last decision); cycle 81 resumed with the same ID and fresh depth. All 51
MAD refusals stayed PAUSE. The accepted diagnostic front distances were
2.514–3.096 m. Driver-declared camera-to-Bridge-decision ages had median/max
0.263350/0.294845 s for AR0234 and 0.267712/0.355332 s for OV9281, all
below 0.5 s. The journal has no source snapshot bundles, no observed RGB
disappearance/reappearance and no two-person scene; it demonstrates continuity
through depth refusals in this one-person scene, not identity across loss,
physical range accuracy or target ownership. Driver SOE is declared, not
independently verified exposure timing. Motor execution was disabled and
`FOLLOW_ELIGIBLE` did not authorize motion. Next MVP depth issue is the 51/97
MAD refusals, especially the 32-cycle pause; investigate source geometry and
a measured physical datum before revising the depth policy or enabling motion.

**Fast local depth in the motor-off Shadow preview, 2026-11-13 (code
candidate):** The mask-only launcher now requests the existing two-fold
same-cycle local disparity-plane diagnostic on every frame. The compact
Bridge journal records its exact source-frame/mask binding, fitted fraction,
held-out axial residual MAD/p90 and the separate `control_depth_quality_shadow`
status alongside legacy whole-person MAD. No local quality or physical-error
limit has been invented; the old `FOLLOW_ELIGIBLE`/PAUSE and motor-disabled
boundary are unchanged. A local replay of the nine archived source bundles
from `shadow_fast_20261112_193449_review.tar.gz` used the saved original mask,
LR-gated disparity, calibration and bbox-derived inner ROI. It reproduced all
nine recorded mask-depth statuses and candidate sample counts. In the two
MAD-refused snapshots, cycles 10 and 30, local held-out residual MAD was
0.035/0.027 m, while absolute residual p90 was 0.143/0.109 m. Thus a low
local MAD alone would conceal substantial tails in these frames. The local
fit took 0.020 s median, 0.080 s maximum on this workstation replay (nine
bundles); this is not Pi timing. Twelve focused existing tests, one Bridge
record check, Python compilation and launcher syntax passed. Next: one
motor-off Pi run of this exact path to observe per-cycle local coverage,
residual tails and driver age; compare against the global-MAD pauses before
designing a versioned shape-aware decision rule. Physical error and person
surface ownership remain unverified.

**Fast local Pi result, 2026-11-13 13:29:08:** The motor-off Bridge journal
`shadow_fast_20261113_132908/bridge.jsonl` (uploaded SHA-256
`b44daf3543514d22fc0f958dd081abcea9a6967757c775eb2e1df39502cc58a5`)
has 96 camera cycles over 25.221 s and 658 no-new-frame polls. Each camera
cycle had one eligible instance, one supported depth layer, an exact
current-cycle/mask-bound local surface assessment, and driver-declared ages
below 0.5 s (AR median/max 0.26167/0.30525 s; OV 0.26737/0.30101 s).
Forty-two cycles were diagnostic `FOLLOW_ELIGIBLE`; 54 remained PAUSE from
whole-mask `DEPTH_MAD_EXCEEDED`, all retaining the same RGB continuity ID.
Local held-out axial residual MAD median/max was 17.14/30.09 mm and absolute
residual p90 median/max 57.80/90.34 mm over all 96. The local MAD medians for
global-MAD pass/refusal were 17.0/17.6 mm, and p90 medians 58.3/57.2 mm;
they do not separate the two groups here. Cycles 79–96 were an uninterrupted
global-MAD PAUSE for 4.500 s from first to last decision; local MAD remained
11.0–12.9 mm on sampled cycles 79–81 and 95–96, and p90 40.1–47.9 mm.
The candidate-layer front p05 was still computed, but no distance was
published during PAUSE. No source snapshot or independently measured person
surface distance was retained in this run. These figures show the existing
whole-mask 50 mm veto is a poor sole *shape-quality* discriminator in this
one-person scene; they do not establish that the low-residual surface is the
person or that its range is physically accurate. Local residual limits cannot
be chosen from these pass/refusal labels as if they were ground truth. Keep
the motor-off boundary while building an explicitly local, source-bound
front-surface candidate and checking it against a physical datum.
