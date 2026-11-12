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
