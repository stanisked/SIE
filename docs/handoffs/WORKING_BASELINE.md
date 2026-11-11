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

Reuse the existing optimized capture, matching and mask-depth path for the
Shadow stream. Bind the current RGB instance, mask, stereo samples, track,
robot-front distance and driver timestamp to one cycle; measure age at the
runner decision. Preserve a PAUSE/STOP on missing or stale evidence. Check
the moving/lost/reacquired target and person surface on Pi before calling
this a working follow loop. No new speed benchmark of the old full-resolution
audit is needed to decide this architecture.
