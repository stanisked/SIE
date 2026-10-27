# V7 supervised bounded person approach

Experimental separate launcher; no canonical Measurement or autonomous admission.
Uses the existing static-v2 camera diagnostic, the V7 one-step observation checks
and the observed ESP32 stop trial. Nothing runs without the explicit `--execute`
flag. There is no interactive `APPROACH-...` code. Default is three steps,
hard cap ten. Each step requests
0.10 m and one /stop; the next step requires a new eligible frame captured after
the previous terminal observation, the same selected person box (IoU >= 0.7),
and a 0.01–0.15 m decrease in optical depth. The final step is also reobserved.
The runner waits up to 60 seconds for the first eligible frame while the
camera pipeline starts. Later observations retain an 8 second timeout. If
no first frame arrives, it exits without a motor request. The first forward
step begins from the first eligible target frame without a steering request. Its target bearing becomes the reference. Before each later
straight step, AR0234 target bearing is estimated from the selected bbox center
and the reviewed V5 intrinsic (fx=2021.546012, cx=924.649158). The runner
requests one bounded ESP32 turn only if bearing changed by more than 1.5 degrees
from the first frame and the target is more than 1.5 degrees from the optical
axis. It checks both conditions again on a fresh frame after controller preflight.
The turn is 1–3 degrees toward the target; its sign follows the image coordinate
and controller turn convention.
It requires terminal READY, zero PWM, valid encoder direction, and then a
fresh eligible camera frame of the same person. The new image bearing must
improve by at least 0.25 degrees before the runner can request the next
forward motion. Before every forward request, the runner reads ESP32
`/status`, then waits for a newly captured eligible frame of the same target.
The stop trial validates the supplied status without another pre-move GET and
rechecks the new frame's 500 ms driver age immediately before dispatch.
The supplied controller status must also be no more than 2 seconds old. A
status failure, changed target, or stale frame stops without a move request.
There is no turn after the final forward step and no blind turn retry.
The original ten-step run moved the bbox center from 982.7 to
683.8 px while the operator observed about 12 degrees of rightward drift.
Those per-step encoder headings were local command estimates, not a global yaw.
The turn sign and image response still need a short on-floor verification.
A refusal, missing frame, target change or unknown controller state ends the session
without a further move request. The summary records the last rejected frame
reason or camera-process exit on timeout, plus image drift and encoder counts.
Separate journals are written per command.

The camera launcher uses a 120 s session for this approach runner; other
preview launches retain 30 s. The operator supplies `--front-clearance-m` from
a freshly measured horizontal front clearance. The value
must exceed 1.1 m plus 0.1 m per permitted step; that is an experimental request
budget, **not** a measured physical upper travel bound. Optical depth and this
clearance have different datums. There is no path-obstacle detection.
The operator must keep the person stationary, the cart facing them, and the
straight path clear with power cutoff accessible. No unattended operation.
The numeric progress window and steering checks are experimental, not validated accuracy.
The 500 ms driver-declared timestamp is not independent exposure-to-actuator
certification. Network/power failures can prevent /stop; firmware stopping
distance is not independently bounded. Stop if unexpected direction occurs.

On Pi after updating the branch, first verify `NO_EXECUTION` without `--execute`.
For an operator-authorized trial, with current ESP32 LAN address and fresh measured
clearance, for example:

```bash
bash vision_core/tools/run_v7_supervised_approach.sh \
  --execute --base-url http://10.13.32.229 \
  --front-clearance-m 2.5 --max-steps 3
```

This is not a route to bypass failed V7 checks: the original one-step
`check_observation` runs on each current frame and again after controller
preflight. The original one-step launcher remains available unchanged.

## Current turn-only diagnostic checkpoint (2026-10-26)

For a single alignment check, use `--execute --diagnostic-turn-only
--max-steps 1` with the operator's current ESP32 address and measured front
clearance. This mode exits before forward dispatch. The preliminary
`GET /status` uses a 1 s timeout and one 2 s retry only for `TimeoutError`.
Both read attempts are recorded in `turn_01_preflight.json`. An invalid,
non-READY or repeatedly unavailable status refuses the run without a turn
POST. After a successful status read, the runner waits for a newly captured
eligible frame, checks target correspondence and turn direction again, and
checks frame age before one possible turn POST. No motion POST is retried.

The most recent physical attempt timed out during the first 1 s status read
after 1.121 s, so it tested neither the turn nor firmware watchdog v3. The
read-only retry was then implemented and covered by seven local tests; no
later physical result is recorded. Inspect `summary.json`,
`turn_01_preflight.json`, the `turn_01` HTTP journal, controller terminal
status and observed chassis displacement together. Repeated status timeouts
need communication diagnosis before any change to turn power or forward scope.

## First-frame startup checkpoint (2026-10-27)

A five-step attempt with 2.5 m operator clearance stopped before any motor
request: `movement_posts=0`, `stop_posts=0`, `steps=[]`, and
`NO_FRESH_ELIGIBLE_FRAME_AFTER_STOP:rejected=0 last=NO_FRAME_RECEIVED`.
At the former 20 second initial timeout, the camera child was still waiting
for its startup `snapshot()`; the runner's shutdown signal produced the child's
`KeyboardInterrupt` traceback. This does not prove that the camera would
subsequently publish. The initial wait is now 60 seconds; a further zero-frame
refusal requires camera/capture diagnosis before another approach attempt.

## Forward status ordering checkpoint (2026-10-27)

The first two-step attempt after the longer camera startup wait selected a
valid first target frame at 3.1648 m, but sent no move or stop POST. ESP32
`/status` returned `READY` and the accepted firmware build; the subsequent
pre-move frame check refused with `DRIVER_AGE_AT_DISPATCH_EXCEEDS_500MS`.
The forward path now reads status first, takes a new eligible frame afterward,
and rechecks that frame just before the one bounded move request. The previous
read-then-old-frame ordering remains in the standalone one-step runner unless
its caller supplies a preflight status; its existing stale-frame veto remains.
The subsequent physical result is recorded below.

## Two-step physical checkpoint (2026-10-27)

With commit `62b1f58`, a supervised `--max-steps 2` trial at operator-declared
2.5 m front clearance sent two 0.10 m forward POSTs and two stop POSTs. Both
controller results were `REMOTE_STOP_OBSERVED_DIAGNOSTIC_ONLY`; the operator
observed straight travel of approximately 0.10 m and a stationary person.
Firmware resets encoder counts at each command start. Terminal counts were
44/38 for step 1 and 63/58 for step 2 (right/left), corresponding to about
0.041 m and 0.060 m using the firmware wheel factors, or about 0.101 m total.
These are wheel-based estimates, not independent physical measurements.

Camera mask depth decreased from 3.1648 to 3.1321 m after step 1 (0.0326 m),
then to 2.9641 m after step 2 (0.1681 m). The second decrease exceeded the
experimental 0.01–0.15 m per-step progress window and produced
`STOPPED_ON_REFUSAL: FINAL_DEPTH_PROGRESS_UNCONFIRMED` after both confirmed
stops. Camera cycles 17–20 remained around 2.9786–3.0081 m, so this was not
a single isolated depth frame. Mask-depth MAD rose from about 0.0326 m before
the second step to 0.0573–0.0868 m in those later frames. The optical depth
change is inconsistent with both the wheel estimate and operator travel
estimate; its cause is unresolved. The depth limit is not raised on this
trial alone.

No turn was requested: bearing drift before step 2 was 0.991 degrees, below
the 1.5 degree trigger. The final center drift was about 1.65 degrees, but
this runner makes no turn after its final forward step. The approach trial
therefore verifies straight startup and bounded stops, but not steering.
Camera journal: `/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_113149`.

## Motor-off manual depth A-B-A check (2026-10-27)

With motors powered off, the operator kept the same person stationary and
measured the camera at the initial cart mark (A), manually translated the
cart 0.10 m forward without turning (B), then returned it to the original
mark (A again). Each camera-only preview ran 30 seconds. The built-in summaries
reported:

| Position | Records | Coherent mask-depth frames | MAD refusals | Median candidate depth | Depth span |
| --- | ---: | ---: | ---: | ---: | ---: |
| A | 102 | 100 | 0 | 3.1484 m | 0.0326 m |
| B, 0.10 m forward | 101 | 83 | 16 | 2.9497 m | 0.0720 m |
| A again | 105 | 94 | 0 | 3.1484 m | 0.0164 m |

The A-to-B depth change was 0.1987 m for the operator-reported 0.10 m cart
translation. Returning to A restored exactly the prior displayed median.
Thus the discrepancy reproduces without motor actuation and depends on cart
position in this setup. The increased MAD refusals and span at B show poorer
mask-depth consistency there. The evidence does not identify whether stereo
disparity, person-mask surface selection, geometry, or another position-linked
factor caused the shift. The 0.01–0.15 m per-step gate remains unchanged; this
person-mask depth median is not yet a reliable travel odometer at this range.
Return-position camera journal: `/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_122336`.
