# V7 supervised bounded person approach

Experimental separate launcher; no canonical Measurement or autonomous admission.
Uses the existing static-v2 camera diagnostic, the V7 one-step observation checks
and the observed ESP32 stop trial. Nothing runs without the explicit `--execute`
flag. There is no interactive `APPROACH-...` code. Default is three steps;
the operator selects a positive step count within the declared front-clearance
budget. Each step requests 0.10 m and one /stop. Before any following step,
the runner requires confirmed terminal READY and zero PWM, positive progress
on both wheel encoders, and a per-command mean encoder travel of 0.01–0.15 m.
It then requires a fresh eligible frame of the same selected person box
(IoU >= 0.7). The final step is also reobserved. Optical depth change is
recorded but is not used as travel odometry or as the per-step progress gate.
The runner waits up to 60 seconds for the first eligible frame while the
camera pipeline starts. Later observations retain an 8 second timeout. If
no first frame arrives, it exits without a motor request. The first forward
step begins after detecting one current RGB person target, without a steering
request. The target bearing in the fresh frame immediately before that first
forward request becomes the reference, wherever it is in the image. Before
each later straight step, AR0234 target bearing is estimated from the selected
bbox center and the reviewed V5 intrinsic (fx=2021.546012, cx=924.649158).
The runner requests one bounded ESP32 turn if bearing changes by more than
1.5 degrees from that first forward frame, regardless of the person's
distance from the optical axis. It checks the change again on a fresh frame
after controller preflight. The 1–3 degree turn reduces displacement from
the original target coordinate; its sign follows the image coordinate and
controller turn convention. It requires terminal READY, zero PWM, valid encoder
direction, and then a fresh eligible camera frame of the same person. The new
image bearing must move at least 0.25 degrees toward the original coordinate
before the runner can request the next forward motion. Before every forward request, the runner reads ESP32
`/status`, then waits for a newly captured eligible frame of the same target.
The stop trial validates the supplied status without another pre-move GET and
rechecks the new frame's 500 ms driver age immediately before dispatch.
The supplied controller status must have been read after operator arming.
The firmware validates boot session, fault and busy state when it receives
the bounded request. A status failure, changed target, or stale frame stops
without a move request.
There is no turn after the final forward step and no blind turn retry.
The original ten-step run moved the bbox center from 982.7 to
683.8 px while the operator observed about 12 degrees of rightward drift.
Those per-step encoder headings were local command estimates, not a global yaw.
The turn sign and image response still need a short on-floor verification.
A refusal, missing frame, target change or unknown controller state ends the session
without a further move request. The summary records the last rejected frame
reason or camera-process exit on timeout, plus image drift and encoder counts.
Separate journals are written per command.

The approach camera session lasts at least 120 s and grows with the operator's
requested step count (`60 + 20 × max_steps` seconds); other preview launches
retain 30 s. The operator supplies `--front-clearance-m` from
a freshly measured horizontal front clearance. The value
must exceed 1.1 m plus 0.1 m per permitted step; that is an experimental request
budget, **not** a measured physical upper travel bound. Optical depth and this
clearance have different datums. There is no path-obstacle detection.
The operator must keep the person stationary and visible in the camera,
the approach path clear and the power cutoff accessible. Manual centering
on the optical axis is not required. No unattended operation.
The encoder progress envelope and steering checks are experimental, not
validated physical accuracy. Encoder distance is a wheel-motion proxy; slip
or calibration error can make chassis travel differ.
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

This separate operator-bounded runner checks current RGB instance binding,
static-v2 profile, target geometry and both driver-declared frame ages on
each frame and again before motion. Masked depth remains diagnostic and its
MAD status does not veto this RGB-coordinate steering experiment. The
original one-step launcher still requires coherent masked depth by default.
Neither path activates metric Measurement or autonomous motion admission.

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
observed straight travel of approximately 0.10 m and reported that the person
stood in place; the person's exact depth and pose were not measured.
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
change exceeded the fixed progress window, but the person's physical
position and pose were not independently tracked to centimetre accuracy. These data do not establish a camera error or actual chassis travel
from optical depth alone. The depth limit is not raised on this trial alone.

No turn was requested: bearing drift before step 2 was 0.991 degrees, below
the 1.5 degree trigger. The final center drift was about 1.65 degrees, but
this runner makes no turn after its final forward step. The approach trial
therefore verifies straight startup and bounded stops, but not steering.
Camera journal: `/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_113149`.

## Exploratory motor-off A-B-A check (2026-10-27)

With motors powered off, the operator made three 30-second camera-only previews
at nominal cart positions A, B (reported 0.10 m forward), and A again. The
operator later clarified that the person in the image also moved, returned
only approximately to the earlier location, and may have leaned or changed
pose. The target's physical depth was therefore not held fixed to centimetre
accuracy. The built-in summaries reported:

| Nominal cart position | Records | Coherent mask-depth frames | MAD refusals | Median candidate depth | Depth span |
| --- | ---: | ---: | ---: | ---: | ---: |
| A | 102 | 100 | 0 | 3.1484 m | 0.0326 m |
| B | 101 | 83 | 16 | 2.9497 m | 0.0720 m |
| A again | 105 | 94 | 0 | 3.1484 m | 0.0164 m |

The displayed A-to-B depth medians differ by 0.1987 m and the final A median
matches the initial A median at four displayed decimals. Because the person
moved and pose may have changed, this difference **cannot be attributed to
the 0.10 m cart translation** or used to infer a camera scale error. Returning
to the same displayed median does not remove that confounder. The B series
had more MAD refusals and a wider depth span; their cause is also unresolved.
This check is inconclusive for relative depth accuracy of the person-mask
estimator. The 0.01–0.15 m per-step gate remains unchanged. A controlled
follow-up needs a fixed target with independent distance references, or a
person constrained to a measured position and pose during both captures.
Return-position camera journal: `/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_122336`.

## Encoder-confirmed approach continuation (2026-10-27)

The per-step optical depth decrease of 0.01–0.15 m was an experimental
person-mask progress heuristic. The operator later clarified that the person
moved during the motor-off A-B-A comparison, so that check cannot establish a
camera scale error or a replacement optical tolerance. The runner now uses
per-command wheel encoder counts after a confirmed remote stop to check
bounded progress, while still requiring a fresh eligible observation and box
correspondence of the same target before another command. Optical depth values
and their differences remain in the audit as diagnostic observations. A zero
or excessive encoder result refuses further motion. The arbitrary ten-step
software cap was removed; the operator-declared clearance budget, explicit
step count, one 0.10 m request at a time, remote stop, camera freshness and
a camera-session timeout sized to the requested count remain. This does not establish an independent
physical stopping bound or autonomous motion admission.

For the next supervised steering check, use three requested steps only after
a fresh front-clearance measurement and with the person maintaining position.
The first step starts straight; a turn before a later step occurs only when
current image bearing both leaves the optical deadband and differs from the
first-frame bearing by more than 1.5 degrees. A turn still requires READY,
encoder direction and fresh camera improvement; it can refuse without a
further forward request. The first physical run with this encoder-based progress policy is recorded below.

## First-step refusal after encoder policy update (2026-10-27)

The next three-step run made zero movement and zero stop POSTs. The first
eligible target was observed at about 2.575 m, and a later fresh frame at
about 2.564 m passed the camera-age check. `GET /status` returned READY in
about 85 ms, but the next eligible frame arrived about 4.1 seconds later.
The host's two-second age cap on that earlier status caused
`Provided controller status is stale or unbound` before the first POST.
That cap is removed. The status must still be from after operator arming;
the camera frame must be fresh immediately before dispatch, and firmware
rechecks session, fault and busy state atomically on the motion request.
The child controller error is now copied into the approach summary. The following three-step trial tested this change with physical motion.

## Three-step supervised floor trial after status-age fix (2026-10-27)

Pi journal: `/home/elwis/.local/state/sie/evidence/v7_supervised_approach_1793111926_44d55c2a/summary.json`.
The runner reported `BOUNDED_SESSION_COMPLETE`: three forward POSTs, three
stop POSTs, and zero turn POSTs. Each controller trial reported observed remote
stop. Per-command mean wheel encoder progress was 0.0481, 0.0392 and 0.0422 m,
for a sum of 0.1295 m. The operator independently estimated 0.13–0.14 m on
the floor and about 1.5 degrees of rightward heading deviation. Camera target
bearing changed from +1.74 degrees at the initial observation to +0.92 and
+0.10 degrees before steps two and three, then -1.15 degrees after step
three. It moved toward and slightly across the optical axis, so no steering
request was eligible. The fresh camera ages immediately before the forward
commands were about 0.245, 0.233 and 0.240 s. The three preliminary ESP32
status ages at dispatch were about 0.405, 1.176 and 0.616 s.

This trial confirms the host age-cap fix allowed bounded forward commands and
that encoder progress roughly matched the operator's floor estimate. It did
not exercise a turn. The diagnostic Measurement policy remains STOP and no
autonomous motion admission changed. The next supervised steering test can
request more steps only with a newly measured clear approach path, stationary
person, clear rotation space and operator access to motor cutoff. It should
record turn request, terminal encoder turn, fresh image response and physical
heading after the session.

## Five-step continuation with target initially right of optical axis (2026-10-27)

The operator supplied the next `summary.json` and reported about 0.32 m of
floor travel with about 1.5 degrees of rightward heading deviation. The
runner reported `BOUNDED_SESSION_COMPLETE`, five forward POSTs, five stop
POSTs and zero turn POSTs. Per-command encoder mean progress was 0.0476,
0.0784, 0.0590, 0.0620 and 0.0695 m, summing to 0.3165 m. The initial
camera target bearing was +2.65 degrees; subsequent bearings before steps
two through five were +2.57, +1.99, +1.66 and +0.87 degrees, and the final
bearing was -0.14 degrees. The person's image moved toward the optical axis
throughout the run. At step five the displacement from the initial image
exceeded 1.5 degrees, but the current target was already inside the optical
1.5-degree deadband. No steering was requested, so turn direction and image
response remain physically untested. That recommendation to center the person is superseded below. The operator
must remeasure front clearance and keep the person stationary with a clear
path and rotation space.

## No-motion steering setup refusal (2026-10-27)

In the next seven-step requested run, the cart did not move and motors did
not start. The runner reported `STOPPED_ON_REFUSAL`, zero movement, stop and
turn POSTs, with `NO_FRESH_ELIGIBLE_FRAME_AFTER_STOP:rejected=40 last=ValueError:
LEGACY_MASK_QUALITY_REFUSED` while waiting for a new frame after the ESP32
preflight. The first accepted camera frame was cycle 9 at 3.1002 m with
bounding-box center x=814.25 px and bearing -3.13 degrees relative to the
reviewed optical axis x=924.65 px. It was not a centered steering start.

Pi camera journal:
`/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_180531`.
The operator's read-only extraction found 49 total frames: one unavailable,
47 with `DEPTH_MAD_EXCEEDED`, and only cycle 9 with coherent mask depth.
The rejected frames had candidate MAD about 0.063–0.096 m, above the
existing 0.050 m mask-depth threshold; cycle 9 had MAD 0.0482 m. The
single passing frame arrived before the controller preflight, and no later
frame met the quality gate in the eight-second wait. The refusal occurred
before any motor command, so this is not a controller-start failure.

Do not infer physical depth error from this mask spread alone. The earlier recommendation to manually center the person is superseded
below. Do not raise the mask MAD threshold based only on this one run.
Steering remains physically untested.

## Initial target coordinate as steering reference (2026-10-27)

The operator clarified the desired behavior: detect the person anywhere in
the camera field, start the first bounded forward step, then steer if later
observations show displacement from that person's original image coordinate.
No manual optical-axis alignment is required. The preceding five-step run
shows why the old two-condition gate was wrong for this task: the target moved
from +2.65 degrees toward the axis, while the cart physically drifted right.
The old gate saw an axis-centered target and never corrected the drift.

The supervised approach now binds the reference to the first fresh frame used
for forward dispatch. Later turns are requested from the signed bearing
change from that reference; turn feedback must reduce the same error. The
turn-only diagnostic retains its explicit optical-axis behavior. The runner
still requires one current RGB instance, exact current-cycle segmentation and
static-v2 profile binding, a fresh driver-declared frame, matching target boxes
across observations, operator-measured front clearance, encoder progress and
a confirmed stop after each bounded command. It no longer requires the legacy
masked-depth MAD status for this coordinate-based experiment. Candidate depth
and optical depth deltas remain in the journal as diagnostics when available;
they are not a motion or target-correspondence gate. The default one-step
launcher retains its prior depth check. Physical turn sign and image response
are still to be verified on the floor. This change is limited to supervised
operator-bounded requests and does not admit Measurement or autonomous motion.

## First coordinate-reference steering request: turn underreach (2026-10-27)

Pi journal: `/home/elwis/.local/state/sie/evidence/v7_supervised_approach_1793114646_3c37824a/summary.json`.
The operator requested up to ten bounded steps with declared 2.5 m front
clearance. The cart made two straight forward commands and two confirmed remote
stops, moving about 0.10 m on the floor by operator observation. Per-command
wheel encoder means were 0.0461 and 0.0630 m, summing to 0.1091 m. The first
forward-dispatch target bearing was -3.324 degrees. After the two stops the
same target moved left relative to that coordinate: before the third step its
bearing was -5.420 degrees, an error of -2.096 degrees. The revised controller
correctly requested `/turn-left` with a 1.0-degree upper bound before another
forward command.

Turn journal: `turn_03/summary.json` under the same root. ESP32 build
`v4_2_micro_turn_watchdog_v3` returned READY with zero PWM and terminal
`PARTIAL_PROGRESS`. A 1-degree request had only three target counts per wheel;
individual braking started at count one, and final settled counts were one
right and zero left. Encoder heading was +0.195 degrees, and the operator
reported no visible chassis turn. The target bearing changed only from
-5.288 to -5.233 degrees after the turn, an approximately 0.055-degree
improvement, below the runner's 0.25-degree feedback requirement. The runner
stopped with `TURN_CAMERA_ALIGNMENT_UNCONFIRMED`; no third forward command
was sent. This establishes that the 1-degree request was ineffective in this
specific floor trial, not that turn direction or an effective 3-degree request
has been physically validated. Next isolate a supervised 3-degree turn-only
diagnostic without forward motion, then use its encoder, camera and floor
observations to choose the next steering command size. Do not weaken the
camera feedback requirement solely to pass this near-zero turn.

## Isolated 3-degree left-turn fault (2026-10-27)

The follow-up turn-only run sent zero forward POSTs, one `/turn-left` POST and
one `/stop` after terminal refusal. Main and child reports were supplied by
the operator as `summary.json` and `turn_01_summary.json`. The terminal ESP32
status reported `FAULT`, latched reason `BREAKAWAY_NOT_FOUND`, and zero PWM.
It used firmware build `v4_2_micro_turn_watchdog_v3`. The three-degree request
had eight target counts per wheel and brake-start counts of two. Right final
count was six; left final count was -1. Right breakaway was detected at PWM
180; left breakaway was not detected (`left_breakaway_pwm=0`). The encoder
heading 0.0168 rad (~0.96 degrees) is not a validated physical chassis turn
because the left wheel did not establish the expected motion. The turn was
not accepted as a safe correction, and no forward command followed.

This is an actuator/encoder-start fault rather than a camera refusal. The
source's bounded breakaway PWM ceiling for this request is computed from the
two-count brake start: `180 + 2*5 = 190`, while the source comment records
loaded breakaway at PWM 215–225. That mismatch is a concrete software
hypothesis for the left non-start; wheel resistance, electrical delivery and
encoder direction remain alternatives until checked physically. The
controller fault is latched. Keep motion trials stopped; inspect the left
wheel, motor power path and encoder with motor power off before deciding
whether firmware needs a revised bounded turn startup procedure. Do not
simply raise PWM or clear the fault and retry in the person-approach run.
