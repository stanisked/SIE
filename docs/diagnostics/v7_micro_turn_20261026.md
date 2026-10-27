# Short turn before the first forward step, 2026-10-26

The supervised V7 approach requested one right turn of 1.278 degrees for a
2.555-degree camera bearing. Firmware returned `PARTIAL_PROGRESS`, with target
counts 4/4, brake-start counts 1/1, and final settled counts 0/1. Its encoder
angle was -0.195 degrees. The operator heard a brief motor pulse and observed
no physical chassis turn. The next camera bearing was 2.477 degrees, an
improvement of only 0.078 degrees. The existing 0.25-degree camera improvement
check refused forward movement; movement_posts=0, turn_posts=1.

The ESP32 micro-turn reserve of six counts makes a four-count request start
braking at the first count. The next diagnostic is one bounded 3-degree turn
within the existing API and a fresh camera observation, without any forward
POST. It does not change firmware PWM, brake reserve, or the camera improvement
threshold. The requested 3 degrees are an experimental bound, not an assertion
that this chassis can achieve a 3-degree physical turn. If the result is again
`PARTIAL_PROGRESS` with negligible camera change, inspect the per-wheel brake
and settled counts before changing the firmware turn profile.

`run_v7_supervised_approach.sh --execute --diagnostic-turn-only --max-steps 1`
uses the current target bearing for left/right choice, then bounded-turn
terminal observation and camera feedback. `--execute` is the operator action;
the runner does not request an additional `APPROACH-...` code.
The diagnostic-only branch exits before the existing forward dispatch. Its
summary records the terminal state and encoder angle for comparison.

The 3-degree diagnostic returned `STARTING` on 33 consecutive status reads.
At controller elapsed 89, 2042 and 4133 ms, both PWMs stayed at 180, and
encoder counts stayed at 1/0. One status GET timed out after 598 ms; the
runner then posted `/stop` (202, pending). Later read-only status was READY,
`last_command_state=STOPPED`, both PWMs zero, no fault latch. The operator
heard motors for several seconds but observed no chassis turn. There was no
forward command in this session.

Source review found that `enforceBoundedEncoderLimit()` entered the
micro-turn output branch during `STARTING` and returned true on each loop.
That skipped `updateStarting()`, including its 450 ms breakaway timeout and
PWM ramp. The firmware fix lets STARTING reach that state handler while the
absolute encoder limit remains checked first. Build ID
`v4_2_micro_turn_startup_v2` distinguishes a flashed fix from the observed
build. This corrects the stuck state; it does not demonstrate that the motors
can physically turn under load or justify increasing PWM. Recheck compile and
flash separately, then observe a single operator-approved diagnostic before
resuming the approach series.

After flashing v2, a single 3-degree turn remained physically stationary.
The controller advanced to `DRIVING`, so the startup bypass was corrected,
but samples at 2552 and 5015 ms showed PWM 180/180 and counts 1/1. The
runner's five-second terminal deadline expired, it sent one `/stop` (202,
pending), and no forward POST was made. This identifies a second source path:
the micro-turn guard returned early during DRIVING before the common command
timeout and periodic encoder stall checks. Build
`v4_2_micro_turn_watchdog_v3` moves the deadline before that guard and routes
an unbraked micro-turn through the existing 100 ms progress check. The small
turn's target speed is below the generic stall-speed threshold, so its
unbraked wheels are checked explicitly. With the existing three-cycle stall
policy, a stationary turn should brake on lack of encoder progress instead
of holding directed PWM until host `/stop`. This is a code-path correction,
not proof of turn torque. Loaded PWM 180 may still be insufficient; inspect
electrical supply and mechanical resistance before raising it.

The next v3 turn-only attempt did not reach the ESP32 motion endpoint. A fresh
RGB target center at 1065.4 px gave a 3.984-degree bearing and a 3-degree
right-turn request, but the first read-only `GET /status` took 573 ms and
raised `TimeoutError` at the previous 500 ms client deadline. Its HTTP journal
contains no POST; `turn_posts=0`, `stop_posts=0`. Camera refusals in adjacent
frames were not the cause of this no-motion result. The status preflight is
now read before choosing a new camera frame. It has a one-second read-only
deadline, and the runner requires a newly captured, eligible frame after the
preflight, checks target correspondence and bearing direction again, then
checks frame age immediately before the one allowed POST. No motion request is
retried. A failed preflight or unavailable new frame still exits without a
motor command. This change addresses the measured network delay; it does not
validate the firmware watchdog or the turn's physical torque.

A subsequent turn-only run stopped before any turn request: the preliminary
`GET /status` timed out after 1.121 s at its 1.0 s deadline. The cart did not
move. The preliminary read now gets one read-only retry (1.0 s, then 2.0 s),
with both attempts recorded in `turn_01_preflight.json`. A failed or non-READY
response still refuses the session. After a successful read, the runner waits
for a fresh camera frame and repeats target and age checks before the single
turn POST. This does not establish reliable ESP32 communication; a repeated
timeout needs network/controller diagnosis rather than more retries.

The retry change is committed through `6578a71`; seven local tests passed.
No physical turn after that commit is recorded. To distinguish a communication
refusal from a firmware or traction result, inspect the run's `summary.json`,
`turn_01_preflight.json`, `turn_01/http.jsonl`, terminal controller status and
the operator's direct observation together. A zero `turn_posts` count cannot
validate watchdog v3 or physical turn capability. Keep the next check to one
supervised turn-only run; do not infer permission for a forward step.

## 2026-10-27 supervised 1-degree and 3-degree left-turn outcomes

After two confirmed short forward steps (~0.10 m observed on the floor),
coordinate-reference steering requested a one-degree left turn. Terminal
`PARTIAL_PROGRESS` showed target counts 3/3, individual brake at 1/1,
settled counts 1/0, and +0.195-degree encoder heading. The operator reported
no visible turn. The camera bearing improved only ~0.055 degrees, below the
0.25-degree feedback rule, so no third forward step was sent. Evidence root:
`/home/elwis/.local/state/sie/evidence/v7_supervised_approach_1793114646_3c37824a`.

The following isolated three-degree left-turn diagnostic sent no forward
POST. ESP32 terminal state was `FAULT` with latched
`BREAKAWAY_NOT_FOUND`; one `/stop` was acknowledged after the fault.
Targets were 8/8 counts, brake starts 2/2, final settled counts 6/-1.
Right breakaway registered at PWM 180, left breakaway did not register.
The firmware's bounded-start ceiling formula gives PWM 190 at a two-count
brake start, below its own source comment's loaded breakaway observation of
215–225. That is a specific explanation to challenge, not proof that the
left motor or encoder is healthy. The operator heard the motors energize but saw no chassis movement during
this three-degree attempt. With motor power off, the operator then checked
that both wheels turned freely by hand and the left motor and encoder
connectors were seated; no visible mechanical obstruction or loose connector
was found. Physical inspection with motor power off and an
electrical/encoder diagnosis are needed before any new motion test.
Do not treat the reported 0.96-degree encoder heading as a verified chassis
turn when one wheel did not break away. The fault remains latched in the
supplied terminal status; no automatic fault acknowledgement or approach
retry is authorized by this result.

## Wheels-lifted actuator isolation prepared (2026-10-27)

The operator confirmed that both wheels rotate freely by hand and motor and
encoder connectors appear seated, then confirmed that the cart can be securely
lifted with both driven wheels clear of the floor. A separate diagnostic
script, `firmware/esp32/tools/run_wheels_lifted_turn_diagnostic.py`, is
prepared for one three-degree left-turn request with no camera or forward
command. Its default path opens no network or motor. With explicit
`--execute --wheels-lifted`, it requires zero PWM, no active command, and either the exact latched
`BREAKAWAY_NOT_FOUND` fault or a clean READY state after a controller reboot.
It acknowledges only that fault when present, checks the same boot session
in READY, sends one bounded
`/turn-left` POST, observes a terminal status, sends one `/stop` even if
the turn POST is ambiguous, and checks zero PWM afterward. It saves the HTTP
journal and summary. No motion POST is retried, and the test grants no person
approach or firmware activation.

After securely lifting the cart, keeping the motor power cutoff accessible,
and arranging to observe both wheels separately, run from the Pi checkout:

```bash
python3 firmware/esp32/tools/run_wheels_lifted_turn_diagnostic.py
python3 firmware/esp32/tools/run_wheels_lifted_turn_diagnostic.py --execute --wheels-lifted --base-url http://10.13.32.229
```

The first command should report `NO_EXECUTION`. The second may clear the
specific latched fault before its one bounded turn. If a reboot has already
cleared the fault, it accepts only a clean READY status and does not send
`/ack-fault`. Any other state refuses without a turn. Record whether
each wheel physically rotated and in which direction, the `STAND_TURN` line,
the `Saved:` path, and the saved `summary.json`. A wheel that moves off-floor
but not under floor load points toward inadequate loaded start torque or
supply, while no observed left-wheel motion off-floor keeps motor drive,
encoder wiring and direction logic in scope. These are diagnostic
interpretations, not established causes.

## Wheels-lifted three-degree result (2026-10-27)

Evidence root:
`/home/elwis/.local/state/sie/evidence/wheels_lifted_turn_20261027_185847_5fb6a04d`.
The bench script acknowledged the exact prior breakaway fault, accepted one
three-degree `/turn-left` request and sent one `/stop`. Terminal status was
`PARTIAL_PROGRESS`, READY, no fault latch and zero PWM. Both individual
brake-starts were recorded at two counts; final settled counts were four
right and six left, versus eight-count targets. Encoder heading was 0.0338
rad (~1.94 degrees), an encoder-only estimate. The stop response was 200 and
the following status retained zero PWM, no active command and READY.

The operator heard a brief motor-on/motor-off sound but saw no wheel motion.
This visual observation and the nonzero encoder counts are not yet reconciled;
a short movement or drivetrain lash may be difficult to see, while encoder
noise or sign errors cannot be ruled out from this trial alone. In contrast,
the loaded floor attempt ended with left final count -1 and a latched
`BREAKAWAY_NOT_FOUND` fault. The differing encoder results suggest load
sensitivity, but do not alone prove inadequate torque or exclude an electrical
problem. Keep person approach paused. A repeat wheels-lifted observation with
visible marks on each wheel and slow-motion video can establish actual wheel
rotation and direction; only then choose a bounded loaded-start firmware
change or electrical repair. Do not infer a physical 1.94-degree chassis
turn from the stand's encoder heading.

## Marked-wheel repeat and startup correction design (2026-10-27)

The operator repeated the one-shot three-degree left turn on a securely
lifted cart with visible wheel marks. Both wheels visibly moved for less than
one second: right forward, left backward. Evidence was supplied as a new
`wheels_lifted_turn_diagnostic.v1` summary with terminal
`PARTIAL_PROGRESS`, READY, no fault, one turn POST and one confirmed stop.
Final settled counts were 4 right / 5 left from 8/8 targets, with individual
brakes recorded at count 2 on both sides. The controller's reported final
angle was 0.0304 rad (~1.74 degrees); this is an encoder estimate, not a
measured chassis yaw. Unlike the first unmarked stand observation, this
confirms physical motor directions and encoder response off-floor.

Combined evidence is now: both wheels and encoders respond unloaded, while
the preceding floor trial failed to start the left wheel and latched
`BREAKAWAY_NOT_FOUND`. A loaded-start deficit is plausible, consistent with
the 190 PWM cap for a 3-degree request versus the source's 215–225 loaded
breakaway note; battery delivery, floor traction and current motor condition
still need checking. Do not infer that simply increasing requested angle or
removing the camera feedback check fixes the floor behavior.

Proposed firmware correction for review, not yet implemented or flashed:
during bounded micro-turn STARTING, brake a wheel as soon as its own
brake-start count is reached; preserve that brake when transitioning to
DRIVING. Only while the leading wheel is braked should the lagging wheel be
allowed a higher bounded breakaway ramp, with the absolute per-wheel encoder
limit, command deadline, stall detection, active-brake settle and latched
fault behavior unchanged. A changed firmware build ID, compile-only check,
wheels-lifted observation, and then a single floor turn with no person in the
path are required before restoring coordinate-based approach. Motor supply
voltage under load and left driver output remain alternative diagnostics if
the revised bounded start still fails.

## Isolated loaded-start candidate prepared (2026-10-27)

The source for build `v4_2_micro_turn_loaded_start_v4` is staged in its own
sketch folder, alongside a compile-only helper. It implements individual
braking during STARTING, raises the lagging wheel's startup ceiling to the
existing 235 PWM limit only after the leading wheel is braked, and preserves
that brake when entering DRIVING. The stand-only diagnostic accepts this
build ID for a later unloaded test. The deployed v3 source and supervised
approach runner were not changed. Compilation, flashing, and physical tests
have not yet occurred; the candidate must not be treated as validated.

## Post-flash stand run identified old firmware (2026-10-27)

The operator reported a successful compile and upload, then ran the marked-wheel
stand test. Both wheels briefly moved in the expected directions, right
forward and left backward. Evidence root:
`/home/elwis/.local/state/sie/evidence/wheels_lifted_turn_20261027_192543_c1791e96`.
The saved initial, terminal, and post-stop statuses all report firmware build
`v4_2_micro_turn_watchdog_v3`, not the candidate
`v4_2_micro_turn_loaded_start_v4`. Thus this run did not exercise the new
startup logic. It ended PARTIAL_PROGRESS at 4/6 settled counts from 8/8
requested, with READY, zero PWM, one turn POST, and one confirmed stop.

The stand diagnostic now requires the exact v4 build ID by default and
refuses before any motor POST if v3 is still active. Verify the sketch, board,
port, and upload result on the computer and confirm the live `/status`
`firmware_build_id` before repeating the stand test. Do not use the v3
result as evidence for v4 or proceed to a floor/person approach test.

## V4 bench preflight refused by shared legacy allowlist (2026-10-27)

After the operator reported the v4 flash, the next wheels-lifted run saved
`/home/elwis/.local/state/sie/evidence/wheels_lifted_turn_20261027_194830_c8b2dd2b/summary.json`.
Its initial status confirmed `v4_2_micro_turn_loaded_start_v4`, READY,
zero PWM, no fault, and boot session `E7295D62E5B1FBA4`. No turn or stop
POST was sent. The refusal `TrialError: Unexpected firmware build` came
from the shared remote-stop preflight's legacy build allowlist after the
stand script had accepted v4. The preflight now accepts an explicit build set
for this stand-only caller; its default allowlist remains unchanged, so the
forward trial and supervised approach do not gain v4 admission. The next
stand test still needs an actual physical result.

## V4 wheels-lifted result and next floor isolation (2026-10-27)

Evidence root:
`/home/elwis/.local/state/sie/evidence/wheels_lifted_turn_20261027_195341_18329b6a`.
The v4 controller accepted one three-degree left turn. A terminal `GET /status`
timed out, so the stand script conservatively reported
`TRIAL_REFUSED_OR_UNKNOWN`; it still sent one `/stop`. The post-stop status
bound to the same command and boot session showed `SUCCESS`, READY, no
fault latch, zero PWM, settled counts 5 right / 6 left from 8/8 targets,
and encoder-estimated final angle 0.0371 rad (~2.13 degrees). The operator
observed both wheels physically rotate in the expected directions, right
forward and left backward, as in the previous v3 stand test. Both breakaway
PWM readings were 185, so the new higher loaded-start branch was not exercised
without floor load. The reported SUCCESS is firmware's threshold result,
not proof of a full physical three-degree chassis turn.

A separate one-shot floor diagnostic is prepared for a clear rotation space
without a person near the cart. It requires explicit execution and operator
assertions, the exact v4 build, a clean READY state, zero PWM, no fault latch,
and one boot session. It sends one bounded three-degree `/turn-left`, never
retries that POST, always attempts one `/stop` after dispatch, and journals
the terminal and post-stop states. It does not acknowledge a fault, request
forward travel, start cameras, or admit person approach. The operator must
observe actual chassis yaw and each wheel's behavior on the floor. If a turn
fails or status/stop confirmation is missing, stop testing and inspect the
journal before any repeat.

## Three-degree v4 floor turn underreached (2026-10-27)

Evidence root:
`/home/elwis/.local/state/sie/evidence/floor_turn_20261027_200212_f1a86665`.
The v4 floor diagnostic sent one bounded three-degree left-turn command and
one confirmed `/stop`; terminal and post-stop statuses were READY, zero PWM,
no fault latch, and `PARTIAL_PROGRESS` for the same command. Both wheels
reached their individual brake-start at two counts and settled at two counts
from 8/8 targets. Breakaway PWM was 180 right / 190 left, and the encoder
heading was 0.0135 rad (~0.77 degrees). The operator observed only a brief
jerk of the chassis, not a useful physical left turn. The higher loaded-start
ceiling did not activate: neither wheel lagged after the other reached its
brake point. This result does not validate camera-guided steering.

The next isolated diagnostic allows an explicitly selected six-degree request,
while retaining the same exact v4 preflight, one-turn/one-stop journal,
encoder limit, fault and timeout behavior. This larger request leaves more
encoder travel before predictive braking and permits the existing PWM ramp to
reach the previously observed loaded breakaway range. It is a diagnostic of
floor turn capability and gearbox lash, not a steering command or a change to
person approach admission. Run it once with ample clear rotation space and
no person near the cart, record physical yaw and both wheels, then inspect
its summary before any additional motion.

## Six-degree v4 floor turn produced visible yaw (2026-10-27)

Evidence root:
`/home/elwis/.local/state/sie/evidence/floor_turn_20261027_201205_b8474c18`.
One bounded six-degree left turn and one confirmed `/stop` were sent. The
controller ended `PARTIAL_PROGRESS`, READY, with no fault latch and zero PWM.
Targets were 16/16 counts; individual braking began at 10/10, and settled
counts were 10 right / 9 left. Breakaway PWM was 210 right / 215 left; the
encoder-estimated final angle was 0.0641 rad (~3.67 degrees). The operator
observed a physical left chassis turn and approximately 1 cm forward travel
of the right wheel and 1 cm backward travel of the left wheel. These coarse
physical estimates are consistent with a few degrees of yaw, but do not
calibrate the exact angle. The six-degree command did not achieve six degrees;
it did establish loaded turn capability at this larger bounded request.

The next isolated floor diagnostic accepts a selected five-degree request,
using the same exact v4 preflight, one turn POST and one confirmed stop. This
maps the smallest useful physical correction before any camera-guided
steering change. The result must be assessed from both the journal and a
physical observation. No person approach or Measurement admission follows
from this single turn.

## Five-degree floor trial interrupted by status timeout (2026-10-27)

Evidence root:
`/home/elwis/.local/state/sie/evidence/floor_turn_20261027_202218_db537383`.
The v4 controller accepted one five-degree left turn. The first terminal
status read timed out, so the diagnostic sent its one `/stop` before a
terminal result was observed. The stop POST returned 202 with
`stop_pending=true`; the immediate stop-status read also timed out. The
operator saw a small physical left turn and perceived weak motor output.
The resulting `TRIAL_REFUSED_OR_UNKNOWN` does not isolate motor torque,
because the host may have ended the turn early on the first read timeout.

A subsequent operator read-only `/status` confirmed the same boot session
`E7295D62E5B1FBA4` and command ID, terminal `STOPPED`, READY, no active
command, `remote_stop_pending=false`, no fault, zero PWM, settled counts
6 right / 4 left, and breakaway PWM 205 on each side. This establishes final
rest, not normal completion of the requested five-degree turn. The diagnostic
now retries only timed-out `GET /status` calls within its original five-second
terminal and two-second stop-confirmation deadlines. It still sends one
turn POST and one stop POST at most, and never extends a deadline due to a
read timeout. The next five-degree floor result must be assessed afresh; do
not infer camera-steering capability from this interrupted trial.
