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
