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
uses the current target bearing for left/right choice, then the same one-time
operator confirmation, bounded-turn terminal observation and camera feedback.
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
