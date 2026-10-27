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
forward motion. There is no turn after the final forward step and no blind
turn retry. The original ten-step run moved the bbox center from 982.7 to
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
