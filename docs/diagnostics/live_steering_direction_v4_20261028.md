# V3 floor result and v4 steering correction

Recorded 2026-10-28. Operator evidence:
`/home/elwis/.local/state/sie/evidence/v7_live_steered_forward_1793187350_8e0ba8d8/summary.json`.
The v3 floor trial completed one 0.20 m bounded steered-forward command.
The operator measured about 0.20 m of floor travel and about 1.5 degrees of
leftward body turn. The camera placed the person 8.5–11.7 degrees right of the
optical axis during the command. Six steering updates were accepted: one zero
bias while STARTING, then five +20 PWM updates while DRIVING. A seventh update
was attempted after the command entered normal braking; the Pi runner observed
the successful terminal state. Final ESP32 status was READY, SUCCESS, no active
command, zero PWM, no bounded or independent-safety fault. Encoder travel was
right 0.2010 m, left 0.1939 m, center 0.1974 m; heading +0.0244 rad. The
encoder heading and operator's direction agree. The diagnostic did not admit
a canonical Measurement or autonomous motion.

V3 boosted the left PWM when the target was to the right, but its speed and
distance synchronization still aimed for equal wheel travel. Right travel
remained greater. The evidence establishes that v3 did not achieve the
requested steering direction in this floor trial; it does not isolate motor
variation from the controller's conflicting setpoint.

The versioned v4 candidate applies the camera bias to a desired encoder wheel
difference accumulated only over subsequent travel. At full bias, 0.18 m of
left-minus-right difference is requested per 1 m of center travel, equivalent
to about 7 degrees over a full 0.20 m segment on a 0.29 m track. If feedback
begins after the straight launch, the requested turn is smaller. Both the
speed and PWM synchronization terms use the error relative to that desired
difference. The preexisting bias boost remains. Straight commands still
request zero difference. V3 movement caps, independent motor guard, camera
feedback deadline and stop path are retained.

The v4 firmware compiled for ESP32 core 3.3.12; twelve dry-run Pi tests passed.
No v4 motor trial is recorded. Next: load v4, confirm exact build ID and
stationary zero-PWM state, then conduct one operator-observed 0.20 m floor
trial with measured clear path. Record target motion, body turn direction,
floor distance and summary. Do not tune gain or declare tracking successful
from compilation alone.
