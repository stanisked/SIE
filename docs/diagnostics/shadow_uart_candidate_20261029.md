# Shadow UART candidate, 2026-10-29

The separate `v4_2_uart_shadow_v1_candidate` image is an operator-supervised engineering candidate. The Raspberry Pi runner deliberately has no public motor `--execute` route. It requires a fresh associated RGB target, calibrated range from the robot front, and target continuity information that the current camera preview does not supply. This checkpoint does not authorize autonomous floor following or canonical Measurement.

The protocol keeps one `command_id` across bounded segments. A target loss or 700 ms steering feedback gap brakes to `PAUSED` with zero PWM; a sequenced `SHADOW_RESUME` on that same ID starts a further bounded segment. An explicit Shadow STOP ends the session. Terminal FAULT remains terminal. Encoder envelope and watchdog protections remain active. The provisional front range gate holds at 0.8 m and resumes above 0.95 m, without a measured physical stopping bound. A future associated LiDAR range can serve as fallback when OV9281 depth is unavailable; raw optical Z or unassociated range cannot.

Hardware evidence: with motors unpowered, the probe confirmed same-ID PAUSE/RESUME and READY. An early lifted-wheels attempt while motor power was off produced `BREAKAWAY_NOT_FOUND`. With motors powered and the wheels lifted, two approximately 0.02 m segments completed under one ID; both wheels stopped during PAUSE, rotated after RESUME, encoder drift while paused was zero, and STOP was confirmed. No Shadow floor trial has occurred.

The sketch is at `firmware/esp32/traction_control_v4_2_uart_shadow_v1/`. The UART facade, supervisor, range gate, runner and probes are in `vision_core/tools/`. Their host tests use an injected controller. The flashed hardware result is stronger evidence for PAUSE/RESUME than the host tests, but it does not validate person tracking, front clearance or an emergency floor stop.

Next: capture synchronized mask and stereo diagnostics on moving/lost/reacquired target frames, establish range from the robot front with age and association checks, then review the runner input and run a supervised floor trial with a physical cutoff and both wheels observed.

## 2026-11-11: bounded Shadow UART floor step

This is the first recorded floor step with the separate Shadow UART candidate,
not the first controlled floor motion of the cart. Earlier bounded-forward
tests reached approximately 0.10 m, the 2026-09-18 supervised person-approach
journal recorded three 0.10 m motor commands and re-observations, and the
October live-steering trials requested 0.20 m. Do not repeat a straight
distance trial merely to establish that the cart can drive forward and stop.

The operator restored the Pi boot configuration from `dtoverlay=uart0-pi` to
the installed `uart0-pi5` overlay. The earlier `UART_REPLY_TIMEOUT` was not
proven to be caused by that typo: a later raw STATUS returned 1344 bytes, then
three parsed STATUS replies returned HTTP 200 in about 0.12 s each. Ten more
STATUS calls passed at the runner's 0.6 s timeout immediately before motion.
The boot session was `180981887BBE5566`, firmware build
`v4_2_uart_shadow_v1_candidate`, state `READY`, and both PWM values zero.

With both wheels on the floor, motors powered, a clear forward path and the
operator at the motor-power switch, one UART `SHADOW_START` requested the
minimum 0.020 m segment under command ID
`shadow-floor-aee80d3b081016cf`. STATUS observed `STARTING` at zero travel,
then `DRIVING` at 0.0050 m left and 0.0049 m right. A subsequent zero-bias
heartbeat returned HTTP 409 `NO_MATCHING_ACTIVE_STEERED_FORWARD`; the one-off
probe classified this as an error and issued terminal `/stop`. STATUS showed
`BRAKING` at 0.0169 m left and 0.0148 m right, settling at 0.0179 m left and
0.0148 m right, then `READY`, `last_command_state=STOPPED`, and zero PWM. The
operator observed a straight floor movement of approximately 2 cm. The UART
journal is
`/home/elwis/.local/state/sie/evidence/shadow_floor_once_1794397743_820d6029.jsonl`.

The 409 is consistent with the short segment settling between a `DRIVING`
STATUS and the next heartbeat. The one-off journal did not capture the state
at the instant of rejection, so this timing explanation remains a hypothesis.
The permanent injected Shadow runner now reconciles this specific late
heartbeat refusal through read-only STATUS polling during `BRAKING` or
`COASTING` for at most 1.6 s. It accepts only if the same command and
transition reach `PAUSED` with zero PWM and no latched fault or pending remote
stop. Otherwise it requests terminal STOP. Eight host tests cover immediate
pause, braking before pause, and a mismatched command; no changed runner has
been deployed or tested on the cart. The floor result establishes one short,
operator-supervised forward step and terminal STOP. It does not establish a
person-following loop, a front-clearance measurement, an emergency stopping
distance, or general UART reliability under camera load.
