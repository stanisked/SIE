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

## 2026-11-11: lead/follower segment boundary in the host runner

For the intended turtlesim-like lead/follower experiment, the follower must
observe the leader again after each completed bounded segment. The Shadow
runner now retains the RGB and associated-range observation times that
authorized START or RESUME. When the same command reaches PAUSED after a
segment, RESUME requires **both** a newer RGB target and a newer associated
range. A cached pair can still be used for steering within its freshness
window during an active segment, but it cannot start the next segment. A
caller may set `max_segments` to a positive integer to end a supervised
session after that many attempted START/RESUME segments; the runner then
requests terminal STOP and records `SEGMENT_LIMIT_REACHED`. No public motor
CLI or live camera-to-runner adapter is added by this change.

The host tests cover stale-pair refusal, a fresh-pair resume, and a
single-segment limit. They use an injected controller, not the Pi cameras or
powered wheels. The existing audit reports optical depth and diagnostic
target continuity, but does not supply validated exposure age, persistent
person ownership, or robot-front clearance for `ShadowObservation`. The
operator-confirmed 0.210 m lens-rim setback and 15 degree mount pitch are
partial geometry only; the remaining optical-centre offset and physical
uncertainties are not filled in. Therefore this code change does not activate
automatic following or establish the proposed 0.8 m hold distance. The next
integration step is to verify a live, target-owned, fresh front-clearance
producer and its timing under camera load before connecting it to the runner.

## 2026-11-11: selectable RGB model in the live audit

The earlier Pi raw-pair capture stopped before opening the cameras because the
ROS observer pinned the v1 ONNX SHA while the installed `/home/elwis/best.onnx`
was the known v2 artifact. The observer and ROS node now accept a named
`model_version` (`v1`, `v2`, or `v3`) and verify that version's exact SHA-256.
The raw-pair wrapper selects the installed v2 by default and can select v3
with `SHADOW_RGB_MODEL_VERSION=v3` and `SHADOW_RGB_MODEL_PATH` pointing to a
locally installed v3 ONNX. It will still reject an absent or mismatched file.
This repairs a configuration mismatch; it does not claim v3 is on the Pi or
that the camera/ESP32 loop is integrated.

## 2026-11-11: live audit to Shadow UART bridge candidate

`vision_core/tools/run_shadow_live_follow.sh` starts the existing calibrated
dynamic AR0234/OV9281 audit and the new `shadow_live_bridge.py` in one terminal.
By default it prints each current range decision without opening UART. An
explicit `--execute` connects its observations to the existing Shadow UART
runner. The bridge accepts only a same-cycle confirmed track, known AR0234 v5
and OV9281 Pi v7 hashes, a finite 3D association, and an audit received within
0.5 s of the reported capture start. It does not reuse an old pair for RESUME.
It converts raw physical-left optical XYZ to the cart-front longitudinal frame
using the operator-reported 15 degree upward pitch and 0.210 m setback. The
optical-centre to rim forward component must be supplied as a separate input;
the previously reported approximately 0.0123–0.0125 m is an estimate, not a
surveyed physical error bound. This path is an experimental control input and
does not publish a canonical SIE Measurement or change calibration admission.
The minimum detector confidence for this experiment is 0.40, while a loss of
target, stale association, controller fault, or UART failure still stops or
pauses the cart. The runner shortens a requested segment near the 0.95 m hold
boundary; its firmware watchdog and terminal STOP are unchanged.

The standalone bridge cannot establish that the current Pi processing takes
less than 0.5 s, that the optical offset is accurate, or that a moving person
has reliable stereo surface coverage. A prior saved 19-cycle two-person replay
yielded two coherent depths in only one frame. The new code has passed host
synthetic adapter tests and 11 runner tests, not a Pi camera or motor test.
One short live preview will expose whether a current single-person stream
produces `FOLLOW_ELIGIBLE`; repeated old captures cannot establish this.

On the Pi, put this branch in a *separate* checkout, with the active cart
checkout untouched. Select v3 only after its ONNX file is present and its SHA
matches `fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5`.
An initial no-motor preview can use the explicitly labelled operator estimate:

```bash
SHADOW_RGB_MODEL_VERSION=v3 SHADOW_RGB_MODEL_PATH=/absolute/path/to/shadow_v3_best.onnx \
  bash vision_core/tools/run_shadow_live_follow.sh \
  --front-rim-forward-of-optical-m 0.0125 --duration-s 30 \
  --journal /home/elwis/.local/state/sie/evidence/shadow_bridge_preview.jsonl
```

For a floor session, use the measured offset and add `--execute`. The bridge
requires the user to supply that number explicitly. Its journal path must be
new for each execution. At present the Pi has not run this bridge, and the
motor-facing path has no physical validation.

### First Pi preview, 2026-11-11

The separate Pi checkout at `cca5ade` started the live cameras successfully.
Thirteen audit cycles arrived in approximately 30 s; all reported
`ASSOCIATION_UNAVAILABLE` and the bridge printed `RGB_TARGET_UNAVAILABLE`.
The latter was a lossy bridge display, not proof that RGB detection failed:
each cycle ran stereo association and full stereo matching. Host processing
was about 1.5–1.8 s, with parallel stereo matching about 1.0–1.2 s.
No UART device was opened and the cart did not move. The audit's specific
refusal reason was omitted from the bridge output, so the cause of the
association failure is still unknown. The preview now prints that original
reason, sample counts, depth distribution and capture-to-receive age. A short
follow-up preview can distinguish an out-of-range target from missing stereo
surface or a geometry mismatch. Even an associated frame from this current
pipeline would exceed the 0.5 s input age limit, so reducing latency is a
separate concrete implementation task before live motion.

### Four-cycle diagnostic range finding, 2026-11-11

The follow-up Pi preview at `a164cd3` showed one current, non-edge RGB target
in each of four reported cycles, with accepted candidate box geometry. Each
cycle had approximately 5,183–5,700 LR-consistent disparity samples in the
frustum, yet zero samples in the activated 0.5–2.0 m Measurement interval.
The stereo depth distributions had central values around 3.2–3.7 m. The
refusal `FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE` therefore explains this particular
run; it is not an RGB detector failure or evidence of zero disparity.
Capture-to-receive age was 1.39–1.66 s. No paired person surface or moving
robot-front clearance was established and no motors ran.

The live follow wrapper now opts into the already declared 0.5–4.5 m
**diagnostic** Shadow range for frustum projection, disparity sample selection
and clustering. The original activation and its 0.5–2.0 m Measurement range
remain untouched. The audit records both ranges and still emits no canonical
Measurement. The separate range may surface background layers or spatial
ambiguity; a successful association and its timing must be observed on Pi.
