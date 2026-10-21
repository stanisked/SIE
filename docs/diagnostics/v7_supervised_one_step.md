# V7 operator-confirmed one-step experiment

Status: separate operator-authorized experiment, requested 2026-10-21. Not
automatic SIE Measurement/Decision/Action admission or actuator qualification.

`run_v7_supervised_one_step.py` composes the current live V7 diagnostic topic
with the existing observed-stop trial. It does not read saved audits to drive,
alter their STOP decision, activate local-quality thresholds or call the legacy
V6 perception runtime. The action source is the operator's explicit one-step
authorization; current camera evidence supplies a veto. No autonomous approach,
turn, repeated step or motion correction is implemented.

## Sequence

1. Default CLI opens no ROS, cameras, HTTP or motors.
2. `--execute` requires an explicit controller URL and operator-measured
   horizontal front-to-target clearance greater than 1.1 m. This is the nominal
   0.10 m request plus the 1.0 m working stand-off; it is not a validated stopping
   guarantee. The operator confirms a stationary target/cart, facing the target,
   clear straight path and accessible power cutoff by entering a generated
   one-use token. Local file locking rejects concurrent runs of this runner.
3. After confirmation, optionally start the existing static-v2 diagnostic
   launcher. Subscribe with volatile QoS and queue depth one. Both driver SOE
   stamps must be later than confirmation on the same local monotonic clock.
4. Require expected V7/AR/extrinsic/model/profile hashes, exact selected
   instance/mask/RGB/fusion/cycle binding, static gate PASS, existing coherent
   masked-depth status and recomputed current diagnostic prerequisites. Preserve
   every unknown physical/policy check and the original STOP decision in evidence.
5. Freeze one current audit. Write it with a SHA-256; perform existing ESP32
   READY/build/API/boot/zero-PWM preflight. Immediately before movement, recheck
   the same source and driver age <=500 ms. A slow preflight refuses without any
   POST. No stale source is silently replaced or retried.
6. One existing `/move-forward` request for 0.10 m. One `/stop` attempt scheduled
   300 ms from the movement POST start. No intermediate status request delays
   that dispatch. A movement transport error or Ctrl+C also attempts this same
   single stop. Neither POST is retried after an ambiguous response.
7. Require same-boot/same-command STOPPED, completed stop observation, READY,
   no fault/latch and zero PWM. Timeout-only terminal GET recovery stays inside
   the existing five-second deadline. Unknown/fault outcomes remain unknown;
   another action is never scheduled. Save the complete controller journal and
   summary, then terminate only the camera process group started by this runner.

## Launch on Pi

Stop other camera consumers and motion programs first. Use the existing Pi
environment, not system Python. Bring the operator-measured clearance to the
command; do not copy optical depth or the partial front-geometry known term.

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
read -rp 'Measured front-to-target clearance, metres: ' clearance
bash vision_core/tools/run_v7_supervised_one_step.sh \
  --execute --launch-cameras --base-url http://10.138.186.229 \
  --front-clearance-m "$clearance"
```

Controller address is DHCP and must still be the intended controller. Without
`--launch-cameras`, start the existing static-v2 launcher in another terminal;
only post-confirmation frames can be used. The runner waits at most 45 seconds
for one usable current observation, then refuses with no movement. Source topic
is local trusted-process diagnostic transport, not cryptographic sensor attestation.

Output: `~/.local/state/sie/evidence/v7_operator_one_step_<time>_<id>/` containing
operator authorization, frozen source audit, source hash, repeated source checks,
controller HTTP journal and both summaries. No image tensors are required.

## Limits preserved

- Driver SOE-to-host POST dispatch is checked. Independent physical exposure
  timing and age at ESP32 acceptance remain unestablished. The controller cannot
  enforce the host deadline with this API. A reply may add network delay.
- The stop deadline is a host dispatch target. A blocking movement POST can
  delay it; the existing transport timeout is 0.5 s and late dispatch is recorded.
  Physical braking time/distance is not guaranteed by this tool. SIGKILL, host
  power loss, process failure or unavailable network can prevent the stop POST.
- The 0.10 m value is a bounded firmware request, not a validated physical upper
  travel bound for this platform. Previous 21.3 mm post-brake encoder progress
  and 608 ms observation do not establish independent stopping bounds.
- Cart orientation, path clearance, stationary scene and front clearance are
  operator statements, not invented camera facts. No footprint/path detector,
  dynamic target qualification or physical error policy is added.
- Frame support/local residuals remain diagnostics. The operator trial does not
  resolve the twelve missing draft checks or promote depth to Measurement.

## Verification

32 offline tests pass across the one-step composition, observed-stop transport,
experimental freshness and current-input/draft binding. Includes stale-after-
preflight no-POST, pre-confirmation/future/missing clocks, mismatched source,
quality refusal, movement timeout/Ctrl+C stop attempt, reboot and invalid terminal
PWM. Four changed Python files compile and launcher shell syntax passes.
No actual ROS/Pi, camera, ESP32 or motor execution was performed during development.
