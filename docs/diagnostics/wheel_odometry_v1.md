# Step 75.3c — ROS 2 wheel odometry and TF

**DECISION — OWNER_APPROVED.** Implement software-only wheel odometry for
Raspberry Pi 5 with ROS 2 Jazzy. Step 75.3b qualified the v15 encoder
telemetry motor-off. This adapter is an opt-in diagnostic ROS producer.
Navigation, Measurement, World State, and execution admission remain disabled.
[Step 75.3b evidence and limits](wheel_encoder_telemetry_v1.md#step-753b-motor-off-qualification-closure)

## Wheel parameters and model

| ROS parameter | Value | Provenance and use |
| --- | ---: | --- |
| `physical_right_wheel_diameter_m` | 0.068 | Operator caliper measurement, robot geometry |
| `physical_left_wheel_diameter_m` | 0.068 | Operator caliper measurement, robot geometry |
| `effective_right_wheel_diameter_m` | 0.0646 | Historical loaded controller scale from a 0.500 m floor experiment, provisional kinematics |
| `effective_left_wheel_diameter_m` | 0.0652 | Historical loaded controller scale from a 0.500 m floor experiment, provisional kinematics |
| `right_encoder_ticks_per_rev` | 205 | Historical controller scale for rising-A events |
| `left_encoder_ticks_per_rev` | 206 | Historical controller scale for rising-A events |
| `track_width_m` | 0.290 | Historical geometric track width, provisional kinematics |

**FACT.** Only the effective diameters enter displacement calculations. The
firmware's turn-specific effective width is outside this adapter's model.
Parameters are read-only for the process lifetime. A change requires a new
launch, journal, and integration origin. Every journal begins with the
actual geometry, covariance, sampling, and freshness parameters. An explicit
YAML profile is available through the launch `config` argument.
[Default ROS profile](../../ros2/sie_ros2/config/wheel_odometry_v1.yaml)

**FACT.** For consecutive samples in the same validated boot, each wheel
displacement is `π × effective diameter × signed tick delta / ticks per
revolution`. Center distance is the wheel average and yaw change is
`(right − left) / track width`. The exact constant-curvature increment uses
a chord of `distance × sinc(yaw_change / 2)` pointing at the midpoint
heading. The analytic zero-angle limit gives straight travel without
subtracting nearly equal cosines. Linear X and angular Z velocity use the
ESP32 monotonic sample interval. X is forward, Y is left, and yaw is
counterclockwise positive. The initial origin is the adapter's local planar
origin. The body datum and wheel scale still require physical validation.
[Jazzy Odometry message definition](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg)

## UART ownership, evidence, and time

**FACT.** The standalone launch uses `sie_wheel_odometry`, with a required
new private JSONL journal. It takes the existing
`~/.local/state/sie/v7_operator_one_step.lock` before opening the UART,
checks visible ownership before and after open, and holds kernel
`TIOCEXCL` until shutdown. Use the normal operator account and keep all
motion runners and independent probes stopped. Later motor integration
must share or replace this acquisition owner as part of its approved
transport design. There is no concurrent production UART consumer here.

Only argument-free `STATUS` and `WHEEL_ENCODERS_V1` are available. The
node verifies the v15 build and boot with STATUS at startup and rechecks
after a new boot is observed. Requests use the existing framing, CRC and
production encoder codec, distinct IDs, and at most one outstanding read.
Sampling defaults to 10 Hz with a 0.25 s transaction timeout. A failed
transaction latches the output off until restart to avoid sending another
request while a late reply could still arrive. The node journals exact
request bytes, written bytes, received bytes, decoded status/sample,
host monotonic request and receipt times, ROS receipt time and disposition.
Journaling is synchronous during this standalone diagnostic and its timing
under Pi load remains unqualified.

**FACT.** The ESP32 monotonic timestamp provides elapsed time within a
boot. Pi monotonic time measures the UART request bracket and age.
`Odometry` and matching TF share the ROS system-time stamp sampled at host
line receipt. It approximates the host receipt event and carries UART,
scheduler, and sampling latency. There is no ESP32-to-Pi clock offset
estimate or hardware clock synchronization. The node refuses simulated ROS
time, non-increasing stamps, ROS offset steps above 50 ms, and samples
with host age or request bracket above 350 ms by default. The device
interval limit is also 350 ms. Stale, absent, or invalid data emits no
fresh odometry or TF.

## Continuity, covariance, and ROS authority

| Input condition | Disposition |
| --- | --- |
| First sample or first usable sample after interruption | Establish a baseline with no odometry or TF output |
| Valid consecutive sample with usable timing | Integrate one interval and publish pose and measured-interval twist |
| Sequence gap or excessive device interval | Keep previous pose, discard the displaced interval, and use the current sample as a new baseline |
| Duplicate, conflict, out-of-order data, invalid timestamp, retired replay, or controller fault | Refuse the sample and clear the integration anchor |
| Counter overflow or sequence exhaustion | Refuse navigation data from that epoch while the production validator marks it terminal |
| Boot change | Keep previous pose, clear the integration anchor and reverify firmware/boot with STATUS before establishing the new baseline |
| Communication or journal failure | Latch output off and report the failure |

**FACT.** The production `EncoderSampleContinuity` instance spans boots and
rejects retired boot replay. Gap and reboot boundaries skip displacement
even if net ticks would be available. A discontinuity is sticky in
diagnostics and adds configurable pose variance. The pose carried into a
new baseline is an estimate with explicitly lost motion continuity. A
stationary consecutive pair can report zero measured wheel-derived twist.
An unavailable sample produces no synthetic twist.

Pose starts with positive planar variances and grows with wheel travel
and accumulated heading change. Each discarded continuity span adds
configurable variance in position and yaw. Twist variances are positive
and configurable. The unobserved axes use high diagonal variance. These
are provisional diagonal assumptions without a calibrated error model or
wheel/clock uncertainty propagation.

`/odom` uses `nav_msgs/msg/Odometry`, with pose in `odom` and twist in
`base_link`. `publish_tf` defaults to false. With it enabled, the same node
broadcasts the `odom → base_link` edge using the identical stamp and pose.
A process lease enforces one instance per operator account. A graph check
refuses another `/odom` or dynamic `/tf` publisher in the standalone
graph. This conservative restriction protects Jazzy, where the Python
receive metadata lacks the publishing GID needed to reliably attribute an
incoming dynamic transform. A foreign static transform assigning a parent
to `base_link` also latches refusal. Downstream static edges such as
`base_link → ld19_sensor` may coexist. DDS discovery is asynchronous.
Use an isolated graph and designate one publisher operationally.
[Jazzy rclpy receive metadata](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/subscription.cpp)
· [Jazzy TransformBroadcaster](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/tf2_ros/transform_broadcaster.py)

`/sie/diagnostics/wheel_odometry` emits `DiagnosticArray` with validity,
reason, boot ID, continuity disposition and loss count. Its stamp is a
diagnostic publication time. Downstream navigation remains disconnected.

## Software verification

**FACT.** A code-only Graphify extraction from clean baseline
`1a63017179e1b2cfc1d853d67716cfe418c8809b` indexed 675 files,
6,155 nodes and 18,636 edges. Graph SHA-256
`92be5c06b5ae87587664a25f82599c63b9dfbe309b367c316e0c2b13f0e4dab3`.
The UART ownership, frame handling and ROS launch findings were confirmed
against the tracked source. Jazzy interface semantics were checked against
the upstream Jazzy source. The installed development runtime is Lyrical.

**FACT — PASS.** 38 focused tests passed. Synthetic cases covered signed
wheel motion, near-zero curvature, immutable configuration, continuity
refusals, receipt-clock validity, serial ownership and framing, and both
TF modes in an isolated ROS graph with a pseudoterminal controller.
Python compilation, the Lyrical colcon build and installed launch argument
inspection passed.

**FACT — existing failures reproduced.** The broader suite reported
390 tests and 104 subtests passed, with three failures. The clean baseline
had 352 tests and 104 subtests passed, with the same three failures.
They concern a legacy remote-stop candidate source hash, an encoder-guard
order assertion, and a v4 shadow-turn terminal result.

```bash
source /opt/ros/lyrical/setup.bash
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest -q --tb=short ros2/sie_ros2/test/test_wheel_odometry{,_uart,_ros}.py
python3 -m pytest -q --tb=short ros2/sie_ros2/test firmware/esp32/tests \
  vision_core/tests/test_{run_shadow_uart,esp32_bounded_motion_contract,shadow_supervisor,shadow_edge_turn_v4,live_steered_forward,supervised_bounded_executor}.py
python3 -m py_compile ros2/sie_ros2/{setup.py,launch/wheel_odometry.launch.py,sie_ros2/wheel_odometry*.py,test/test_wheel_odometry*.py}
git diff --check
```

**FACT — software verification scope.** The private Pi evidence files were
unavailable locally and the read-only SSH probe timed out. Software
verification ended before physical-log replay, Jazzy runtime testing, or Pi
deployment. The subsequent Pi integration is recorded below.

## Motor-off integration verification procedure

**FACT — original smoke-check procedure.** Under separate authorization on
Pi 5, confirm physical traction power is disconnected and keep both wheels
stationary. Stop
motion runners and independent UART readers. Keep the ROS graph isolated
from other dynamic TF or odometry publishers. Use a new evidence directory.
The repo checkout supplies the existing firmware codec on `PYTHONPATH`.

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/SIE
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
colcon build --base-paths ros2 --packages-select sie_ros2
source install/setup.bash

export ROS_DOMAIN_ID=176
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
fuser -v /dev/ttyAMA0
ros2 topic info /odom -v
ros2 topic info /tf -v
evidence=$(mktemp -d /home/elwis/.local/state/sie/evidence/step75_3c_motor_off.XXXXXX)
ros2 launch sie_ros2 wheel_odometry.launch.py \
  uart_port:=/dev/ttyAMA0 publish_tf:=true journal_path:="$evidence/wheel_odometry.jsonl"
```

Run the launch only after the preceding inventory shows no competing
serial owner or relevant graph publisher. From another terminal on the
same isolated ROS domain, record odometry, TF, and diagnostic samples.
Confirm exactly one writer on `/odom` and `/tf`, compare messages with the
same stamp, verify `odom` and `base_link`, and check diagnostic validity and
`navigation_admitted=false`. At rest expect stable accepted counts and
wheel-derived zero twist on consecutive fresh samples. Confirm the journal
shows the first baseline and the actual sample intervals. Stop the adapter
and hash the evidence. No flash, reset, manual movement, or fault clearing
is needed for this smoke check.

## Step 75.3c motor-off integration closure

**DECISION — OWNER_APPROVED INTEGRATION, 2026-11-27.** Close Step 75.3c as
`ROS_WHEEL_ODOMETRY_MOTOR_OFF_QUALIFIED` based on the Step 75.3c.1
stationary integration retry with traction power physically disconnected.
The implementation was `e9820ad66442b9f5539b6a8f14b768d2447797ae` on
Raspberry Pi 5, Ubuntu 24.04.5 LTS, and ROS 2 Jazzy, with firmware
`v4_2_uart_shadow_v15_cumulative_encoder_telemetry`.

**FACT — operator-reported initial attempt, BLOCKED.** Evidence remains at
`/home/elwis/.local/state/sie/evidence/step75_3c_1_integration.4sj1ufww`.
Jazzy dependency checks and the build passed. The UART STATUS request timed
out before the adapter launched. The operator reported Pi battery depletion
and shutdown. UART subsequently recovered, and the observed ESP32 boot ID
changed from `E41B90553131A237` to `5144699EFD0B1E9B`. The precise physical
cause of the boot change and initial timeout remains unverified.

**FACT — operator-reported retry, PASS.** Evidence is at
`/home/elwis/.local/state/sie/evidence/step75_3c_1_integration.0mm3614j`.
Deployment mode was `VERIFIED_REUSE`. The retry used boot
`5144699EFD0B1E9B`. The private Pi artifacts and their SHA-256 digests
below were reported by the operator and were unavailable for independent
inspection.

| Qualification metric | Reported result |
| --- | --- |
| Qualification window | 32 s |
| ROS header span | 31.789687958 s |
| Matching steady-state odometry/TF pairs | 290 |
| Steady-state encoder samples | 290 |
| Total decoded encoder samples | 347 |
| Total UART attempts | 348 |
| Journal dispositions | `BASELINE` 1, `FIRMWARE_VERIFIED_AWAITING_SAMPLE` 1, `VALID` 346 |
| Total subscriber messages | 301 odometry, 301 TF, 303 diagnostics |
| Refusals | None |
| Stationary cumulative encoder totals | Right −27 ticks, left −17 ticks |
| Pose extrema | X, Y, and yaw all zero |
| Twist extrema | Linear and angular all zero |

| Measured stream | Rate (Hz) |
| --- | ---: |
| ESP32 samples | 9.091194781490616 |
| Odometry | 9.090998325677871 |
| TF | 9.090998325677871 |
| UART polling | 9.090911839630218 |

| Measured interval | Median (ms) | Maximum (ms) |
| --- | ---: | ---: |
| UART request to host receipt | 38.327366 | 39.169264 |
| Host UART receipt to odometry subscriber | 1.429258 | 1.961948 |
| Host UART receipt to TF subscriber | 2.1818625 | 2.801756 |

These latencies describe host transactions and message delivery. The physical
measurement epoch relative to the host and hardware clock synchronization
remain unverified.

**FACT — operator-reported shutdown and integrity.** Shutdown was
`SIGINT_CLEAN` with adapter exit code 0. Publishers were removed and no
UART owners remained. The operator reported that all 14 evidence file
checksums passed.

| Primary evidence file | Operator-reported SHA-256 |
| --- | --- |
| `qualification.json` | `74159e644f6d3dc89e0d11223a4a9a684b3778092d45fff68ce5c1606dfaa312` |
| `wheel_odometry.jsonl` | `1b13e97f2b18252ba1642583724011171cc6980e7a75270a4569ae4724825107` |
| `odometry_samples.jsonl` | `d3a7ce5e1bba1a604782f2e00cd17a8b22110b575577ac2193a86028fa527e93` |
| `tf_samples.jsonl` | `eb60a32ba399d3a57003869707f092ea9e850af7c388285e0147c4de84338ce5` |
| `diagnostics.jsonl` | `b87e82e6679839f0930c835458fbbde23d1860416887d71891cd1aa06e83ccad` |

**OPEN.** Physical displacement, heading accuracy, and powered-motion
telemetry performance remain unqualified. The physical wheel diameters are
0.068 m on both sides. The effective diameters of 0.0646 m right and 0.0652 m
left and the 0.290 m track width remain provisional for odometry. The earlier
unexplained ESP32 change from `3BBA9D5615754514` to `BCC97F6E304A8459`
remains open, separately from the boot change above. The v13 braking defect
remains open for Step 76. Navigation admission remains false.

## Step 75.3d proposal for independent physical odometry validation

**PROPOSAL — separate owner approval required.** Evaluate endpoint accuracy
during slow manual floor motion with traction power physically disconnected.
Use the existing configuration and the isolated diagnostic ROS graph.
Keep motion runners disabled and log the firmware identity, boot ID,
configuration, ROS graph, continuity, and stationary samples before and
after each trial. Stop and preserve evidence on an unexpected boot ID,
continuity loss, controller fault, or loss of a physical reference.

1. On a level floor, lay out orthogonal axes using a tape measure and floor
   marks. Use two fixed chassis marks with a measured baseline to determine
   heading, and measure their offsets to the drive-axle midpoint used for
   the comparison. Mark the floor projections before and after each trial.
   Repeat static reference readings to estimate position and heading
   uncertainty from marker placement, projection, and measurement resolution.
2. Repeat slow manual forward and backward straight traverses of about 1 m,
   and repeat slow manual left and right turns of about 90 degrees. Use the
   floor marks to guide the operator. Record the actual endpoint position
   and heading in each trial, including translation during turns. Record
   the floor surface, visible slip, and handling disturbances alongside
   the reference measurements.
3. Use each trial's stationary start to align odometry and the physical
   reference. Compare signed longitudinal and lateral displacement and
   wrapped signed heading change against the independently measured
   endpoints. Report per-trial signed errors and repeatability with the
   reference uncertainty. Preserve floor measurements, trial markers, ROS
   odometry and TF, encoder journals, and rejected trials with reasons and
   checksums.

Agree on trial counts and accuracy criteria before execution. Keep wheel
scales and track width fixed through the validation. Evaluating any later
parameter fit would require new reference trials. This experiment addresses
motor-off endpoint accuracy on the tested surface. Powered accuracy,
long-term drift, and navigation admission remain outside its scope.
