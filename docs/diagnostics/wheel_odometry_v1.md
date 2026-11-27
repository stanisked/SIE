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

**OPEN.** The private Pi evidence files were unavailable locally and the
read-only SSH probe timed out. No physical-log replay, Jazzy runtime test,
Pi deployment, or hardware test was performed.

## Proposed motor-off integration verification

**PROPOSAL — separate authorization required.** On Pi 5, confirm physical
traction power is disconnected and keep both wheels stationary. Stop
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

**OPEN.** Step 75.3d physical odometry validation requires separate approval.
The earlier unexplained ESP32 boot ID change remains open. The v13 braking
defect remains open for Step 76.
