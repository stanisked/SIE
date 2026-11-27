# SIE ROS 2 phase 1

This package layers ROS 2 transport on top of SIE Data Contracts.

## Scope

`perception_contract_node` accepts a canonical JSON Measurement on
`/sie/perception/raw_measurement`, validates it, and publishes it to
`/sie/perception/measurement`.

`navigation_contract_node` consumes only that validated Measurement and emits
a non-executing recommendation on `/sie/navigation/decision`.

`supervisor_contract_node` publishes the final phase-1 state on
`/sie/supervisor/state`. It has no actuator client, serial port access, HTTP
transport, or motor command path. Every output declares
`motor_command_performed: false`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/SIE
colcon build --base-paths ros2
source install/setup.bash
ros2 launch sie_ros2 sie_pipeline.launch.py \
  config:=$PWD/ros2/sie_ros2/config/sie_pipeline.yaml
```

The first runtime adapter will publish the existing JSONL evidence as
`/sie/perception/raw_measurement`. It is intentionally not included in phase 1:
the legacy fusion runtime still has laptop-specific paths and an archived AR0234
v3 binding which must be refactored to v5 before being connected.

## Standalone wheel odometry diagnostic

`wheel_odometry.launch.py` is a separate read-only v15 UART launch. It
publishes `/odom`, diagnostics, and an opt-in `odom → base_link` transform.
See the [Step 75.3c guide](../../docs/diagnostics/wheel_odometry_v1.md)
for parameter provenance, exclusive ownership, continuity and the proposed
motor-off procedure.

## Optional STL-19P / LD19 raw sensor bring-up (2026-11-25)

`sie_pipeline.launch.py` now has an **opt-in, motor-off** LD19 extension.
Its default is `enable_ld19:=false`, so the three phase-1 contract nodes and
their behaviour remain unchanged unless the option is explicitly enabled.
The supervised motion launch is intentionally untouched.

When enabled, it starts exactly one `ldlidar_stl_ros2_node` with these
previously tested Pi settings:

- LD19 via CP2102 UART, 230400 baud, configured port (default `/dev/ttyUSB0`)
- `/sie/lidar/scan_raw` (`sensor_msgs/msg/LaserScan`), `ld19_sensor`, CCW, no angle crop
- a provisional `base_link → ld19_sensor` static transform:
  translation `(-0.023, 0.000, +0.212) m`; RPY `(0, 0, 0) rad`

The original LD19 driver is **external** to this package and was separately
built at `~/siee_lidar_ws` on the Pi. It is not vendored or installed by
`sie_ros2`. Source its overlay before the SIE overlay; check availability
before opting in. No serial device is opened when `enable_ld19:=false`.

### Operator preflight (stationary cart, drive power disconnected)

Stop the *manual* `ros2 run ldlidar_stl_ros2 ...` and *manual*
`static_transform_publisher` processes from prior interactive tests first.
Two publishers for the same sensor frame or competing serial readers are not
allowed. Do not change the cart's motor/Fault state as part of LiDAR bring-up.

```bash
source /opt/ros/jazzy/setup.bash
source ~/siee_lidar_ws/install/setup.bash
cd /path/to/SIE
source install/setup.bash

test -c /dev/ttyUSB0
test -r /dev/ttyUSB0 && test -w /dev/ttyUSB0
ros2 pkg executables ldlidar_stl_ros2
ros2 pkg executables tf2_ros
ros2 node list

ros2 launch sie_ros2 sie_pipeline.launch.py \
  enable_ld19:=true ld19_port:=/dev/ttyUSB0
```

For a stable CP2102 `/dev/serial/by-id/...` path, supply
`ld19_port:=/dev/serial/by-id/<actual-device-name>` after inspecting the
real symlink. Do not assume it exists or invent an identifier.

In another shell with the same ROS environment:

```bash
ros2 topic hz /sie/lidar/scan_raw
ros2 topic info /sie/lidar/scan_raw --verbose
ros2 run tf2_ros tf2_echo base_link ld19_sensor
```

If a driver package, port, or TF dependency is missing, leave the optional
sensor disabled and investigate the relevant dependency. The launch does not
perform a device/permissions preflight automatically.

### Evidence and acceptance boundary

On 2026-11-25 the operator confirmed on a Pi 5 with Ubuntu Jazzy:
LD19 USB CP2102 on the Waveshare VL805 (kernel `6.8.0-1065-raspi`,
**temporary workaround** for VL805 init failure on `1068`), raw scans
approximately 9.887 Hz, remote receive on a ROS 2 Lyrical laptop via
Tailscale/static peers, and RViz2 LaserScan display relative to `base_link`.
Physical 815-mm cardboard tests and left/right relocation agreed with X+/Y
orientation. The `(-0.023, 0, 0.212) m` transform was verified as
**published**, and the lateral/front scan orientation verified qualitatively.
Z is based on the *centre of the optical window*, not an independently surveyed
laser plane; RPY=0 is a first-order alignment, not a precision calibration.

### Integrated-launch acceptance, 2026-11-25 (operator evidence)

The operator performed the **combined** `enable_ld19:=true` launch on the Pi,
built from isolated worktree HEAD `8515f31dd9806a21993faffd7f89accaa2dcb1da`:

- `python3 -m py_compile` on the changed launch and test: PASS.
- `python3 -m pytest -q ros2/sie_ros2/test/test_ld19_launch.py`: 4/4 PASS.
- `colcon build --packages-select sie_ros2 --symlink-install`: 1 package PASS.
- Five processes started: three unchanged phase-1 contract nodes, LD19 driver
  (`ldlidar node start is success` and `communication is normal`) and one
  `static_transform_publisher`. Supervisor logged
  `phase 1: actuator bridge disabled`.
- On the Pi, the receiving `ros2 topic hz /sie/lidar/scan_raw` settled at
  approximately 10.000 Hz in the provided short sample; transient intervals
  0.000 to 0.433 s were recorded near startup.
- The ROS 2 Lyrical laptop, connected by Tailscale/static peers, received
  `/sie/lidar/scan_raw` at a final reported mean 9.779 Hz, with observed
  intervals 0.089 to 0.201 s in that short sample.
- Pi and laptop `tf2_echo base_link ld19_sensor` repeatedly returned translation
  `[-0.023, 0, 0.212]` m and identity quaternion; an initial
  `Invalid frame ID` message cleared when the static TF was discovered.

**Acceptance:** integrated launch, raw scan publication and cross-device
reception, and provisional TF are confirmed on the actual hardware.
Prior manual-launch RViz2/physical X+ and Y+/Y- tests are separate evidence;
RViz2 was not repeated with the new combined launch in the recorded run.
No prolonged delivery/reliability benchmark, independent laser-plane survey,
precision yaw calibration, collision-safe obstacle logic, fusion or odometry
was demonstrated. The log confirms the software actuator bridge is disabled;
physical motor-power disconnection was not independently re-confirmed in the
submitted launch output.

The raw `LaserScan` is diagnostic sensor evidence only. It is **not**
an admitted SIE canonical Measurement/Fact/World State update, obstacle-safety
decision, fused AR0234/OV9281 output, odometry, or SLAM result.
