# Step 75.4 — Robot geometry and footprint v1

**DECISION — 2026-11-28.** Close Step 75.4 as
`ROBOT_GEOMETRY_AND_FOOTPRINT_MOTOR_OFF_QUALIFIED` for the surveyed cart
configuration and isolated static TF. The footprint and transform are
practical MVP inputs for motor-off integration. See the Pi evidence
and scope below.

**FACT — operator survey, 2026-11-28.** The current cart's body datum is
the midpoint of the drive-wheel axle, with X forward, Y left, and Z up.
The axle midpoint is 0.035 m above the floor. The original chassis
front measurement was 0.0255 m ahead of the axle. The front and side
photographs were reported by the operator to show the drive wheels
extending beyond the aluminium chassis. The wheel-inclusive front
extent is 0.034 m, derived from the measured 0.068 m wheel diameter.
The rear extent is 0.2645 m. The 0.320 m width was measured between
the outermost drive-wheel edges, 0.160 m to either side. The resulting
candidate length is 0.2985 m. Survey uncertainty is unknown.
[Versioned diagnostic profile](../../ros2/sie_ros2/config/diagnostics/robot_geometry_footprint_v1.json)

**FACT — physical footprint.** The surveyed polygon uses the ground
projection of the axle midpoint as its plan-view origin and is expressed in
the `base_link` XY coordinates. Its vertices in metres are
`[[0.034, 0.160], [-0.2645, 0.160], [-0.2645, -0.160], [0.034, -0.160]]`.
The vertices are counterclockwise, enclose the origin, and give an area of
0.09552 m². Physical extents belong to the geometry profile. Navigation
padding, inflation, and stopping allowances require separate evidence and
configuration. The operator photographs indicate that the rear caster and
installed equipment fit within the candidate. Coverage of the full caster
swivel envelope is visually assessed and has no independent measurement.

## Frame datum and TF ownership

**FACT — source review.** A code-only Graphify extraction of the tracked ROS
package and relevant geometry code at
`010bca91de0d350a27ec102cabaa235e22ce4486` scanned 74 source files.
Graphify traced the odometry node to `WheelOdometry` and its publication
path to `ros_messages`. Direct source inspection verified the frame
names, transforms, and publisher conditions.
[Wheel odometry node](../../ros2/sie_ros2/sie_ros2/wheel_odometry_node.py) ·
[Planar wheel model](../../ros2/sie_ros2/sie_ros2/wheel_odometry.py) ·
[Optional LD19 launch](../../ros2/sie_ros2/launch/sie_pipeline.launch.py)

| Edge or frame | Source and interpretation | Proposed diagnostic authority |
| --- | --- | --- |
| `odom → base_link` | The existing wheel adapter integrates differential-drive motion about its modeled axle midpoint in XY. It publishes matched odometry and TF with Z = 0 and yaw-only orientation. Interpreting `odom` as the plane through the axle midpoint at initialization is consistent with this leveled planar model. The adapter does not establish the floor height or independently establish the installed physical `base_link` origin. | Existing wheel adapter only, through its opt-in TF path. It is unnecessary for the isolated static check below. |
| `base_link → ld19_sensor` | The operator's 2026-11-28 correction places the scan centre 25 mm behind the axle, at X = −0.025 m and Y = 0.000 m. The scan plane is +0.222 m above the floor, so Z = 0.222 − 0.035 = +0.187 m in the proposed axle-centred frame. The operator checked the installation horizontal. RPY = [0, 0, 0] is nominal alignment for the practical MVP. | One isolated `tf2_ros/static_transform_publisher` named `sie_step754_ld19_candidate_tf`. |
| Floor projection | The floor is Z = −0.035 m in the candidate axle-centred frame. The footprint uses only XY. | No additional TF edge is needed for this diagnostic. |

**FACT — prior orientation evidence, 2026-11-25.** With a cardboard target
approximately 0.815 m ahead, the captured RViz2 observation showed forward
on +X, left on +Y, and right on −Y. Zero scan angle aligned with forward
for the practical MVP. The tracked driver arguments for `ld19_sensor`,
CCW scan direction, and disabled angle crop were unchanged from the
recorded integrated-launch source. Reuse the qualitative orientation
result while the installed mount and orientation-relevant driver settings
agree with that record. Repeat the orientation experiment only if a change
is identified. [Historical experiment and launch provenance](../../ros2/sie_ros2/README.md#evidence-and-acceptance-boundary)

**FACT — historical transform.** The optional `enable_ld19:=true` path
in `sie_pipeline.launch.py` starts the driver and publishes the historical
`base_link → ld19_sensor` transform at [−0.023, 0.000, +0.212] m with zero
RPY. Its Z value was based on the optical-window centre. The historical
translation and the candidate use different datum evidence. Preserve
both records and leave the historical publisher out of the candidate's
isolated graph. The camera calibration and separate optical and lens-rim
datums keep their existing provenance.
[Historical provisional transform](../../ros2/sie_ros2/README.md#optional-stl-19p--ld19-raw-sensor-bring-up-2026-11-25)

## Minimal motor-off validation

The standalone Pi diagnostic bundle reads the existing
`/sie/lidar/scan_raw` publisher on the operator-selected driver domain
and checks the candidate in a separate localhost-only domain 177. It
queries the active publisher's read-only parameter service and observes a
LaserScan header. The bundle compares the product, baud rate, topic, frame,
scan direction and crop settings to the 2026-11-25 baseline. It records
the device path and clock setting. Applying the historical orientation
evidence also requires operator confirmation that the mount and driver
implementation are unchanged. Missing data leave applicability
unverified. A changed setting or mount calls for review. The bundle
starts only one candidate static TF publisher and checks both lookup
directions and publisher cleanup. It neither starts a sensor driver nor
opens a serial device.

The following manual procedure exposes the checks for review.

1. Confirm the cart is stationary and traction power is physically
   disconnected. Record the operator's visual coverage assessment and
   the installed configuration. Review independently measured caster and
   equipment extrema only where needed to resolve a clearance or safety
   requirement.
2. Compare the active LD19 configuration with the 2026-11-25 record.
   Use the axle-centred Z convention and assess whether any intended
   consumer requires a separate floor-referenced datum.
3. In each of two Pi terminals use an unused, localhost-only diagnostic
   ROS domain. Confirm no incumbent publisher on `/tf_static` or `/tf`
   before starting the single candidate authority. An absent topic is
   acceptable at this point. Keep the combined LD19 pipeline and supervised
   launch off in the test graph.

   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=177
   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
   ros2 topic info /tf_static --no-daemon --verbose
   ros2 topic info /tf --no-daemon --verbose
   ```

4. Run the candidate publisher in the first terminal. This command starts
   the static TF publisher and uses the exact values in the diagnostic
   profile.

   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x -0.025 --y 0.000 --z 0.187 --roll 0.0 --pitch 0.0 --yaw 0.0 \
     --frame-id base_link --child-frame-id ld19_sensor \
     --ros-args -r __node:=sie_step754_ld19_candidate_tf
   ```

5. In the second terminal verify exactly one named publisher on
   `/tf_static` and one `base_link → ld19_sensor` transform with
   translation [−0.025, 0.000, 0.187] m and identity quaternion
   [0, 0, 0, 1]. Allow discovery to settle and record the resulting
   graph and lookup. A duplicate authority, unexpected frame, or
   mismatched value is a refusal. Stop the publisher and verify its
   disappearance from the graph.

   ```bash
   ros2 topic info /tf_static --no-daemon --verbose
   timeout 8s ros2 topic echo /tf_static tf2_msgs/msg/TFMessage \
     --no-daemon --once --qos-durability transient_local
   timeout 8s ros2 run tf2_ros tf2_echo base_link ld19_sensor
   ```

**FACT — local software verification.** Thirty-eight targeted tests
passed. The tests checked the wheel-inclusive footprint and candidate
command, read parameters and a LaserScan from a simulated driver,
verified forward and inverse lookup from one static authority, and
refused an occupied test domain. The test graph was clean after
shutdown. A direct message check confirmed odometry Z = 0 and yaw-only
orientation.

## Pi qualification evidence

**FACT — operator-reported Pi trials.** Both runs retain their original
`BLOCKED` result. Isolated static TF passed in both runs. Evidence is
stored under `/home/elwis/.local/state/sie/evidence/`. The private Pi
files were unavailable for independent inspection.

| Run | Static TF | Recorded disposition |
| --- | --- | --- |
| `step75_4_geometry_tf.w9eFf5` | `PASS` | `BLOCKED` because the LD19 driver was inactive. |
| `step75_4_geometry_tf.rCWlq5` | `PASS` | `BLOCKED` solely by the parameter read timeout. The operator reported all 12 evidence file checksums passed. |

The operator reported one active `sensor_msgs/msg/LaserScan` publisher,
`/siee_ld19` on `/sie/lidar/scan_raw`. The header frame was
`ld19_sensor`, the scan had 501 beams, and its reported angular
span was approximately 0 to 2π rad. The operator confirmed that the LD19
hardware, mounting, and driver implementation were unchanged since
2026-11-25.

**FACT — launch-argument provenance.** The operator inspected the command
line of the running publisher process, PID 5272. These six launch
arguments match the historical orientation test configuration.
[2026-11-25 driver and orientation baseline](../../ros2/sie_ros2/README.md#evidence-and-acceptance-boundary)

| Launch argument | Value |
| --- | --- |
| `product_name` | `LDLiDAR_LD19` |
| `port_baudrate` | `230400` |
| `topic_name` | `/sie/lidar/scan_raw` |
| `frame_id` | `ld19_sensor` |
| `laser_scan_dir` | `true` |
| `enable_angle_crop_func` | `false` |

The command line also showed `use_sim_time:=false` and
`port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
The ROS parameter services were advertised, but
`timeout 8s ros2 param get /siee_ld19 laser_scan_dir` returned exit
code 124. This establishes launch-argument evidence, with the topic and
frame corroborated by the live scan report. Effective runtime parameter
values were unavailable through the service. The timeout cause is unknown.

**DECISION — applicability.** The unchanged hardware and mount, matching
launch arguments, and live scan metadata support reuse of the 2026-11-25
qualitative orientation evidence for this motor-off MVP configuration.
Preserve the isolated axle-centred `base_link → ld19_sensor` edge with
translation [−0.025, 0.000, +0.187] m and nominal zero RPY. A mounting or
orientation-relevant driver change requires review. The historical
provisional transform remains a separate record.

**OPEN — qualification limits.** Full rear caster swivel coverage has
visual support only. The nominal transform carries no precision extrinsic
claim. A live `odom → base_link → ld19_sensor` chain at scan timestamps
and map quality remain to be checked in Step 75.5. Navigation admission
and powered motion remain disabled. Step 75.3d's backward-motion limits and
the v13 braking defect remain open.
[Task-Sufficient Accuracy](../../PROJECT_CONTEXT.md#task-sufficient-accuracy-over-component-level-precision) ·
[Hardware Independence and Capability-Aware Adaptation](../../PROJECT_CONTEXT.md#hardware-independence-and-capability-aware-adaptation) ·
[ES-005 Spatial Model](../architecture/SIE_SPATIAL_MODEL.md) ·
[Step 75.5 minimum integration plan](../handoffs/SIE_SPATIAL_NAVIGATION_HANDOFF_20261127.md#step-755-slam-toolbox-integration)
