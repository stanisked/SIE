# Step 75.5a — Isolated SLAM Toolbox mapping

**DECISION — diagnostic implementation.** Use one opt-in asynchronous SLAM
Toolbox mapper. Its occupancy grid and map-frame robot pose support inspection of
mapping and localization in the map being built. Saved-map relocalization,
mapping accuracy and navigation admission require separate evidence. Use the
qualified geometry for the surveyed cart. Keep the measured physical footprint
separate from navigation padding. The wheel odometry profile and its
backward-motion limitations carry forward. Navigation admission and powered
motion remain disabled. The v13 braking defect remains open.

[Step 75.4 geometry and applicability](robot_geometry_footprint_v1.md#pi-qualification-evidence) ·
[Task-Sufficient Accuracy](../../PROJECT_CONTEXT.md#task-sufficient-accuracy-over-component-level-precision) ·
[Hardware Independence and Capability-Aware Adaptation](../../PROJECT_CONTEXT.md#hardware-independence-and-capability-aware-adaptation)

## Dependencies and frame ownership

**FACT — source review.** Code-only Graphify indexed 57 ROS package files from
baseline `2a00bd80348e8d10d4577e886f354d30e24057c1`. It traced
`WheelOdometryNode` through the wheel model, ROS message construction and the
graph checks. Direct source review confirmed the wheel adapter's standalone
guard refuses another publisher on its resolved dynamic TF topic. Putting an
unmodified wheel adapter and a mapper directly on the same `/tf` therefore
does not work.

| Edge | Authority and route |
| --- | --- |
| `map → odom` | SLAM Toolbox broadcasts the mapping transform on `/tf`. |
| `odom → base_link` | The existing wheel adapter is the pose authority. Remap its TF topic to `/sie/step75_5a/wheel_tf`. The diagnostic forwards only this edge to `/tf`, with the original stamp and transform values. The adapter's private-topic exclusivity, existing UART lease and read-only protocol remain in force. A synthetic fixture substitutes for the adapter in offline modes. |
| `base_link → ld19_sensor` | A single static broadcaster uses the qualified translation `[-0.025, 0.000, 0.187]` m and nominal zero RPY from the Step 75.4 profile. Keep the historical LD19 static publisher outside the diagnostic graph. |

The launcher loads the installed distribution's async YAML and overlays
the diagnostic frames, scan topic, queue depth of one and wall-clock setting.
It also disables the interactive interfaces and map-saver convenience service.
The mapper retains the installed solver, range, map-update and travel thresholds.
Its scan subscriber's DDS QoS is checked directly against the active publisher.
Verify transforms at each original `LaserScan.header.stamp`, never by substituting
a latest-time query. SLAM Toolbox's own `map → odom` output may carry its normal
transform-timeout offset even with `restamp_tf=false`. The incoming scan header
continues to represent the LD19 driver's host ROS conversion time. Physical
LD19 acquisition time remains `UNKNOWN`.

[Standalone wheel authority guard](../../ros2/sie_ros2/sie_ros2/wheel_odometry_node.py) ·
[Async profile overlay](../../ros2/sie_ros2/config/diagnostics/slam_toolbox_async_v1.yaml) ·
[Jazzy launch](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/launch/online_async_launch.py) ·
[Jazzy mapper TF and scan handling](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)

## Pi dependency inventory

**FACT.** SLAM Toolbox was absent in the Step 75.2 Pi inventory.
An earlier unattended SSH probe could not authenticate. The operator
subsequently reported a successful Pi synthetic qualification on ROS 2 Jazzy
with SLAM Toolbox 2.8.5. The earlier local Ubuntu 26.04/ROS 2 Lyrical runs
are separate evidence. Recheck the Pi package before Step 75.5b.
[Pi synthetic qualification evidence](slam_toolbox_step75_5a_jazzy_attribution.md#pi-synthetic-qualification-closure)
[Jazzy package status](https://repo.ros2.org/status_page/ros_jazzy_default.html)

Run this read-only probe from the laptop with interactive SSH:

```bash
ssh -t elwis@100.70.100.29 'source /opt/ros/jazzy/setup.bash && { ros2 pkg prefix slam_toolbox; dpkg-query -W ros-jazzy-slam-toolbox; apt-cache policy ros-jazzy-slam-toolbox; }'
```

If the package is absent, the minimum isolated dependency installation is
`sudo apt-get install --no-install-recommends ros-jazzy-slam-toolbox`.
This is a dependency-only command. Use the same probe afterward to record the
actual installed version. SLAM Toolbox may bring declared dependencies.
The launcher starts no Nav2 nodes.

## Synthetic verification

On Jazzy, build the diagnostic receipt subscriber and pass
`--tf-audit-executable`; the retry bundle performs these steps automatically.
[Jazzy attribution and retry](slam_toolbox_step75_5a_jazzy_attribution.md)

Use a sourced ROS environment with SLAM Toolbox and an unused domain or
two distinct domains. From the SIE source checkout:

```bash
python3 tools/diagnostics/step75_5a_slam.py synthetic --domain 179
python3 tools/diagnostics/step75_5a_slam.py missing-tf --domain 180
```

These modes publish a generated 501-beam planar room scan and, in the positive
case, matching synthetic wheel TF. No suitable recorded scan/odometry pair
was found locally. Both modes use the actual mapper, hold the qualified static
edge fixed and verify lifecycle activation. Negative mode withholds the dynamic
wheel edge. A passing negative trial requires missing scan-time input TF,
logged drops and no map, mapper poses, or map TF. Neither mode accesses any
UART or interprets synthetic output as physical mapping evidence.

The evidence includes `qualification.json`, the effective YAML and process
logs, graph endpoint inventories before, during and after the run, scan-time
checks in `scan_timing.jsonl`, the first occupancy grid when present, and
`SHA256SUMS.txt`. A mapper output pose with the same stamp is positive
evidence of a processed scan. Scans without a matched pose can include scans
skipped by installed motion/time thresholds, DDS loss or TF issues. The
mapper's drop-log counts are observed events and can be rate-limited.

**FACT — local verification, 2026-11-28.** The two final cases used the real
SLAM Toolbox 2.10.0 binary from the ROS Lyrical Debian package
`2.10.0-2resolute.20260921.014246`. The positive run's reliable, volatile
synthetic publisher was compatible with the mapper's best-effort,
volatile subscriber. Each emitted TF edge had one publisher GID.

| Final evidence directory | Scan-time input TF | Confirmed scan-stamped SLAM poses | Occupancy grids | Mapper TF-filter drop log events |
| --- | --- | --- | --- | --- |
| `step75_5a_dvqxz2gv` | 64/64 | 2 | 3, last 120 × 80 at 0.05 m | 0 |
| `step75_5a_iisshzd6` | 0/32 | 0 | 0 | 2 queue-full log events |

The positive run observed the full map-to-LiDAR chain at 62/64 original
scan stamps and 62 scans without a confirmed new SLAM pose. The latter
is a separate count from logged drops. Both runs completed lifecycle
shutdown and their publisher inventories were empty after cleanup.
An earlier positive development run produced a map and logged one
queue-full event. Its evidence and original failure verdict are retained
locally. Five targeted unit and ROS graph tests passed, including
the unchanged wheel adapter guard with the private remap and a
duplicate private-topic publisher. Actual live UART release, Pi timing
under real load and physical map accuracy remain unverified.

## Step 75.5b stationary live motor-off runbook

**PROPOSAL — execute after separate authorization.** Keep the platform
stationary with physical traction power disconnected. Use the verified
Pi baseline checkout `/home/elwis/dev_ws/sie_step75_5a_baseline` at
`2a00bd80348e8d10d4577e886f354d30e24057c1`. The launcher transfers only
the committed diagnostic files and verifies the archive on the Pi.
Existing deployment directories and evidence are preserved.

Verify interactive SSH access to `elwis@100.70.100.29`. Use the qualified
Jazzy 2.8.5 SLAM Toolbox installation, the existing
`~/siee_lidar_ws/install` LD19 overlay, the ROS C++ headers and CMake.
Free both serial devices by stopping their existing users normally.
Confirm the established LD19 device identity, unchanged mounting and
configuration, no competing odometry or motion runners, and no serial
getty on `/dev/ttyAMA0`. Leave the historical LD19 static TF launcher
off. The launcher uses unused, localhost-only ROS domain 178.
[Qualified LD19 arguments and device path](robot_geometry_footprint_v1.md#pi-qualification-evidence)

From the committed SIE checkout on the laptop, run the read-only STATUS
preflight. Confirm the physical conditions at the prompt.

```bash
bash tools/diagnostics/step75_5a_tf_audit/launch-live-from-laptop.sh status
```

Review the current v15 boot session ID and its STATUS evidence before
continuing. A change from the last independently verified boot requires
operator review. Verify READY, PWM 0/0, no active command, no fault, no safety
latches and no pending stop. The production transport checks exclusive UART
ownership and uses the configured 0.25 s timeout. No controller reset is
needed.

Enter the verified current ID from that STATUS output. Confirm the
physical conditions again when prompted for the live run.

```bash
read -r -p 'Verified ESP32 boot ID: ' VERIFIED_BOOT_ID
bash tools/diagnostics/step75_5a_tf_audit/launch-live-from-laptop.sh live "$VERIFIED_BOOT_ID"
```

The launcher starts the known sensor-only LD19 executable on its CP2102
device with baud 230400, topic `/sie/lidar/scan_raw`, frame `ld19_sensor`,
CCW scan direction, crop disabled and host ROS time. It waits for a raw
scan, then the diagnostic repeats safe STATUS on `/dev/ttyAMA0` and
requires the same boot ID. The unchanged read-only wheel adapter provides
the private odometry TF edge. SLAM Toolbox receives the original scan
headers and the qualified static transform.

During the 30-second stationary observation, preserve map and scan-stamped
pose output, input-TF results at the original stamps, startup full-chain
misses, any later misses, queue-full log events, and one attributed GID for
each edge. The steady-state review should distinguish scan-time input
continuity from mapper output. Review every live refusal, including any
unexpected boot or continuity loss. The physical LD19 acquisition time
remains unknown.

The diagnostic verifies its ROS shutdown and `/dev/ttyAMA0` release in
`qualification.json`. The wrapper stops its own LD19 process and preserves
its log and final graph inventory in the printed staging directory. A
stationary map establishes live input connectivity and smoke-test output.
Physical mapping and mapping accuracy over a moved route require separate
evidence. Navigation admission and powered motion remain disabled.
