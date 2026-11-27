# SIE | Step 75.1 Spatial Navigation Architecture Audit

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Close Step 74 as
`LIVE_ENTITY_CENTRIC_DIAGNOSTIC_INTEGRATION_COMPLETE` and start Step 75, ROS 2
Spatial Navigation Integration. The immediate goal is the shortest defensible
path to the Autonomous Person Following MVP. Step 75.1 audits and proposes.
Implementation, execution, and admission require their existing approvals.
[Step 74.5 closure and evidence](SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md#step-745--live-entity-centric-diagnostic-integration-complete)

**FACT.** The audited source is `35e103d700a8d764241f08c9eb770dd51b8983dc`.
Local and GitHub matched at the start. The private Step 74 primary files were
unavailable locally. Pi SSH rejected authentication, so package installation,
ROS graph, and loaded configuration on the Pi remain unverified. Historical
runtime results below are explicitly operator-reported. The available
2026-11-16 Graphify AST report names ancestor `db8097ea4239dcf45ef7598c831a45648d2498fd`.
The graph file matched recorded SHA-256
`9e068e27a46cfca02382370cb21465c749a3e6ad35fc1d16e948ab96e04c4e93`.
It predates the live three-sensor work. The findings below use the current
tracked manifests, launchers, interfaces, profiles, and firmware.

## ROS 2 implementation inventory

| Capability | FACT — tracked implementation and existing evidence | PROPOSAL — gap and dependencies |
| --- | --- | --- |
| Package and launch | [`sie_ros2` package.xml](../../ros2/sie_ros2/package.xml) is the only tracked ROS package manifest. [setup.py](../../ros2/sie_ros2/setup.py) installs two launches and the contract-node entry points. [`sie_pipeline.launch.py`](../../ros2/sie_ros2/launch/sie_pipeline.launch.py) starts Perception, Navigation Contract, and Supervisor, with optional LD19 and static TF off by default. Its composed sensor launch was reported working on the Pi. The separate [supervised launch](../../ros2/sie_ros2/launch/sie_pipeline_supervised_mvp.launch.py) explicitly enables the bounded-forward executor path. No launch starts Nav2, SLAM, odometry, or a state publisher. [Launch evidence](../../ros2/sie_ros2/README.md#integrated-launch-acceptance-2026-11-25-operator-evidence) | Inventory Pi overlays and installed Jazzy packages first. Preserve both existing launches and keep any future motor-off navigation bringup separate from the supervised motion launch. |
| Perception interfaces | [`PerceptionContractNode._callback`](../../ros2/sie_ros2/sie_ros2/perception_node.py) validates JSON from `/sie/perception/raw_measurement` for `/sie/perception/measurement`. [`Ar0234Ov9281MetricMeasurementNode`](../../ros2/sie_ros2/sie_ros2/ar0234_ov9281_metric_measurement_node.py) is a separate producer with a conditionally activated static or stop-and-measure recommendation profile. It is outside the default phase 1 launch. Step 74 instead consumes the existing camera audit and raw scan and publishes two isolated diagnostic String streams via [`DiagnosticSession`](../../ros2/sie_ros2/sie_ros2/live_entity_centric_node.py). Its Pi trials are operator-reported. [Static activation scope](../calibration/ar0234_ov9281_decision_chain_activation_v1.md) · [Step 74 interface](../diagnostics/live_entity_centric_integration_v1.md#ros-2-interface) | Preserve source evidence. Qualify the moving-target position and validate its effect on navigation separately from the activated static path and from diagnostic readiness. |
| Navigation and Decision Engine | [`NavigationContractNode._callback`](../../ros2/sie_ros2/sie_ros2/navigation_node.py) calls [`navigation_decision`](../../ros2/sie_ros2/sie_ros2/contracts.py#L656) directly from a validated Measurement. It emits `STOP`, `TURN_REOBSERVE`, or `FORWARD_REOBSERVE` with `execution_authorized=false` on `/sie/navigation/decision`. [Contract tests](../../ros2/sie_ros2/test/test_contracts.py) cover the recommendation boundary. [`PersonApproachDecisionEngine`](../../vision_core/person_approach/decision.py) is a separate bounded stabilization prototype; `sie/decision_engine/engine.py` is a [scaffold](../../sie/decision_engine/engine.py). This branch has no implemented ARCH-006 navigation loop using modeled-world facts from World State. | Reuse the recommendation and fail-closed test patterns. Put admitted moving-target facts through the World State Manager and use existing decision roles. Use Nav2 to plan and control within an admissible request. A topic name or class name alone supplies no authority. |
| World State, Knowledge Engine, Task Evaluator | [`sie/world_state/updater.py`](../../sie/world_state/updater.py), [state](../../sie/world_state/state.py), [Evidence Graph](../../sie/evidence/graph.py), and [Task Evaluator](../../sie/task_evaluator/evaluator.py) are scaffolds. No wired ROS World State or Knowledge Engine interface was found. The [approved architecture rules](../architecture/SIE_RULES.md) assign factual updates to the World State Manager and require the approved Knowledge Engine and Task Evaluator boundaries. | Implement only the approved minimum for current target and task evaluation after the input qualification path is defined. Reuse Data Contract boundaries and evidence links. |
| Supervisor and actuator bridge | [`SupervisorContractNode._callback` and `_execute_decision`](../../ros2/sie_ros2/sie_ros2/supervisor_node.py) default to execution disabled. The separately configured supervised mode checks its profile, session, timestamps, controller state, and re-observation, then uses one bounded HTTP forward action. The newer [`run_session` / `_run_session_unlocked`](../../vision_core/tools/run_shadow_uart.py) uses [`ShadowSupervisor`](../../vision_core/tools/shadow_supervisor.py) and [`UartHttp`](../../vision_core/tools/uart_http_transport.py) for guarded segments and status. Earlier supervised motion evidence applies to those envelopes, and v13's short turn still faulted. No tracked ROS velocity-to-actuator bridge or validated continuous velocity contract was found. [Approach evidence](../diagnostics/v7_supervised_approach.md) · [v13 evidence](SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md) | Reuse bounded feedback and stop patterns as evidence for a later reviewed executor. Keep motion output disconnected until a suitable actuator interface, stop behavior, authorization, and an independent protection path are verified. |
| Robot description, TF and navigation configuration | The tracked launch inventory contains no URDF, Xacro, robot state publisher, robot footprint, map, Nav2 costmap, planner/controller, or SLAM configuration. The only tracked navigation-relevant broadcaster is the optional LD19 `static_transform_publisher`. [`sie_pipeline.yaml`](../../ros2/sie_ros2/config/sie_pipeline.yaml) configures JSON transport and recommendation thresholds. It is not a navigation-stack configuration. | Verify installed packages independently, establish frame ownership and measured geometry, then create the smallest separate motor-off Nav2/SLAM configuration. A minimal description can publish the measured static frames without a detailed visual model. |

## Frames and datums

**FACT.** ES-005 requires explicit spatial frames and deterministic
transforms. Use the existing calibrated functions and source identities.
Display aliases, provisional assumptions, and approximate offsets do not
supply validated TF edges. [Spatial Model](../architecture/SIE_SPATIAL_MODEL.md)

| Frame or link | FACT — source and status | PROPOSAL — required disposition |
| --- | --- | --- |
| `map → odom` | Neither a map producer nor this TF edge is tracked in the launches. No persisted map/localizer configuration was found. | Use one mapping or localization authority once the odometry and scan TF chain are verified. No fabricated static map alignment. |
| `odom → base_link` | No continuous odometry producer, `nav_msgs/msg/Odometry` publisher, or dynamic robot TF broadcaster was found. `base_link` is named as an LD19 TF parent with no tracked wheel-centred datum definition. | Establish the body origin relative to the wheel geometry, with X forward, Y left, and Z up. Give one odometry source authority for dynamic TF and twist. |
| `base_footprint` | No tracked SIE definition or transform was found. | Use the measured `base_link` convention initially if it meets the chosen stack's configuration. Introduce a ground projection only if required and supported by the measured datum. |
| `base_link → ld19_sensor` | [`generate_launch_description`](../../ros2/sie_ros2/launch/sie_pipeline.launch.py) optionally publishes `(-0.023, 0, +0.212) m` with zero RPY. The operator confirmed publication and qualitative orientations. Z uses optical-window centre and the alignment is provisional. | Verify actual origin, laser plane, alignment, and applicability to the chosen base origin before using this as navigation TF. One broadcaster owns the child frame. |
| AR0234 and OV9281 camera frames | `ar0234_image_frame` is pixel space. `ar0234_optical_frame` is the camera optical frame named by the offline replay. Its extrinsic to OV9281 physical left has scoped static evidence. The stereo pipeline uses `rectified_left_optical_frame`, `R1ᵀ` into physical left, and physical right as part of the stereo calibration. Source labels include `ov9281_physical_left_optical_frame` in the camera audit and `physical_left_optical_frame` in the diagnostic geometry model. There is no tracked camera-to-base TF tree. [Activation](../calibration/ar0234_ov9281_decision_chain_activation_v1.md) · [Stereo reconstruction](../diagnostics/stereo_lidar_joint_static_v1.md) | Bind the exact source label and calibration revision at the adapter. Validate the camera-to-base pose for navigation while preserving the camera-internal calibration and distinction between optical and pixel coordinates. |
| `physical_left_lens_front_rim_frame` | [`mechanical_transform`](../../sie_core/geometry/camera_lidar_mechanical_geometry_v1.py) uses the reported LD19 offset from the rim `(+0.158, +0.030, +0.195) m` in robot-aligned axes and camera pitch `20°`. Yaw and roll have provisional assumptions and geometric uncertainty is unknown. It refuses an optical-centre transform. [`estimated_ld19_transform`](../../sie_core/geometry/camera_lidar_optical_datum_v1.py) adds an estimated optical datum of about `13.3022 mm` using the historical `12.5 mm` forward component and an optical-axis assumption. This returns `estimated_physical_left_optical_frame` for diagnostics. | Keep both profiles out of navigation TF. Reuse their provenance in a narrowly scoped transform review. Physical rim, estimated optical origin, and calibrated stereo frames remain distinct. |
| Robot front-plane datum | [Front mount profile](../../vision_core/config/diagnostics/robot_front_mount_operator_v1.json) records `0.21 m` rim setback and `15°` pitch with unknown optical-centre-to-rim forward offset. [`describe_front`](../../vision_core/person_localization/robot_front_geometry.py) outputs a partial value in `robot_longitudinal_axis_from_front_plane` with `distance_from_front_m=null`. Step 74 uses this profile independently of the separate 20° camera–LiDAR mechanical profile. | Reconcile the 15° and 20° datums and their mount revisions against the installed configuration before any base transform or clearance is admitted. Preserve both historical profiles. Do not construct a footprint or clearance by chaining them. |

## Odometry and motion feedback

**FACT.** The audited [v13 sketch](../../firmware/esp32/traction_control_v4_2_uart_shadow_v13_brake_continuity/traction_control_v4_2_uart_shadow_v13_brake_continuity.ino)
contains signed per-wheel counters. `onRightEncoderA` and `onLeftEncoderA`
trigger on A rising and use B phase for sign. `readMotionCounts` documents
forward sign correction `right=-rightRaw`, `left=leftRaw`, then applies the
active command direction. The source records effective loaded diameters
`0.0646 m` right and `0.0652 m` left (effective radii `0.0323 m` and
`0.0326 m`), `205/206` rising counts per wheel revolution, and
`TRACK_WIDTH_M=0.290`. `TURN_EFFECTIVE_TRACK_WIDTH_M=0.2941` is a
turn-specific fit. These are recorded controller parameters and limited trial
calibrations. Their accuracy for accumulated navigation pose and twist is
unverified. Neither track width defines footprint width.

**FACT.** `startMotion` calls `resetEncoderCounts`; `updateDistancesFromEncoders`
computes per-command wheel distances, average progress, and encoder-derived
heading. The regulated loop calculates wheel speed from count deltas over
its local interval. The HTTP status builder exposes wheel/profile speed
fields. The v13 UART `buildUartStatus` and `buildUartStatusCompact` expose
boot and command context, raw wheel counts, command-local distances and
heading, without continuous signed twist or a count-snapshot time. Fault
and steering timing fields concern specific controller events.
[`UartHttp.__call__`](../../vision_core/tools/uart_http_transport.py)
provides request identity and CRC. [`run_shadow_uart.call`](../../vision_core/tools/run_shadow_uart.py)
journals host `started_s` and `completed_s`; these bound the request interval
where the response is valid. They are not a firmware encoder sample clock.
[`expand_status`](../../vision_core/tools/shadow_uart_compact_status.py)
expands compact names with the same telemetry limits.

**FACT.** Current evidence supports per-command progress and controller
diagnostics. Continuous ROS odometry is not established. The counters
reset at motion start within a boot. UART response snapshots lack a
continuous count epoch and sample timestamp that would disambiguate
resets, gaps, direction, and stopped intervals. No usable IMU acquisition,
calibration, topic, or hardware evidence was found in the audited branch.
Pi-side external sources remain unknown. The v13 supervised right-turn
trial latched `BOUNDED_DISTANCE_LIMIT` and ended FAULT, STOP confirmed and
PWM 0/0. Physical traction-power state and independent off-Pi v13 archive
were not confirmed in that record. [v13 handoff](SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md)

**PROPOSAL.** First inspect the installed graph and existing telemetry.
If no adequate source exists, separately review a minimally invasive,
boot-scoped signed cumulative wheel-count sample with firmware monotonic
sample time, sample/reset identity, and fault provenance. Keep bounded
motion counters and the recorded fault intact. Use the recorded wheel
scales and physical sign as candidates to check. Survey the effective
wheel separation and chassis datum only where existing measurement
records leave gaps. Verify rollover, reset, reverse, monotonicity, timeout
and invalid-output behavior on fixtures and saved records before proposing
any physical calibration. A later separately authorized validation should
compare encoder-derived forward progress and heading with independent
distance and angle references under load and establish drift, timing and
covariance. One odometry adapter should publish consistent `Odometry` and
`odom → base_link`, with pose and twist expressed in their declared frames.
An IMU or filter is optional if wheel-only odometry meets the validated
MVP envelope. [Nav2 Jazzy state estimation](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/)

## Raw LD19 to SLAM, localization, costmaps and protection

**FACT.** The external `ldlidar_stl_ros2` package was previously built on
the Pi under `~/siee_lidar_ws`. The default launch opens no LD19 serial
port. Its opt-in configuration identifies `/dev/ttyUSB0` at 230400 baud,
`/sie/lidar/scan_raw` as `sensor_msgs/msg/LaserScan`, `ld19_sensor`,
CCW and no crop. Runtime scan publication and the provisional TF have
operator evidence. Actual device binding, active publisher, QoS and
loaded `use_sim_time` need an inventory rather than an assumption.
LD19 header time describes the driver's ROS conversion/publication.
Physical acquisition time is `UNKNOWN`. The Step 74 obstacle topic is
an independent JSON diagnostic and carries no admitted navigation or
safety status. [Sensor launch provenance](../../ros2/sie_ros2/README.md) ·
[LD19 timestamp policy](../ros2/ld19_timestamp_provenance_freshness_v1.md)

| Use of the existing raw scan | PROPOSAL — minimum route and prerequisites |
| --- | --- |
| SLAM and mapping | Configure SLAM Toolbox Jazzy to subscribe directly to `/sie/lidar/scan_raw` after verifying scan geometry, timestamp/TF compatibility, and frame ownership. Supply time-aligned `odom → base_link → ld19_sensor`. Validate with already available replay or isolated outputs before an authorized live stage. SLAM owns `map → odom` when mapping. [SLAM Toolbox Jazzy](https://github.com/SteveMacenski/slam_toolbox/tree/jazzy) |
| Localization | For a repeatable route, preserve the map and tested initialization; use the verified SLAM Toolbox localization mode on its saved pose graph if it meets the MVP requirement. Select one `map → odom` authority for the chosen localization mode. An alternative localization package can wait. [SLAM Toolbox Jazzy localization](https://github.com/SteveMacenski/slam_toolbox/tree/jazzy) |
| Local obstacles and Nav2 costmaps | Feed the same raw scan to the configured Nav2 LaserScan obstacle layer. Use a measured footprint and tested inflation policy. Use local `odom` and global `map` when available. Check frame/time transforms, valid hit marking, and cleared/unknown cells independently. Keep missing or invalid returns unknown and verify any proposed finite-ray clearing policy before relying on it. Avoid a second beam projection solely for costmaps. [Nav2 Jazzy Costmap 2D](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/) |
| Independent runtime protection | Use validated scan and feedback freshness and stop behavior independent of target identity, decision approval, and planning success. Evaluate a direct-scan software gate such as Nav2 Collision Monitor with verified coverage, fail-closed stale/invalid data and stop-chain tests. It is an additional CPU-level tool with no hard real-time certification. Preserve the physical cutoff and independent firmware protections, and validate the end-to-end stop envelope separately. [Nav2 Jazzy Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/) |

**FACT.** No SIE Nav2/SLAM configuration or dependency declaration was found,
and the Pi package query did not authenticate. Installation of Nav2, SLAM
Toolbox, Collision Monitor, or robot-state publisher remains unknown.
An upstream launch or installed package is a dependency candidate, with no
evidence of a working SIE stack. No planner, controller, footprint, command
topic type, lifecycle setup, or Pi navigation configuration is validated.

**PROPOSAL.** Check Jazzy package versions and existing overlays before
choosing exactly one shipped planner and one differential-drive-compatible
controller. Verify message type, frame IDs, rate, and lifecycle defaults
against the installed versions. Use the same Pi 5, headless, and isolate or
disconnect motion outputs in initial software verification. Measure CPU,
memory, and diagnostic latency with perception and navigation software
together before drawing capacity conclusions from the Step 74 Pi rates.
Do not introduce a second path planner, a custom localization engine, or a
robot-localization filter solely to satisfy an example.

## Moving target evidence and role boundaries

**FACT.** [`semantic_from_camera`, `spatial_from_camera` and `camera_view`](../../ros2/sie_ros2/sie_ros2/live_entity_centric_v1.py)
retain source cycle, session-local track, instance, mask/model and calibration
references, packet IDs, camera timestamps, age, and method-specific quality.
The live spatial output is scalar. The preferred stable torso-core median is
a partial longitudinal known term. The fallback is rectified optical Z.
The upstream [control preview](../../ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py)
can emit `shadow_target_bearing_deg` from an undistorted AR0234 centre ray.
That bearing is camera-relative, and Step 74 does not package it as a base-frame
target vector. The separate full-resolution [audit `_frustum_associate`](../../ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py)
and static [Metric Measurement `_associate`](../../ros2/sie_ros2/sie_ros2/ar0234_ov9281_metric_measurement_node.py)
paths have physical-left XYZ or bearing candidates in their own scopes.
They do not validate dynamic base-relative XY, uncertainty, target validity
time, or odometry alignment for this Step 74 stream.

**PROPOSAL.** For the first moving-target adapter, reuse same-cycle
camera geometry to qualify planar position or a defensible bearing/range
pair referenced to one validated frame. Optical depth, AR0234 ray bearing,
and the partial front term have distinct origins and axes; combining them
as if they were one vector would lose datum and lateral information.
Produce a goal only from an accepted current target fact after applying
observable uncertainty, source freshness, time-appropriate base/odom TF,
and the approved standoff and visibility constraints. Use the existing
[depth-admission draft](../diagnostics/control_depth_admission_draft_v1.md)
as a checklist for focused validation, including lateral/axial error,
target loss and reacquisition, measurement age, and applicable transform.
The 500 ms diagnostic window supplies no navigation freshness threshold.
A planar position is sufficient to evaluate the MVP; a full torso pose,
permanent identity, or LD19 membership is optional for this milestone.

**DECISION.** Preserve the approved [Fundamental Architecture Baseline](../../PROJECT_CONTEXT.md#fundamental-architecture-baseline)
and [Responsible Autonomy and Decision Governance Principles](../../PROJECT_CONTEXT.md#responsible-autonomy-and-decision-governance-principles).
Only the World State Manager writes accepted modeled-world facts, with
provenance and confidence traceable through the Evidence Graph. The Decision
Engine obtains modeled-world facts from World State and retains its approved
Knowledge Engine and Task Evaluator inputs under ARCH-006. The Knowledge
Engine owns approved engineering knowledge and rules. The Task Evaluator
assesses task state and outcomes. Independent
admissibility validation and authorized execution remain distinct.
Independent runtime protection must work through camera loss or semantic
uncertainty. [Evidence Graph](../architecture/SIE_EVIDENCE_GRAPH.md) ·
[Architecture rules](../architecture/SIE_RULES.md#arch-006)

**PROPOSAL.** Use this data flow when its prerequisites are accepted.

```text
Camera evidence → applicable Measurement → interpretation → verifiable Fact
  → World State Manager → World State → Decision Engine
                                        ↑ Knowledge Engine rules and Task Evaluator outputs
  → independent admissibility validation → authorized execution request
  → Nav2 goal or path processing → protected executor → outcome feedback

LD19 raw scan + qualified TF/odometry → SLAM or localization + Nav2 costmaps
LD19 raw scan + independent freshness/stop inputs → independent runtime protection
```

Use one reviewed, cancellable goal interface supported by the installed
version. Nav2 supplies planning and local control for the authorized
request, while SIE owns target selection, task sufficiency, and admissible
interaction. Keep target-loss/cancel handling and outcome re-observation
within approved boundaries. Earlier `FORWARD_REOBSERVE` decisions and
Step 74 diagnostics cannot serve as live goals or bypass this chain.

## Critical path and actual blockers

1. **PROPOSAL — Step 75.2, read-only odometry-input and Pi navigation preflight.**
   Identify the installed Jazzy stack, currently visible TF/odom authorities,
   and the actual signed counter/timing fields in saved v13 status evidence.
   Record observed, absent-from-graph, or unverified status for each item.
   This resolves dependency and telemetry uncertainty without reopening
   perception work.
2. **PROPOSAL — establish the minimum frame and odometry foundation.** Once
   Step 75.2 resolves inputs, qualify one signed-wheel odometry source with
   timestamp/reset semantics and a measured base/LD19 transform and footprint.
   Validate first using software and recorded data. These are prerequisites for
   useful SLAM, costmaps, and a stable local navigation reference.
3. **PROPOSAL — qualify moving target evidence in parallel.** Reuse the
   existing camera observations and calibration. Limit new work to the
   missing base-relative position, clocks, error/age, and approval evidence.
   Route accepted data through the World State and decision boundaries.
4. **PROPOSAL — verify navigation with outputs isolated.** First verify the
   local `odom`-relative path on the validated frames, then add one
   SLAM/localization authority for map-based routing. Configure costmaps
   and one installed planner/controller. Verify error and cancellation behavior,
   target expiry, independent obstacle handling, and Pi resource use under
   comparable load. Keep executor commands disconnected.
5. **PROPOSAL — qualify execution and independent protection before moving.**
   The v13 braking fault, controlled actuation/feedback, stop/cancel
   behavior, independent collision protection, and owner authorization are
   actual blockers to autonomous motion. Resolving them is separate from
   configuring a planner. Until then, qualifying execution remains open.

**FACT.** Diagnostic perception, independent raw LD19 geometry, and
conditional static recommendations already have scoped evidence. Full
camera–LiDAR extrinsics, semantic matching of obstacles, an IMU or
multi-sensor odometry filter, detailed URDF visuals, an additional
localization stack, and generalized reasoning can be deferred unless the
chosen, validated MVP requires them.

## Step 75.2 exact read-only Pi preflight

**PROPOSAL.** Run this once from the Pi's existing Jazzy environment with
its normal deployed overlay already sourced. The commands inspect packages,
an existing ROS graph, and private saved telemetry. They do not install
packages, launch navigation, open a UART, or send controller commands.
If the graph is idle, record the resulting nonvisibility.
`ros2 pkg prefix` probes overlays as well as binary installs, while
`dpkg-query` identifies installed Debian packages.

```bash
source /opt/ros/jazzy/setup.bash
if test -r /home/elwis/siee_lidar_ws/install/local_setup.bash; then
  source /home/elwis/siee_lidar_ws/install/local_setup.bash
fi

repo=/home/elwis/dev_ws/sie_live_identity_20261119
git -C "$repo" rev-parse HEAD
git -C "$repo" status --short --branch
printf 'ROS_DISTRO=%s ROS_DOMAIN_ID=%s\n' "$ROS_DISTRO" "${ROS_DOMAIN_ID:-0}"
for pkg in sie_ros2 ldlidar_stl_ros2 tf2_ros nav_msgs slam_toolbox \
  nav2_bringup nav2_msgs nav2_planner nav2_controller nav2_costmap_2d \
  nav2_collision_monitor robot_state_publisher robot_localization; do
  printf '%s: ' "$pkg"
  ros2 pkg prefix "$pkg" 2>/dev/null || printf 'not visible in sourced overlays\n'
done
for pkg in ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-collision-monitor ros-jazzy-robot-state-publisher; do
  dpkg-query -W -f='${binary:Package} ${db:Status-Abbrev} ${Version}\n' "$pkg" 2>/dev/null ||
    printf '%s: not present in dpkg records\n' "$pkg"
done

ros2 node list --no-daemon
ros2 topic list --no-daemon -t
for topic in /sie/lidar/scan_raw /odom /tf /tf_static /map /robot_description /cmd_vel; do
  timeout 6s ros2 topic info "$topic" --no-daemon --verbose || true
done
timeout 6s ros2 topic echo /sie/lidar/scan_raw sensor_msgs/msg/LaserScan \
  --no-daemon --once --field header --qos-reliability best_effort || true
timeout 6s ros2 topic echo /tf_static tf2_msgs/msg/TFMessage \
  --no-daemon --once --qos-durability transient_local || true
timeout 6s ros2 topic echo /tf tf2_msgs/msg/TFMessage --no-daemon --once || true
timeout 6s ros2 topic echo /odom nav_msgs/msg/Odometry --no-daemon --once || true
```

For saved v13 status evidence, report filenames, hashes and field coverage
without exposing command payloads or serial identifiers. Keep the raw files
private.

```bash
evidence=/home/elwis/.local/state/sie/evidence/v13_right8_20261123_205406
if test -d "$evidence"; then
  find "$evidence" -maxdepth 2 -type f -printf '%P\n'
  python3 - "$evidence" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
fields = ("boot_session_id", "right_encoder_raw_count", "left_encoder_raw_count",
          "right_speed_m_s", "left_speed_m_s", "heading_rad")
found = False
for path in sorted(root.glob("**/*.jsonl")):
    n = paired = bracketed = 0
    covered = set()
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if not isinstance(row, dict) or row.get("method") != "GET" or row.get("code") != 200:
            continue
        if row.get("endpoint") not in ("/status", "/status-compact"):
            continue
        body = row.get("body")
        if not isinstance(body, dict):
            continue
        n += 1
        covered.update(key for key in fields if key in body)
        paired += (("right_encoder_raw_count" in body and "left_encoder_raw_count" in body)
                   or (body.get("_") == 1 and "x" in body and "w" in body))
        bracketed += "started_s" in row and "completed_s" in row
    if n:
        found = True
        print(path.relative_to(root), "sha256", hashlib.sha256(path.read_bytes()).hexdigest(),
              "status rows", n, "count pairs", paired, "host brackets", bracketed,
              "full status fields", sorted(covered))
if not found:
    print("No usable saved status rows found")
PY
else
  printf 'v13 evidence directory unavailable\n'
fi
```

Pass forward the package/graph inventory, current TF authorities and
visible frame names, and status telemetry coverage. Record missing
snapshot time, count continuity across resets, and any unverified
physical scale explicitly. Step 75.2 ends at this evidence report.

## Step 75.2 closure and Step 75.3 owner decision (2026-11-27)

**FACT — operator-reported Pi preflight.** Jazzy, ROS_DOMAIN_ID=0, detached perception checkout 35e103d700a8d764241f08c9eb770dd51b8983dc. Sourced LD19 diagnostic overlay and /opt/ros/jazzy. sie_ros2, ldlidar_stl_ros2, tf2_ros, nav_msgs and robot_state_publisher visible; robot_state_publisher Debian version 3.3.4-1noble.20261106.031427. SLAM Toolbox, Nav2 bringup/messages/planner/controller/costmap/collision monitor and robot_localization not visible in sourced overlays; queried SLAM Toolbox and Nav2 Debian packages not present. Sole visible node /siee_ld19, one reliable volatile publisher on /sie/lidar/scan_raw, LaserScan header frame ld19_sensor. /odom, /tf, /tf_static, /map, /robot_description and /cmd_vel not visible in this graph. No installation or navigation launch performed.

**FACT — operator-reported saved telemetry audit (75.2a/b).** Evidence directory /home/elwis/.local/state/sie/evidence/v13_right8_20261123_205406 contains events.jsonl (26587 bytes), fault_diagnostics.json, initial_status.json, turn_terminal_status.json and final_status.json. Initial schema-specific JSONL query found no matching rows; subsequent inspection found 34 events (17 request and 17 response rows), with host_time and response data. Raw right/left pairs: READY 0/12; forward BRAKING -2/+2 then -3/+4; PAUSED -3/+4; right turn STARTING +1/0; terminal +27/+25. Forward physical sign convention from audited firmware is right=-raw, left=raw. Command-local resets prevent continuous odometry; no firmware encoder snapshot timestamp established. Host event timestamps are not firmware sample timestamps. No new physical trial or independent primary file inspection. V13 fault remains BOUNDED_DISTANCE_LIMIT / ENCODER_LIMIT_REACHED, final PWM 0/0; brake request 15/13, final 27/25, targets 21/21, limits 22/22. Do not infer loaded odometry accuracy from this trial.

**DECISION — OWNER_APPROVED.** Step 75.2 closes as `NAVIGATION_PREFLIGHT_COMPLETE_ODOMETRY_IMPLEMENTATION_REQUIRED`. Start Step 75.3 Continuous Wheel Odometry. Preserve the existing motor-control and command-local counter semantics. Add an independent boot-scoped signed cumulative encoder telemetry path; calculate navigation pose on Pi, not ESP32. Physical forward-positive signs must be independent of command direction. Proposed fields: boot_session_id, sample_sequence, sample_monotonic_us, right_total_signed_ticks, left_total_signed_ticks, controller_state, fault_latched. Use a coherent count-pair/time snapshot; explicit version, reset/overflow semantics and fault provenance. Avoid changes to braking, actuator logic, limits, and safety. Qualify ISR/critical-section overhead; do not assert timing neutrality without evidence.

**NEXT — Step 75.3a, software implementation only.** Inspect v13 firmware and existing UART framing, compact/full status compatibility, ISR synchronization, and boot identity. Preserve v13 unchanged; create a separately versioned derivative with minimal read-only odometry telemetry, preferably opt-in. Test signed increments/decrements, command-local resets without cumulative reset, idle/coast pulses, reboot identity, sequence/time monotonicity, rollover or explicit invalidation, coherent snapshots, CRC/framing and existing motion/safety regressions. Compile if toolchain available; report unrun checks honestly. No flash/reset, hardware commands, motor operation, fault clearing, or authorization changes. No invented sample rates or validated wheel scale. Document deployment and motor-off validation plan, not execution. Subsequent steps: 75.3b telemetry qualification; 75.3c ROS Odometry adapter and one TF authority; 75.3d independent physical validation under separate approval. Frame/footprint qualification, camera base-relative target and SLAM/Nav2 remain open. Step 74 stays closed; no perception/calibration repetition without new defect evidence. PR #13 untouched.

## Step 75.3a — Continuous wheel encoder telemetry software

**FACT.** Candidate `v4_2_uart_shadow_v15_cumulative_encoder_telemetry`
adds the read-only `WHEEL_ENCODERS_V1` exchange and an offline host validator.
A coherent count pair and ESP32 sample time use lock-free, revision-checked
publication from the existing GPIO callbacks. The v13 directory, motion
runners, and admission boundaries are unchanged.
[Protocol, firmware audit, and verification](../diagnostics/wheel_encoder_telemetry_v1.md)

**OPEN — Step 75.3b.** Start with physical verification that motor power is
disconnected and SHA-256 verification of archived v13 fault evidence.
Any flash or reboot is a separately authorized action. Qualify the candidate
using stationary reads and manual rotations with motor power off before
considering an odometry adapter. The v13 braking fault remains open.
[Motor-off qualification plan](../diagnostics/wheel_encoder_telemetry_v1.md#step-753b-motor-off-qualification-plan)
