# LD19 diagnostic evidence adapter v0 (draft, 2026-11-25)

Status: **DRAFT / diagnostic-only / NOT an approved SIE Data Contract**.

## Goal and scope

Capture bounded `sensor_msgs/msg/LaserScan` streams from `/sie/lidar/scan_raw`
as append-only, source-bound engineering evidence. This is the first
**non-authoritative internal diagnostic step** toward a canonical LD19
`Observation` adapter; it is not an approved Observation contract.

Approved architecture: ES-002 `ARCH-001`, ES-004 Observation fields, ES-005
reference frames, ES-006 evidence traceability. No input from this draft is
allowed to enter a canonical Measurement, Fact, World State, navigation
admission, steering or motor path.

## Implementation

- Pure conversion: `ros2/sie_ros2/sie_ros2/ld19_diagnostic_evidence.py`
- ROS subscriber + append-only capture:
  `ros2/sie_ros2/sie_ros2/ld19_diagnostic_evidence_node.py`
- Unit tests: `ros2/sie_ros2/test/test_ld19_diagnostic_evidence.py`
- No change to the existing `sie_pipeline.launch.py`: this collector
  must be invoked explicitly and is not enabled by default.
- Best-effort subscription uses ROS 2 sensor-data QoS.
- The default capture limit is 100 scans. Journal file permissions when
  created are owner read/write only; repeated runs append rather than overwrite.
- Each record preserves header stamp (nanoseconds), receive timestamp (UTC),
  `ld19_sensor` reference frame, ranges, intensities, angular/timing metadata,
  units, quality counts and a deterministic SHA-256 of its normalized JSON
  scan representation. The SHA is **not a hash of original UART bytes**.
- NaN and +/-Infinity scan values are encoded as explicit JSON strings to
  prevent silently dropping invalid samples. No range scaling is performed.
- `confidence=0` explicitly marks **not estimated**, with
  `quality.assessment=NOT_QUALIFIED`; valid_fraction is a diagnostic metric,
  **not** reliability or observation acceptance.
- No `std_msgs/String` output and no canonical topic. No client for ESP32,
  motion, World State or Decision Engine.

The internal file schema `sie.ld19.diagnostic_observation_candidate.v0` is
not an approved Data Contract and must not become an inter-module API by
accident. The `evidence_ids` field supports local source linking but does not
claim a fully integrated Evidence Graph or calibration acceptance.

## Pi read-only verification (not yet executed)

Start with the existing integrated `enable_ld19:=true` launch from the
previously verified PR #12, with drive power physically disconnected.
**Do not start a second LD19 driver or second static TF publisher.**

Test the draft from a *separate* worktree and build/install location. After
creating that worktree at the current branch head and verifying clean checkout:

```bash
source /opt/ros/jazzy/setup.bash
source ~/siee_lidar_ws/install/setup.bash
cd /home/elwis/dev_ws/sie_ld19_diagnostic_v0_20261125
colcon --log-base "$PWD/log" build --base-paths ros2 \
  --packages-select sie_ros2 --symlink-install \
  --build-base "$PWD/build" --install-base "$PWD/install"
source install/setup.bash
python3 -m pytest -q ros2/sie_ros2/test/test_ld19_diagnostic_evidence.py
python3 -m py_compile ros2/sie_ros2/sie_ros2/ld19_diagnostic_evidence.py \
  ros2/sie_ros2/sie_ros2/ld19_diagnostic_evidence_node.py
```

For a short capture, in a new shell with the same overlay:

```bash
ros2 run sie_ros2 ld19_diagnostic_evidence_node --ros-args \
  -p max_records:=30 \
  -p output_jsonl:=/home/elwis/.local/state/sie/streams/ld19_diagnostic_v0_20261125.jsonl
```

Inspect privately on Pi: count 30 JSONL records, validate SHA recomputation
for sample records, check source timestamp/frame and status
`DIAGNOSTIC_ONLY_NOT_ADMITTED`. Do not push raw scene scans to public GitHub;
store source evidence in the private SIEE evidence archive, with checksums
and a separately verified backup before claiming durable evidence.

## Open engineering work

1. Approve or adapt an LD19-specific **canonical Observation Data Contract**
   based on ES-004 and existing project contracts. This draft is **not it**.
2. Define sensor calibration ID/version/quality applicability and a bounded
   time-alignment/freshness policy with AR0234/OV9281, using explicit
   frames and traceability.
3. Introduce a validated Observation producer and Evidence Graph linkage.
4. Only after quality and applicability gates, define independent Measurement
   and sensor-fusion acceptance. This draft never authorizes motor commands.

The hardware TF `base_link -> ld19_sensor` remains provisional:
translation (-0.023, 0, 0.212) m, yaw/roll/pitch zero by first-order
alignment; Z is based on optical-window centre, not a laser-plane survey.
