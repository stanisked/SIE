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

## Pi read-only verification (executed on 2026-11-25; commands retained for reproducibility)

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

## Pi diagnostic capture evidence (2026-11-25, operator report)

**FACT / input and run:** The operator ran the manual-only
`ld19_diagnostic_evidence_node` against the existing
`/sie/lidar/scan_raw` topic on Raspberry Pi 5. The collector logged
`DIAGNOSTIC_ONLY`, `limit=30`, and `capture complete: 30 records`.
This was the first reported physical scan capture using this adapter, and
does not constitute a canonical SIE Observation validation.

**FACT / operator-side verification output:**
- Build and executable availability: `SIEE_LD19_ADAPTER_BUILD: PASS`,
  with six unit tests passed in 0.03 s.
- JSONL file:
  `/home/elwis/.local/state/sie/evidence/ld19_diagnostic_v0_20261125T132230Z.jsonl`
- `RECORDS: 30`; `UNIQUE_OBSERVATIONS: 30`.
- `REFERENCE_FRAME: ld19_sensor`.
- `VALID_FRACTION_MIN: 0.839`,
  `VALID_FRACTION_MAX: 0.8608`. These are per-scan geometric validity
  ratios, **not estimated confidence or safety reliability**.
- `EVIDENCE_SHA256: PASS`: the operator recomputed each digest over
  canonical JSON of its recorded `payload.scan`, verifying correspondence
  between the persisted payload and stored digest.
- `MEASUREMENT_ADMISSION: DISABLED`,
  `WORLD_STATE_UPDATE: DISABLED`,
  `MOTOR_CONTROL: DISABLED`.
- Full JSONL file SHA-256:
  `d647283c478600c02574a2a6ff3cea38ccbd662200dca3444b45a46df8e5d158`.
  This checksum was reported by `sha256sum` on Pi; an independent
  off-Pi verification has not yet been reported.

**FACT / temporal audit (2026-11-25, same 30-record file):** The
operator independently audited stored `payload.scan.header` time fields,
record source timestamps, receive timestamps and angular metadata:

- `RECORDS: 30`; duplicate source stamps `0`; non-monotonic intervals `0`.
- Consecutive source-header stamp intervals (29 differences): min
  `99.987 ms`, median `99.999 ms`, max `100.029 ms`, mean `100.000 ms`.
  Thus the source **header stamps** imply approximately 10 Hz over this
  short, about 2.9-second interval, not a verified physical spin cadence.
- `received_at_utc` minus ROS scan-header stamp: min `0.471 ms`,
  median `0.519 ms`, max `0.803 ms`. This measures only the
  timestamp difference as published/stored by this software stack;
  it is **not** evidence of hardware laser acquisition-to-host latency.
- `SOURCE_TIMESTAMP_MATCH: PASS`, when reconstructing ISO-8601 UTC
  timestamps from scan header sec/nanosec.
- `ANGLE_SPAN_ERROR_RAD_MAX: 0.0` when checking stored
  `angle_max - angle_min - (N - 1) * angle_increment`. This proves
  internal metadata consistency in the saved records, **not** an
  independently calibrated physical beam bearing.
- The operator reported `SIEE_LD19_TEMPORAL_AUDIT: COMPLETE`.

**BOUNDARY / OPEN:** The LD19 driver's source code path that assigns
`LaserScan.header.stamp` has **not yet been audited**. Its apparent 100-ms
regularity might reflect scheduling or driver stamp generation, not
independent LiDAR rotation timing. The sampled timestamp delta must not
be used as full sensor latency or cross-modal synchronization uncertainty.
Scan drops, true capture timestamps, long-duration transport stability,
extrinsic precision and independently verified off-Pi file backup remain OPEN.
Keep raw data private and the diagnostic contract non-authoritative.

**NEXT VERIFIABLE STEP:** Inspect, read-only, the exact
`ldlidar_stl_ros2` driver revision used on Pi to identify the assignment
of `LaserScan.header.stamp` and whether it represents acquisition, SDK
completion or ROS publish time. Do not change calibration or downstream
admission on the basis of the short temporal audit.

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
