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

**FACT / driver timestamp provenance audit (2026-11-25,
operator-provided source grep):** The Pi's local
`~/siee_lidar_ws/src/ldlidar_stl_ros2` checkout reported Git HEAD
`bf668a89baf722a787dadc442860dcbf33a82f5a`. Its worktree
**is not clean**: `ldlidar_driver/src/logger/log_module.cpp` is modified,
and two `*.before_pthread_fix` backup files are untracked. These unrelated
changes must not be overwritten or assumed included in the Git SHA.

The matching `src/demo.cpp` excerpts show:
- `ToLaserscanMessagePublish(...)`: `start_scan_time = node->now();`
  (line 155), and `output.header.stamp = start_scan_time;` (line 173).
  Thus the ROS `LaserScan.header.stamp` represents a host ROS clock
  sampled during scan conversion, **not a directly exported physical
  laser-acquisition timestamp**.
- `scan_time = start_scan_time - end_scan_time` uses consecutive
  conversion-time clock samples; `output.time_increment` is computed as
  `scan_time / (beam_size - 1)`, not from individual SDK timestamps.
- The scan is then published (`lidarpub->publish(output)`, line 240)
  and the previous conversion-time state is updated (line 241).
- Separately, SDK `lipkg.cpp` assigns per-point
  `data.stamp` from `get_timestamp_()`-derived packet times
  (lines 154-180), and `ldlidar_driver.cpp` sets
  `dst.stamp = recvpcd.front().stamp` (line 273). The shown
  ROS publishing function does **not** use this SDK stamp as the
  `LaserScan.header.stamp` field.

**FACT / SDK callback and ROS call-chain audit (2026-11-25,
operator-provided source grep from same Pi checkout):**

- `src/demo.cpp:91` registers
  `ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp))`.
  `src/demo.cpp:245-249` defines `GetSystemTimeStamp()` as
  `std::chrono::system_clock::now()` cast to nanoseconds since epoch.
  Thus SDK packet timestamps are **host system-clock nanoseconds**, not
  independent hardware LiDAR timestamps.
- `ldlidar_driver/src/core/ldlidar_driver.cpp:298-299` registers this
  callback, then `...:84` forwards it to
  `comm_pkg_->RegisterTimestampGetFunctional`;
  `ldlidar_driver/src/dataprocess/lipkg.cpp:299-300` stores it in
  `get_timestamp_`.
- `lipkg.cpp:154-161` samples `get_timestamp_()` for the first and
  each successive parsed packet; the code estimates a per-point timestamp
  by interpolation between adjacent packet callback times
  (`lipkg.cpp:176`). It does **not** prove actual laser-exposure
  timing or hardware clock synchronization.
- `src/demo.cpp:119-122` calls
  `ldlidarnode->GetLaserScanData(laser_scan_points, 1500)`, where
  `laser_scan_points` is a `Points2D`, then calls
  `ToLaserscanMessagePublish`. This selects the
  `GetLaserScanData(Points2D&)` overload
  (`ldlidar_driver.cpp:240-249`), which reads the accumulated point array
  from `LiPkg::GetLaserScanData(Points2D&)`
  (`lipkg.cpp:281-286`).
- The separate `GetLaserScanData(LaserScan&)` overload can copy
  `recvpcd.front().stamp` to `dst.stamp` (`ldlidar_driver.cpp:263-274`)
  but is **not used in the observed ROS publication path**.
- As recorded above, `ToLaserscanMessagePublish` obtains
  `node->now()` and writes it to `LaserScan.header.stamp`;
  point timestamps are not exported in the published
  `sensor_msgs/msg/LaserScan` message. `scan_time` and
  `time_increment` describe a conversion-time-derived uniform spacing,
  not independently observed point timing.

**INTERPRETATION / LIMIT:** The observed 99.987–100.029-ms intervals
describe the ROS conversion clock's cadence over a 2.9-second sample,
**not** validated physical rotation or laser-acquisition cadence.
The 0.471–0.803-ms `received_at_utc - header.stamp` describes
software timing between conversion and diagnostic receipt, **not**
the full latency from a laser measurement. The SDK timestamp callback
timebase is now identified as the Pi host wall clock, but its relationship
to ROS time when `use_sim_time` changes, the timing of UART packet
receipt/parse and laser exposure, and any inter-camera offset have not
been empirically measured. Do not infer a fixed temporal compensation
factor. Packet loss, long-duration drift, extrinsic precision and
off-Pi independently verified backup remain OPEN. Keep raw scan
evidence private and the draft `DIAGNOSTIC_ONLY_NOT_ADMITTED`.

**NEXT VERIFIABLE STEP:** Without touching the working third-party
driver, define a versioned **timestamp provenance and freshness**
contract distinguishing ROS conversion time, host packet-processing
timestamps and unknown hardware acquisition time; use this
distinction in future camera/LiDAR alignment checks. First test the
contract with synthetic stale/reordered scans, then collect real
cross-sensor evidence only after the contract boundaries are reviewed.

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
