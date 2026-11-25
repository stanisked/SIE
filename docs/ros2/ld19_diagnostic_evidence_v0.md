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

**OPEN:** Inter-record source timestamp ordering, time intervals and
source-to-receive time deltas were **not evaluated** by the above check.
No claim about exact scan frequency, packet-loss rate, laser-plane extrinsics
or long-duration temporal stability follows from this short capture.
No off-Pi evidence backup has been confirmed. Keep the raw journal private.

**NEXT VERIFIABLE STEP:** Run a read-only audit of the existing JSONL to
verify timestamp monotonicity, duplicate source stamps, per-record angular
array consistency, and receive latency, without capturing a new scene or
running motors. Any apparent time offset must be reported as an observation,
not silently corrected.

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
