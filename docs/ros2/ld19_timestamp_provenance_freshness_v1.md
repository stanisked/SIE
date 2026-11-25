# LD19 Timestamp Provenance & Freshness Policy v1

**Status: DRAFT / internal diagnostic policy / NOT an approved SIE Data
Contract.** Date: 2026-11-25. Applies only to the LD19
`sie.ld19.diagnostic_observation_candidate.v0` record format and
the driver source path inspected at
`ldlidar_stl_ros2` HEAD `bf668a89baf722a787dadc442860dcbf33a82f5a`.
The inspected third-party driver worktree has unrelated local logger changes.
This version does **not** claim an exact installed binary/source hash.

## Reason for this policy

The motor-off 30-scan archive passed JSONL digest validation and had
monotonic ROS-header-stamp intervals of 99.987–100.029 ms. The median
`received_at_utc - LaserScan.header.stamp` was 0.519 ms.

Read-only source audit established three **different** timing meanings:

1. **SDK packet / point time:** `GetSystemTimeStamp` uses Pi
   `std::chrono::system_clock::now()` nanoseconds. The SDK interpolates
   point timestamps between host packet-processing callbacks.
   Neither value is demonstrated to be an actual hardware laser exposure
   time; point stamps are not present in the published ROS LaserScan.
2. **ROS source/header timestamp:** `demo.cpp` takes
   `start_scan_time = node->now()` inside
   `ToLaserscanMessagePublish`, then assigns
   `LaserScan.header.stamp = start_scan_time`.
   This marks ROS conversion-path time, **not** physical acquisition.
   `scan_time` is measured between conversions;
   `time_increment = scan_time / (beam_size - 1)` is an approximation.
3. **Adapter receive time:** the diagnostic subscriber records
   `datetime.now(timezone.utc)` independently at receipt. The difference
   from ROS conversion time measures only software timestamp separation
   **when** clocks are comparable. It is NOT full sensor latency.

Do not replace or silently rename any of these clocks as
`hardware_acquisition_time`.

## Internal draft interface

Implementation: `ros2/sie_ros2/sie_ros2/ld19_timestamp_policy_v1.py`.
Tests: `ros2/sie_ros2/test/test_ld19_timestamp_policy_v1.py`.

`TimestampPolicyV1` requires an explicit nonempty `policy_id` and
three **nonnegative integer nanosecond** thresholds. **There are no
approved default operating limits**:

- `max_receive_age_ns`: maximum `evaluation_time_utc_ns -
  received_at_utc`. Host-side freshness since diagnostic receipt.
- `max_future_skew_ns`: maximum allowed receive-time lead over
  evaluation time or ROS conversion-time lead over receive time
  (only if clock domains are verified).
- `max_conversion_to_receive_ns`: maximum host delay from
  `LaserScan.header.stamp` to adapter receive timestamp when clock
  domains are verified. Not a physical sensor latency bound.

The function `evaluate_ld19_timestamp(record, ...)` accepts explicit
`evaluation_time_utc_ns`, `previous_source_stamp_ns` (last
**diagnostically passing** stamp, or `None`) and
`same_host_clock_domain_verified` (defaults to **False**). It never
reads the current wall clock, modifies state, looks up TF or publishes
ROS messages. It validates exact diagnostic status, topic,
`ld19_sensor`, source/header timestamp consistency and an existing
evidence digest link. It **does not** recompute the payload SHA-256;
integrity remains the v0 adapter/independent audit's responsibility.

An explicit `same_host_clock_domain_verified=True` is an external
test assertion, not a fact established by this module. It is valid only
after verifying the ROS node clock is based on the same host UTC system
clock as the adapter (e.g. not simulated ROS time). Do not pass it on
unverified hardware or across independent hosts.

**Outcome codes:**

- `INVALID_SOURCE`: malformed/untrusted metadata, wrong topic/frame,
  inconsistent stamp or unsafe input admission.
- `DUPLICATE_SOURCE_STAMP` and `OUT_OF_ORDER_SOURCE_STAMP`:
  non-forward source stamps compared with explicit prior passing stamp.
- `RECEIVE_TIME_IN_FUTURE`, `STALE_RECEIPT`: host receipt age or
  future skew violates the explicit policy.
- `CLOCK_DOMAIN_UNVERIFIED`: metadata otherwise valid, but clock
  comparison is not proven; **fail closed**.
- `ROS_STAMP_AHEAD_OF_RECEIPT`,
  `CONVERSION_TO_RECEIVE_TOO_OLD`: verified host clock domain but
  conversion-to-receipt timing violates the policy.
- `HOST_TIMING_WINDOW_PASS_DIAGNOSTIC_ONLY`: **only** the defined
  host clock and receipt-time window passed. It does not prove laser
  physical timing, safety-grade freshness, or cross-sensor alignment.

On every outcome, including the last one, the returned record has:
`measurement_authorized=False`,
`world_state_update_authorized=False`,
`motor_control_authorized=False`. The returned
`next_accepted_source_stamp_ns` advances **only** on the
diagnostic timing-window pass; on refusal it preserves the prior
high-water mark. Deliberate reset of that mark after clock discontinuity
requires explicit policy/session review, not silent recovery.

The record also states source clock `ROS_NODE_CLOCK`, source stamp
meaning `ROS_SCAN_CONVERSION_TIME_NOT_ACQUISITION`, SDK host packet
clock semantics, receive clock `UTC_HOST_SYSTEM_CLOCK`, and
`physical_acquisition_time=UNKNOWN`. This is an implementation-specific
internal diagnostic result, not a new cross-module SIE Data Contract.

## Synthetic validation

The tests use controlled timestamps and **test-only** limits of
`max_receive_age_ns=250000000`,
`max_future_skew_ns=5000000`,
`max_conversion_to_receive_ns=20000000`. These are arbitrary
test fixtures. They are **not** admitted SIEE sensor-fusion or
motion-safety operating values.

Coverage includes valid host timing window, duplicate, out-of-order,
stale, future receipt, ROS conversion ahead of receipt, excessive
software delay, unknown clock domain, exact threshold boundaries,
clock rollback, inconsistent frames/timestamps/evidence/admission,
invalid policy parameters, deterministic replay and unconditional
downstream no-admission.

No hardware read, launch modification, motor command, SDK patch,
calibration change or World State update is part of these tests.

### Operator verification on Pi

The code is on the **draft PR #13 branch**, not merged. Use a new
isolated worktree (or an independently checked-out test branch),
leaving the existing LiDAR, motor and main SIE checkouts unchanged.

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/isolated/checkout
python3 -m py_compile ros2/sie_ros2/sie_ros2/ld19_timestamp_policy_v1.py \
  ros2/sie_ros2/test/test_ld19_timestamp_policy_v1.py
python3 -m pytest -q ros2/sie_ros2/test/test_ld19_timestamp_policy_v1.py
```

Source-level tests do not require live LiDAR or ROS graph activity.

## Offline verification on Pi (operator-reported, 2026-11-25)

**FACT:** The operator fetched exact draft branch HEAD
`d2fdb70eb400fe335ddc0bcb95ea24b41792cc4f` into a new
detached worktree
`/home/elwis/dev_ws/sie_ld19_timestamp_v1_20261125`.
The fetch/checkout reported the expected SHA. Python syntax checks
on `ld19_timestamp_policy_v1.py` and
`test_ld19_timestamp_policy_v1.py` succeeded (the shell continued
under `set -e`).

**FACT:** From that isolated worktree, the operator ran:

```bash
export PYTHONPATH="$PWD/ros2/sie_ros2${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q \
  ros2/sie_ros2/test/test_ld19_timestamp_policy_v1.py \
  ros2/sie_ros2/test/test_ld19_diagnostic_evidence.py
```

Output: `34 passed in 0.08s`,
`SIEE_LD19_TIMESTAMP_V1: PASS`; Git status output contained
no file modifications in the isolated worktree.
The 34-test total covers **both** the new synthetic timestamp
tests and the six previously verified v0 diagnostic adapter tests.
No ROS graph, LiDAR hardware, physical clock-domain verification
or motor run was part of this test.

**NEXT:** Use the **existing private 30-scan JSONL** in a read-only,
offline replay. Evaluation time must be anchored to the original
per-record receipt time rather than the current wall clock;
otherwise every historical record would trivially be stale.
Keep `same_host_clock_domain_verified=False` because the v0
journal contains no independent proof of its ROS time domain.
Expect `CLOCK_DOMAIN_UNVERIFIED` for structurally valid,
within-window records; do **not** report this as a timing-policy
admission failure or enable a production threshold. No raw
scene scan samples should be uploaded to public GitHub.

## Real 30-scan offline replay (operator-reported, 2026-11-25)

**FACT:** The operator replayed the same archived private
`ld19_diagnostic_v0_20261125T132230Z.jsonl`, using source HEAD
`d2fdb70eb400fe335ddc0bcb95ea24b41792cc4f` in the
isolated `sie_ld19_timestamp_v1_20261125` worktree.
The replay rechecked the original full-file SHA-256
`d647283c478600c02574a2a6ff3cea38ccbd662200dca3444b45a46df8e5d158`
and the canonicalized scan SHA-256 for every record.

Operator output:
- `RECORDS: 30`
- `FILE_SHA256: PASS`; `SCAN_SHA256: PASS`
- `SOURCE_STAMPS_MONOTONIC: PASS`
- `CLOCK_DOMAIN_VERIFIED: False`
- `STATUS_COUNTS: {'CLOCK_DOMAIN_UNVERIFIED': 30}`
- `SYNTHETIC_STALE_CHALLENGE: PASS`
- `MEASUREMENT_ADMISSION: DISABLED`
- `WORLD_STATE_UPDATE: DISABLED`
- `MOTOR_CONTROL: DISABLED`
- `SIEE_LD19_REAL_REPLAY_V1: PASS`

The replay selected an **artificial** evaluation time of each stored
receipt timestamp + 10 ms to test the historically recorded samples;
this is **not** a measured acquisition/transport delay. Its
250-ms receipt-age, 5-ms future-skew and 20-ms conversion-to-receipt
limits are explicit **synthetic test values**, not authorized operating
limits. With no independently verified same-host ROS/UTC clock domain,
all 30 valid records correctly **failed closed** at
`CLOCK_DOMAIN_UNVERIFIED`. This is the intended behavior, **not**
a claim of 30 actual clock failures or inability of the LiDAR to scan.

**No result establishes physical acquisition timestamps, camera/LiDAR
synchronization, authorized freshness, validated lidar-only
Measurement, or actuator safety.** No new scans or motion occurred.

## Read-only camera clock-source inspection (source evidence, 2026-11-25)

At the current PR branch, the native camera capture path is:
- `vision_core/tools/stream_v4l2_mjpeg.c`: extracts
  `v4l2_buffer.timestamp` into `driver_timestamp_s`, and samples
  `CLOCK_MONOTONIC` after `VIDIOC_DQBUF` into
  `host_dequeue_monotonic_s`. Its JSON has `sequence`,
  `flags`, both timestamps and a copied immutable JPEG payload.
- `ros2/sie_ros2/sie_ros2/latest_v4l2_capture.py`:
  `validate_packet` requires the driver flags
  `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC` and
  `V4L2_BUF_FLAG_TSTAMP_SRC_SOE` (driver-declared
  start-of-exposure), and `driver_timestamp_s <=
  host_dequeue_monotonic_s`. Native snapshots retain
  driver sequence and metadata; pair skew is computed as
  the difference between AR0234 and OV9281 V4L2
  monotonic timestamps. The code explicitly declares
  `physical_exposure_timing_independently_verified=False`.
- `ar0234_ov9281_control_preview_node.py` uses native
  `driver_timestamp_s` for its camera frame-time comparisons
  while capturing a separate UTC `stamp` for the host cycle.
  The less-specific standalone `ar0234_observation_node.py`
  uses `datetime.now(timezone.utc)` after `capture.read()`;
  do not conflate these two camera capture profiles.

**Critical cross-sensor distinction:** the LD19 ROS
`LaserScan.header.stamp` uses the ROS node clock (typically
host wall/ROS time; simulation mode must be checked), while the
native camera source timestamps are driver-declared
`CLOCK_MONOTONIC` values. **These time values must not be
subtracted directly**; doing so would compare different clock
domains. A future adapter needs a measured, versioned
`CLOCK_REALTIME ↔ CLOCK_MONOTONIC` mapping on the **same Pi**
with a finite uncertainty bound (e.g., bracketing clock reads),
clock-step/jump detection and explicit source/receive provenance.
Such a bridge still cannot retroactively prove laser physical
acquisition time; it remains diagnostic until evidence and a
canonical Data Contract are reviewed.

**NEXT VERIFIABLE STEP:** Use read-only Pi diagnostics to confirm
`use_sim_time` for the running LiDAR ROS node and collect a short
series of bracketed `CLOCK_REALTIME/CLOCK_MONOTONIC` pairs on
the same host, without changing any camera or LD19 driver and
without motor power. Treat readings as a proposed host-clock
bridge, not sensor fusion, until cross-modal evidence is tested.

## Raspberry Pi host clock bridge diagnostic (operator-reported, 2026-11-25)

**FACT / bounded host-clock sampling:** The operator executed a
read-only Python script that bracketed each `CLOCK_REALTIME` sample
with two `CLOCK_MONOTONIC` reads, using their midpoint to compute
`offset_ns = realtime_ns - monotonic_midpoint_ns`.
There were **200 samples** at nominal 10-ms spacing (~2 s);
no camera, LD19 or motor capture was needed.

Reported output:

- `SAMPLES: 200`
- `OFFSET_MEDIAN_NS: 1795611548984341504`
- `OFFSET_SPAN_NS: 13601` (13.601 µs between observed extremes)
- `OFFSET_FIRST_LAST_DELTA_NS: -417` (-0.417 µs)
- `BRACKET_MEDIAN_NS: 982` (0.982 µs)
- `BRACKET_MAX_NS: 18907` (18.907 µs)
- `SIEE_HOST_CLOCK_BRIDGE: MEASURED`

These readings **demonstrate a sampled mapping** between host wall
and monotonic clocks on this Pi over the short observation window.
The measured offset span is **not a validated clock-mapping
uncertainty bound**, not a long-term drift estimate and not a
camera/LD19 physical measurement-time error. Per-sample bracket
durations provide a lower-level timing diagnostic; scheduling,
clock adjustments, V4L2 timing semantics, and the LD19
acquisition-to-ROS conversion interval have not been bounded.

**FACT / ROS graph and parameter-service diagnosis (2026-11-25):**
The live Pi process list identified the LD19 publisher as PID `1282`,
executable `~/siee_lidar_ws/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node`,
name `/siee_ld19`; the process remained alive for over an hour.
`ros2 topic info /sie/lidar/scan_raw --verbose` found one
`sensor_msgs/msg/LaserScan` publisher, with zero subscribers after
the bounded diagnostic collector had exited.

The second bounded ROS discovery check returned:
- `ROS_DOMAIN_ID=0`; `RMW_IMPLEMENTATION=default`;
- `ros2 node list --no-daemon` listed
  `/sie_navigation_contract`, `/sie_perception_contract`,
  `/sie_supervisor_contract`, `/siee_ld19`,
  `/siee_ld19_static_tf`; `NODE_LIST_EXIT_CODE=0`;
- `ros2 service list -t` found all seven expected
  `/siee_ld19` parameter services; exit code `0`;
- `ros2 param get /siee_ld19 use_sim_time` had exit code `124`
  (12-second timeout); **no parameter value was returned**;
- the active `/tmp/launch_params_2hnhwcyf` had no explicit
  `use_sim_time` entry (per operator grep).

**FACT / source-level explanation:** The matching third-party
`ldrobotSensorTeam/ldlidar_stl_ros2` `src/demo.cpp` at local
checkout HEAD `bf668a89baf722a787dadc442860dcbf33a82f5a`
calls `rclcpp::WallRate r(10)` (line 113), loops in
`while (rclcpp::ok())` and `GetLaserScanData(...)`
(lines 118-133), then calls `r.sleep()`. The file contains
**no** `rclcpp::spin()`, `spin_some()` or executor spin
for the created ROS node. This explains why graph-discoverable
parameter services may time out: callback processing is not
serviced by the publisher's foreground loop. No driver patch or
restart was attempted.

**CORRECTION / timing semantics:** The previously observed extremely
stable ~100-ms ROS header-stamp intervals also track the explicit
`WallRate(10)` publishing loop. They do not independently
measure true LD19 rotation period, SDK accumulation frequency,
laser hardware timestamp or scan drop rate.

**OPEN:** Because the parameter service cannot be queried, the
**runtime** `use_sim_time` value remains unverified. Its absence
from launch overrides and the standard node-clock default are
consistent with system-time operation but do not constitute a
live parameter read. Similarly, the host clock bridge measurement
does not prove the exact time from laser acquisition to ROS
conversion. Keep `same_host_clock_domain_verified=False` in
the diagnostic evaluator until a separately documented test
establishes clock compatibility. Do not waste additional runs
on the unchanged blocking parameter service.

**NEXT VERIFIABLE STEP:** Build a separate, pure-Python experimental
host timebase conversion helper with explicit
`CLOCK_REALTIME` and `CLOCK_MONOTONIC` samples and uncertainty
metadata; test jump/reorder/expired-bridge refusals. If a live
ROS time sanity check is required, use a **read-only** subscriber
to inspect scan header timestamps against host wall-clock time
and keep the conclusion limited to consistency, not a parameter
service confirmation. Do not change the working driver, cameras
or motors.

## Engineering gates before any adoption

1. Confirm actual driver revision, ROS `use_sim_time` and shared clock
   domain at the host when diagnosing real scans. The current v0
   evidence file does not contain a verified clock-domain claim.
2. Review timestamp policy thresholds against real camera/LiDAR
   correspondence and explicit uncertainty; do not extrapolate
   the ~0.519 ms software timing into physical acquisition delay.
3. Version and review a canonical LD19 `Observation` Data Contract
   under ES-004/ARCH-001 and Evidence Graph linkage. This draft
   cannot authorize canonical `Measurement`.
4. Do not use this policy result for obstacle safety, motion, navigation
   admission, sensor fusion or automatic World State modification.

`PROJECT_CONTEXT.md` and the Graphify handoff remain unchanged; their
coordinated update requires the operator's explicit `обнови документ`
command.
