# SIEE Host Timebase Bridge v1 (DRAFT, 2026-11-25)

**Status:** INTERNAL DIAGNOSTIC ONLY, **NOT** an approved SIE Data
Contract, no production sensor-fusion or actuator admission.

## Why this is needed

The LD19 ROS driver at inspected revision
`bf668a89baf722a787dadc442860dcbf33a82f5a` stamps
published `LaserScan.header.stamp` with ROS `node->now()` while
converting a scan, not with a hardware laser exposure timestamp.
The SDK's host `system_clock` packet-processing timestamps are
interpolated for points, but not published by its current ROS
`Points2D` conversion path. The ROS driver loop uses
`rclcpp::WallRate(10)`; approximate 100-ms header intervals do
not prove the physical spin cadence.

The FAST camera capture backend reads V4L2 driver-declared
`CLOCK_MONOTONIC` start-of-exposure metadata from AR0234 and
OV9281. The physical accuracy of the driver-declared SOE flag
is not independently established. The camera and LiDAR header
stamps are on potentially different clock domains and **must not
be subtracted directly**.

## Already confirmed evidence, separate from this code

On 2026-11-25, the Pi operator sampled 200 bracketed host
`CLOCK_REALTIME` and `CLOCK_MONOTONIC` pairs over ~2 seconds:
median offset `1795611548984341504 ns`, observed offset span
`13601 ns`, first-to-last difference `-417 ns`,
median bracket `982 ns`, max bracket `18907 ns`.
These are **observed statistics**, not a certified global
clock error bound, drift model, or a permanent offset. They
**must not be reused across a Pi restart**.

The later read-only LD19 live ROS clock probe received **5/5**
`LaserScan` records with host `CLOCK_REALTIME` receive minus ROS
header-stamp deltas `0.864, 0.718, 0.660, 0.667, 0.632 ms`.
Minimum `0.632 ms`, median `0.667 ms`, maximum `0.864 ms`;
this supports same-host real-time consistency in that run,
**not** a proof of physical laser-to-host latency or
`use_sim_time` parameter value.

`ros2 param get /siee_ld19 use_sim_time` timed out (124)
despite the node and seven parameter services being discoverable.
The driver's main loop does not spin an executor and so does
not service parameter callbacks. Do not continue probing the
unchanged nonresponsive endpoint or claim it returned false.

## V1 implementation boundaries

Pure Python module:
`ros2/sie_ros2/sie_ros2/host_timebase_bridge_v1.py`.

Pure Python synthetic tests:
`ros2/sie_ros2/test/test_host_timebase_bridge_v1.py`.

Optional read-only Pi clock collector (no ROS / LiDAR / cameras):
`ros2/sie_ros2/sie_ros2/host_timebase_clock_probe_v1.py`.
Collector unit tests:
`ros2/sie_ros2/test/test_host_timebase_clock_probe_v1.py`.

The bridge consumes explicitly supplied bracketed samples:

```text
monotonic_before_ns = CLOCK_MONOTONIC sampled before
realtime_ns         = CLOCK_REALTIME sampled between
monotonic_after_ns  = CLOCK_MONOTONIC sampled after
```

It computes the offset `realtime_ns - floor((before + after)/2)`.
For even-size sample collections it computes the median with
**integer arithmetic**, avoiding float precision loss at large
nanosecond epochs.

Only after validation does it report
`DIAGNOSTIC_BRIDGE_READY` with an offset, the first/last
monotonic sample points, observed offset span, maximum bracket,
and empirical envelope
`max(abs(individual_offset - median_offset)
    + ceil(individual_bracket_width/2))`.

**The empirical envelope is not a rigorously certified future
error bound or LiDAR acquisition uncertainty.** The current
implementation has no automatic holdover/drift guarantee.
Callers must supply the unique Pi boot ID and the timebase
policy explicitly; no system clock, ROS graph, or sensor is
queried by this function. An unverified or mismatched boot ID
is not an authority token; it is a reboot-mismatch guard.

The versioned `HostBridgeTestPolicy` requires, with no
defaults, `policy_id` and nonnegative integer thresholds for
max bracket width, sample gap, observed offset span, consecutive
offset jump, between-window offset change, bridge age,
extrapolation and future stamp skew. Thresholds in tests are
**arbitrary fixtures only**, not accepted SIEE runtime values.
The bridge serializes every policy threshold as `policy_limits`;
a later conversion or cross-window comparison refuses the same
`policy_id` with different limits (`POLICY_LIMITS_MISMATCH` or
`INVALID_PREVIOUS_BRIDGE`).

A subsequent `convert_monotonic_stamp()` maps an input
`CLOCK_MONOTONIC` timestamp to an estimated host
`CLOCK_REALTIME` timestamp if the **same** boot and policy
are declared, the bridge is current, the timestamp is within
the bounded interpolation/extrapolation window and the time
is not from the future. The only success status is
`HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY`.

Refusal statuses include `INVALID_BRACKETS`,
`MONOTONIC_SEQUENCE_INVALID`, `REALTIME_CLOCK_ROLLBACK`,
`BRACKET_TOO_WIDE`, `SAMPLE_GAP_EXCEEDED`,
`CLOCK_STEP_SUSPECTED`, `OFFSET_SPAN_EXCEEDED`,
`INVALID_PREVIOUS_BRIDGE`, `BOOT_ID_CHANGED`,
`BRIDGE_WINDOWS_OUT_OF_ORDER`, `BRIDGE_UNAVAILABLE`,
`BOOT_ID_MISMATCH`, `BRIDGE_FROM_FUTURE`,
`BRIDGE_EXPIRED`, `STAMP_FROM_FUTURE` and
`STAMP_OUTSIDE_BRIDGE_WINDOW` and `POLICY_LIMITS_MISMATCH`.

Every result, including success, carries
`physical_acquisition_time=UNKNOWN`,
`measurement_authorized=False`,
`world_state_update_authorized=False`,
`motor_control_authorized=False`.

The module emits **no ROS topic** and never writes, flashes,
configures or drives any device. Its internal Python objects
cannot be treated as canonical inter-module Data Contracts
under ARCH-001 / ES-004.

## Verification status

Code and synthetic tests are committed to the existing
**draft PR #13** branch; hardware/software verification on Pi
is **pending operator output**, and the module is not installed
into the active ROS pipeline. Existing 34 PASS results belong
to the separate LD19 timestamp policy v1 and v0 adapter tests.
Do not claim this bridge v1's tests passed until actually run.

## Reproducible Pi test (no hardware access)

Use an independent, clean, detached worktree at the latest
PR head. No `colcon`, ROS launch or live LiDAR is needed:

```bash
cd /path/to/new/isolated/worktree
export PYTHONPATH="$PWD/ros2/sie_ros2${PYTHONPATH:+:$PYTHONPATH}"
python3 -m py_compile \
  ros2/sie_ros2/sie_ros2/host_timebase_bridge_v1.py \
  ros2/sie_ros2/test/test_host_timebase_bridge_v1.py
python3 -m pytest -q \
  ros2/sie_ros2/test/test_host_timebase_bridge_v1.py \
  ros2/sie_ros2/test/test_host_timebase_clock_probe_v1.py \
  ros2/sie_ros2/test/test_ld19_timestamp_policy_v1.py \
  ros2/sie_ros2/test/test_ld19_diagnostic_evidence.py
```

### Optional new-boot Pi clock evidence (after synthetic PASS)

```bash
mkdir -p "$HOME/.local/state/sie/evidence"
python3 -m sie_ros2.host_timebase_clock_probe_v1 \
  --samples 200 --interval-ms 10 \
  --output "$HOME/.local/state/sie/evidence/host_timebase_probe_v1_$(date -u +%Y%m%dT%H%M%SZ).json"
```

The collector reads `/proc/sys/kernel/random/boot_id`, then
brackets 200 host clock readings with `CLOCK_MONOTONIC`
and `CLOCK_REALTIME`. The result is an explicitly
**experimental** clock mapping with integer-valued offsets,
an observed bracket envelope and validation statuses.
The JSON file is created with mode `0600` using exclusive
create (never overwritten), and its SHA-256 is printed.
Do **not** upload its raw samples to public GitHub.
The fixed trial thresholds in this script are NOT approved
SIEE runtime limits. If `BRIDGE_STATUS` is a refusal, that
is a valid diagnostic outcome, not permission to force PASS.
No ROS node or hardware driver is started.

No active mapping should be claimed solely from the old
200-sample offset after the Pi has powered down. The
next separate task after synthetic PASS is new bracketed
read-only host-clock sampling **on the current Pi boot**,
preserving unique boot_id and sample evidence privately,
followed by bounded source-to-host camera timestamp
conversion. A real cross-modal comparison still requires
verified ROS clock-domain semantics and physical time
uncertainty, so **no Measurement or motion admission**.

`PROJECT_CONTEXT.md` and the Graphify handoff remain
unchanged; their coordinated update requires the operator's
explicit `обнови документ` command.
