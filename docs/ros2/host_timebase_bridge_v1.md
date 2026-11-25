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

## Pi verification, current boot (operator report, 2026-11-25)

**FACT / offline validation:** Operator executed the four relevant
Python test modules from the isolated timestamp v1 checkout,
after `py_compile`. Final operator output:
`SIEE_HOST_TIMEBASE_V1_TESTS: PASS`.
No count or individual pytest trace was supplied, so this report
does **not** assert a specific number of tests or independent CI PASS.

**FACT / fresh-boot host clock samples:** The operator then invoked
`sie_ros2.host_timebase_clock_probe_v1` with 200 bracketed
`CLOCK_MONOTONIC / CLOCK_REALTIME` samples.
The collector returned:

- `SIEE_HOST_TIMEBASE_PROBE_V1: COMPLETE`;
- `BOOT_ID: 07e5f3a5-3d20-470a-8e8b-ab30fa2a9b8a`;
- `SAMPLES: 200`;
- `BRIDGE_STATUS: DIAGNOSTIC_BRIDGE_READY`;
- `OFFSET_NS: 1795619749801903953`;
- `OBSERVED_SPAN_NS: 2574` (2.574 µs);
- `EMPIRICAL_ENVELOPE_NS: 4963` (4.963 µs);
- `PHYSICAL_ACQUISITION_TIME: UNKNOWN`;
- `MEASUREMENT_ADMISSION: DISABLED`;
- `WORLD_STATE_UPDATE: DISABLED`;
- `MOTOR_CONTROL: DISABLED`;
- `SIEE_HOST_TIMEBASE_V1_DIAGNOSTIC: COMPLETE`.

**Private evidence location (Pi only):**
`/home/elwis/.local/state/sie/evidence/host_timebase_v1_20261125T152802Z_1353.json`.
The sampler printed SHA-256
`84007fd71391d83c4d18cefb59bbc8ae791058c49c9d2ff0b926e752acad336e`;
the operator's subsequent `sha256sum` command returned the
same SHA-256 for this path. This verifies the local saved bytes
against the same-session printed digest; **no off-Pi backup
has yet been established**.

**INTERPRETATION / LIMIT:** This sampled empirical bridge is
associated with the Pi boot ID above, not the earlier boot's
200-sample offset. The `4963 ns` empirical envelope refers to
spread and bracket uncertainty of the sampled host clock
mapping, **not** the uncertainty in laser acquisition time,
physical V4L2 exposure, a future long-duration holdover,
nor a guaranteed cross-sensor synchronization accuracy.
The script's fixed trial limits are unapproved test policy,
not runtime safety thresholds.

**NEXT / camera input:** Before invoking camera devices,
read-only inspect the current Pi ROS graph, V4L2 by-id
device mapping and any existing camera capture processes.
To compare AR0234/OV9281 driver-declared
`CLOCK_MONOTONIC` stamps with LD19 ROS time, obtain
new **same-boot** source metadata and a temporally adjacent
host-clock bridge. Historical camera evidence from another
boot MUST NOT be converted with this bridge. Respect the
bridge's explicit expiry and extrapolation guards. Keep
fusion/admission and motors disabled.

## Pi camera metadata conversion check (operator output, 2026-11-25)

**FACT:** Operator performed a bounded, cameras-only native V4L2
metadata test after explicitly verifying `/dev/video0`
AR0234 at 1920x1200 MJPEG/30 fps and restoring `/dev/video2`
TSTC OV9281 at 2560x800 MJPEG/60 fps. Both cameras had
no reported V4L2 device owner before start. The existing
`stream_v4l2_mjpeg.c` native capture was compiled to a
temporary executable. Both `LatestV4L2Capture` workers
ran while 200 new host clock brackets were collected;
subsequently the last live packet of each camera was
selected. JPEG payloads were **not persisted**.

Reported output:

- `BOOT_ID: 07e5f3a5-3d20-470a-8e8b-ab30fa2a9b8a`
  (matches the same-session Pi boot);
- `BRIDGE_STATUS: DIAGNOSTIC_BRIDGE_READY`;
- `OFFSET_NS: 1795619749801904074`;
- `BRIDGE_ENVELOPE_NS: 3102` (3.102 µs observed);
- `AR0234_FRAMES_RECEIVED: 17`;
- `AR0234_TIMESTAMP_STATUS:
  HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY`;
- `OV9281_FRAMES_RECEIVED: 40`;
- `OV9281_TIMESTAMP_STATUS:
  HOST_TIMEBASE_CONVERTED_DIAGNOSTIC_ONLY`;
- `CAMERA_SNAPSHOT_SKEW_MS: 16.24`;
- `SIEE_CAMERA_TIMEBASE_V1: PASS`.

**Private metadata evidence (Pi only):**
`/home/elwis/.local/state/sie/evidence/camera_timebase_v1_20261125T154936Z_1490.json`.
SHA-256 `35442d72f2d78c59df3ea108e95ba002e2e20ff200579911467d36bf7934c52f`
was emitted by the collector and reproduced independently
by `sha256sum` on that file.

The new offset differs by 121 ns from the earlier same-boot
sample; this is a bounded observation, not a long-term drift
guarantee. The native camera metadata was accepted by
the diagnostic bridge under the **experimental** policy.
The 16.24-ms pair skew is the difference between two selected
driver-declared SOE timestamps, **not** verified hardware
exposure synchronization nor camera calibration error.
Frame receive counts do **not**, alone, establish frame loss
or delivery frame rate.

**LIMIT / NEXT:** The sample and bridge prove only a successful
camera-to-host timestamp conversion for that bounded session.
The `3102 ns` empirical envelope does not bound uncertainty
of physical exposure timing, nor the LD19 laser's measurement
time. The LD19 node was previously observed stopped after Pi
restart, so there is no simultaneous triple-sensor evidence
yet. Before a three-sensor capture, check the current LD19
ROS process/serial port and restore its validated startup
profile **without** connecting actuators. Keep Measurement,
World State and motor admission disabled. No scene-image
payload or raw private clock samples were uploaded to GitHub.

## Reproducible Pi test (no hardware access)

Use an independent, clean, detached worktree at the latest
PR head. No `colcon`, ROS launch or live LiDAR is needed:

```bash
cd /path/to/new/isolated/worktree
export PYTHONPATH="$PWD/ros2/sie_ros2${PYTHONPATH:+:$PYTHONPATH}"
python3 -m py_compile \
  ros2/sie_ros2/sie_ros2/host_timebase_bridge_v1.py \
  ros2/sie_ros2/sie_ros2/host_timebase_clock_probe_v1.py \
  ros2/sie_ros2/test/test_host_timebase_bridge_v1.py \
  ros2/sie_ros2/test/test_host_timebase_clock_probe_v1.py
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
