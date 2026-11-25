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
