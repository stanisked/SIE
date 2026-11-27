# Step 74 | Live Entity-Centric Integration v1

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Add an opt-in diagnostic
companion to the existing motor-disabled perception pipeline. Step 73's
offline replay remains a separate tool.
[Step 73.4 closure and Pi evidence](../handoffs/SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md#step-734--offline-entity-integration-complete)

## Live evidence and source independence

The companion runs in a separate process. ROS callbacks hand off immutable
serialized messages to two existing latest-wins diagnostic writers, one
per source path. Each path has one processing item and at most one pending
item. Deserialization, projection, publishing, and disk writes run on the
respective worker. A slow camera task does not occupy the LD19 worker.
The camera producer continues to publish through its original interface.
The companion starts no drivers, runs no inference or SGBM, and requires
no Step 71 snapshots.

- **Camera.** The existing AR0234 audit supplies the same-cycle semantic
  instance, mask reference, track confirmation or current RGB continuity,
  and camera-packet identity. The observer consumes existing live
  RGB-mask-owned OV9281 spatial estimates. It preserves the actual
  half-resolution method, calibration hashes, source timing, quality, and
  units. A bound stable torso-core estimate remains a partial longitudinal
  known term in `robot_longitudinal_axis_from_front_plane`, in metres. A
  coherent mask-depth estimate retains its optical-Z semantics in
  `rectified_left_optical_frame`, in metres. The live quantities have
  different methods and datums from Step 73's full-resolution component.
- **Temporal checks.** Run the camera producer and companion on the same
  Pi and boot. The observer checks the selected camera packet identities,
  V4L2 monotonic timestamp flags, sequence and receipt provenance, and
  retained producer bindings. Spatial evidence requires an actual camera
  pair from the same selection within the existing 25 ms trial skew. At
  the host publication-preparation endpoint, reuse the existing 250 ms
  camera freshness trial. Preserve stale and reused dispositions without
  presenting them as current candidates. The existing Step 57 causal
  zero-wait comparison runs separately. Step 74 neither waits for that
  journal nor claims that the actual pair won its offline comparison.
- **LD19.** The independent worker converts each available
  `/sie/lidar/scan_raw` callback using the existing diagnostic converter
  and Step 73 geometry adapter. Valid beams preserve bearings in radians
  and range and XY in metres in `ld19_sensor`. Invalid beams provide no
  free-space evidence. The worker retains receipt sequence, monotonic
  receipt, ROS header and digest. It flags duplicate or reversed headers
  within the session. The 250 ms local receipt-age screen reports queuing
  delay from the observer callback. Header age is unassessed without a
  comparable clock, and physical laser acquisition time remains
  `UNKNOWN`. The existing Step 57 header policy is unchanged.
- **Entities.** One current, uniquely bound camera target and stereo
  estimate can support a provisional entity through Step 73's diagnostic
  integration. A source track ID remains scoped to the producer and
  observation. A missing or ambiguous spatial estimate leaves semantic
  evidence available. The separate LD19 stream carries no camera join,
  Surface-Level test, synchronized three-source observation, or entity
  membership claim.

## ROS 2 interface

The producer of both new topics is the explicitly launched
`sie_live_entity_centric_diagnostic` companion. Both use
`std_msgs/msg/String` carrying diagnostic JSON under
`sie.live_entity_centric.v1.diagnostic`. QoS is volatile, best effort,
keep last depth 1. The camera audit publisher and LD19 publisher keep
their existing QoS. The companion subscribes with compatible best-effort
depth 1 to the camera `std_msgs/msg/String` audit and the LD19
`sensor_msgs/msg/LaserScan`. Expected consumers are an operator and offline
diagnostic tools.

| Topic | Messages and freshness |
| --- | --- |
| `/sie/diagnostics/entity_centric/camera` | One diagnostic event per processed camera audit, including target loss and refusal. The row contains the actual camera pair and independent stamp ages. |
| `/sie/diagnostics/entity_centric/obstacles` | One diagnostic event per processed LD19 callback, including invalid, reused or stale dispositions. Source-local scan geometry is independently available, with nominal host receipt freshness and unknown header age. |

The companion is off unless launched explicitly. It has no approved Fact,
Metric Measurement, World State, Decision Engine, navigation, or actuator
interface. Admission and authorization flags are false. No calibration,
track lifecycle, camera selection, Step 57 temporal policy, or motor
behavior changes.

## Timing and comparable modes

The same launcher starts the same 30-second camera producer in both
`baseline` and `enabled` modes. The baseline companion observes source
cadence and camera producer dispositions. It does not compose entities,
project LD19 geometry, or create the new ROS publishers. The baseline
companion's subscriptions and camera parsing are part of that measurement.
The original camera launcher alone is the fully absent-companion case.

The evidence directory contains two `preview/*/events.jsonl` files and
`preview/summary.json`. Compare callback frequency, inferred RGB and
spatial producer update frequencies observed by the subscriber, accepted
entity and obstacle rates, and actual successful publisher submissions.
Rates use first and last monotonic sample instants and distinguish observed
callback counts from producer timestamps and DDS delivery. Each event
records queue wait, worker processing cost, callback-to-preview time, and
source ages at publication preparation. Camera records include
producer packet-receipt-to-preview intervals. LD19 records use callback
receipt age. JSON serialization and DDS send time are outside the reported
pre-publication processing interval. Physical exposure and laser acquisition
latencies remain unverified. The summary includes stale and reused
dispositions, latest-wins replacement counts, queue occupancy, and errors.

## Software verification

**FACT.** 247 targeted tests and seven subtests passed, including 16
focused Step 74 cases. The bounded-worker test held the camera worker
while LD19 processed independently and replaced 78 pending scans under
a controlled burst. Python compilation and Bash syntax checks passed.

**FACT / synthetic ROS smoke.** The local ROS 2 Lyrical test sent the same
generated 20 camera audits and 60 scans in each mode over localhost. Both
modes received all inputs, observed about 10 Hz RGB and spatial producer
updates and about 30 Hz scan input, and had zero pending replacements.
Baseline advertised neither output. Enabled produced 20 camera entities
at 9.96 Hz and 60 obstacle updates at 30.05 Hz. Mean callback-to-preview
time was 1.59 ms camera and 0.69 ms LD19 in baseline, and 4.24 ms camera and
3.32 ms LD19 enabled. The test used generated camera stamps and did not
measure Pi performance, acquisition latency, or safety.

## Raspberry Pi deployment and validation commands

Prepare the cart with traction motor power physically disconnected. Leave
the unresolved ESP32 v13 fault, firmware, and authorization state alone.
Use the existing sole LD19 driver on `/sie/lidar/scan_raw` and release other
camera owners on the Pi. Keep the navigation and actuator paths stopped.
The existing launcher supplies device bindings, model, activation profile,
and calibration paths. Perform the following on the Pi running ROS 2 Jazzy.

```bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
repo=/home/elwis/dev_ws/sie_live_identity_20261119
branch=codex/siee-three-sensor-spatial-association-v1-20261126
checkout=/home/elwis/dev_ws/sie_step74_preview
read -r -p 'Exact Step 74 implementation commit SHA: ' step74_sha
[[ "$step74_sha" =~ ^[0-9a-f]{40}$ ]]
git -C "$repo" fetch origin "$branch"
git -C "$repo" merge-base --is-ancestor "$step74_sha" FETCH_HEAD
git -C "$repo" worktree add --detach "$checkout" "$step74_sha"
cd "$checkout"
test "$(git rev-parse HEAD)" = "$step74_sha"
test -z "$(git status --porcelain)"
ros2 topic info /sie/lidar/scan_raw -v
```

Confirm one LD19 publisher and the same-host camera producer. If using the
existing Step 57 same-host LD19 clock evidence, set
`SIE_TEMPORAL_LD19_CLOCK_EVIDENCE` to its current verified
`sha256:<digest>` in both runs. The independent Step 74 obstacle stream
does not use that attestation to infer acquisition timestamps.

For the first 30-second static-person trial, place one visible stationary
person in the common viewing area with the cart stationary. Run the two
modes with the same target pose, illumination, clock setup, and publishers.

```bash
bash vision_core/tools/run_live_entity_centric_v1.sh baseline
bash vision_core/tools/run_live_entity_centric_v1.sh enabled
```

Record each `Step 74 evidence:` path. In the enabled summary, check for
nonzero entity and independent obstacle updates during the same trial.
Use individual timestamps to check overlapping live activity. Zero fresh
entities leaves the first trial unconfirmed. Review timing and provenance
dispositions before any change to an existing gate.

For the subsequent trial, keep the cart stationary and have the target
move through the existing camera viewing area while staying clear of the
cart. Repeat both modes with a comparable route and pace.

```bash
bash vision_core/tools/run_live_entity_centric_v1.sh baseline
bash vision_core/tools/run_live_entity_centric_v1.sh enabled
```

Summarize either enabled run and verify its hashes. The launcher also prints
the full summary and the hashes when it exits.

```bash
read -r -p 'Step 74 evidence directory: ' run
(
  cd "$run" || exit
  sha256sum -c SHA256SUMS.txt && sha256sum -c camera-temporal.sha256 || exit
  /home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python - <<'PYTHON'
from collections import Counter
import json
from pathlib import Path
summary = json.loads(Path("preview/summary.json").read_text())
print("mode", summary["mode"])
for source in ("camera", "ld19"):
    stream = summary[source]
    print(source, "callback Hz", stream["input"]["hz"],
          "rates", {name: value["hz"] for name, value in stream["rates"].items()},
          "dispositions", stream["dispositions"], "replaced", stream["dropped_pending"])
tracks = Counter()
prior = None
changes = 0
with Path("preview/camera/events.jsonl").open() as events:
    for line in events:
        row = json.loads(line)
        track = row["producer"].get("track_id")
        if track is not None:
            tracks[track] += 1
            changes += prior is not None and prior != track
            prior = track
print("producer track occurrences", dict(tracks), "track changes", changes)
PYTHON
)
```

For the moving trial, compare producer track changes and semantic gaps
alongside live spatial availability, stale dispositions, and pending
replacements. Track identifiers describe the existing tracker output.
Independent LD19 updates should continue through any camera gaps.
