# RGB-anchored Temporal Association v1 (DIAGNOSTIC DRAFT)

**Status: not an approved SIE Data Contract, NOT Measurement/Fusion/motor admission.**
Implementation is offline, pure Python, and has no ROS, camera, LiDAR, or
motor runtime dependency. It replays a private `sie.three_sensor_warmed.v1.diagnostic`
metadata snapshot. Source image and laser range-array payloads are not read.

## Verified continuation: Steps 43–46, 2026-11-25

**FACT:** The original source SHA256 `6a12065b778f3e3476e8fab4286f542d0dfa6c7bbe9d79f227002817fe22f836` was verified by Pi and the real pure-Python association replay produced `44/44` unique RGB↔Stereo links and `44/44` independent RGB↔LD19 ROS header links using 30 unique headers. Original offline association result SHA256 `ad0864c74d8998d67ca8eee57bb8a462e0dc4620c048e9e0cb5ec8f0c0e7a213`. Separate explicit-time freshness replay evaluated 44 historical rows at the **single** clock instant 10 ms after the bridge, marked 3 camera-pair links and 3 LD19-header links recent (250ms *trial* source age, not live uptime performance), SHA256 `65449331056ce5e2ad25c300c6403fb8ac6f4b7087db639452fc867bbea9b381`; ages explicit and all admission flags false. Detailed `pytest` count was not pasted, although the `set -e` test script completed.

**FACT / Step 46 causal availability check:** On the selected latest-V4L2 metadata sequence, each of 44 RGB anchors could choose a Stereo packet **received no later than** the RGB receipt instant, and a separate ROS LD19 header whose callback had already arrived. Both independent relations passed trial checks for 44/44 anchors at wait 0ms, with 44 unique Stereo and 30 unique LD19. Camera stamp abs median 0.239ms/max 16.185ms; max per-pair camera age median 36.846ms/max 52.889ms. A simulated 20ms delay produced **no extra matches**, increased age median to 56.846ms/max 72.889ms. Historical selected packets only, not live YOLO/segmentation cycle latency, no proof of sensor physical synchronization.

**DESIGN DECISION (diagnostic ONLY):** Prefer RGB-anchored **source-available** camera-pair matching, keep LD19's ROS conversion-header link independently recorded without a third-edge gate. Do not silently change existing `select_latest_pair()` or `select_stereo_during_rgb()` runtime policy; those workflows differ from the offline replay. Never promote these links to canonical Measurement/World State/motor. Live read-only Shadow integration is the next **unverified** step, not a completed test.

**Newest new-chat continuation:** [SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md](../handoffs/SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md). PR #13 remains DRAFT, based on the 20-Nov person-follow branch, separate from the unresolved ESP32 v13 FAULT.

## Motivation: verified Step 42 edge case (Pi, 2026-11-25)

Operator's SHA-verified Step 38 evidence contains 30 LD19 ROS header stamps.
An LD19-header-anchored nearest-camera trial yielded 17/30 candidate triples,
with 13 failures solely because independently chosen nearest camera frames
were ~32.25 ms apart (trial max RGB↔Stereo 25 ms).

A separate RGB-anchored replay over the same evidence had
44 RGB frames within the LD19 coverage interval,
44/44 nearest Stereo frames within 25 ms, and
43/44 satisfying **an additional arbitrary** third edge
`|Stereo - LD19 ROS header| <= 50 ms`.
Step 42 isolated the one rejected triple:

- RGB–Stereo: 16.185 ms (PASS);
- RGB–LD19 ROS header: 39.764 ms (PASS);
- Stereo–LD19 ROS header: 55.949 ms (would FAIL).

The last number equals 16.185 + 39.764 ms due to time ordering.
It is **not** a physical sensor timing error. All 44 RGB anchors chose
unique Stereo packets. All 30 LD19 ROS headers were used at least once,
at most twice.

**Trial design:** use RGB as the association anchor;
select Stereo by nearest RGB timestamp with an explicit independent
RGB↔Stereo trial check. Independently compare the RGB timestamp to the
nearest LD19 ROS conversion/publication header. Do **not** require
Stereo↔LD19 closeness to admit or refuse the *diagnostic RGB–Stereo pair*.
A LiDAR header can be reused and cannot become multiple independent
laser measurements.

## Inputs and recorded provenance

- Source file: private Pi evidence from `three_sensor_warmed_v1`.
  Caller must verify exact source SHA-256 before invoking the pure function.
- Clock mapping: same-boot `DIAGNOSTIC_BRIDGE_READY` sampled
  `CLOCK_MONOTONIC ↔ CLOCK_REALTIME`; input cameras use driver-declared
  monotonic SOE stamps plus the recorded bridge offset.
  Camera timestamps outside the bridge sample window are excluded.
- LiDAR stamp: ROS `LaserScan.header.stamp` taken at SDK→ROS
  conversion/publication, **not** time of physical laser acquisition.
  The actual node `use_sim_time` parameter was not directly read;
  cross-clock compatibility remains provisional.
- Source references: `private-capture-sha256:<digest>#camera_frames.X[index]`
  or `#ld19[index]` refer to exact JSON array locations in the
  privately verified source file. These are candidate evidence pointers,
  **not registered SIE Evidence Graph IDs**.
- `reference_frame` for camera metadata is explicitly
  `UNVERIFIED_IN_CAPTURE` because the private timing series
  contains no authoritative camera optical frame. The LD19 ROS
  message states `ld19_sensor`. **No spatial transform is inferred.**
- `confidence: null` / `confidence_basis: NOT_ESTIMATED`; no
  inferred spatial accuracy or camera–laser physical acquisition time.
- All records and top-level outputs set
  `measurement_authorized=False`,
  `world_state_update_authorized=False`,
  `motor_control_authorized=False`.
  Even a pair labelled `MATCH_DIAGNOSTIC_ONLY` never changes those flags.

The explicit 25 ms RGB↔Stereo and 50 ms RGB↔LD19
bounds are experimental reproducibility parameters, not
validated Sensor Fusion or motion acceptance thresholds.
No temporal compensation is inferred from nearest-header deltas.

## Step 43/44 replay and temporal freshness gap (Pi, 2026-11-25)

Operator-reported SHA-verified Pi offline replay of the warmed session:
44 RGB anchors, 44 unique matched Stereo frames, 44 diagnostic
RGB↔LD19 ROS-header links referencing 30 unique LD19 messages,
all without Measurement / World State / motor admission.
Private association report:
`/home/elwis/.local/state/sie/evidence/temporal_association_v1_20261125T163318Z_2125.json`,
SHA-256
`ad0864c74d8998d67ca8eee57bb8a462e0dc4620c048e9e0cb5ec8f0c0e7a213`
verified by separate `sha256sum` on Pi.

Step 44 offline audit confirmed 44 rows, 30 unique LD19
evidence pointers and max LiDAR header reuse 2,
`reference_frame=UNVERIFIED_IN_CAPTURE`,
`confidence=null`, proper driver/ROS timestamp provenance,
and all admission flags false. **Missing in this snapshot:**
an explicit `age_ns` and a stale/future refusal at
the time a downstream consumer evaluates the data.

## Temporal Association Freshness v1: independent DRAFT evaluator

A new **pure offline**, **explicit-time** evaluator is kept
separate from the Step 43 matcher to preserve that verified
historical association report byte-for-byte.
`temporal_association_freshness_v1.py` requires a versioned
diagnostic association report, its source host clock bridge,
an explicit `evaluation_monotonic_ns`, source `boot_id`,
and an explicit experimental `FreshnessTrialPolicy`.
It returns each row's `age_ns` for RGB, Stereo and LD19
**ROS header** separately, statuses/reasons for unavailable,
stale and future timestamps, plus independent pair/header-link
recency; it never uses the Stereo↔LD19 third edge as a gate.

There are whole-evaluation refusals for a future/expired
clock bridge, a changed boot ID, or monotonic evaluation
rollback. The bridge's **own** max-age policy further caps
the evaluation's trial holdover; a stale bridge cannot be
used to fabricate current ages. No device or system clock
is read inside the evaluator. All field and whole-report
admission flags remain false.

**Important:** The offline CLI's explicitly chosen evaluation
timestamp is `bridge.last_sample_monotonic_ns + N ms`
on the *original Pi boot*. It demonstrates **historical**
freshness/refusal behavior. It does NOT assess whether
the November 25 measurements are fresh *now*.
A recent ROS LD19 conversion header still does not
establish the age of any physical laser measurement.
The ROS clock's `use_sim_time` has not been read through
the running driver; cross-domain compatibility is provisional.
The camera optical reference frame also remains unverified.
These fields cannot authorize canonical SIE Measurements
or motion.

### Pi offline verification (historical command; operator replay completed 2026-11-25)

Refresh the existing clean isolated worktree from PR #13,
compile both new modules, run synthetic tests, then replay
the two SHA-verified private files. No camera, ROS or motor
process is started.

```bash
set -e
set +u
WT="$HOME/dev_ws/sie_temporal_assoc_v1_20261125"
test -d "$WT/.git" -o -f "$WT/.git"
cd "$WT"
test -z "$(git status --porcelain)" || { echo "WORKTREE_DIRTY: STOP"; exit 1; }
git fetch origin codex/siee-ld19-observation-diagnostic-v0-20261125
git switch --detach FETCH_HEAD
export PYTHONPATH="$PWD/ros2/sie_ros2${PYTHONPATH:+:$PYTHONPATH}"
python3 -m py_compile \
  ros2/sie_ros2/sie_ros2/temporal_association_freshness_v1.py \
  ros2/sie_ros2/sie_ros2/temporal_association_freshness_offline_v1.py \
  ros2/sie_ros2/test/test_temporal_association_freshness_v1.py
python3 -m pytest -q \
  ros2/sie_ros2/test/test_temporal_association_diagnostic_v1.py \
  ros2/sie_ros2/test/test_temporal_association_freshness_v1.py
E="$HOME/.local/state/sie/evidence"
python3 -m sie_ros2.temporal_association_freshness_offline_v1 \
  --capture "$E/three_sensor_warmed_20261125T161626Z_2035.json" \
  --expect-capture-sha256 6a12065b778f3e3476e8fab4286f542d0dfa6c7bbe9d79f227002817fe22f836 \
  --associations "$E/temporal_association_v1_20261125T163318Z_2125.json" \
  --expect-associations-sha256 ad0864c74d8998d67ca8eee57bb8a462e0dc4620c048e9e0cb5ec8f0c0e7a213 \
  --evaluation-lag-ms 10 \
  --output "$E/temporal_freshness_v1_$(date -u +%Y%m%dT%H%M%SZ)_$.json"
```

These are test-only 250ms per-source age limits and 1s
bridge holdover bounds. Expect some earlier rows to be
marked `SOURCE_STALE` because the historical replay
evaluates all 44 frames **at one instant after a
~3-second capture**, not at each frame's original
arrival. This must NOT be interpreted as a camera
failure. The Pi operator ran this replay successfully and verified the private SHA-256,
but no numeric pytest summary was pasted. Re-check source/boot/clock if rerunning.

## Files

- `ros2/sie_ros2/sie_ros2/temporal_association_diagnostic_v1.py`:
  deterministic, immutable-input style, pure matcher.
- `ros2/sie_ros2/sie_ros2/temporal_association_offline_probe_v1.py`:
  explicit SHA-checked offline CLI; optional create-only private JSON
  report with mode 0600 and printed SHA-256.
- `ros2/sie_ros2/test/test_temporal_association_diagnostic_v1.py`:
  synthetic checks: Step 42 arithmetic edge, independent refusal
  branches, LiDAR absence, reuse, evidence links, malformed source,
  trial policy and hard diagnostic boundaries.

## Initial Pi verification command (historical, already completed 2026-11-25)

Use an isolated worktree with current PR #13 head, not a modified
running vision workspace. This is entirely offline; LD19 launch can
remain running in another shell, but no process is opened here.

```bash
cd /home/elwis/dev_ws/sie_ld19_timestamp_v1_20261125
git fetch origin codex/siee-ld19-observation-diagnostic-v0-20261125
TEST_WT=/home/elwis/dev_ws/sie_temporal_assoc_v1_20261125
test ! -e "$TEST_WT" || { echo "EXISTING_WORKTREE: STOP"; exit 1; }
git worktree add --detach "$TEST_WT" FETCH_HEAD
cd "$TEST_WT"
export PYTHONPATH="$PWD/ros2/sie_ros2${PYTHONPATH:+:$PYTHONPATH}"
python3 -m py_compile \
    ros2/sie_ros2/sie_ros2/temporal_association_diagnostic_v1.py \
    ros2/sie_ros2/sie_ros2/temporal_association_offline_probe_v1.py \
    ros2/sie_ros2/test/test_temporal_association_diagnostic_v1.py
python3 -m pytest -q ros2/sie_ros2/test/test_temporal_association_diagnostic_v1.py
python3 -m sie_ros2.temporal_association_offline_probe_v1 \
    --input "$HOME/.local/state/sie/evidence/three_sensor_warmed_20261125T161626Z_2035.json" \
    --expect-sha256 6a12065b778f3e3476e8fab4286f542d0dfa6c7bbe9d79f227002817fe22f836 \
    --output "$HOME/.local/state/sie/evidence/temporal_association_diagnostic_v1_$(date -u +%Y%m%dT%H%M%SZ)_$$.json"
```

**Expected only if live Pi verification reproduces the operator's
metadata:** `RGB_ANCHORS: 44`,
`RGB_STEREO_MATCHES: 44`,
`RGB_STEREO_UNIQUE_STEREO: 44`,
`RGB_LD19_HEADER_LINKS: 44`,
`RGB_LD19_UNIQUE_HEADERS: 30`,
`BOTH_DIAGNOSTIC_LINKS: 44`.
These are trial associations, not canonical cross-sensor Measurements.

The separate coordinated 2026-11-25 handoff documentation update now adds
summaries to `PROJECT_CONTEXT.md` and the Graphify handoff (no Graphify
extraction). Camera calibration, LD19 third-party driver and motor code remain
unchanged by these documentation updates.
