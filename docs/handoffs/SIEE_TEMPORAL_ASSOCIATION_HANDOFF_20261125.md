# SIEE | Three-Sensor Timebase + RGB-Anchored Temporal Association | Handoff 2026-11-25

**Purpose:** verified continuation point for a NEW ChatGPT/Codex conversation. This is an **engineering handoff**, not an approved Engineering Specification, Data Contract, sensor-fusion admission or motion authorization.

**Project:** Spatial Intelligence Evidence Engine (SIEE), repository `stanisked/SIE`. English is canonical for code/interfaces; historical `sie` technical identifiers are unchanged.

**DECISION — current entry point:** [Step 57 diagnostic baseline freeze](#step-57-diagnostic-baseline-freeze-2026-11-26).

**Source state checked 2026-11-25 (GitHub):**
- PR **#13**, OPEN, DRAFT: https://github.com/stanisked/SIE/pull/13
- PR head branch: `codex/siee-ld19-observation-diagnostic-v0-20261125`.
- PR base: `codex/siee-person-follow-mvp-20261120` (not `main`).
- Base branch check at inspection: `1d1b44b62491b4271c02c316cfdc769e6b8a10c3`.
- PR head before this documentation handoff: `9fb9c5ac14979f3f0f76470bbb284a81a1840483`. New handoff/docs commits advance the head: **re-check current head on every new chat**.
- **New clean continuation branch created for the NEXT chat:** `codex/siee-live-temporal-association-shadow-v1-20261125`, initialized from verified PR #13 diagnostic/doc checkpoint; it does NOT merge PR #13 into main or change the existing person-follow runtime. Inspect its actual HEAD and clean worktree before editing.
- This diagnostic branch contains **additive** host-timebase, LD19 diagnostic and temporal association Python modules, tests and documentation. No merge into base/main is evidenced.
- Existing FAST/person-follow branch and firmware state are a **separate operational track**. Do not force-merge, rebase, push or change a different branch merely because this handoff exists.

## Canonical rules and status

Read `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, `docs/handoffs/WORKING_BASELINE.md`, `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`, applicable approved ES/Data Contracts, this handoff and the code at verified PR HEAD, in their documented priority order.

**Hard boundary:** all work in this handoff is `DIAGNOSTIC_ONLY_NOT_ADMITTED`: no canonical `Observation`/spatial `Measurement` promotion, no World State update, no Decision Engine or motor authorization, and no calibration modification. A source-data link is not a validated 3D transform. Any standalone diagnostic runner must not send UART/ESP32 motion commands. Physical motor power should remain disconnected for the next live sensor-only experiment; verify rather than assume that the operator disconnected it.

**Independent unresolved motor issue from 2026-11-23:** the latest documented ESP32 v13 `v4_2_uart_shadow_v13_brake_continuity` raised-wheel RIGHT 8° test ended in `ENCODER_LIMIT_REACHED`, latched `BOUNDED_DISTANCE_LIMIT` and FAULT (PWM 0/0), without proving an effective brake fix. Physical motor power OFF and v13 off-Pi raw-evidence backup had not been reconfirmed as of the source snapshot. The 20-Nov bounded FAST floor-follow observation does not override this newer fault. No new motor test/flash/reboot/FAULT reset is authorized by timebase results. Read `docs/handoffs/SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md`.

## Hardware and verified runtime topology

- Pi 5 `piecake`, Ubuntu 24.04 / ROS 2 Jazzy. Captures from `/dev/video0` AR0234 RGB 1920×1200 MJPG (configured 30 FPS) and `/dev/video2` OV9281 dual stereo packed 2560×800 MJPG (configured 60 FPS). Use installed calibration artefacts only in the existing operational pipeline, not this timestamp audit.
- LD19 (LDROBOT D500/STL-19P), CP2102 at `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`, 230400 baud, ROS topic `/sie/lidar/scan_raw`, `frame_id=ld19_sensor`.
- `ros2 launch sie_ros2 sie_pipeline.launch.py enable_ld19:=true ld19_port:=<by-id> ld19_baudrate:=230400` in the earlier first SSH terminal gave successful LD19 normal communication and read-only phase-1 supervisor (actuator bridge DISABLED). **Do not assume that process is still running in a later chat**: inspect actual hosts, ports, ROS publishers and motor power state before live capture.
- `LaserScan.header.stamp` from inspected LD19 ROS driver is **ROS scan conversion/publication time**, NOT physical laser beam acquisition. SDK-internal interpolated host-time per-point data are not exported in current ROS LaserScan. Physical LiDAR acquisition time `UNKNOWN`.
- Native `stream_v4l2_mjpeg.c` uses driver-declared MONOTONIC START_OF_EXPOSURE buffer stamps; those are not independently metrologically verified physical exposure times. `LatestV4L2Capture` drains MJPEG and retains only latest packet: gaps in **selected** V4L2 sequence do not alone mean camera dropouts.

## Confirmed chronological checkpoints, 2026-11-25

**FACT / Step 29 host↔camera mapping:** same-boot bracketed CLOCK_MONOTONIC ↔ CLOCK_REALTIME `DIAGNOSTIC_BRIDGE_READY` validated both cameras. Sampled bridge offset is an empirical mapping, not a future guarantee. Camera evidence SHA256 `35442d72f2d78c59df3ea108e95ba002e2e20ff200579911467d36bf7934c52f`.

**FACT / Step 33 three-sensor initial read-only probe:** simultaneous camera + LD19 data in a ~2s bridge, `SIEE_THREE_SENSOR_TIMEBASE_V1: PASS_DIAGNOSTIC_ONLY`; SHA256 `dc13132790de917185a983fce08d1fadc3a67f2a140ecf82408be1902446e5cf`. Not hardware sync.

**FACT / Steps 35–36:** 3.012s rolling bridge had 33 RGB and 55 Stereo selected frames but only 16 LD19 headers covering 1.500s, beginning +1423.055ms from window start. All 15 adjacent LD19 header gaps ~100ms, max 100.047ms; 0 gaps >150ms. Prior ~0.6s naive nearest-neighbor outliers were due to matching RGB frames **outside LiDAR header coverage**, NOT confirmed LD19 scan drops. Evidence `timing_series_v1_20261125T160558Z_1854.json`, SHA256 `5b8418486283a99c6ddbd0da5069ac3c0279f16aaff25cdcff9153e3f612f51a`.

**FACT / Step 37 warmed standalone LD19 subscriber:** first callback 8.086ms; after 3-scan warmup, **30 scans in 3s**, median ROS header gap 100.000ms, max 100.035ms, 0 gaps >150ms.

**FACT / Step 38 warmed concurrent read-only three-sensor test:** bridge `3011.770ms`; **30/30** LD19 header stamps/ROS receives inside window, max header gap `100.038ms`, max receive gap `100.086ms`, first receipt +75.689ms. RGB: 46 snapshots / 45 valid converted stamps; Stereo: 92 / 91. Nearest header differences: RGB 44 comparisons (abs median 28.366ms, P95 39.720ms), Stereo 88 (median 28.137ms, P95 35.956ms). `SIEE_THREE_SENSOR_WARMED_V1: PASS_DIAGNOSTIC_ONLY`.
- Private file `/home/elwis/.local/state/sie/evidence/three_sensor_warmed_20261125T161626Z_2035.json`
- SHA256 `6a12065b778f3e3476e8fab4286f542d0dfa6c7bbe9d79f227002817fe22f836` verified with separate `sha256sum`.

**FACT / Step 39 camera selected-sequence audit on that file:** AR0234 45 selected frames, seq 34→122, median step 2, 44 intermediate IDs unselected, selected driver-stamp median gap 68.000ms, median stamp→packet receipt 33.120ms. OV9281 91 selected, seq 65→244, median step 2, 89 intermediate IDs unselected, selected median gap 32.005ms, median stamp→packet receipt 16.233ms. Selection policy accounts for observing ~half the nominal native cadence; not an independent native-throughput/USB-loss test.

**FACT / Steps 40–42 nearest-timestamp offline matching:** independent LD19-header anchoring gave **17/30** trial triples under RGB↔LD19 ≤50ms, Stereo↔LD19 ≤50ms, RGB↔Stereo ≤25ms. RGB anchoring gave **44/44** unique nearest Stereo pairs within 25ms; **43/44** under redundant all-three-edge rule. The single false third-edge refusal (`rgb_index=42`) had RGB↔Stereo **16.185ms**, RGB↔LD19 **39.764ms**, Stereo↔LD19 **55.949ms**. The third delta is the sum of the first two due to ordering and **does not** invalidate RGB↔Stereo. One ROS LD19 header can serve two RGB anchors: 30 unique LD19 links among 44; never count reuse as independent measurements. Trial thresholds are NOT approved runtime limits.

**FACT / Step 43 reproducible offline diagnostic module:** from same private warmed metadata, SHA-verified `RGB_ANCHORS=44`, `RGB_STEREO_MATCHES=44`, `RGB_STEREO_UNIQUE_STEREO=44`, `RGB_LD19_HEADER_LINKS=44`, `RGB_LD19_UNIQUE_HEADERS=30`, `BOTH_DIAGNOSTIC_LINKS=44`. Status `DIAGNOSTIC_ONLY_NOT_ADMITTED`. Private derived file:
`/home/elwis/.local/state/sie/evidence/temporal_association_v1_20261125T163318Z_2125.json`
SHA256 `ad0864c74d8998d67ca8eee57bb8a462e0dc4620c048e9e0cb5ec8f0c0e7a213` independently rehashed.

**FACT / Steps 44–45 explicit freshness:** contract audit: camera `reference_frame=UNVERIFIED_IN_CAPTURE`, `confidence=None`, source timestamp basis and evidence IDs present, all admission flags false, original association rows had no `age_ns` field. Additive `temporal_association_freshness_v1.py` evaluates ages at *explicit caller-selected* `CLOCK_MONOTONIC` instant, independent RGB↔Stereo vs RGB↔LD19 header recency and stale/future/bridge-expiry/monotonic-rollback refusals. Pi historical replay at bridge end +10ms: 44 evaluated, **3** RGB↔Stereo recent, **3** RGB↔LD19 ROS-header recent, **3** both recent with experimental 250ms age policy. This describes an endpoint evaluation after a 3s recording, not a camera failure nor present-time freshness.
- Private `/home/elwis/.local/state/sie/evidence/temporal_freshness_v1_20261125T163927Z_2246.json`, SHA256 `65449331056ce5e2ad25c300c6403fb8ac6f4b7087db639452fc867bbea9b381`, independently rehashed.
- Operator completed a `set -e` test-and-replay script but did not paste numeric `pytest` summary. Do NOT invent exact number of passed tests.

**FACT / Step 46 causal availability audit (metadata only):** 44 RGB anchors; a Stereo selected frame had been received by every RGB packet's receipt time, with 44 unique Stereo matches and 44 independent ROS-header links using 30 distinct LD19 messages, no refusals under *trial* bounds. Immediate decision: median RGB↔Stereo abs delta `0.239ms`, max `16.185ms`; median/max older camera age `36.846/52.889ms`. Simulated extra **20ms** wait produced no additional pairs: camera age `56.846/72.889ms`. Thus **zero additional wait helped none of these historical selected packets**, not proof of live YOLO cycle or universal policy.

## Code / docs at PR #13 (verify exact current HEAD)

- `ros2/sie_ros2/sie_ros2/host_timebase_bridge_v1.py`, `host_timebase_clock_probe_v1.py`, tests.
- `ros2/sie_ros2/sie_ros2/ld19_timestamp_policy_v1.py`, evidence adapter and tests.
- `ros2/sie_ros2/sie_ros2/temporal_association_diagnostic_v1.py`
- `ros2/sie_ros2/sie_ros2/temporal_association_offline_probe_v1.py`
- `ros2/sie_ros2/sie_ros2/temporal_association_freshness_v1.py`
- `ros2/sie_ros2/sie_ros2/temporal_association_freshness_offline_v1.py`
- `ros2/sie_ros2/test/test_temporal_association_diagnostic_v1.py`
- `ros2/sie_ros2/test/test_temporal_association_freshness_v1.py`
- `docs/ros2/host_timebase_bridge_v1.md`
- `docs/ros2/ld19_timestamp_provenance_freshness_v1.md`
- `docs/ros2/ld19_diagnostic_evidence_v0.md`
- `docs/ros2/temporal_association_diagnostic_v1.md`

Existing live observation path, inspected on the PR branch:
- `ros2/sie_ros2/sie_ros2/latest_v4l2_capture.py`: `select_latest_pair()` vs `select_new_rgb()` / `select_stereo_during_rgb()`; these are different policies, do not silently replace.
- `ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py`: native capture and per-cycle `frame_freshness`, topics and journals; online current-target segmentation and depth must remain unchanged on diagnostic-only integration.
- `ros2/sie_ros2/launch/sie_pipeline.launch.py`: LD19 opt-in `enable_ld19:=true`, phase-1 actuator bridge disabled.
- Existing FAST launcher can use motors through separate paths; do **not** launch it with `--execute` and do not quietly change its `run_shadow_uart.py` path for timing work.

## What IS and IS NOT accepted

**FACT:** same-host bracketed timebase, warmed concurrent metadata collection and timestamp provenance, source-available causal pairing on selected historical camera packets, separate LD19 ROS header link, explicit-time trial freshness/refusals and private SHA evidence. Camera-to-camera metadata closeness ~0.239ms median in selected RGB-anchor pairs.

**NOT VERIFIED:** synchronized physical camera exposure; physical laser point acquisition timing; ROS `use_sim_time` state of the driver through actual parameter query; camera optical `reference_frame` from the timing file; laser-to-camera extrinsics/applicability; real 3D fusion accuracy; offline test counts when pytest output not provided; live end-to-end scheduler/YOLO latencies with 3 sensors; production thresholds; perception-to-motor timing and safety. No new Graphify AST extraction: latest previously documented extraction remains **2026-11-16**, `db8097ea4239dcf45ef7598c831a45648d2498fd`.

## Live Temporal Association Shadow Monitor v1 implementation, 2026-11-26

**FACT** The preview has an opt-in `preview_temporal_shadow_enabled` parameter, default `false`. It uses the existing native camera selectors and subscribes to `/sie/lidar/scan_raw` on a separate ROS node and executor. The journal records the actual selected camera sequences, driver-declared exposure stamps, host dequeue and packet receipt, selection request and completion, result-ready time before serialization, and the time the preview publisher call returns. It records the LD19 header stamp and subscriber-local receipt sequence and time. Physical laser acquisition time is `UNKNOWN`.

**DECISION** The zero-wait cutoff is receipt of the selected AR0234 packet on the Pi's monotonic clock. Search only metadata already received at that cutoff and independently find the nearest fresh OV9281 stamp and LD19 conversion header. The trial bounds are 25 ms for stereo, 50 ms for the LD19 header link and 250 ms for source age. The session samples the existing Host Timebase Bridge on the local boot and uses only the most recent bridge observation available at each endpoint. Missing, invalid or expired clocks, future and stale stamps, and incomplete causal history produce explicit refusal statuses. The LD19 header link refuses by default with `ROS_CLOCK_DOMAIN_UNVERIFIED`. `preview_temporal_ld19_clock_evidence` accepts a current evidence reference attesting that the sole publisher uses the same host's realtime clock. The monitor records the attestation and requires one discovered publisher at callback time. `live-shadow:` references identify private session metadata. Every row stays `DIAGNOSTIC_ONLY_NOT_ADMITTED`.

**DECISION** The monitor keeps 128 scalar records per camera, 64 LD19 headers and 64 clock windows. The asynchronous journal has eight pending slots and drops incoming records when full. It writes `control-preview-<run_id>/temporal-association-shadow.jsonl` with mode `0600`, including queue drops, evaluation errors and write errors. Shutdown reports the counters even when the file cannot be created. The launcher `vision_core/tools/run_live_temporal_shadow_v1.sh` starts the existing mask-based camera preview and opt-in subscriber, uses a dedicated preview topic outside the existing Shadow bridge inputs, and expects an existing LD19 publisher. Its default run is 30 seconds with the existing latest-pair selector and new-RGB triggering. `SIE_TEMPORAL_CAMERA_SELECTION=fresh_stereo_during_rgb` selects the existing overlap workflow. `SIE_TEMPORAL_LD19_CLOCK_EVIDENCE` supplies the optional evidence reference.

**FACT** Local verification on baseline `bdb29621d698d8e3192ef5fc6c0eeff59f6e2c97` plus this change passed 121 targeted tests and 2 subtests. The eight changed Python files compiled, the launcher passed Bash syntax validation, and the existing native helper compiled on the development host. No historical experiment, live sensor run or motor test was performed. The current publisher's ROS clock domain, live diagnostic overhead and sensor association yield remain unverified.

## Step 57 diagnostic baseline freeze, 2026-11-26

**DECISION** The project owner closed Live Three-Sensor Temporal Association at
the diagnostic level on 2026-11-26 as
`TEMPORAL_ASSOCIATION_DIAGNOSTIC_COMPLETE`. Preserve the existing
`latest_pair` camera selection, with new-RGB arrival triggering for the
frozen run, and introduce no additional camera association wait. Preserve
current AR0234 and OV9281 calibrations, segmentation, tracking, stereo depth,
Vision Core processing and control-preview behavior. Preserve independent
RGB–Stereo and RGB–LD19 associations and bounded asynchronous diagnostics.

**FACT** Working branch `codex/siee-live-temporal-association-shadow-v1-20261125`.
Monitor implementation `c2403f8b7b0771bc7c3c3858b05387714db90df5`.
Deployment baseline `23dbfdb4a60fbdca8eec5480d9d19751a7e2c317`.
Git ancestry and the one-line deployment correction were verified. The
correction resolves `comparison.json` from the parent of the ONNX model
directory. The implementation task reported 121 targeted tests and 2 subtests
passing, compilation of 8 Python files, Bash syntax validation and a native
V4L2 build. Step 57 independently reproduced the 121 tests, 2 subtests and Bash
syntax validation on the deployment baseline. The previous compilation results
remain prior evidence. No new sensor or motor experiment was run for Step 57.

### Evidence inventory

**FACT** The operator reports traction motor power was physically disconnected
for these experiments. Pi evidence is private. The attempted read-only Pi
connection could not reach the host. The final journal was available as a
local copy and matched the recorded SHA-256.

| Evidence | Original Pi path and SHA-256 | Verification |
| --- | --- | --- |
| First live integration | `/home/elwis/.local/state/sie/evidence/live_temporal_shadow_jHvElzbt/control-preview-23196e9a573d959e/temporal-association-shadow.jsonl` · `68bdd1210aa9acb60da3eca203ea8708c675dbbdc4bd7f3d5663d8c81aeed14e` | `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE` |
| LD19 clock evidence | `/home/elwis/.local/state/sie/evidence/ld19_live_clock_evidence_1795684354520643532.json` · `669ad80b115d4bfbeb2a576504147ed2cca801e495a5a458d00b0910e7dfb3c7` | `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`. The final journal references this digest. |
| Final three-sensor journal | `/home/elwis/.local/state/sie/evidence/live_temporal_shadow_Vw2GO5LR/control-preview-480bb7d8aba07a4e/temporal-association-shadow.jsonl` · `0fad36deec285929166ca4679b7dad833aa09a89027f9abc8b08f9b2ef785938` | SHA-256 matched local copy `/home/kestabor/SHADOW_TESTS/temporal-association-shadow.jsonl`. Metrics recalculated from that copy. |

**FACT / operator report** The first session had 50 cycles, 49 RGB–Stereo
diagnostic matches, 49 cycles with LD19 candidates, an unverified LD19 clock
domain, median RGB publication age about 312.9 ms and zero journal drops or
monitor errors.

**FACT / operator report** The clock session observed 30 unique, strictly
increasing ROS headers from one local publisher. Header-to-callback receipt
separation was 0.316446 / 0.3520755 / 0.796911 ms (min / median / max), median
header interval 99.999741 ms and maximum bracket 2352 ns. Boot ID
`a1702b2f-2b79-41d8-aa13-bfa13cb568d9` agrees with the final journal's
`boot_id`. The inspected driver revision was
`bf668a89baf722a787dadc442860dcbf33a82f5a` with local logger changes.
The reported active binary SHA-256 was
`b00b2aba235db265e7577c5fc080c86101ac0f1ba9fdf165d5b25ab9696b19be`.
The driver uses `start_scan_time = node->now();` and
`output.header.stamp = start_scan_time;`. Launch specified
`use_sim_time:=false`. The runtime parameter query timed out. The empirical
realtime compatibility observation applies to the observed publisher instance
and host session. Publisher uniqueness alone does not establish identity continuity.

### Recalculated final-journal metrics

| Metric | Verified from SHA-matched journal |
| --- | --- |
| Rows and publication | 83 cycle rows and one session summary. 82 preview publications. |
| Independent matches at RGB receipt | RGB–Stereo 82/83, RGB–LD19 header 80/83, both 80/83. |
| Accepted absolute deltas | RGB–Stereo median 4.365 ms, max 15.661 ms. RGB–LD19 header median 34.4131855 ms, max 40.326383 ms. |
| Actual camera selection | 82/82 accepted RGB–Stereo links selected the same stereo sequence as Vision Core. |
| Journal | 83 written, zero drops and zero recorded write, evaluation, hook, clock or subscriber errors. |
| Preview results | 41 `MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY`, 41 `TARGET_UNAVAILABLE`, one `CANCELLED_SHUTDOWN`. |
| Publication rate and age | 4.05442 Hz over the first-to-last publisher-return interval. Median RGB age 252.260936 ms, masked-depth median 306.865836 ms. |
| Three-source publication freshness | 40/82 published cycles had recent RGB, zero-wait stereo and zero-wait LD19 header at the experimental 250 ms limit. |

**FACT** No material discrepancy with the supplied final-session metrics.
The cancelled row has no publication timestamp and is included in the
83-row association totals. The startup row refused both associations because
the bridge was unavailable. Two further rows exceeded the RGB–LD19 delta
limit. To recalculate, SHA-256 the journal, separate `cycle_id` rows from
the `SESSION_SUMMARY`, count `MATCH_DIAGNOSTIC_ONLY` under `zero_wait`,
and take the absolute `candidate_minus_rgb_ns` of matches only. Count
`same_stereo_as_zero_wait` only for accepted RGB–Stereo links. Use non-null
`endpoint_monotonic_ns.publication` for publication metrics and
`(N - 1) / elapsed_seconds` for rate. Read RGB ages from
`freshness_by_endpoint.publication.rgb.age_ns`, and require
`RECENT_HOST_TIMESTAMP_DIAGNOSTIC_ONLY` in the publication entries
`rgb`, `zero_wait_stereo` and `zero_wait_ld19_header` for the simultaneous
freshness numerator.

### Limits and continuation

**FACT** Camera stamps are driver-declared exposure metadata. Physical camera
exposure timing has no independent verification. The LD19 ROS header marks
conversion/publication and physical laser acquisition time remains
`UNKNOWN`. The 25 ms, 50 ms and 250 ms limits are experimental. All rows,
including the summary, keep `measurement_authorized = false`,
`world_state_update_authorized = false` and `motor_control_authorized = false`.

**OPEN** End-to-end latency and the root cause of `TARGET_UNAVAILABLE`
remain unresolved. Publication rate alone cannot establish faster full Vision
Core processing. Temporal links establish no spatial correspondence. The
camera-to-LiDAR extrinsic remains unverified. Session-local diagnostic
references require separate admission to become canonical Evidence Graph IDs.

**FACT** The existing Graphify AST artifacts `graph.json`, `GRAPH_REPORT.md`
and `graph.html` locally matched the November 16 recorded hashes. Their
source snapshot is `db8097ea4239dcf45ef7598c831a45648d2498fd` with 4,599
nodes, 13,658 edges and 300 communities. No new graph extraction occurred.
See the [Graphify provenance index](SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md).

**DECISION** Reopen temporal experiments only for a new engineering question.
The next entry point is the [SIEE Three-Sensor Spatial Association v1
handoff](SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md).
PR #13 and the MVP integration branch are outside the freeze.

## Historical continuation proposal, 2026-11-25

**Implement and run an opt-in, read-only LIVE Temporal Association shadow monitor** alongside the *existing* RGB/OV9281 person-following preview and LD19 ROS topic. Ground the design in the current `LatestV4L2Capture` and `ar0234_ov9281_control_preview_node.py` source. It should record per-cycle input `sequence`, driver stamp, packet receipt monotonic, RGB/Stereo pair-selection time, ROS LD19 header conversion stamp and callback receipt time, observation/evidence source IDs, processing-result time, host same-boot bridge policy/status, ages and explicit refuse reasons. Compare zero-wait latest-available vs current fresh-stereo-during-RGB policy **as read-only diagnostics**, not by modifying the active target/depth/control choices. Keep the sensor calibration, physical motor-power OFF, `SHADOW_IDENTITY_PREVIEW=0` unless separately enabled for a diagnostic, and all admission bits false.

Before any live capture: inspect Pi runtime, actual ROS topics, camera owners, working-tree diffs, safe motor-power state and PR head. Do not start a duplicate LD19 publisher or modify the ROS driver. If source clock bridge is expired or ROS clock meaning unverified, issue an explicit refusal rather than fabricate source freshness. Require a new source/boot clock-bracket sample for each fresh session. Preserve private raw metadata and hashes, publish only summaries/paths/hashes to public GitHub. Follow SIE's evidence-graph discipline; private file-array pointers do not become canonical Evidence Graph node IDs by assumption.

**Do not merge PR #13 or promote Diagnostic → Measurement without separate approved review.**

## Historical new-chat starter, 2026-11-25

> Сью, продолжаем **SIEE | Three-Sensor Timebase + Temporal Association | Live Shadow Integration**. Для новых правок используем ветку `codex/siee-live-temporal-association-shadow-v1-20261125` (от контрольной точки PR #13, не main), а доказательства сверяем с веткой `codex/siee-ld19-observation-diagnostic-v0-20261125` и PR #13 DRAFT. Прочитай handoff `docs/handoffs/SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md`. Репозиторий `stanisked/SIE`, исходная диагностическая ветка `codex/siee-ld19-observation-diagnostic-v0-20261125`, PR #13 DRAFT (база `codex/siee-person-follow-mvp-20261120`). Прочитай `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, `docs/handoffs/WORKING_BASELINE.md`, Graphify-handoff и новый handoff, затем проверь актуальный GitHub HEAD и инструменты. Последний подтверждённый результат: шаг 46, `44/44` причинно-доступных RGB↔Stereo и `44/44` отдельных ссылок RGB↔LD19 ROS header на старых метаданных; `0ms` ожидание не ухудшило соответствия, `20ms` добавило возраст. Физическое время лазера НЕИЗВЕСТНО; код диагностический, `Measurement/World State/motors=DISABLED`. Последний независимый ESP32 v13 тест торможения закончился `FAULT`, питание приводов после этого не подтверждено как OFF. **Задача: подготовить и проверить один следующий шаг: read-only live Shadow Temporal Association monitor** без изменения существующей сегментации/глубины/управления. Не повторяй уже завершённые офлайн-тесты и не включай двигатели. Для любого вывода укажи ФАКТ/РЕШЕНИЕ/ГИПОТЕЗА, evidence и ограничение.
