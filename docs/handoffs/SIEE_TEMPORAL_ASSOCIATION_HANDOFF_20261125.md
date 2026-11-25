# SIEE | Three-Sensor Timebase + RGB-Anchored Temporal Association | Handoff 2026-11-25

**Purpose:** verified continuation point for a NEW ChatGPT/Codex conversation. This is an **engineering handoff**, not an approved Engineering Specification, Data Contract, sensor-fusion admission or motion authorization.

**Project:** Spatial Intelligence Evidence Engine (SIEE), repository `stanisked/SIE`. English is canonical for code/interfaces; historical `sie` technical identifiers are unchanged.

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

## Exactly one next testable step

**Implement and run an opt-in, read-only LIVE Temporal Association shadow monitor** alongside the *existing* RGB/OV9281 person-following preview and LD19 ROS topic. Ground the design in the current `LatestV4L2Capture` and `ar0234_ov9281_control_preview_node.py` source. It should record per-cycle input `sequence`, driver stamp, packet receipt monotonic, RGB/Stereo pair-selection time, ROS LD19 header conversion stamp and callback receipt time, observation/evidence source IDs, processing-result time, host same-boot bridge policy/status, ages and explicit refuse reasons. Compare zero-wait latest-available vs current fresh-stereo-during-RGB policy **as read-only diagnostics**, not by modifying the active target/depth/control choices. Keep the sensor calibration, physical motor-power OFF, `SHADOW_IDENTITY_PREVIEW=0` unless separately enabled for a diagnostic, and all admission bits false.

Before any live capture: inspect Pi runtime, actual ROS topics, camera owners, working-tree diffs, safe motor-power state and PR head. Do not start a duplicate LD19 publisher or modify the ROS driver. If source clock bridge is expired or ROS clock meaning unverified, issue an explicit refusal rather than fabricate source freshness. Require a new source/boot clock-bracket sample for each fresh session. Preserve private raw metadata and hashes, publish only summaries/paths/hashes to public GitHub. Follow SIE's evidence-graph discipline; private file-array pointers do not become canonical Evidence Graph node IDs by assumption.

**Do not merge PR #13 or promote Diagnostic → Measurement without separate approved review.**

## New-chat starter (copy into ChatGPT)

> Сью, продолжаем **SIEE | Three-Sensor Timebase + Temporal Association | Live Shadow Integration**. Для новых правок используем ветку `codex/siee-live-temporal-association-shadow-v1-20261125` (от контрольной точки PR #13, не main), а доказательства сверяем с веткой `codex/siee-ld19-observation-diagnostic-v0-20261125` и PR #13 DRAFT. Прочитай handoff `docs/handoffs/SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md`. Репозиторий `stanisked/SIE`, исходная диагностическая ветка `codex/siee-ld19-observation-diagnostic-v0-20261125`, PR #13 DRAFT (база `codex/siee-person-follow-mvp-20261120`). Прочитай `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, `docs/handoffs/WORKING_BASELINE.md`, Graphify-handoff и новый handoff, затем проверь актуальный GitHub HEAD и инструменты. Последний подтверждённый результат: шаг 46, `44/44` причинно-доступных RGB↔Stereo и `44/44` отдельных ссылок RGB↔LD19 ROS header на старых метаданных; `0ms` ожидание не ухудшило соответствия, `20ms` добавило возраст. Физическое время лазера НЕИЗВЕСТНО; код диагностический, `Measurement/World State/motors=DISABLED`. Последний независимый ESP32 v13 тест торможения закончился `FAULT`, питание приводов после этого не подтверждено как OFF. **Задача: подготовить и проверить один следующий шаг: read-only live Shadow Temporal Association monitor** без изменения существующей сегментации/глубины/управления. Не повторяй уже завершённые офлайн-тесты и не включай двигатели. Для любого вывода укажи ФАКТ/РЕШЕНИЕ/ГИПОТЕЗА, evidence и ограничение.
