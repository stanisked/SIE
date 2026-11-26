# SIE Project Context

**DECISION — 2026-11-26:** The owner froze `STEREO_LIDAR_LOCAL_SPATIAL_CORRESPONDENCE_DIAGNOSTIC_COMPLETE`. Step 71 adds source-bound AR0234 semantics to the existing OV9281–LD19 diagnostic. Preserve the calibrated artifacts, existing diagnostic policies and disabled admission flags. [Step 70 local correspondence freeze](docs/diagnostics/stereo_lidar_local_correspondence_v1.md#step-70-freeze-2026-11-26).

**DECISION — current sensor checkpoint, 2026-11-26:** The owner closed Live Three-Sensor Temporal Association as `TEMPORAL_ASSOCIATION_DIAGNOSTIC_COMPLETE` on `codex/siee-live-temporal-association-shadow-v1-20261125`. Preserve `latest_pair`, zero added association wait, current calibrations, Vision Core processing and independent bounded diagnostics. Measurement, World State and motor admission remain disabled. Read the [Step 57 evidence and limits](docs/handoffs/SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md#step-57-diagnostic-baseline-freeze-2026-11-26) and [Spatial Association v1 handoff](docs/handoffs/SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md).

**FACT** The final SHA-matched journal reproduced 82/83 RGB–Stereo, 80/83 RGB–LD19 header and 80/83 joint matches. All 82 accepted camera links agreed with Vision Core's actual stereo selection. Forty of 82 publications met experimental three-source freshness. The earlier first-live and LD19 clock files are `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`. The Graphify snapshot is still 2026-11-16 at `db8097ea4239dcf45ef7598c831a45648d2498fd`.

**OPEN** Runtime `use_sim_time`, end-to-end latency, the cause of `TARGET_UNAVAILABLE` and the camera-to-LiDAR extrinsic remain unverified. Physical laser acquisition time remains `UNKNOWN`. The next stage begins with a read-only frame and calibration inventory. The 2026-11-23 ESP32 v13 fault remains an independent blocker.

## 2026-11-25: Historical three-sensor timebase and RGB-anchored association checkpoint (FACT / DECISION / OPEN)

**FACT / provenance:** The Pi's LD19 ROS conversion/publication `LaserScan.header.stamp` is not independently measured physical laser acquisition time; physical acquisition remains `UNKNOWN`. The AR0234/OV9281 V4L2 metadata declare `CLOCK_MONOTONIC START_OF_EXPOSURE`, which was bounded to host UTC with a same-boot empirical bridge. The camera optical reference frame in this private timestamp capture is explicitly `UNVERIFIED_IN_CAPTURE`, no spatial transform asserted. ROS `use_sim_time` status of the active LD19 driver was not directly verified by a successful parameter query. Live hardware synchrony and physical laser delay are not established.

**FACT / read-only combined capture:** The initial rolling 3s series observed 16 LD19 headers in a 1.500s interior span; all adjacent headers were ~100ms apart. Therefore the previously large naive nearest-header differences came from unmatched coverage boundaries, not confirmed LiDAR publishing dropouts. A separate warmed ROS subscriber received 30 messages in 3s, max 100.035ms header gap, no gaps >150ms. After warming both camera streams and LD19 before opening the timebase window, a second 3.011770s concurrent trial captured 30/30 LiDAR receives + headers in the window (max receipt gap 100.086ms, max header gap 100.038ms) alongside 45/46 converted RGB and 91/92 converted Stereo metadata snapshots. Captures were metadata only; camera packets were not stored. Original private Pi evidence `/home/elwis/.local/state/sie/evidence/three_sensor_warmed_20261125T161626Z_2035.json`, SHA256 `6a12065b778f3e3476e8fab4286f542d0dfa6c7bbe9d79f227002817fe22f836`, independently rehashed.

**FACT / camera producer vs consumer:** V4L2 selected RGB `sequence` from 34→122 (45 rows; median step 2, selected timestamp gap 68.000ms) and stereo 65→244 (91 rows; median step 2, gap 32.005ms). This documents that the *diagnostic latest-wins selector* took roughly each second sequence number, not that cameras dropped those frames. Median source stamp→packet receipt was RGB 33.120ms and Stereo 16.233ms, not independently verified physical exposure latency.

**FACT / association:** LD19-header anchoring produced 17/30 candidates at experimental trial thresholds. RGB-anchored selection found 44/44 unique nearby Stereo frames; independent RGB↔LD19 header links for 44/44 anchors referred to 30 distinct ROS messages (max reuse 2). One apparent rejection under a redundant third-edge constraint at RGB index 42 had RGB↔Stereo 16.185ms and RGB↔LD19 39.764ms, both within trial thresholds, while Stereo↔LD19 was 55.949ms (sum of the other two). The third edge is NOT a valid reason to reject the independent RGB↔Stereo diagnostic pair. Offline `Temporal Association v1` replay reproduced 44/44 pair + 44/44 LD19 links: private output `temporal_association_v1_20261125T163318Z_2125.json` SHA256 `ad0864c74d8998d67ca8eee57bb8a462e0dc4620c048e9e0cb5ec8f0c0e7a213`.

**FACT / historical freshness:** Original association rows did not include age; an additive explicit-time `Temporal Association Freshness v1` diagnostic evaluator now does. With all 44 recorded frames evaluated at the **same** historical instant 10ms after the bridge window ended, 3/44 RGB↔Stereo pairs and 3/44 independent RGB↔LD19 header links passed 250ms **trial** source-age checks. This is expected because the record covers ~3s; NOT a 3/44 live-performance ratio. Private result `temporal_freshness_v1_20261125T163927Z_2246.json` SHA256 `65449331056ce5e2ad25c300c6403fb8ac6f4b7087db639452fc867bbea9b381`. Output `age_ns` present, all admission bits disabled. Numerical `pytest` pass count not pasted, though operator's `set -e` script reached completion.

**FACT / causal available-input replay:** Step 46 explicitly filtered Stereo packets by actual `received_monotonic_ns` <= RGB packet receipt (and ROS LiDAR by callback receipt). Immediate decision achieved 44/44 RGB↔Stereo and 44/44 independent RGB↔LD19 header links, zero refusals, 44 unique Stereo and 30 unique LD19 headers. Pair stamp abs median 0.239ms / max 16.185ms; the older camera age median 36.846ms / max 52.889ms at decision. Adding 20ms wait created no new valid pair and increased ages to 56.846ms median / 72.889ms max. This is a historical subset of *selected* camera packets and does NOT establish continuous online YOLO/segmentation performance or physical sensor synchronization.

**DECISION / diagnostic design only:** Keep RGB as target/time anchor for the experimental person-follow metadata association; find eligible Stereo from already received packets, then optionally attach a separate nearest LD19 ROS conversion-header link with explicit provenance, age/clock basis, evidence and refusal. Do not require Stereo↔LD19 proximity for the independent RGB↔Stereo pair. A reused ROS header never becomes an independent physical scan. Trial bounds (25ms RGB↔Stereo, 50ms RGB↔LD19 header, 250ms host ages) are NOT approved canonical Measurement or action policies.

**OPEN / historical next step at that checkpoint:** Implement an opt-in **read-only live Temporal Association shadow monitor** with existing `LatestV4L2Capture`/`ar0234_ov9281_control_preview_node.py` timing hooks and the existing LD19 topic, without affecting target selection, depth, activated calibration or UART motor actions. Verify physical motor-power disconnected, no duplicate camera owners or LD19 ROS publishers, clean working tree and fresh same-boot clock brackets. The existing preview's `select_latest_pair` and `select_stereo_during_rgb` have different runtime semantics; do NOT replace them solely from an offline zero-wait result. Log only metadata/Evidence IDs and host processing latencies; maintain `Measurement/World State/motors=DISABLED`. Separate actuator track remains blocked by the v13 23-Nov bounded encoder fault; existing Graphify AST remains the 2026-11-16 `db8097e` snapshot, not rescanned for these changes.

**Primary handoff:** [SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md](docs/handoffs/SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md); **detail:** [temporal_association_diagnostic_v1.md](docs/ros2/temporal_association_diagnostic_v1.md); **PR:** https://github.com/stanisked/SIE/pull/13 (DRAFT).

---

**Current hardware checkpoint (2026-11-23):** ESP32 `v4_2_uart_shadow_v13_brake_continuity` was flashed, boot/read-only UART checked, and then tested in exactly one supervised raised-wheel RIGHT 8° experiment. Outcome: `ENCODER_LIMIT_REACHED` / latched `BOUNDED_DISTANCE_LIMIT` / terminal FAULT, PWM 0/0, STOP confirmed. Brake request R/L 15/13 counts, output-write commands completed after 38 µs, final R/L 27/25 against target 21/21 and hard limit 22/22. **v13 is not a validated braking fix.** Read [current v13 handoff](docs/handoffs/SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md); previous v11 handoff is historical. Operator has not yet reconfirmed physical motor-power OFF after the latest run; v13 raw evidence archive/off-Pi backup is not yet confirmed. No further motion, flash, FAULT reset or reboot is authorized by this document. A diagnostic v14 remains a proposal.

## 2026-11-23: ESP32 v11-v13 short-turn brake diagnosis (FACT / DECISION / HYPOTHESIS / OPEN)

**FACT / versions:** Active Git branch `codex/siee-person-follow-mvp-20261120`; firmware source commits v11 `b07fbd9c0274235473dc2db6e0abdda20986d34a`, v12 `1de64385df1f002a668c7454acfd205fd73f9651`, v13 `77067e47db98bf64d8101c4e8bbccd9822a1cc5b`. On 23-Nov v13 compiled in Arduino IDE 2.3.11 (516175 program bytes, 72480 globals), uploaded with flash hash verification and confirmed over the physical Pi UART (`GET /status` and `GET /fault-diagnostics` HTTP 200, matching v13 build ID, READY/PWM0/0/no fault before the experiment). Operator separately authorized exactly one raised-wheel RIGHT 8° test.

**FACT / measurements:** All trials used target 21/21 and hard cutoff 22/22 encoder counts, `NEAR_TARGET` guard and latched terminal `ENCODER_LIMIT_REACHED`:
| Firmware | Date | Git source commit | Brake request R/L | Request→write | Fault R/L | Final R/L | Additional R/L | Outcome |
|---|---|---|---:|---:|---:|---:|---:|---|
| v11 | 2026-11-20 | `b07fbd9c` | 15/14 | not measured | 26/26 | 28/27 | 13/13 | FAULT |
| v12 | 2026-11-23 | `1de64385` | 14/15 | 37 µs | 26/27 | 27/28 | 13/13 | FAULT |
| v13 | 2026-11-23 | `77067e47` | 15/13 | 38 µs | 26/25 | 27/25 | 12/12 | FAULT |

v13 `shadow_turn_brake_request_elapsed_ms=70`, request-to-mutex 3 µs, mutex-to-output-command completion 35 µs, R/L counts at brake-request and completed write 15/13, snapshot valid/matched fault; terminal STOP confirmed, PWM0/0 and FAULT retained. The one-count reduction versus v12 is **not evidence of meaningful braking improvement**. These are encoder/software observations, not electrical or chassis-yaw ground truth.

**FACT / code:** v12 introduced brake timing telemetry. v13 removes `stopMotors()` immediately before `brakeMotors()` within `beginBoundedBraking()` to avoid an intermittent Brake→Coast→Brake handoff. Other GPIO, hard limits, PWM, 400-ms active-brake hold, UART and FAULT rules are unchanged. v13 on Pi passed 16/16 v13, 8/8 v12, 12/12 v11 targeted Python tests and minimal-diff checks. Actual driver input voltage and current were not independently measured; the operator has no oscilloscope.

**FACT / powertrain and provenance:** The operator identified GB37-520 6-V nominal 200-RPM gearmotors and 6-V TA6586 drive supply; 200 RPM is a nominal specification, **not measured speed** during these trials. v13 raw private evidence `/home/elwis/.local/state/sie/evidence/v13_right8_20261123_205406/` (command `sie-v13-r8-c537ca8fa82e`) has **no confirmed archived SHA or off-Pi backup yet**. v12 Pi evidence archive SHA256 `1d2b15c185bb0996412c53842f4b9e6b51e3d27cb29185fcdf27b8ca3b9a0760`; an off-Pi v12 copy is not confirmed. Operator has not yet reconfirmed physical motor-power OFF after the v13 trial.

**DECISION:** v13 source/flash checkpoint is saved, but the short-turn braking problem remains OPEN. Do not widen 22/22 cutoff, relax overshoot, automatically ACK fault, flash, reboot or energize motors without separate operator consent. Keep the supervised FAST floor person-follow/steer result from 20-Nov separate from these failed raised-wheel turns. Canonical SIEE Data Contracts, current calibration and Face/Body motor-admission boundaries are unchanged.

**HYPOTHESIS / OPEN:** Residual 12–13 encoder counts may reflect motor speed/inertia, actual TA6586 braking current/torque, other software output changes or combinations. Programmatic `analogWrite` completion (37–38 µs) is NOT proof of sustained electric braking; PWM0/0 does not distinguish Brake from Coast. `updateBoundedBraking()` intentionally switches to Coast after 400-ms brake hold, but that operation has not been linked causally to this fault. A possible **v14 is only a design proposal**: bounded encoder timestamps and trace of software output mode transitions to estimate trigger speed and deceleration without an oscilloscope. No v14 code, upload or tests have been completed.

**NEXT VERIFIED SAFE STEP:** Confirm physical motor power is disconnected after v13; preserve v13 fault evidence with SHA256 and an independent off-Pi backup before any ESP32 reset; then perform motor-off source-only output-path audit and review proposed v14. Last Graphify AST still 2026-11-16 `db8097ea4239dcf45ef7598c831a45648d2498fd` (4599 nodes, 13658 edges, 300 communities); no extraction for v13.

References: [operational v13 handoff](docs/handoffs/SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md); [detailed brake diagnostics](docs/diagnostics/siee_esp32_brake_v11_v13_20261123.md); [historical v11 handoff](docs/handoffs/SIEE_PERSON_FOLLOW_V11_TURN_FAULT_HANDOFF_20261120.md).

**Snapshot date:** 2026-11-20 (FAST camera capture and supervised physical person following; previous sections preserved)
**Purpose:** durable entry point for a human or AI agent continuing SIE work.  
**Status:** The 20-Nov operator-supervised FAST floor test confirmed forward approach and operator-observed steering toward a person moving left and right. UART evidence records 19 START/RESUME, 41 STEER and 8 PAUSE, terminal READY with PWM 0/0 and no FAULT. The experimental camera-to-motion chain is verified for this bounded scenario, not accepted canonical Measurement, LiDAR fusion, independent trajectory accuracy or sustained unattended navigation. See 2026-11-20 update.


## 2026-11-20: FAST V4L2 capture and physical person following (FACT / DECISION / OPEN)

**Revision:** 2026-11-20. Preserve all earlier entries, including the 2026-11-19 Face/Body diagnostic results; these describe a different optional identity worker and are not superseded by a motor-off FAST configuration or a v10 motor test.

**Git provenance (FACT):** The Raspberry Pi 5 working branch is `codex/siee-person-follow-mvp-20261120`, checkout `/home/elwis/dev_ws/sie_live_identity_20261119`. Source checkpoint `cc64873d766ee8ab77df54ca071aa4cce931e3c8` contains exactly five changed/new files: `vision_core/tools/run_shadow_fast_preview.sh`, `vision_core/tools/stream_v4l2_mjpeg.c`, `vision_core/tools/run_shadow_uart.py`, `vision_core/tools/uart_http_transport.py`, and new `vision_core/tools/shadow_uart_compact_status.py`. Evidence checkpoint `315d61c5b76c4a25a4f68fc9c3d206b732dbc775` contains `docs/diagnostics/siee_person_follow_physical_mvp_20261120.md`, `docs/evidence/20261120/SHA256SUMS.txt` (816 source evidence files), and `.gitignore` excluding private raw logs. On 2026-11-20, operator `git ls-remote` verified the GitHub branch SHA equals `315d61c5b76c4a25a4f68fc9c3d206b732dbc775`. These are not a merge to `main`.

**FACT — reproducible camera profiles:** `SHADOW_FAST_RUNTIME_PROFILE=baseline` retains the original YOLO 256x384 with one V4L2 MMAP buffer per camera. `SHADOW_FAST_RUNTIME_PROFILE=fast_192x320_4buf` explicitly selects the previously offline-compared 192x320 ONNX model and requests four V4L2 buffers on each camera. The native producer/consumer measured ~15.54/30.15 FPS for AR0234/OV9281 in the initial one-buffer run and ~29.98/59.98 FPS for the FAST four-buffer configuration. The per-cycle input shape `(1,3,192,320)` and four buffers were independently verified for all 163 valid FAST motor-off cycles in `shadow_fast_20261120_105226`. Derived 500-ms stale-window share was 23.1% of 31.35 s for 256x384/1 buffer (`shadow_fast_20261119_224755`), 7.2% of 40.37 s for the integrated FAST profile; maximum stale interval 151.1 to 53.1 ms respectively. The scenes, runtimes and resolutions differed, so this does not isolate the effect of each change or guarantee an equivalent motor-PAUSE reduction. A 256x384/4-buffer control produced a worse 34.4% stale fraction in its own 40.37-s run. Baseline also passed a separate 75-cycle motor-off regression in `shadow_fast_20261120_105656` (73 FOLLOW_ELIGIBLE, two PAUSE). FAST produced 161 reported camera cycles in the integrated test (157 FOLLOW_ELIGIBLE, four PAUSE, 0 dropped and 0 journal errors); 163 JSON files were usable for the independent freshness calculation. Do not conflate these denominators.

**FACT — UART initialization and controller:** ESP32 reported flashed runtime build ID `v4_2_uart_shadow_v10_turn_guard_settle` with `shadow_api_version=v2_paused_edge_turn`. Software adds compact status decoding and one retry for `UART_REPLY_TIMEOUT` *only on the first read-only GET /status*. Offline tests: timeout then success PASS; two timeouts refusal PASS; send timeout no retry PASS. A real no-motion `/status` read in `uart_initial_status_real_bx4bqfyx` returned HTTP 200, READY, PWM 0/0 and no fault on the first request (retry did not trigger on hardware in this test). An earlier motor attempt `shadow_fast_20261120_110052` stopped before any segment with UART_REPLY_TIMEOUT; this did not validate steering. No POST motion command is automatically retried after ambiguous delivery.

**FACT — physical motor experiment:** Operator-authorized supervised `SHADOW_FAST_RUNTIME_PROFILE=fast_192x320_4buf`, segmentation mask, `--execute --supervised-experimental --edge-turn`, requested 0.300-m maximum segments, 60-s nominal bounded session and firmware v10. Primary journal `/home/elwis/.local/state/sie/evidence/shadow_fast_20261120_111329/bridge.jsonl` (1118 JSONL records, SHA-256 `2525732072d85c2ecb10182af751a7190ae116449374c85eff50ac2f84af0302`) records `SESSION_ENDED`, 19 START/RESUME, 41 STEER transmissions, eight PAUSE, zero edge turns and zero feedback-timeout PAUSE. Per-segment encoder-derived travel: initial five ~0.267-0.287 m, maximum ~0.300 m, median across all 19 ~0.084 m; sum of segment values ~2.77 m is NOT a separately measured net chassis displacement. Final ESP32 READY, PWM left/right 0/0, `fault=null`. No persistent UART failure in this successful trial.

**Operator physical observation (FACT):** First the cart moved in long segments. When the operator stepped left and right, the cart steered toward the person on the floor. At approximately 1 m it switched to shorter segments. With a wall behind the operator and no room to step back, it stopped at an operator-estimated ~0.60 m straight-line gap from the cart base front edge to the operator's leg. This is an approximate physical observation, not a calibrated stereo optical reference-frame standoff or independent trajectory measurement. The experiment supports bilateral visible steering in one bounded scene, not indefinite continuous following or verified obstacle detection.

**DECISION:** Preserve and use the explicit FAST profile as the best measured candidate for the next supervised MVP trial; retain baseline and the existing 500-ms freshness gate, versioned calibration, current distance/short-segment guards, ESP32 v10 and physical drive-power cutoff. Do not treat diagnostic `FOLLOW_ELIGIBLE`/torso depth as admitted canonical SIEE `Measurement`; do not silently enable the separate late Face/Body identity worker for motor admission.

**Evidence storage (verified off-Pi backup, 2026-11-20):** Public GitHub includes the diagnostic report and hashes, not raw JSONL. The private archive on Pi is `/home/elwis/.local/state/sie/checkpoints/siee_person_follow_evidence_20261120.tar.gz`, SHA-256 `25025941d2bbf93708838461ba39cafbdc28aa185f90bef581d2ff0eee6c5376` (about 6.6 MiB), covering five run directories and 816 files. The operator copied it to the independent laptop path `/home/kestabor/siee_private_backups/siee_person_follow_evidence_20261120.tar.gz`; laptop `sha256sum` matched the Pi SHA-256 exactly, file permissions were `-rw-------`, and the transfer ended `SIEE_PRIVATE_EVIDENCE_BACKUP: PASS`. This confirms a second copy of that archive, not archival durability or automatic backup of future experiments.

**FACT — ESP32 v10 source recovered and preserved (2026-11-20):** The laptop's Arduino sketch directory `~/Arduino/traction_control_v4_2_uart_shadow_v10_turn_guard_settle/` was inspected; only the main `.ino` and two required local `.h` files were transferred to Pi (the `.ino.before_turn_start_pwm` backup was excluded). `sha256sum -c` passed on both devices; source markers for `v4_2_uart_shadow_v10_turn_guard_settle`, `STATUS_COMPACT`, bounded turn guard, remote STOP settle and both headers passed. Git commit `d6ad93c848e73d9459ee3ea76c7b4e3a909c88e8` on `codex/siee-person-follow-mvp-20261120` contains exactly these three files; the operator confirmed local HEAD equals GitHub branch HEAD after push. Source SHA-256 values:

- `firmware/esp32/traction_control_v4_2_uart_shadow_v10_turn_guard_settle/traction_control_v4_2_uart_shadow_v10_turn_guard_settle.ino`: `7fd5433b6157b5f8664834d6c26a3df11b4c39c8d8bd2abc83bc02a07c7adde4`
- `firmware/esp32/traction_control_v4_2_uart_shadow_v10_turn_guard_settle/command_replay_guard.h`: `be68d55c7052fc836780000ba7604164a9d5b43b3714f270ec94a7b27244e3d4`
- `firmware/esp32/traction_control_v4_2_uart_shadow_v10_turn_guard_settle/remote_stop_observer.h`: `b67dc0dda8702b30852723469fcc7db8440e354c76ace3e2bd7ccbfc59e641e3`

**Provenance limit:** The source constant `FIRMWARE_BUILD_ID` matches the runtime status returned by ESP32 during the physical trial. This does **not** independently prove that those exact source bytes produced the installed firmware binary: neither a compiled binary SHA-256 nor a readback of ESP32 flash was verified. No new firmware compile, upload, motor test or Graphify AST extraction was performed as part of this preservation step.

**OPEN / next testable step:** Keep the verified off-Pi evidence backup and the newly committed v10 source checkpoint intact; the compiled/flashed ESP32 image hash and same-build reproducibility are not established. With the existing safe supervised setup, use a clear longer floor route where the person can keep ~1.5-2.0 m ahead, change bearing both ways and retreat safely, recording fresh RGB/depth track, UART decisions, encoder distances and final STOP, while independently observing trajectory. Test target loss and identity re-entry separately. LiDAR D500 fusion, canonical Measurement admission, sustained collision-safe autonomous following and calibrated physical standoff remain OPEN. No new Graphify extraction was performed; the last verified AST remains the 2026-11-16 `db8097ea4239dcf45ef7598c831a45648d2498fd` snapshot (4599 nodes, 13658 edges, 300 communities).

## 2026-11-19: live Face/Body diagnostics and supervised floor following (FACT / DECISION / HYPOTHESIS / OPEN)

**Primary source and branch:** `docs/diagnostics/siee_floor_follow_face_body_status_20261119.md`; new-chat recovery `docs/handoffs/SIEE_PERSON_FOLLOWING_CHAT_HANDOFF_20261119.md`. Development branch `codex/siee-live-identity-preview-v1-20261119`, pre-documentation implementation SHA `8e079a97c37db9f78e614f9e165b3639180954b0`. Raspberry Pi checkout `/home/elwis/dev_ws/sie_live_identity_20261119`, Python `/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python`, UART `/dev/ttyAMA0`. Pi firmware build in uploaded motor evidence `v4_2_uart_shadow_v8_short_forward_completion` (UART runtime ID; source/flashed SHA not independently recovered).

**FACT — offline face/body lineage:** 143-cycle motor-off episode `shadow_fast_20261119_170524`: RGB Track `:1` lost after cycle 33, `:2` starts 73, face observed in saved post-reentry images from cycle 80. Eleven source image inputs; seven SFace face observations. OSNet cross-track masked background median 0.77758 (24 comparisons). Offline `identity_coordinator_replay_20261119.json` has 143 journal cycles, only 11 retained source images and 132 cycles without retained identity imagery (NOT 132 real RGB target losses); identity coordinator 11 `AWAITING_FACE` and 132 `TARGET_LOST`. Preflight 23/23 targeted tests PASS, but positive-only Face/Body comparisons do not constitute verified identity or a calibrated threshold.

**FACT — live integration with motors OFF:** Opt-in `SHADOW_IDENTITY_PREVIEW=1` runs the source-bound crop/cycle/mask path into asynchronous latest-wins YuNet/SFace + OSNet and existing `FaceAnchoredTarget`, with diagnostic ROS topic `/sie/diagnostics/ar0234_ov9281/identity_preview`. Raw embeddings are not emitted; the result is not motor admission. In `shadow_fast_20261119_180036/identity-preview.jsonl` (uploaded SHA256 `6b9f570222c98647eeffeae3daef65dfc7cebeb3b49aa7b10fe788868e7b210e`): 18 outputs, 15 face observations, three no-face; Track samples `:1`=12, `:2`=1, `:3`=5. OSNet `:1→:2` median 0.73561 across eight comparisons and `:2→:3` 0.8717 for one comparison; neither proves a persistent person identity. **18/18 LATE_DIAGNOSTIC**, submit-to-result min/median/max 546.61/625.605/762.86 ms and 18/18 `TARGET_UNAVAILABLE/STALE_OR_UNBOUND_RGB_MASK`; `identity_supported=false`, `motor_authorized=false`. Camera/Bridge summary under worker load: 112 camera cycles, 67 FOLLOW_ELIGIBLE, 45 PAUSE, 19 driver-to-decision >500 ms, host median/max 267.66/696.76 ms. **HYPOTHESIS:** contention for CPU/processing, not proven by profiling.

**FACT — supervised floor motion with Identity Worker OFF:** `SHADOW_IDENTITY_PREVIEW=0`, `SHADOW_FAST_SEMANTIC_SOURCE=segmentation_mask`, `--execute --supervised-experimental --single-person-reacquire --max-segments 6 --segment-m 0.40`. Pi evidence `/home/elwis/.local/state/sie/evidence/shadow_fast_20261119_181125/bridge.jsonl`, operator-uploaded `bridge(20261119-151320).jsonl` SHA256 `e48fa59526e2f882649156036f89537a425c943dbf1e73fbc3794615c24476e6` (281 records). `SESSION_ENDED/SEGMENT_LIMIT_REACHED`; six START/RESUME segments, three PAUSE, 12 STEER attempts (10 HTTP 200 accepted, two HTTP 409 `NO_MATCHING_ACTIVE_STEERED_FORWARD`). ESP32 reported applied steering PWM `last_steer_biased_drive_cycles=34`, `last_steer_update_seq=12`. Terminal STOP accepted: `READY`, `last_command_state=STOPPED`, `fault=null`, `bounded_fault_latched=false`, PWM left/right 0/0. `single_person_reacquires=0`, `edge_turns_started=0`.

**Operator's ground truth (FACT):** Standing facing the cart, the operator saw it approach. When the operator stepped **left relative to themselves** (rightward in the AR0234 camera image), the cart visibly corrected toward them. This confirms physical forward cart approach and steering toward the person in **that one direction** on the floor. No independent validation of bidirectional correction, sustained uninterrupted follow, physical front-edge stand-off accuracy, loss→STOP→fresh reappearance→resume, or actual identity of reappearing Track IDs is claimed. Encoder distances are per segment, not summed ground-truth displacement.

**DECISION / OPEN:** Preserve the verified bounded floor path with `SHADOW_IDENTITY_PREVIEW=0` until Identity Worker freshness is fixed and identity calibration is independently validated. Do not loosen time or match thresholds to hide lateness. Maintain all existing calibration files, SIE Data Contracts, diagnostic-only torso depth semantics, motor cutoff and ESP32 STOP. **Next checkable floor milestone:** under operator supervision and physical drive power cutoff, test disappearance → confirmed STOP → fresh RGB/depth reacquisition (and direction reversal) using the existing bounded motor path; record per-command and final UART evidence. Separately profile live worker stages motor-off. Graphify AST has **not** been refreshed: last extraction 2026-11-16 at `db8097ea4239dcf45ef7598c831a45648d2498fd`, 4599 nodes / 13658 edges / 300 communities.

## 2026-11-17: current executable branch, physical FOLLOW/STEER and ESP32 short-brake v7 (FACT / DECISION / OPEN)

**Primary detailed checkpoint:** [`docs/diagnostics/siee_status_20261117.md`](docs/diagnostics/siee_status_20261117.md), authored from the operator's 17 Nov floor journal (`bridge(3).jsonl`, 157 JSONL records), local test outputs, GitHub code and ESP32 UART status. Keep its FACT / HYPOTHESIS distinctions when reconstructing state.

**Branch / recovery:** The original operator-designated baseline branch remains `codex/shadow-turtlesim-follow-20261111`. New 17-Nov executable work resides on `codex/shadow-rgb-edge-recovery-20261117` and its descendant `codex/shadow-short-segment-brake-v7-20261117`. These branches are not silently merged into the baseline. Most recent *flashed* firmware build independently identified via UART: `v4_2_uart_shadow_v7_short_forward_brake`. Pi checkout path is `/home/elwis/dev_ws/sie_shadow_follow_tools`, Python venv `/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python`, serial `/dev/ttyAMA0`. ESP32 firmware build/upload uses laptop Arduino IDE 2.3.11, `ESP32 Dev Module`, `/dev/ttyUSB0`, sketched folder copied from Pi via scp. An earlier proposal to invoke an uninstalled `arduino-cli` on Pi was incorrect; do not reintroduce it as the standard procedure.

**FACT — real floor FOLLOW and STEER:** In the reviewed 157-event `bridge(3).jsonl`, 6 forward segments, 4 STEER transmissions and one original person track `ar0234-audit-track:8144989e0438cf10:1` across reported FOLLOW_ELIGIBLE ranges. Current associated person distance reduced in the journal from 1.7403 to 0.7320 m (diagnostic sensor-front inference, not independently measured physical ground truth). The operator directly observed the cart rotate when the person was at the RGB frame edge and forward motion had stopped. **Important limitation:** no `POST /shadow-turn`, `edge_turns_started=0`, `shadow_turn_result=NONE`. Thus **physical reorientation occurred**, but it is *not* a confirmed independent paused RGB Edge Recovery pivot or successful fresh-depth reacquisition. Wheel steering/coast is one possible explanation, not a certified diagnosis.

**FACT — 17-Nov hard-limit fault on short v5 command:** Last `SHADOW_RESUME` requested 0.042 m; active braking began at 0.0466 m, the original hard chassis-center envelope was 0.062 m, and final encoder travel was 0.0635 m. Firmware latched `STEERED_CENTER_LIMIT`, ended FAULT and PWM 0/0. Do not call this a successful 0.5 m stand-off.

**FACT — code and tests:** Same-ID RGB Edge Recovery preserves the forward-depth freshness requirement, bounds a single paused pivot to at most 8 degrees, rejects a new/different track, and expands stale depth allowance *only for a bounded turn* from 2.5 s to 3.0 s based on floor timing. After corrections to test fixtures, 13/13 targeted Edge Recovery tests passed. The exact physical `SHADOW_TURN` path on the floor remains unconfirmed.

**FACT — v7 short-start brake:** New ESP32 sketch `firmware/esp32/traction_control_v4_2_uart_shadow_v7_short_forward_brake/` adds an independent predictive brake for steered-forward segments <=0.100 m during STARTING, leaving original hard wheel/center/heading limits unchanged. Python accepted v7 build and 4/4 targeted tests passed. Laptop Arduino IDE compile: 508199 program bytes (38%), 72320 global bytes (22%); written 508352 B, `Hash of data verified`. Raspberry Pi UART reported HTTP 200, v7 firmware ID, READY, no fault, PWM 0/0. A single requested 42-mm segment ended with 19.3 mm encoder travel on raised wheels (brake trigger 26 / threshold 26 counts) and 22.8 mm on floor (trigger 27 / threshold 26); both `short_forward_brake_requested=True`, final READY, no FAULT, but **`PARTIAL_PROGRESS`**, not SUCCESS. The actual v7 brake reserve is **30 mm** (`STEERED_SHORT_BRAKE_RESERVE_M = 0.030f`). Proposed **17 mm** reserve (`0.017f`) is **HYPOTHESIS / NOT IMPLEMENTED**, not flashed or tested; ~35 mm prospective final travel is an estimate, not evidence.

**OPEN / next verifiable step:** Keep v7 as saved baseline. If adopted, change early-brake reserve in a *separate firmware revision*, then Arduino IDE compile/flash and one bounded 42-mm encoder test (raised wheels, followed by isolated floor test) recording trigger, final counts, partial/success/fault, PWM zero. Preserve physical cutoff. Only after short motion completes consistently without a fault, return to an explicit `SHADOW_TURN` → fresh associated AR0234/OV9281 RGB/depth → `RESUME` floor experiment. Stop distance remains 0.5 m **from the cart front edge**; 0.5–4.5 m is diagnostic stereo range. Calibrations, canonical SIE Measurement policy, and planned LiDAR fusion are unchanged.

**Graphify freshness:** The Drive folder `Graphify_AST_db8097e_20261116` with `graph.json`, `graph.html`, and `GRAPH_REPORT.md` remains a frozen graph of `db8097ea4239dcf45ef7598c831a45648d2498fd` (4599 nodes / 13658 edges / 300 communities). This 17-Nov documentation update does **not** mean Graphify was recomputed from v7. The Graphify handoff carries a matching dated addendum.

## 2026-11-16: source-code preservation and current Graphify AST snapshot (FACT / LIMITS)

**FACT (2026-11-16, source-code checkpoint):** The operator committed the Fast Shadow Python/UART runtime and ESP32 source `v4_2_uart_shadow_v2_initial_bias` as `a815c262dc71c95a99e4280792e39a89bbf31933`, merged the 16 November documentation, and pushed the result to GitHub branch `codex/shadow-turtlesim-follow-20261111`. The Pi local and remote Git heads matched exactly at `db8097ea4239dcf45ef7598c831a45648d2498fd`. An independent clean detached Git worktree on the laptop was checked at this SHA. This is source preservation, not a rerun of physical following or proof that a flash binary/calibration/evidence is backed up. The non-versioned `.before_*` files, patch and helper remain outside Git; a 57 KB worktree archive was copied from Pi to laptop and both copies had SHA-256 `a5a81a6b462c34ff0e939e9bbf05765037aae32e80ef8aa8299f08df84da61a8`.

**FACT (Graphify AST rebuilt locally):** Graphify CLI `0.9.107`, `extract --code-only` on the exact clean checkout `db8097ea4239dcf45ef7598c831a45648d2498fd` (535 code files, no Graphify Cloud/LLM extraction), followed by `cluster-only --no-label`: **4,599 nodes, 13,658 edges, 300 communities**. Historical GitHub-code graph at `60eeade`: 4,500 nodes, 13,379 edges, 290 communities; deltas are +99/+279/+10, not a direct quality or complexity measure. Graphify reported 9 files with partial syntax extraction (including ESP32 `remote_stop_observer.h` variants), 65 code files without symbols, and 750 inferred edges; this is not evidence of a firmware compilation failure.

**Graph provenance (operator SHA-256, laptop local files only):** `~/graphify_sie_mvp_20261116/graphify-out/graph.json` = `9e068e27a46cfca02382370cb21465c749a3e6ad35fc1d16e948ab96e04c4e93`; `GRAPH_REPORT.md` = `0ef3a084d0089913ddf06dd6a7ddd174b303d3c77b24c16aaf0c03ad85cfdf29`; `graph.html` = `570c285102b1a798d1e933c417214abef21ab883866687c23a5a1267e78ba636`. The graph files were **not yet copied to GitHub or Drive**; these hashes are provenance, not remote-backup verification.

## 2026-11-16: ESP32 UART v2 Initial Bias and bilateral physical steering (FACT / OPEN)

**FACT — implemented local runtime:** The operator built and flashed ESP32 `v4_2_uart_shadow_v2_initial_bias` using Arduino IDE 2.3.10 on the Ubuntu laptop. Successful compilation: 504463 program bytes (38%) and 71880 global bytes (21%). Flash write hash verified; Raspberry Pi 5 `/dev/ttyAMA0` preflight reported HTTP 200, `firmware_build_id=v4_2_uart_shadow_v2_initial_bias`, `shadow_api_version=v1_paused_segment`, READY, PWM=0/0, no faults. The v1 firmware directory was preserved. The current working Pi source changes for firmware, `vision_core/tools/uart_http_transport.py` and `vision_core/tools/run_shadow_uart.py` have **not** been independently verified as committed to GitHub; the documentation update is not an executable-code deployment.

**FACT — control mechanism:** UART SHADOW_START/SHADOW_RESUME accepts a fifth command argument `bias_pwm` (eight core UART fields before CRC), validated as integral in [-20,+20] and saved into the ESP32 `CommandRecord` before `startMotion()`; `shadowLastBiasPwm` preserves initial values for duplicate checks. `pollUart()` runs before the loop's `startRequested` handler; STARTING uses `updateStarting()` and only DRIVING's `updateDriving()` applies nonzero steering bias to PWM. Python START/RESUME now sends bias in the same UART command, eliminating dependence on a late separate STEER packet. UART five-operation field test and Python static checks passed. Standalone mid-segment STEER remains implemented but was not proven applied in these runs (`last_steer_update_seq=0`).

**FACT — raised wheels 2026-11-16:** `shadow_fast_20261116_194409`: 9 accepted requested 0.100-m START/RESUME segments, each `bias_pwm=-20` and HTTP 202, terminal SESSION_ENDED, READY, STOPPED, PWM=0/0 and no faults; last command shows 9 biased drive cycles, first/last applied bias -20, right/left PWM sums 2016/1215. Last-command encoder distance is not total session travel.

**FACT — floor steering:** `shadow_fast_20261116_195100`: operator visually observed a slight left turn in one segment; right/left encoder distances 0.1198/0.1094 m, `heading_rad=+0.0359` (~+2.06°), applied bias -20; READY, PWM 0/0. `shadow_fast_20261116_195358`: person right, bias +20 applied, but right/left distances 0.1000/0.0994 m and `heading_rad=+0.0019` do **not** prove a rightward turn alone. `shadow_fast_20261116_195714`: operator visually observed rightward steering over three accepted 0.100-m START/RESUME segments, all bias +20; last-segment distances 0.0950/0.0964 m, `heading_rad=-0.0049` (~-0.28°), 3 biased cycles, READY/PWM=0/0. Encoder heading is not an independent measurement of chassis yaw.

**FACT — dynamic moving-person, raised wheels:** `shadow_fast_20261116_200624`: first Shadow session transmitted 14 accepted START/RESUME commands at biases `+20,+20,+20,+20,+20,+11,-15,-18,-20,-20,-20,-20,-13,+2`; two sign changes within the same command session (segments 7 and 14). On a new RGB Track ID the runner recorded `NEW_TRACK_REQUIRES_NEW_SESSION` and STOP, then started a second Shadow session. Second session reported 19 segments, terminal SESSION_ENDED and READY/PWM=0/0; final record has 17 biased drive cycles and first/last applied bias +20/-20. Do not mistake the first session's 14-segment sequence for the second session's 19-segment sequence. The complete per-segment bias history of the second session and exact track continuity require further primary-journal inspection.

**OPEN / boundaries:** The camera node continues reporting frequent `DEPTH_MAD_EXCEEDED`; its diagnostic-only `motors=DISABLED` is not a statement about the independent supervised Shadow UART actuator path. The experimental torso/front range is **not** an admitted canonical SIE Measurement or independently verified moving-person physical distance. No approved LiDAR fusion, independent total yaw/trajectory measurement, robust identity through full crossing, verified dynamic target-loss STOP on the floor, obstacle clearance, or reliable autonomous moving-person following has been demonstrated. Do not change active calibration or Engineering Specifications based on these trials.

**Primary evidence on Pi:** `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_194409/bridge.jsonl`, `shadow_fast_20261116_195100/bridge.jsonl`, `shadow_fast_20261116_195358/bridge.jsonl`, `shadow_fast_20261116_195714/bridge.jsonl`, `shadow_fast_20261116_200624/bridge.jsonl` (new session 2 also reported by launcher). See `docs/diagnostics/siee_uart_v2_initial_bias_steering_20261116.md` for detailed evidence and runtime boundaries.

**NEXT verifiable MVP step:** supervised and bounded floor-follow experiment with moving person and synchronized RGB `track_id`, bearing, OV9281 range provenance, accepted per-segment bias, wheel odometry, independently measured chassis heading and verified terminal motor-off. Test target loss/reacquisition and maintain physical motor cutoff; no new firmware tuning until behavior is measured.

## 2026-11-15: physical Fast Shadow Follow Person and asynchronous steering (FACT / OPEN)

**FACT (real Raspberry Pi 5 and ESP32, operator-observed):** The established `run_shadow_fast_preview.sh` with `SHADOW_FAST_SEMANTIC_SOURCE=segmentation_mask`, `--execute --supervised-experimental`, AR0234 RGB, OV9281 depth and GPIO UART `/dev/ttyAMA0` drove the physical cart. Earlier physical runs completed six 0.10 m segments (`shadow_fast_20261115_200627`), and subsequent runs completed multiple segments, handled `PAUSE` and confirmed final `READY`, PWM=0/0, `STOPPED`, no latched faults. Encoded segment distances are command bounds; the last per-segment odometry report is **not** a verified cumulative travel distance.

**FACT (failure analysis):** With the original synchronous observation generator, UART `issue()` blocked incoming RGB/stereo frames while waiting for `STARTING → DRIVING → BRAKING → PAUSED`. In the 2026-11-15 `shadow_fast_20261115_204105` evidence, two command sessions ran 4 and 6 segments, respectively, after confirmed STOP and new Track ID; no `/steer-update` was transmitted and `last_steer_biased_drive_cycles=0`. Another run reached `STEER` but returned `NO_MATCHING_ACTIVE_STEERED_FORWARD` when the short forward segment had ceased to be eligible, with final STOP confirmed. UART timing showed `STARTING` around 128–297 ms, `DRIVING` around 455–592 ms, and `BRAKING` by ~736–770 ms after START/RESUME. No assumption of a sign error was justified from these motor-off-steering runs.

**FACT (architecture implementation):** A local, as-yet-uncommitted asynchronous source patch separated the ROS observation producer from the serial motion consumer using a latest-wins observation buffer. During STARTING steering now reads a fresh observation of the **same** RGB Track ID, not the target frozen at segment start; firmware protocol, sequential UART owner, STOP, staleness checks and session identity remain in place. Operator-reported local verification:
`PASS async producer continues during slow consumer, latest-wins`;
`simulation SESSION_ENDED ['START', 'STEER'] [20, 20, 20] None`;
`PASS fresh steering dispatched during STARTING and STOP confirmed`.
These are local tests, **not** evidence that the local patch has been pushed to GitHub.

**FACT (real hardware after patch):** With drive wheels safely lifted, the cart's wheels rotated through 9 short segments. ESP32 reported `last_steer_update_seq=5`, `last_steer_biased_drive_cycles=3`, `last_steer_first_applied_bias_pwm=-20`, `last_steer_last_applied_bias_pwm=-20`, `last_steer_right_pwm_sum=666`, `last_steer_left_pwm_sum=405`. Final status was `READY`, `STOPPED`, PWM=0/0, no faults. The operator stood **to the left of the cameras**. This confirms actual nonzero steering PWM applied by ESP32, not just a command or simulation. A greater right-wheel PWM sum is consistent with a commanded left turn in a conventional forward differential drive, but the true sign of body yaw **has not** been verified on the floor.

**OPEN / next physical check:** Confirm a leftward **actual chassis turn** toward the person on an unobstructed surface with physical emergency power-off available, and confirm STOP on target loss. Existing tests demonstrated real cart movement and real applied steering separately; they did **not** yet certify reliable moving-person following, unobstructed path detection, or physical steering direction. Preserve exact Track ID, freshness, STOP and provenance semantics. The actuator-facing torso-core range remains diagnostic in `--supervised-experimental`, not an admitted SIE Measurement. Do not rewrite the calibration evidence as a measurement of the moving person's torso-front datum.

**Primary run evidence on Pi:** `~/.local/state/sie/evidence/shadow_fast_20261115_200627/bridge.jsonl`, `shadow_fast_20261115_201443/bridge.jsonl`, and `shadow_fast_20261115_204105/{bridge.jsonl,bridge.session-002.jsonl}`; last lifted-wheel run is recorded in the operator-provided 2026-11-15 terminal log (its full evidence directory was truncated in that excerpt). **Local uncommitted code paths:** `vision_core/tools/shadow_live_bridge.py`, `vision_core/tools/run_shadow_uart.py`, `vision_core/tools/shadow_supervisor.py`, and `vision_core/tools/run_shadow_fast_preview.sh`. The document change does not claim these executable changes were committed.

## Update rule

This file is the change-controlled snapshot of the active project state.

- The current `PROJECT_CONTEXT.md` is stored in two places: this local laptop file and its matching [private Google Drive copy](https://drive.google.com/file/d/1mRj9UluoMptShOMPMz4h_MbCFczlVn0o/view) in the `SIE Active Projector` folder. Keep their contents synchronized when an update is authorized.
- The local file is the working copy. The Drive copy is the second current copy, not a public release. Historical copies elsewhere do not replace either current copy.
- The operator commands **`обнови документ`** and **`обновить документ`** are equivalent. **Each command requests one coordinated update of TWO documents:** this `PROJECT_CONTEXT.md` and `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`. Neither document is optional in this update workflow.
- Keep both documents synchronized between their working GitHub copies and their existing private Google Drive copies. `PROJECT_CONTEXT.md` is the cross-project status and verified continuation point; the [Graphify handoff](https://drive.google.com/file/d/1LcviPpSIXe5U26ugRF6PkCTq2Rr2_x6M/view) is the code-dependency map, experiment history, evidence index, and LLM continuation journal. Preserve their distinct roles and do not discard later evidence from either copy.
- For each command, read current copies first; incorporate only supported new facts, decisions, source hashes, experiment results, blockers, and next steps. Record the update date in both revision histories; confirm each GitHub/Drive pair has identical content. If a write or verification fails, report which copies are out of sync rather than claiming the update completed.
- Refresh local Graphify graphs only when the corresponding source tree and tool are accessible and the refresh is relevant; updating the Markdown handoff is not automatic graph regeneration. Record the last verified graph snapshot and source commit honestly. No additional activation, runtime change or test is implied by this documentation command.
- A ChatGPT Library copy, if present, is optional reading material. It may be outdated and is not an authoritative or required synchronized copy. Do not recreate or update it as part of the required maintenance workflow.
- Do not update the repository or Drive copy without an explicit operator instruction to update this document or its maintenance rule.
- Do not silently update it after code edits, tests, experiments, or conversation summaries.
- When updating, preserve accepted evidence, distinguish fact from hypothesis, and state the revision date and what changed.
- This file describes current implementation and safe continuation. It never overrides approved Engineering Specifications, canonical Data Contracts, or calibration artifacts.

## Read-only engineering-memory recovery command (2026-11-14)

The exact operator command **`восстанови память SIEE`** requests **read-only**
reconstruction of the active engineering context, including in a new chat.
It is distinct from `обнови документ` / `обновить документ` and does **not**
authorize any documentation write, Graphify extraction, code change, Pi command,
camera or motor activation.

On this command, read the freshest accessible `AI_CONTEXT.md`, this
`PROJECT_CONTEXT.md`, applicable approved ES / Data Contracts, `AGENTS.md`,
`docs/handoffs/WORKING_BASELINE.md`, and the current
`docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`; for Fast Shadow read
`docs/diagnostics/shadow_uart_candidate_20261029.md`, linked primary evidence,
relevant code and tests. Prefer the current working branch
`codex/shadow-turtlesim-follow-20261111` until newer operator-confirmed
information supersedes it. Compare the GitHub and existing private Drive copies
of this context and the Graphify handoff where available. Check exact
branch/commit and distinguish historical graph snapshots from current code.

Reply in Russian with: current goal; implemented and confirmed results;
accepted decisions; hypotheses and missing evidence; source dates/commits and
any conflicts; one next verifiable MVP step. Explicitly mark inaccessible
sources and unverified Pi runtime. Never turn this command into a new experiment
or claim physical tests were executed. When source access is unavailable, ask
for the current documents instead of claiming recovery succeeded.

## 2026-11-15: moving-person RGB track and stereo torso depth

The operator's motor-off Pi archive `shadow_fast_20261115_134056_review.tar.gz`
(SHA-256 `4d85b02bf2efe3f9e26545aee62c2a8b58e7fa10afd6839ea223419e876db5f1`)
contains 215 camera cycles, 15 validated current-source bundles, zero journal
drops/errors, and 143 `FOLLOW_ELIGIBLE` decisions. In cycles 1–41 track `:1`
persisted; cycles 42–111 had no selected target (61 with no segmentation
instance, nine with raw instances at the right boundary); cycles 112–113
were deliberate no-range bootstrap `PAUSE` cycles; cycles 114–215 carried
track `:2` continuously. In 32 eligible cycles the RGB person box touched
the upper image edge while the torso remained visible. Of 24 cycles with
whole-mask `DEPTH_MAD_EXCEEDED`, 22 supplied a current locally fitted
torso-front diagnostic with the RGB track, and two were the bootstrap pauses.
There were no two eligible people. AR0234 and OV9281 driver-declared ages
at Bridge decision stayed below 329 ms; this is not measured physical exposure
latency. The selected front diagnostic spanned 1.633–2.352 m but has no
independent physical distance error bound. No motors were enabled.

**Accepted finding:** RGB current-frame segmentation maintains the moving
person's diagnostic ID through top cropping while current OV9281 stereo points
continue to support a torso-depth estimate. After a long exit, two fresh
matched masks with torso support seed a new diagnostic ID; the following
cycle can use current depth even when whole-mask MAD is high. This closes
the tracked-person/re-entry question for the motor-off preview, not canonical
Measurement admission or physical following. The exact chronology, evidence
and limitations are in
`docs/diagnostics/shadow_fast_moving_person_track_20261115.md`. Code commit
`d00d4e0949c291ae86b45f80c5c7c0a49ce1b472` on
`codex/shadow-turtlesim-follow-20261111` was used for this Pi run. Next MVP
step: connect the current Fast Shadow target/range to the existing bounded
UART motor/steering path and verify a supervised camera-driven movement and
steering segment. The operator accepts centimetre-scale distance error for
this MVP and plans LiDAR as an additional source. Use the existing stereo
calibration and board/translation evidence instead of repeating calibration
or static-person tests. This implementation choice does not certify the
moving torso-front statistic as an admitted Measurement.

## 2026-11-13: current-frame local surface and corrected Fast Shadow launcher

The active OV9281 Pi v7 stereo calibration, AR0234 v5 intrinsic and
AR-to-left extrinsic remain unchanged. The OV9281 activation SHA-256 is
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
The extended RGB and stereo decision profile is
`runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json`,
range 0.5 to 4.5 m, SHA-256
`c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9`
and status `ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY`.
This status does not authorize canonical Measurement or movement.

The motor-off Pi evidence at
`/home/elwis/.local/state/sie/evidence/shadow_fast_20261113_181718` was
reviewed from `shadow_fast_20261113_181718_review.tar.gz`, including the
Bridge journal and 64 exact audit JSON files. All 64 camera cycles produced
`FOLLOW_ELIGIBLE`: 31 used `CURRENT_FAST_MASK_OBSERVATION` with coherent
whole-mask depth and 33 used `CURRENT_STABLE_LOCAL_PREVIEW` after
`DEPTH_MAD_EXCEEDED`. In all 33 fallback cycles the exact audit retained a
current `shadow_target_bearing_deg` and an unambiguous matched RGB track.
The same track ID was present in all 64 cycles. There was no target loss,
re-entry or two-person interval in this run, so ownership and reacquisition
remain unproved.

Maximum driver-timestamp-to-Bridge-decision age was 337.188 ms for AR0234
and 333.264 ms for OV9281. These are driver-declared ages, not independently
measured exposure-to-command latency. The stable local front surface used
2,619 to 4,008 points and the Bridge reported 2.174 to 2.613 m. These values
show that the current-frame local surface can keep the motor-off decision
path available when whole-mask MAD is high; they are not a physical distance
error bound or proof of robot-front clearance.

The reviewed run still loaded the old v1 activation profile with a 0.5 to
2.0 m authorized range while the preview range was configured to 0.5 to
4.5 m. Commit `a1b02b960639c25b218b67bf70b987b717b484a2` corrects the Fast
Shadow launcher to require the exact conditional v2 profile and SHA above.
Commit `fcb1959fc637e6cfdbec3659edb1d1d6a650c0b2` also makes the diagnostic
segmentation-mask launcher explicitly disable the unused AR v3 detector, so
this path no longer depends on a YOLO v3 file. The separate
`v3_mask_binding` path remains strict and SHA-pinned. Syntax, compilation and
the new launcher contract checks passed; full pytest was unavailable in the
working environment. No new Pi run has yet verified these two corrections.

The next single MVP check is a motor-off mask-only run with the exact v2
profile at approximately 2.5 to 2.65 m, including controlled person motion.
It must verify the loaded activation SHA and range, local depth, track
continuity, driver-declared age and the stable-front estimate against the
measured robot-front clearance or another explicit physical datum. Do not
return to whole-mask MAD tuning or download YOLO v3 for this mask-only check.
Motors remain disabled.

## 2026-11-12: repository and Drive are the current copies

The operator removed the requirement to maintain a ChatGPT Library copy.
The repository file and private Drive copy remain the two synchronized current
copies. A Library copy may be read for convenience, but its contents are
historical unless checked against the current repository or Drive file.
The earlier Library synchronization rule is superseded. This changes no
camera calibration, measurement admission or motor state.

## 2026-11-12: fast Shadow source evidence and decision age

The active OV9281 Pi v7 stereo calibration (SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`),
AR0234 v5 intrinsic and AR-to-left extrinsic remain unchanged. The optimized
native-capture, half-resolution stereo, exact COCO instance-mask path is
separate from the combined v3-box-to-mask diagnostic; a v3 box is never
treated as an instance mask. Earlier fast-path results below 0.5 s remain
valid within their measured endpoints, while the slower full-resolution
dynamic audit and the combined v3-plus-mask run do not replace them. The
camera-only launcher cannot execute motors.

On the Pi, the 75 s mask-only run at
`/home/elwis/.local/state/sie/evidence/shadow_fast_20261112_193449`
produced 296 camera cycles after a 13.382 s startup interval. The archive
`shadow_fast_20261112_193449_review.tar.gz` has SHA-256
`2072ba9230f9ed79a0863ba203eba89519a8d719e03b807eeadf612c2d65d43c`.
All nine requested original-RGB/stereo, individual-mask, disparity/LR and
calibration bundles passed cycle, hash, track, instance, model and driver
timestamp checks; offline replay reproduced their mask-depth and front p05.
No source images were added to Git. The Bridge journal has 296 camera rows
and 1,269 polling rows with no new camera cycle. Driver-timestamp-to-decision
median/max were 0.176763/0.359337 s for AR0234 and 0.175512/0.355097 s
for OV9281; all 296 camera rows were below the experimental 0.5 s limit.
For the 48 one-person segmentation rows, medians were 0.284523/0.282028 s.
These are driver-declared ages at the Bridge decision, not independently
verified exposure-to-motor-command ages.

The segmentation found one person on cycles 1–48 and none on 49–296. It did
not record reappearance after the person left, and no simultaneous two-person
scene was staged. Forty-three cycles evaluated mask depth: 13 were coherent,
30 failed the unchanged 0.05 m MAD gate. Five different track IDs occurred
while the same person was visibly present; depth refusals broke confirmation.
The snapshot reason `FIRST_COHERENT_AFTER_TRACK_LOSS` describes those breaks,
not post-exit reacquisition. In this scene the all-mask front p05 and numerical
candidate-layer p05 differed by no more than 0.029194 m across 43 evaluated
cycles, so the earlier archived large two-layer mismatch did not recur. The
13 coherent cycles still failed `FRONT_LAYER_BINDING_UNVERIFIED`; all 1,565
Bridge decisions remained PAUSE. Projected near-p05 points were generally
inside the selected mask, but physical person ownership, front clearance,
depth error and the optical-to-rim datum remain unproven. The diagnostic
0.5–4.5 m range is not the activated 0.5–2.0 m Measurement range.

Next: use the saved RGB/stereo/mask bundles and an independently measured
datum to establish whether the selected depth surface belongs to the person
and has a bounded physical error before choosing a front-range statistic.
Keep the current MAD threshold, point-set refusal and motor boundary. The
live post-exit reacquisition and two-person cases remain untested; do not
repeat the fast timing proof or a straight-wheel motion probe. Detailed
evidence: [source snapshot report](docs/diagnostics/shadow_fast_source_snapshot_run_20261112.md)
and [working baseline](docs/handoffs/WORKING_BASELINE.md).

## 2026-11-11: equivalent context-update commands

The operator confirmed that `обнови документ` and `обновить документ` are
equivalent commands for this change-controlled file. This clarification
changes no camera calibration, measurement admission or motor state.

## 2026-11-10: Shadow RGB v3 independent review and two-person stereo diagnostic

The active camera calibration is unchanged: **OV9281 Pi v7**, SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`;
AR0234 v5 intrinsic SHA-256
`e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096`;
AR0234-to-physical-left extrinsic SHA-256
`10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc`.
No new calibration or admitted metric Measurement follows from these tests.

**Independent RGB detection check.** A fresh projector-off two-person
AR0234 session saved 1200 frames; a verified 5 s subset contains 120 images.
Source subset SHA-256:
`2e508576c2a9582a523c19d86023c33e3a38f29a46720411ed888afe0ad8b7b8`.
The operator reviewed all 120 LabelMe records and adjusted boxes on eight
frames. Reviewed archive SHA-256:
`b976002ee51f0e41a8f1a7de85826fd57cf16e98551e7b54b02506a5df477efe`.
The reviewed sample has 32 empty, 26 one-person, and 62 two-person frames
(150 labeled people). In offline CPU ONNX evaluation with the same confidence
0.40 and NMS IoU 0.45, both v2 and v3 detect a person in all 88 positive
frames and none of the 32 empty frames. On two-person frames v2 returns at
least two boxes in 56/62, v3 in 60/62. V3 still returns only one box in
`ar0234_000601.jpg` and `ar0234_000660.jpg`; v2 returns two in those frames.
The v2 and v3 ONNX SHA-256 values are respectively
`5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b`
and `fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5`.
The evaluated boxes largely originated as v3 drafts, though all were
operator-reviewed, so IoU localization comparisons favor v3; the count and
presence figures are the more useful comparison. Five-second samples do not
prove uninterrupted detection, identity across occlusions, or tracking. The
[reviewed comparison](https://chatgpt.com/library?file_id=libfile_67d846ee50008191b583fe03e4699304&account_id=personal)
has SHA-256
`1f63ca416d20c11702e29b849a30fd73a84294a3e1e4ec6b3d8a8a0d068cfdfc`.

**Separate stereo feasibility check.** The older 2026-10-31 two-person
stereo archive (SHA-256
`b8d0437995ff81b9b6ddcd24029ecfb0857571729614566fa97c70796b2b1cbb`)
contains 29 RGB frames with two v3 detections each, but only 19 have saved
LR-consistent disparity. This archive is separate from the 2026-11-10 blind
RGB subset and may overlap v3 training; it is not an independent detector
test. Offline projection of v7 stereo samples into the two inner RGB box
regions gives 1201–3982 depth samples per region in those 19 frames. The
nearer region has median optical z 1.653 m (range 1.640–1.697 m), the
farther 2.293 m (range 2.246–2.355 m); the smallest per-frame median
separation is 0.549 m. In both recorded setups the nearer region is image
left, so closer-person selection with the nearer person on the right is
untested. Rectangular boxes do not establish person-specific ownership of
depth samples; no pair of box regions jointly passes the existing 50 mm
global MAD gate in the 19 frames. Ten frames lack saved disparity, and
physical distance ground truth and exposure synchronization are missing.
These optical depths are not measured robot-front clearance or evidence for
the intended 0.8 m hold. Full method and per-cycle data are in the
[stereo diagnostic](https://chatgpt.com/library?file_id=libfile_979024524dc88191886866502a76b22a&account_id=personal),
SHA-256 `4937eed7f074a6b405426174fcb8f2eff68fe72337cb7f84e83ea964537f3906`.

**Next:** with motors off, collect a fresh paired two-person sequence with
current-frame individual masks and their RGB-to-stereo bindings, calibrated
depth and quality per person, exposure age, crossing/occlusion, and a scene
where the closer person is image right. Choose the nearer person only when
both person-owned depths pass a validated quality and separation rule;
otherwise report `AMBIGUOUS_OR_UNAVAILABLE` and keep Shadow `PAUSED`.
Preserve the active v7 calibration and the existing MAD gate. Continuous
target ownership, front-relative range, physical stopping bound, and
camera-driven floor motion remain unvalidated; this check does not authorize
motor execution.

Older dated sections below preserve what was known at those checkpoints.

## 2026-11-01: Shadow RGB v3 training result and next check

The active stereo calibration remains **OV9281 Pi v7**, SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
No new stereo calibration, admitted metric Measurement, target identity,
robot-front clearance or physical stopping bound has been established.
Nothing in this RGB training authorizes Shadow motor execution.

The operator corrected six two-person LabelMe images and confirmed review.
The reviewed archive SHA-256 is
`8b79e947ef381825440bfe2a826b463ac4568d42182b62917e1cb606b792438f`.
Their source JPEGs were unchanged. A derived training addendum with six
images and 12 upper-body boxes has SHA-256
`8e5038c5eb2c98150a5ab5c1110fb5ce69d111a0a7b939c76ec2175bf8445612`.
It extends the 230-image reviewed training pool to 236 images. The six
images were chosen after seeing v2 misses and are training data, never an
independent holdout. The original LabelMe archive retains its review flags;
the derived addendum carries the operator-review record.

The [Shadow v3 Colab notebook](https://drive.google.com/file/d/1rKV1cHTnkp3uewSWEPTdkud6NIqa-Gi_/view?usp=drivesdk)
started from the saved v2 `best.pt` (SHA-256
`575c480b03b1722222211f474fae2dc8c7c6fe5e02c9078433d5d9e16667b395`)
with `resume=False`, trained on a Tesla T4 with Ultralytics 8.4.152 and
torch 2.11.0+cu130, and exported v3. The new checkpoint SHA-256 is
`e5d7e67f7ab2b53f6a8548e3eec13fdf52e84d390cf0e58088ba40dca79989db`;
the [v3 ONNX](https://drive.google.com/file/d/1cX7vboJDX9u4rfAf80zmMtnSZAmrtfyk/view?usp=drivesdk)
SHA-256 is
`fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5`.
The supplied ONNX bytes matched that digest. Colab checked the export with
`onnx.checker`; inference on Pi with the production preprocessing remains
pending. The v3 hash is now accepted by the offline one-person evaluator as
a diagnostic model, without changing the active camera runner's model.

On the same previously inspected 120-image development set, v2 localized
88/89 positive frames at confidence 0.40 and IoU >= 0.50, with detections
on 1/31 empty frames. V3 localized 89/89 and detected on 2/31 empty frames.
The Colab mAP50–95 changed from 0.78947 (v2) to 0.81349 (v3). On the six
selected *training* frames, both people matched in 1/6 with v2 and 6/6 with
v3. These results do not establish generalization or continuous tracking;
the extra empty-frame false positive needs inspection.

The original Colab [`comparison.json`](https://drive.google.com/file/d/1UiXGi6Z0xDdXcJ5Jzowc0jeh34XdZr9r/view?usp=drivesdk)
mistakenly labeled its per-frame entries `v1`/`v2`, although its actual
`old_model` and `new_model` were v2/v3. Its numeric values and the correctly
labeled `ap` section were unaffected. The original remains unchanged,
SHA-256 `3595740d77535415a60ddc08657efd2ea0749ca0b968e49e08550fb867c40ee1`.
The [corrected derivative](https://drive.google.com/file/d/1KrxcwhtNdqBKoS0RFC3CyQJUbBWwWDHM/view?usp=drivesdk),
SHA-256 `10a1a14c280bf45ba929e10058b8eab2265974c5eedf8b81ae8b3c6d4399086a`,
records the source hash and exact label mapping. The notebook source was
fixed. The [result handoff](docs/diagnostics/shadow_rgb_v3_two_people_training_result_20261101.md)
and [v3 hash pin](vision_core/tools/evaluate_shadow_rgb_dev_onnx.py)
are committed on branch `codex/shadow-rgb-detector-20261030`, commit
`edafd6df40ea9e881764827017a8b1a0fb51a527`.

**Next:** with motor power off and projector off, collect a *new* 10-minute
AR0234 session of one moving person entering, crossing and leaving the frame,
with empty-room intervals. The operator currently has only one person
available. Freeze the reviewed labels before comparing v2 and v3 ONNX with
the same Pi preprocessing and threshold; assess misses, empty-frame false
positives and timing. This checks single-person regression only. Later,
record a separate two-person session with both people simultaneously visible
and build a matching multi-person evaluator; the existing reviewed-subset
evaluator rejects more than one labeled person per frame. Nearer-person
selection, cross-person identity, stereo/robot-front range, 0.8 m stopping
and automatic Shadow motion remain unvalidated.

Older dated sections below preserve what was known at those checkpoints.

## 2026-10-31: Shadow RGB evidence and model-training handoff

The active camera calibration is still **OV9281 Pi v7**, SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
No new calibration, admitted metric Measurement, robot-front clearance, or
autonomous motion claim follows from the RGB work below.

Saved moving-person stereo evidence remains insufficient for reliable Shadow
range: a 14-cycle masked-depth replay yielded two coherent diagnostic frames,
three MAD refusals, and nine frames without stereo geometry. A separate
26-cycle moving-person series had 15 RGB candidate cycles and only two
coherent masked-depth cycles (2/15), separated by 15.55 s and seven refused
cycles. A saved matcher comparison and a median-filter replay did not change
the relevant acceptance/refusal classification. The 650 nm projector was
present in the reviewed moving-person series; it is **not** used for the new
RGB two-person training capture or the planned blind evaluation. Existing
stereo calibration and the 0.05 m diagnostic MAD gate were not relaxed.

The operator recorded three 10-minute AR0234 RGB sessions, each with 1200
frames and no skipped slots, then selected 120 frames per session for manual
review. The first training session and the separate validation session were
recorded with the projector on; the later session with people appearing in
turn had the projector off. The checked two-person LabelMe archive has SHA-256
`4ee79b52c1c421fdeb91be9abedf938c6bf85afc777f97819a3a8cfdd41d97af`.
The combined reviewed training pool contains 230 images (208 positive upper-
body boxes, 22 clean negatives); ten ambiguous frames were excluded from the
240 selected source frames. Session prefixes prevent filename collisions.
The separate 120-image development validation has 89 positive and 31 negative
frames and zero byte-identical overlap with training. It was already inspected
and used in diagnosis, so it is not a blind test. The existing ONNX detector
(SHA-256 `dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c`)
matched 71/89 positives at confidence 0.40 and box IoU >= 0.50, with 0/31
false-positive negative frames in this development set. The original Colab
model's stronger internal test result came from a single capture environment
and is not evidence of generalization.

The previous training artifacts were located in Drive: YOLO11n Detect,
one `person_upper_body` class, Tesla T4, input 640, batch 16, up to 100 epochs,
patience 20, best epoch 27 and early stop after 47. Crucially, its `best.pt`
checkpoint still exists. The prepared Shadow v2 Colab notebook starts from
that checkpoint with `resume=False`, trains on the 230-image pool, compares
both checkpoints on the 120-image development validation, and exports a
640x640 static ONNX with opset 17. Its data packages and notebook are in the
[Drive Colab v2 folder](https://drive.google.com/drive/folders/1d9OYCDuvjT4cKxyN86foeagra9lVpd_p).
The three archive SHA-256 values are
`9a6a65a800c0847bdac646cd05c631de92aeacc976226160f9ec720ea03e4ebb`
(training session 1),
`76550684e0826f210d93989a62f233443ae12af179066365e6bd53f38c191feb`
(training session 2), and
`80b74757e35c0a76a7d13f21c43bc0f9ef3813c5af0f4156053c9a26ed706c6c`
(development validation). At this dated checkpoint the notebook was prepared
and syntax checked but had not yet run. The later training result is recorded
above. The reproducibility record is
[Shadow Colab v2 handoff](docs/diagnostics/shadow_rgb_yolo11n_v2_training_handoff_20261031.md).

The next step at that checkpoint was to run the notebook on Colab T4,
review `comparison.json` and inspect per-frame misses, then collect a fresh
blind projector-off two-person session
and check deployment ONNX with the same Pi preprocessing and timing. Do not
infer stable person identity, valid range, 0.8 m stopping, or motor readiness
from RGB detector scores. Shadow motor execution remains disabled pending
fresh target ownership, associated robot-front range, timing, disagreement,
and physical stopping evidence.

## 2026-10-29: current Shadow checkpoint

The operator's Shadow objective is to detect a person with AR0234, begin moving
after a fresh observation, follow a moving person, and pause the *same*
`command_id` whenever the target is lost or scene and encoders disagree. After
fresh reacquisition and a distance check, that command resumes through further
bounded segments; the intended hold distance is 0.8 m from the robot front.
RGB detection may initiate or reacquire a target, including when OV9281 depth is
temporarily unavailable; a future LiDAR may supply associated range. Neither an
unassociated depth sample nor optical camera Z is robot-front clearance. The
0.8 m hold and 0.95 m resume thresholds in the candidate range gate are
experimental, without an established physical stopping bound. The runner has
no public motor `--execute` route until fresh target ownership, range-to-front,
age, and disagreement checks can be supplied and validated.

The flashed ESP32 candidate reports
`FIRMWARE_BUILD_ID=v4_2_uart_shadow_v1_candidate` and
`shadow_api_version=v1_paused_segment`. A motor-power-off probe confirmed
PAUSE/RESUME with the same `command_id`. An initial lifted-wheels attempt with
motor power off correctly reached terminal `BREAKAWAY_NOT_FOUND`; it is not a
successful movement test. After powering the motors and lifting the wheels, a
second probe observed two approximately 0.02 m segments with the same ID,
zero encoder drift during PAUSED intervals, both wheels stopped on PAUSE and
rotating again after RESUME, and confirmed STOP. This proves those transitions
in that setup, not floor following or physical stopping at a person.

The opt-in camera-only dynamic Shadow preview preserves existing static-v2
gate results in its audit while reporting separate current-RGB and masked-depth
diagnostics. It can consider one high-confidence person clipped by an image
edge and record adjacent-box continuity; that continuity cannot establish
persistent person identity. The initial 130-record preview yielded 42 selected
RGB targets and 88 refusals, including 55 refusals with detections; only 4 of
the 42 selected targets had coherent masked depth. The next 94-record preview
admitted three clipped targets but exposed seven near-right-edge geometry
refusals. The subsequent 104-record preview exercised the near-edge diagnostic
override: 16 geometry overrides, 10 clipped selections, 93 mask diagnostics,
20 coherent and 73 `DEPTH_MAD_EXCEEDED` results. One person was detected in
all 104 records; nine frames fell below confidence 0.7 and one box was too
narrow. Adjacent boxes stayed continuous for 92 frames, with a loss around
frames 39–48 and a new segment at frame 49. This is diagnostic association,
not proven reacquisition of the same person. The 0.05 m mask-depth MAD gate
remains unchanged. The compact preview did not retain synchronized raw stereo
and mask images, so the cause of MAD refusals remains open. None of these
camera records authorizes a motor command or admits metric Measurement.

The active stereo calibration remains **OV9281 Pi v7**, SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`;
V6 is historical. The immediate next engineering step is a camera-only capture
of matched masks, disparities and timing around coherent and MAD-refused
frames, followed by a reviewed front-clearance and target-continuity input for
the Shadow supervisor. Keep the motor runner disabled while those inputs are
unavailable; later perform a supervised floor trial with independent power
cutoff and explicit observation of both wheels.

The implementation candidate is the separate Shadow firmware, UART facade,
supervisor, range gate and probes (commit `6370a23`), plus an opt-in ROS preview
node with a diagnostic helper and wrapper (commit `b935d13`). The last camera
overlay archive has SHA-256
`904f072f47298448ee5e3310c4ce2fa0940448ed8701a632fc636e2b5a1bafa8`.

## 2026-10-29: earlier UART and moving-target control checkpoint

This is the current continuation point. The 2026-10-28 v4 handoff below is
historical. **OV9281 Pi v7 remains the active stereo calibration** with SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`;
no new calibration or metric Measurement admission has occurred.

The operator's control objective is unchanged: detect the person anywhere
usable in the AR0234 image, start one bounded forward command straight, then
adjust left/right drive from later fresh observations during that same command.
The person may move. Current segmentation is person-specific, and this
supervised diagnostic is not autonomous navigation.

The v4 Wi-Fi-controlled firmware was loaded and one supervised 0.20 m floor
run was measured by floor marks at about 0.20 m. The person stood with slight
sway; the cart turned right in that run. This single observation does not
establish reliable tracking of a laterally moving person. A later run sent a
movement POST but stopped after an HTTP timeout. Its journal showed the
`/move-forward-steered` POST itself timed out at 0.403 s, followed by a
successful `/stop`; the operator observed no movement. Other camera-loaded
trials showed intermittent HTTP status and steering-update timeouts.

With motors off, 30 read-only `/status` calls succeeded 30/30 without camera
load (maximum 67 ms). Under camera preview, one series succeeded 23/30,
another 25/30 with Pi Wi-Fi power save off, and a later one 27/30; 0.4 s
timeouts remained. A simultaneous 30-ping series lost 4/30 before disabling
Pi Wi-Fi power save and 0/30 in later series. The mobile hotspot is
`SIE-Mobile`, 2.4 GHz only; Pi and ESP32 share it. These observations show an
intermittent control-path delay under camera load, not a proven cause in the
camera process, Pi scheduling, radio, phone hotspot, or ESP32 HTTP server.

The alternative GPIO UART link uses Raspberry Pi 5 pin 8/GPIO14 TX to ESP32
RX2/GPIO16, Pi pin 10/GPIO15 RX from ESP32 TX2/GPIO17, and common GND. On the
Pi, `dtoverlay=uart0-pi5` is active, serial console was removed from kernel
arguments, `/dev/ttyAMA0` exists, and `serial-getty@ttyAMA0` is inactive.
The motor-power-off diagnostic image `v4_2_uart_diag_v1` returned 30/30
matching PINGs while idle (median 7.93 ms, max 7.96 ms) and 100/100 during
30 s camera preview (median 7.92 ms, max 10.71 ms); preview exited normally.
Both series reported boot session `81621C29240F9A3E`. This validates the
physical serial link for diagnostics, not active motor control. See
`firmware-build/uart-link-diagnostic-20261028/README.md`.

The first active UART image `v4_2_uart_motion_v1` was flashed with motors
unpowered. Its very first host STATUS request timed out. USB serial evidence
showed a repeated `xQueueSemaphoreTake` assertion and reboot immediately
after `UART_ONLY_WIFI_DISABLED`. Decoding its backtrace located the crash in
`WebServer::begin()`/`NetworkServer::begin()`: the image started an HTTP
listener after disabling Wi-Fi. That image is withdrawn. The follow-up
`v4_2_uart_motion_v1b` removed the server start, compiled and passed 21 host
tests, but the operator supplied a long unreadable serial capture (`□p`
repeated) with no version or readiness lines. The cause of that capture has
not been established; do not claim v1b passed hardware startup or UART.

The v1c Arduino sketch is
`firmware-build/uart-motion-v1c-20261029/source/traction_control_v4_2_uart_motion_v1c/traction_control_v4_2_uart_motion_v1c.ino`,
with its two headers in the same folder. V1c removes Wi-Fi initialization,
HTTP listener registration and service, and the Wi-Fi credential dependency
from its startup path. Local ESP32 core 3.3.12 compilation succeeds (496811
bytes flash, 70552 bytes globals); 21 Python host transport/runner tests pass.
The matching private merged image has SHA-256
`e54c325da4f15d97407fd9cd6d6802195c1bdd88e8acc2ea1bc38e089363cc15`.
The operator flashed v1c and supplied a fresh USB boot log showing
`FIRMWARE_BUILD_ID=v4_2_uart_motion_v1c`,
`UART_ONLY_NETWORK_NOT_STARTED`, and `SIE_UART_READY`, with one
`POWERON_RESET` and no repeated reset in the supplied capture. A direct
UART STATUS read returned HTTP 200, exact v1c build ID, boot session
`27F906E8AAFBC92F`, `READY`, no active command or latched fault, and zero
left/right PWM. The first probe attempt failed before I/O because `elwis`
lacked membership in the Pi's `dialout` group. After access was corrected,
the installed `probe_uart_motion_off.py` rejected the v1c response because
its expected build string was still `v4_2_uart_motion_v1`; the operator
changed that exact check to `v4_2_uart_motion_v1c` locally and retained a
`.py.bak` copy. This local probe edit has not been reported as committed.

With motor power off, the corrected probe completed 100/100 UART STATUS
requests at idle (median 61.94 ms, max 61.97 ms) and rejected a deliberately
wrong boot-session movement request with HTTP 409
`BOOT_SESSION_MISMATCH`; final state was `READY`. A second 100/100 series
ran during a 45 s camera-only preview, with operator-confirmed overlap:
median 61.93 ms, max 68.25 ms, the same boot session, wrong-boot request
rejected, and final state `READY`. The preview exited normally with 186
frames, zero journal drops/errors, and motor access disabled. It recorded
48 `TARGET_UNAVAILABLE` frames and 138 masked-depth diagnostic frames;
the control decision was `STOP` in all 186. Its 125 coherent masked-depth
results are diagnostic only, not admitted metric Measurement.

**Next:** preserve motor power off. Put the exact v1c probe expectation into
the versioned source and inspect the UART command/stop path in a bounded
motor-off test before considering any floor run. The invalid-boot rejection
and read-only STATUS series do not establish valid-command behavior,
stopping behavior under motor power, or reliable target following. Keep
supervised diagnostics separate from admitted Measurement and autonomous
operation; do not infer that the earlier Wi-Fi timeout cause has been proved.

## 2026-10-28: active calibration and moving-target control checkpoint

This dated section is the current continuation point. Older dated sections and
their `Next` or `Current` labels below are preserved as historical checkpoints;
they do not supersede this section.

**Active stereo calibration is OV9281 Pi v7**, not V6. The Pi artifact is
`/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz`,
expected SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
The AR0234 intrinsic remains `ar0234_intrinsic_v5_final_20260910` with SHA-256
`e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096`;
the AR0234-to-physical-left-OV9281 extrinsic remains v1 with SHA-256
`10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc`.
These identities are bound in
[decision-chain activation](docs/calibration/ar0234_ov9281_decision_chain_activation_v1.md).
OV9281 v8 and v8b were rejected and are not active. The
`vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md`
describes an earlier V6 calibration/runtime and remains relevant when changing
V6 files; it is **not** the active stereo calibration for this RGB + stereo
experiment. V7 far-range metric accuracy and current-frame rectification
remain under investigation. A plausible diagnostic depth is not an admitted
metric Measurement or a physical error bound.

The intended target behavior remains: acquire the person at any usable RGB
position, start one bounded forward command straight, then use later fresh
frames to adjust the two forward wheels within that same command. The target
may move. Current segmentation is a person-specific adapter; arbitrary object
classes and autonomous tracking are not established.

The latest **v3 floor trial** requested 0.20 m and ended in ESP32 `SUCCESS`,
`READY`, zero PWM, no active command, no bounded fault, and no independent
steered-safety latch. The operator measured about 0.20 m floor travel and
about 1.5° leftward body turn. During the command the RGB target bearing was
8.5–11.7° **right**; five accepted `DRIVING` updates requested the maximum
+20 PWM rightward correction. Encoders nevertheless reported right 0.2010 m,
left 0.1939 m, center 0.1974 m and heading +0.0244 rad, consistent with the
operator's leftward observation. Thus v3 completed and stopped normally but
did not steer toward the target in that trial. Evidence:
`/home/elwis/.local/state/sie/evidence/v7_live_steered_forward_1793187350_8e0ba8d8/summary.json`.
The preceding v3 raised-wheel trial accepted two negative steering updates;
its third update received HTTP 409 because the controller had already entered
`BRAKING`. The runner's normal-braking handling was corrected in commits
`10eecc4` and `a2ab40b`. A separate initial read-only `/status` timeout sent
zero movement posts; its preliminary timeout was raised from 0.4 to 2.0 s in
`a3dd2c6`. These host fixes did not change the installed firmware.

**V4 is the next unflashed diagnostic candidate**, committed to
`codex/sie-ros2-integration-v1` as `d58aaec5e254f70fd85af67948f66b431ee11ec3`.
The v3 wheel synchronization still targeted equal travel while the camera
requested a curve. V4 instead accumulates the desired left-minus-right
encoder travel difference from subsequent center travel and the latest
command-bound camera bias; full bias requests 0.18 m wheel difference per
metre of subsequent center travel. The initial command still starts straight.
Existing center, wheel and heading caps, independent 300 ms motor guard,
700 ms feedback deadline and `/stop` remain. The Pi runner requires the exact
`v4_2_live_steering_v4` build. Source, reasoning and limits:
[v4 steering record](docs/diagnostics/live_steering_direction_v4_20261028.md).
The v4 sketch compiled for ESP32 core 3.3.12; twelve Pi runner dry-run tests
passed. Its private 4 MB merged image is on the operator computer at
`/home/kestabor/Документы/ChatGPT/SIEE/firmware-build/live-steering-v4-20261028/sie_live_steering_v4_esp32_4mb.bin`,
SHA-256 `a386d6bae8a20b51594aac7734ee13c574529804345b18727fae76fec5e02c94`.
It embeds local Wi-Fi credentials and must not be published. No v4 `/status`
or physical motor observation has been supplied; do not claim that v4 steers
correctly on the floor.

**Next:** with motor power off, load v4 and verify `/status` reports exact v4
build, `READY`, no active command, zero PWM and no faults. Update the Pi branch
to `d58aaec`, measure the available front path, and conduct one operator
observed 0.20 m floor trial with accessible motor cutoff. Record the camera
bearing/update sequence, final encoder distances and heading, measured floor
travel, actual turn direction and confirmed stop. If the target is lost, the
status faults, or the stop is unconfirmed, stop and inspect the saved evidence
before another movement. This supervised experiment does not activate
canonical Measurement or autonomous motion admission.

## 2026-10-27: historical moving-target approach checkpoint

The operator's target-following requirement is **acquire a moving semantic
target anywhere usable in the RGB image, start moving straight, then update
left/right wheel drive from later fresh observations while the same forward
command remains active**. The target need not be centered before starting;
separate stop-and-turn commands are the historical diagnostic, not the intended
controller. A changed, lost, stale, or ambiguous target requires a stop.
The current camera adapter detects a person; arbitrary object classes remain
future adapter work. The target's image coordinate is updated on every frame
and is not assumed fixed.

The active branch is `codex/sie-ros2-integration-v1`. The implementation
checkpoint before this context revision is `66ac08a`. The versioned
`traction_control_v4_2_live_steering_v1` firmware candidate, based on the
loaded-start v4 controller, adds `/move-forward-steered` (0.02–0.30 m) and
command/boot-session-bound `/steer-update` calls with a rising sequence
number. It starts with zero steering bias; later observations may request
bounded differential forward PWM (±20). A nonzero update is accepted only
after the drive enters `DRIVING`. The ESP32 feedback watchdog stops directed
drive after 700 ms without an update. Existing encoder cutoffs, bounded
timeout, fault latch, and observed `/stop` remain in place. The Pi runner is
`vision_core/tools/run_v7_live_steered_forward.sh`; it also has a
`--preview-only` camera mode with no HTTP or motor requests. Code and
contract: [moving-target approach diagnostic](docs/diagnostics/continuous_moving_target_approach_20261027.md).

The first camera-only preview recorded 44 unique frames over 8.77 s, with
one straight acquisition and subsequent proposed bias of +5 or +6 PWM for a
person about 1.19–1.42 degrees right of the optical axis. It posted no motor
or network requests. A moving-person preview then recorded 67 frames but
accepted only one under the older static-v2/mask check. The saved camera
journal showed `MOTION_DETECTED` during lateral movement; from frame 11
onward the person's box repeatedly touched the top image edge (top y about
0–3 px). The segmentation still reported one person, generally about
0.84–0.93 confidence. These are observed refusal causes, not proof of
continuous tracking. Evidence roots:
`v7_live_steering_preview_1793126913_6ed87990`,
`v7_live_steering_preview_1793127837_e6838a67`, and
`control_input_preview_half_640x400_single_segmentation_onnxruntime_20261027_220358`
under the Pi's `/home/elwis/.local/state/sie/evidence/`.

Commit `66ac08a` separated the supervised moving-target RGB check from the
static-v2 stillness and stereo-mask checks. It binds current segmentation,
calibration and AR0234 driver timestamp, accepts a top-clipped person if it is
the sole high-confidence instance, and checks adjacent-frame box
correspondence. Side clipping, multiple instances, stale frames, and
unconfirmed correspondence still refuse or stop the trial. This change
does not alter the static profile, admitted metric Measurement, or autonomous
motion admission. Seven local tests and Python compilation passed; C++
static analysis passed for the earlier firmware candidate. There is **no
camera preview after `66ac08a` and no compiled, flashed, or physically
tested live-steering firmware evidence yet**. The actual ESP32 build now
installed must be rechecked from `/status`; do not infer it from source.

Next: update the Pi branch and repeat the 20 s **camera-only**
`--preview-only` trial while the person moves across the optical axis.
Inspect the new summary for sustained tracking, positive and negative
proposed bias, target-loss events, and zero motor/network posts. If that
passes, compile and flash the versioned candidate on the operator's
computer, confirm its exact build and zero-PWM READY state through
`/status`, then begin a separately observed short bounded motor test.
The physical steering response, stopping distance, and dynamic target
continuity remain unestablished. Do not promote the preview to an admitted
Measurement or autonomous approach.

## 2026-10-26: historical supervised V7 turn checkpoint

The active branch is `codex/sie-ros2-integration-v1`. The latest source and
documentation checkpoint before this update is `6578a71`. The separate
`run_v7_supervised_approach.sh` experiment requires `--execute`; its
`--diagnostic-turn-only --max-steps 1` mode cannot dispatch a forward POST.
This supervised experiment does not change the camera diagnostic's STOP output,
admit a canonical Measurement, qualify the actuator, or approve autonomous
navigation. Earlier statements below that the active stage excludes motor
control describe the canonical diagnostic path; this separately authorized
supervised trial has since been added.

The first short right turn (1.278 degrees requested) ended in
`PARTIAL_PROGRESS` with no observed chassis turn. A later 3-degree test held
PWM at 180/180 while encoder counts stayed at 1/1 until the host sent `/stop`.
Firmware `v4_2_micro_turn_watchdog_v3` repairs the identified path that skipped
the command deadline and stall checks; it has not yet passed a physical turn
test. Do not infer turn torque or safe stopping from source-level correction.

Two subsequent turn-only attempts sent no motion POST. In the first, read-only
`GET /status` timed out after 573 ms at the former 500 ms deadline. In the
second, it timed out after 1.121 s at the new 1 s deadline. The latest runner
now performs preliminary status reads at 1 s and, on `TimeoutError` only, one
2 s retry. It records both attempts in `turn_01_preflight.json`, then obtains
a new camera frame and rechecks target identity, bearing direction and age
before the single possible turn POST. Motion POSTs are never retried.
Seven local tests passed for the latest retry change; no physical run after
`6578a71` is recorded. The next evidence should be one operator-supervised,
turn-only attempt after confirming the Pi has this branch and the intended
ESP32 watchdog build. Inspect `summary.json`, `turn_01_preflight.json`, the
turn HTTP journal, terminal status and observed chassis motion before any
forward trial. A repeated status timeout calls for network/controller
diagnosis, not additional retries. Details: [V7 micro-turn record](docs/diagnostics/v7_micro_turn_20261026.md)
and [approach runbook](docs/diagnostics/v7_supervised_approach.md).

## 2026-10-26: current network and NVMe handoff

The operator confirmed OpenSSH from Kestabor to Raspberry Pi 5 over Tailscale: `ssh elwis@100.70.100.29` (PASS). The laptop is `kestabor-hn-wx9x` at Tailscale `100.96.180.119`; the Pi is `piecake` on mobile hotspot `SIE-Mobile`. They do not need the same physical Wi-Fi network. Tailscale SSH is disabled; `tailscaled` is running. These facts do not establish Tailscale access to ESP32 or authorize motion.

A post-boot Tailscale/SSH timeout recovered on 2026-10-26. Mobile-network control-plane sync and port 80 involvement are hypotheses, not established causes; a separate port 80 probe returned HTTP 302. The accepted action is observation: preserve Netplan, DNS, service ordering, Tailscale and SSH. Revisit only recurring delays over 2–3 minutes with working internet, using current-boot logs. The Pi's NVMe APST is disabled by `nvme_core.default_ps_max_latency_us=0`; retain the setting. A 1 GiB read and SMART/boot-log checks were clean in the observed interval, but original fault cause remains unknown. Do not run motors during network diagnosis or change NVMe settings without a specific need. Full operator-provided evidence and commands: [network infrastructure handoff](docs/diagnostics/network_infrastructure_20261026.md).

## Project name and compatibility

**DECISION — `OWNER_APPROVED`, 2026-11-26.** **SIE** is the single canonical
name of the project, architecture, and future product. The full name is
**Spatial Intelligence Engine**, pronounced **SEE**. Use SIE in current and
future architectural descriptions, development, and documentation.

**SIEE** (Spatial Intelligence Evidence Engine) is the historical designation of
an earlier experimental stage of SIE. It refers to the same project and
architecture. Retain historical wording where it records evidence provenance,
supports experimental reproducibility, or preserves development history.

The freeze preserves existing branches, tags, packages, modules, files, ROS 2
nodes and topics, schemas, calibration artifacts, evidence directories and IDs,
and other runtime identifiers. Established names such as `/sie/...` and `sie.*`
continue to use `sie`. Previously approved architecture principles remain in
force.

## Read this first

Read in this order before changing SIE:

1. `AI_CONTEXT.md` for the normative architecture and precedence rules.
2. This `PROJECT_CONTEXT.md` for the current cross-project state.
3. `AGENTS.md` for operational rules.
4. The relevant version-specific handoff. Read `vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md` before modifying V6 files; check the active v7 artifact and activation record for current RGB + stereo work.
5. The implementation, tests, reports, activation profile, and `git status` for the task at hand.

If these sources conflict, use the precedence in `AI_CONTEXT.md`; do not guess.

## What SIE is

SIE is an evidence-based spatial-intelligence architecture for a construction object. It integrates project and BIM/CAD data, surveying, inspections, sensor observations, material and zone state, work records, and reports from admitted robotic complexes, machines, and mechanisms into a traceable model of the physical world. SIE uses that model for safe engineering decisions and authorized actions. It is not a particular neural model, camera, ROS 2 package, or robot base.

Canonical flow:

```text
Raw Observation
  -> Observation
  -> Measurement
  -> Interpretation
  -> Fact
  -> World State
  -> Decision
  -> Action
```

Important invariants:

- Every spatial value has a unit, explicit reference frame, timestamp, confidence, and provenance.
- Calibration is an applicability gate, not a shortcut to trust.
- Diagnostic evidence is never an approved Measurement, Fact, World State update, Decision, or Action.
- Models are replaceable. Data contracts and frame semantics are not.
- Missing, stale, ambiguous, or low-quality evidence fails closed.

## Fundamental Architecture Principles

**DECISION — `OWNER_APPROVED`, 2026-11-26.** These principles apply to the
entire SIE architecture and its future development, including the current MVP.

> **SIE integrates independent knowledge about the physical world obtained from multiple sources, forms a coherent representation of its environment, and uses this representation to make decisions about how to interact with it.**

1. **Knowledge over Sensor Data.** Integrating knowledge takes precedence
   over directly fusing sensor data. Sensor fusion is an enabling mechanism
   for Spatial Intelligence, the objective of SIE.
2. **Sensor Independence.** Each sensor is an independent source of
   information with its own capabilities and limitations.
3. **Entity-Centric Representation.** An entity is the central element of
   the physical-world representation. Individual measurements supply
   evidence about entities.
4. **Complementary Evidence.** Different sources can contribute
   complementary knowledge about the same entity while observing
   different physical surfaces.
5. **Understanding Before Action.** The robot makes decisions based on its
   knowledge of the situation, the current task, applicable instructions,
   and its own capabilities.

### Architectural consistency

- **Data Contracts and provenance.** Modules exchange data through approved
  Data Contracts. Observations stay bound to their sources. Measurements
  remain independent quantitative results with explicit units and reference
  frames, applicable calibration or method, and evidence provenance.
  The Evidence Graph preserves links to the originating Observations.
- **Entity lifecycle and World State.** Entities retain stable identity,
  state, properties, and lifecycle. Only the World State Manager writes
  World State, using verifiable Facts with source, timestamp, confidence,
  and evidence links. An unverified hypothesis remains a hypothesis.
- **Decision inputs and roles.** The Decision Engine consumes modeled-world
  facts only from World State. It also uses approved Knowledge Engine
  rules and Task Evaluator outputs as required by ARCH-006. The Knowledge
  Engine owns domain knowledge, constraints, and approved rules. The
  provisional Task Evaluator assesses task state and outcomes. Decision
  responsibility stays with the Decision Engine.
- **Verification and safety.** Preserve deterministic, reproducible cycles
  and Challenge Before Acceptance. When the available evidence is
  insufficient, acquire additional information or safely refuse the
  interaction. Independent collision avoidance and safety behavior remain
  in place regardless of semantic association or certainty.

[Architecture Rules](docs/architecture/SIE_RULES.md) ·
[Engineering Principles](docs/architecture/SIE_ENGINEERING_PRINCIPLES.md) ·
[Spatial Model (ES-005)](docs/architecture/SIE_SPATIAL_MODEL.md) ·
[Evidence Graph (ES-006)](docs/architecture/SIE_EVIDENCE_GRAPH.md)

### Surface-Level Association and Entity-Level Association

**Surface-Level Association** tests whether observations or measurements
correspond to the same physical surface. It depends on applicable spatial,
temporal, and evidence checks. **Entity-Level Association** tests whether
source-bound evidence concerns the same entity. It can connect observations
of different parts at different coordinates when the evidence supports
common identity. A candidate at either level remains provisional until
its claim is verifiable.

### Practical MVP interpretation

- **LD19** provides spatial geometry, obstacle observations, and mapping
  inputs. An obstacle may require further semantic investigation before
  choosing an interaction. Any independent collision-avoidance response
  continues to apply without waiting for semantic identification.
- **AR0234** provides semantic observations and target tracking.
- **OV9281** provides depth and spatial structure.
- These observations can complement each other. For example, LD19 may
  observe a person's legs while camera evidence concerns the torso. The
  coordinates can differ and still support an entity-level association
  when common identity is established by evidence.

**FACT / implementation status.** Step 70 freezes the reported local
Stereo–LiDAR diagnostic results. Step 71 implements a three-sensor
semantic spatial diagnostic and has synthetic verification. These
checkpoints do not establish physical target membership, validate a
camera–LiDAR extrinsic, or admit Measurement, World State updates, or
motor control. The principles above govern future architecture beyond
these experiments. [Step 70 evidence](docs/diagnostics/stereo_lidar_local_correspondence_v1.md#step-70-freeze-2026-11-26) ·
[Step 71 handoff](docs/handoffs/SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md#step-71--three-sensor-semantic-spatial-diagnostic)

## Knowledge Architecture Principles

**DECISION — `OWNER_APPROVED`, 2026-11-26.** This freeze applies across
the SIE architecture and its future development, including the current MVP.

> **Knowledge in SIE is structured, justified, and applicable information about the physical world, its entities, properties, relationships, regularities, and interaction possibilities, enabling the system to interpret observations, understand situations, and make decisions.**

Knowledge may originate from sensor observations, verified facts, approved
domain knowledge, instructions, engineering specifications, and other
authorized information sources. Knowledge supports interpretation and
decision-making. Evidence remains necessary to establish claims about
the physical world.

### Conceptual definitions

| Concept | Meaning | Question |
| --- | --- | --- |
| **Observation** | Information obtained from a specific source at a specific time. It remains source-bound and retains its provenance. | What do I observe? |
| **Measurement** | A quantitative characterization of an observed property obtained through an identified method. It retains independent provenance, units, explicit reference frames, calibration identity, and applicable quality information. | How much? Where? How far? |
| **Fact** | A statement about the physical world accepted by the system on the basis of sufficient evidence. It remains subject to validity, confidence, and temporal applicability, and can be revised as new evidence becomes available. | What has been established? |
| **Knowledge** | Structured and justified information about entities, properties, relationships, regularities, and interaction possibilities. | What does this mean? |
| **Intelligence** | The ability to interpret available knowledge, assess situations, identify missing knowledge, and make justified decisions about interactions with the physical world. | What should be done, and why? |

These are conceptual distinctions. Existing Data Contracts and the
canonical evidence chain govern engineering objects and their exchange.
The list establishes no mandatory linear processing pipeline.

### Categories of knowledge

| Category | Scope and architectural boundary |
| --- | --- |
| **Situational Knowledge** | The current physical-world situation, including entities, positions, states, relationships, events, and environmental conditions. World State represents accepted modeled-world facts. |
| **Domain Knowledge** | Types of entities, their properties, relationships, regularities, constraints, and interaction possibilities. The Knowledge Engine owns approved domain knowledge and rules. |
| **Procedural Knowledge** | How actions, tasks, and interactions can be performed, including procedures, instructions, applicable methods, and action prerequisites. Existing responsibilities of the Knowledge Engine, Task Evaluator, Decision Engine, and executors apply. |
| **Self-Knowledge** | The system's own capabilities, available sensors, physical characteristics, operational constraints, and technical state. Approved capability descriptions and current operational facts remain distinct. |

These categories describe kinds of knowledge. Existing ownership and
storage boundaries govern their representation. The categories do not
assign all knowledge to a single module.

### Knowledge and evidence

Knowledge enables SIE to interpret independent observations, identify
potential relationships between entities, form explanatory and testable
hypotheses, evaluate the applicability of instructions, understand
possible interactions, and recognize insufficient information.

Domain knowledge can suggest a candidate association. Applicable evidence
is required to establish a fact about a specific observed entity. For
example, knowing that a person has legs and a torso does not establish
that particular LiDAR reflections and a stereo torso observation concern
the same person. Keep hypotheses, candidate associations, accepted facts,
and approved rules distinct. Confidence must remain evidence-backed.

### Knowledge sufficiency and acquisition

> An intelligent system must be capable of identifying when its available knowledge is insufficient for a proposed task or interaction, determining what additional information is required, and selecting appropriate means to obtain that information.

The conceptual cycle is:

1. Observe.
2. Interpret observations using applicable knowledge.
3. Assess knowledge sufficiency relative to the current task.
4. If sufficient, select an authorized action.
5. If insufficient, identify the knowledge gap.
6. Acquire additional information.
7. Reassess the situation.
8. Observe and evaluate the result of any executed action.

Additional information can come from re-observation, another sensor, a
safe change of observation position, approved knowledge sources, or
clarification from a human. Information acquisition actions remain
subject to authorization, capabilities, and independent safety
constraints. If the necessary information cannot be obtained, select a
safe alternative or refuse the interaction.

**OPEN / implementation status.** The acquisition principle defines
approved architectural behavior. Knowledge Engine and Active Perception
mechanisms are outside this documentation freeze.

### Consistency with approved architecture

The [Fundamental Architecture Principles](#fundamental-architecture-principles)
continue to apply. Deterministic and reproducible cycles, Challenge Before
Acceptance, and independent collision avoidance and safety behavior
remain in force. Collision avoidance and safety behavior do not depend
on successful semantic understanding.

Entity lifecycle, the Evidence Graph, and confidence requirements remain
in force. The World State Manager is the sole writer of World State and
accepts only verifiable Facts. The Decision Engine receives modeled-world
facts only from World State and continues to use Knowledge Engine rules
and Task Evaluator outputs under ARCH-006. Engineering knowledge remains
separate from perception under ARCH-010. The Task Evaluator assesses
tasks and outcomes. The Decision Engine retains decision responsibility.

[Architecture Rules](docs/architecture/SIE_RULES.md) ·
[Engineering Principles](docs/architecture/SIE_ENGINEERING_PRINCIPLES.md) ·
[Data Model](docs/architecture/SIE_DATA_MODEL.md) ·
[Spatial Model](docs/architecture/SIE_SPATIAL_MODEL.md) ·
[Evidence Graph](docs/architecture/SIE_EVIDENCE_GRAPH.md)

## Current product direction

The long-term product is a construction-object spatial intelligence system: it maintains an evidence-backed spatial state of the project and coordinates data and authorized actions for admitted robotic complexes, machines, and mechanisms. A mobile base that follows a semantic target is one future executor and validation scenario, not the definition or boundary of SIE. **Canonical dynamic fusion diagnostics exclude mobile-base navigation and motor control.** A separate supervised bounded trial is recorded in the current checkpoint; it grants no autonomous or canonical motion admission. No output of the current dynamic fusion diagnostic may authorize motion.

The intended, future operating model is:

```text
BIM / project / schedule
+ surveying / inspections / sensors
+ material, zone, and quality state
+ reports from admitted robotic complexes, machines, and mechanisms
  -> evidence-backed SIE World State
  -> Task Evaluator and Knowledge Engine
  -> Decision Engine
  -> authorized action for a specific executor
  -> evidence-backed re-observation and state update
```

The current RGB-to-stereo person experiment is one Vision Core validation path within this broader system. The target may be a person today and another object later. The geometry and decision layers must not be designed around `person_upper_body`.

### Current RGB + stereo architecture

| Responsibility | Current implementation | Architectural rule |
| --- | --- | --- |
| Semantic target | AR0234 RGB; current 640x400 preview uses one COCO YOLO11n-seg inference for person bbox and mask; custom `person_upper_body` ONNX remains the reference backend | AR0234 is the sole semantic authority in the new fusion path. The model label is a temporary adapter, not a system-wide object definition. |
| 2D target region | Current-frame segmentation bbox and exact-index instance mask in the diagnostic preview; bbox-only reference backend remains available | A bbox is an area of uncertainty, not a depth point. Do not synthesize a mask from a bbox. |
| Geometry and depth | DECXIN dual OV9281 stereo, rectification, disparity, calibrated 3D reconstruction | Stereo provides geometry, not a second mandatory semantic person detector. |
| Cross-camera association | Validated AR0234-to-OV9281 physical-left extrinsic transform | Project the RGB target region through calibrated geometry into an admissible stereo frustum; do not measure the bbox centre. |
| Quality | LR consistency, target-mask ownership, depth layers, global MAD comparison and local shape diagnostics in the preview; connected-component gates remain in the reference audit | Refuse a result when more than one plausible 3D surface, poor disparity, or inadequate evidence remains. |

The current dynamic diagnostic node intentionally does **not** require MediaPipe on OV9281. Earlier live metric code still contains an OV9281 MediaPipe detector; it is legacy static-MVP behavior and must not become a requirement for the intended RGB-semantic + stereo-geometry architecture.

### Required evidence for every fusion diagnostic cycle

A diagnostic result is useful only when it can be audited. Each cycle must either retain the complete evidence set named in the approved next sequence or explicitly identify which source was unavailable. Its outcome must be one of: `ASSOCIATED_DIAGNOSTIC_ONLY`, `TARGET_UNAVAILABLE`, `ASSOCIATION_UNAVAILABLE`, or a more specific evidence-backed rejection. Diagnostic evidence is not a Metric Measurement and must never be silently promoted to World State, navigation, or an actuator command.

## Historical continuation checkpoint: 2026-10-14

This checkpoint supersedes older next-step descriptions about repeating manual
board sweeps, diagnosing the same six saved MAD refusals, or using the old
range-gated shadow path as the only implementation. Preserve those observations
as history; do not repeat them without a new concrete question.

### Implemented diagnostic path

- Branch `codex/sie-ros2-integration-v1`, implementation commit
  `45007ad5d9850d1126a52903db75a3d3bcb8d714`.
- `ar0234_ov9281_control_preview_node.py`, dedicated launcher
  `vision_core/tools/run_control_input_preview_640x400.sh`, topic
  `/sie/diagnostics/ar0234_ov9281/control_preview`. Full runbook:
  [control_input_preview.md](docs/diagnostics/control_input_preview.md).
- Actual stereo capture/calibration remain 2560x800 combined, physical RIGHT
  first and LEFT second. Rectified matching alone is reduced to 640x400 per
  camera, with independent SGBM64 matchers and LR tolerance 0.75 small pixels.
  Scale, focal-baseline units and pixel-centre projection are explicit. No
  upsampled/fill depth, hidden correction or second full-resolution pass.
- One COCO person segmentation inference on each current AR frame provides bbox
  and mask. Stereo runs concurrently. The node selects a current target, then
  fuses the exact model-result mask index, bound to cycle/RGB SHA/model SHA.
  Ambiguity, confidence, edges, geometry and fresh-track matching remain checked;
  no prior depth or mask substitutes for current observations.
- COCO `person` is an internal diagnostic schema, not relabelled as the frozen
  custom `person_upper_body` Observation. No canonical raw observation is
  published by this backend. The old ONNX observer is not called per frame;
  parent startup still checks its artifact before releasing the unused session.
- The legacy full-resolution component association is marked `NOT_EVALUATED`
  in half mode. Nested mask-depth status and local shape diagnostics are separate.
  Shape diagnostics being available does not imply quality admission. Local
  residuals describe agreement with a shape model, not physical accuracy.
- Diagnostic range 0.5–4.5 m is separate from the original active 0.5–2.0 m
  Measurement profile. Calibration/activation/operating authority are unchanged.
- Compact journals are bounded/background. Temporary arrays are removed, so these
  journals are not replayable full image evidence. Use older frozen captures for
  offline model comparisons. Shutdown bug fixed in commit `52086d1`; final
  shutdown logging does not use an invalid ROS publisher.

### Evidence and measured performance

The saved resolution challenge
`stereo_resolution_comparison_20261014_224621.json` processed 22 cycles per scale,
with source hashes unchanged. Median stereo+LR: 851.51 ms at 1280x800,
73.49 ms at 640x400, 12.69 ms at 320x200. Half mask median differences from the
full JPEG-based reference were -21.28 to +10.51 mm; all 22 half mask diagnostics
passed the existing 50 mm global-MAD comparison. This is numerical agreement
on one scene, not physical-depth certification. Whole-frame common-point depth
delta p90 has median 257 mm; the responsible regions have not been isolated.
Quarter resolution shifted mask medians by a median +64.92 mm and is not exposed
in the live mode. Do not generalize person-median agreement to all scene points.

Pi live comparison, post-first completed cycles:

| Path | Completed cycles | Host median | Host max | Mask-depth coherent |
| --- | ---: | ---: | ---: | ---: |
| 640x400, detector plus segmentation | 17 | 1.2852 s | 1.3324 s | 17/17 |
| 640x400, single segmentation | 25 | 0.8674 s | 0.9060 s | 25/25 |

Latest journal:
`/home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_20261014_231200`.
There are 26 records: 25 completed and one `CANCELLED_SHUTDOWN`; warm latency
statistics use 24 completed post-first cycles. Zero journal drops/errors.

| Latest stage | Median | Meaning |
| --- | ---: | --- |
| Separate RGB detector | 0 s | No per-frame call |
| Segmentation round trip | 0.6318 s | Includes one model inference and request/response |
| Segmentation inference | 0.5925 s | Included in that round trip |
| Stereo worker | 0.2116 s | Overlaps segmentation; includes rectify/resize/matching/LR |
| Local surfaces | 0.1933 s | Included in subsequent fusion round trip |
| Subsequent fusion round trip | 0.2024 s | No second model inference |

Host latency improved about 32.5% relative to the preceding two-model live run.
No completed timed cycle meets 250 ms (0/24). Segmentation alone exceeds the
whole target budget. Candidate median depth 2.71265 m, cycle-median span
12.1 mm; neither physical accuracy nor exposure freshness follows from that
span. Global MAD 50 mm remains a team-selected comparison parameter, not a
camera specification or physical error bound. No local-residual cutoff has been
invented to promote these results.

### Working motion targets and next step

Operator-selected implementation targets: base speed up to 0.15 m/s; stand-off
at least 1.0 m from robot front to target surface; distance error <=100 mm in
1–2 m and <=5% above 2 m within a validated range; exposure-to-command age
<=250 ms; stop travel <=25 mm and stop time <=0.5 s. These are targets, not
proven abilities. A proposed extra 100 mm reserve is not a measured value.
See [requirements draft](docs/diagnostics/vision_motion_requirements_draft_v1.md).

Next: export the SAME segmentation model to ONNX in an isolated environment,
then compare PyTorch/ONNX on already saved AR frames for inference/round-trip
time, detection/selection changes, bbox agreement and mask overlap. Preserve
model/export hashes and settings, input hashes and failures; do not change
thresholds to obtain agreement. No new person/distance run is needed for this
comparison. Do not switch the live backend until results are reviewed.

Retain half-resolution stereo. No need to repeat a checkerboard or multi-range
series merely to tune MAD. Every proposed restriction/change must name the
measured deficiency or explicitly unknown prerequisite it addresses and what
removes it. Each next action should advance camera-derived robot operation.

The current output remains diagnostic: no admitted robot-front clearance/error
bound, established exposure age, validated generic dynamic-scene performance,
or qualified 0.15 m/s drive/stopping interface is claimed. The existing bounded
ESP32 distance command is not a velocity setpoint. `Measurement`, navigation
and motor commands remain disabled. Last implementation verification: 27 targeted
tests, changed-file py_compile and launcher bash -n passed; this live run adds
real Pi evidence but does not validate physical accuracy or dynamic safety.

## Hardware and runtime

| Item | Current fact |
| --- | --- |
| Compute | Raspberry Pi 5 `piecake`, Ubuntu Server 24.04.5 LTS ARM64, kernel `6.8.0-1065-raspi`, headless over SSH |
| Middleware | ROS 2 Jazzy, normally `ROS_DOMAIN_ID=42` |
| RGB camera | AR0234 UVC, 1920x1200, runtime auto exposure must be `3` |
| Stereo camera | DECXIN dual OV9281 UVC, combined 2560x800 at 60 FPS, MJPG, auto exposure `3` |
| Stereo half mapping | combined first half is physical **RIGHT**; combined second half is physical **LEFT** |
| Stereo physical baseline | 65.1 mm |
| Current camera-block pose | The rigid AR0234 + OV9281 assembly is pitched approximately 7° upward relative to the floor for the controlled semantic tests recorded on 2026-10-05. Its inter-camera mount was not changed. |
| Motor capability | ESP32 bounded-forward API exists; its use is separately supervised and is not enabled by diagnostic fusion |

Do not independently detach, loosen, rotate, bend, or otherwise change AR0234-to-OV9281 mechanical geometry without a new calibration plan. A rigid-body adjustment of the complete two-camera block preserves the AR0234-to-OV9281 extrinsic, but changes the block pose relative to the robot base/floor and must be recorded. It is not, by itself, a validation of a base/world-frame measurement configuration.

## Calibration and proven measurement envelope

The following artifacts are bound into the current ROS2 work. Their SHA-256 values must be verified from the actual files before use:

| Artifact | Identifier / SHA-256 |
| --- | --- |
| AR0234 intrinsic | `ar0234_intrinsic_v5_final_20260910`, `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` |
| OV9281 stereo calibration | **active v7** at `/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz`, SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`; its far-range metric accuracy is under re-validation |
| OV9281 v8 candidate | `ov9281_pi_stereo_candidate_v8`, SHA-256 `6b037ec3a79bd6d340875586353ea0554cc78f5332db8b4929462e80e944d956`; **rejected**, not active, after comparative physical-depth checks |
| OV9281 v8b candidate | `ov9281_pi_stereo_candidate_v8b`, SHA-256 `6691a86dd8daaa84d87cf5c124ba4309471ff92416f200bf4799843299dcf76d`; **rejected**, not active, after same-setup 1.5 m comparative check |
| AR0234 -> OV9281 physical-left extrinsic | `ar0234_ov9281_physical_left_extrinsic_v1`, `10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc` |
| RGB + stereo decision profile | `runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json` |

The 0.5 to 4.5 m v2 profile remains a conditional supervised diagnostic/recommendation constraint. It is not current evidence that v7 has physically validated metric accuracy across that whole interval, and it is not an authorization for dynamic fusion, autonomous navigation, or motors.

The V6 handoff and V6 runtime policy describe an earlier, separate calibration
chain. They must not be substituted for the SHA-bound active OV9281 v7 artifact
in this RGB + stereo diagnostic. V7 is the calibration version here; the
`run_v7_*` runner names also identify supervised experiment scripts and do not
by themselves select a calibration.

The current Pi copy of the 0.5 to 4.5 m v2 profile has SHA-256 `c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9` and status `ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY`. A diagnostic run outside 2.0 m must not silently fall back to the frozen original profile `ar0234_ov9281_decision_chain_v1`, whose authorized range is only 0.5 to 2.0 m.

### Static planar-depth evidence and datum limitation

The following checks compare the validator's `z` in `rectified_left_optical_frame` with laser distance from the **front edge of the physical-left OV9281 lens to the chessboard centre**. They are valuable screening evidence, but they are not yet a strict metric-depth certification: the laser datum is not the left optical centre, the rigid camera block is pitched upward, and the board was not mechanically constrained perpendicular to the left optical axis. A `z` coordinate, a slant range, and the median depth of a tilted board's corners are different quantities.

| Reference distance, m | Median depth, m | Absolute error, m | Per-run depth MAD, m | Evidence SHA-256 |
| ---: | ---: | ---: | ---: | --- |
| 1.500 | 1.497994 | 0.002006 | 0.007302 | `8a8344ab188457ba57d1ef6ac337eb125f5d9e7f7ee3f1ea1cae9968870821d4` |
| 2.000 | 2.052748 | 0.052748 | 0.012017 | `549b6bd5890d711524cb03e33a4f654de4c58b7c07715f62d64798bd3e0c2c1b` |
| 2.500 | 2.606853 | 0.106853 | 0.011260 | `19814eafe433178eee63acc3143b28e82ec361f6364db1565ac070a137de5c96` |
| 3.065, small-board repeat after 180-frame warm-up | 3.250298 | 0.185298 | 0.028556 | `8eb6de8ebd4973f9a2a1153b622428063cc443f51e3ea0770ed73dc8caa44480` |
| 3.065, new matte 48 mm board after 180-frame warm-up | 3.230612 | 0.165612 | 0.020497 | `e43525e697702d463887aa5825230d6243e4ca31ed0d6438da0ee55646aad64f` |

The 3.065 m matte-board run is important: replacing the small/glossy target reduced the spread and reduced the apparent positive difference only slightly. This excludes neither datum/pose error nor a calibration problem. It is **not proof of a physical cause** and must not be used alone to certify or reject v7 at a given range.

With `P2[0,3] = -75954.18816496264`, the implied disparity departure from the reference increased with distance: +0.068 px at 1.5 m, -0.976 px at 2.0 m, -1.245 px at 2.5 m, and -1.375 px at 3.065 m. This calculation inherits the same datum/pose limitation. Do not apply a scale or offset correction: it is a diagnostic signal, not a runtime patch.

### 2026-10-07 matte-board screening series

With the same 48 mm matte 9x6-inner-corner board, active v7, and 30 frames per run, the operator collected the following screening series. The 2.5 m first repeat that returned 2.046 m was taken before the board was correctly repositioned and is excluded; the output path was then overwritten by the valid run.

| Laser reference, m | Median `z`, m | Difference `z - reference`, m | Depth MAD, m | Run |
| ---: | ---: | ---: | ---: | --- |
| 1.500 | 1.500170 | +0.000170 | 0.014973 | current same-setup v7 control |
| 2.000 | 2.032131 | +0.032131 | 0.012147 | run01 |
| 2.000 | 2.048996 | +0.048996 | 0.015191 | run02 |
| 2.500 | 2.550983 | +0.050983 | 0.019792 | run01 |
| 2.500 | 2.558118 | +0.058118 | 0.020823 | run02 |
| 3.000 | 3.064197 | +0.064197 | 0.022729 | run01 |
| 3.000 | 3.079035 | +0.079035 | 0.026277 | run02 |
| 3.500 | 3.631426 | +0.131426 | 0.035391 | run01 |
| 4.000 | 4.268161 | +0.268161 | 0.046942 | run01 |

The series shows a repeatable, distance-growing difference between current v7 `z` and the stated laser procedure. Its frame-to-frame spread remains much smaller than the growing mean difference through 3.0 m. This is enough to motivate a pose-aware validation; it is **not enough** to declare a 2%, 3%, or 5% physical-depth pass/fail at those ranges. In particular, a camera pitch or an off-axis board normally makes optical-axis `z` less than slant range, whereas the observed difference is mostly positive; any eventual explanation must model all reference points and board pose rather than assume that one effect explains all values.

The earlier low-board and chest-board observations remain separate semantic/ROI evidence. In particular, the chest-board run where AR0234 selected a curtain is semantic-target-selection failure, not a v7 calibration diagnosis. All results still require separation of semantic selection, RGB-to-stereo association, stereo observability, and metric calibration.

### Rejected OV9281 v8 candidate (2026-10-06)

The first v8 capture combined 144 raw 48 mm matte-board pairs. Its initial audit passed the then-current centre-span criteria: horizontal centre span about 789 px, vertical span about 205 px, board-area range 5.70×, and strong sharpness. The independent holdout geometry was also superficially strong: stereo RMS 0.3396 px, physical-baseline difference 0.0683 mm, median holdout reprojection RMS 0.3580 px, and median rectified vertical residual 0.0987 px.

That solver result is **not** sufficient for acceptance. Direct physical checks, both with 180-frame warm-up, rejected the candidate:

| Candidate and reference | Distance datum, m | Median depth, m | Absolute cross-frame difference, m | Depth MAD, m |
| --- | ---: | ---: | ---: | ---: |
| v8, matte 48 mm board | 3.065 | 2.926988 | 0.138012 | 0.012358 |
| v8, matte 48 mm board | 1.500 | 1.428038 | 0.071962 | 0.010431 |
| active v7, same matte board and same 1.500 m setup | 1.500 | 1.524911 | 0.024911 | 0.015382 |

The same-setup 1.5 m A/B comparison rules out the new matte board or simple datum placement as a sufficient explanation for v8's 72 mm underestimate. v8 must remain a rejected evidence artifact; do not activate, copy, or correct it with a scale/offset.

One data-backed explanation to test, not an established root cause: the first v8 calibration set had no board centres low enough in the image. The physical-left and physical-right centre ranges were only Y=175.9..381.1 px and Y=163.5..368.5 px in 800 px-high images. v8's rectified `P1/P2` Y centre became about 421.8 px, versus about 372.2 px in v7, while its depth constant fell from 75,954.2 to 62,926.8 mm·px. These values motivate the controlled lower-field recapture; they do not by themselves identify the physical cause.

For v8b the audit now requires a board centre near the upper field and a board centre at or below the lower-field condition (`y_min <= 250 px`, `y_max >= 550 px`) in both physical images. A third 72-pair lower-field session improved combined vertical span to about 355 px and board-area diversity to 11.49×, but the combined 216-pair audit correctly rejected it: maximum centres were only 531.6 px (physical-left) and 518.1 px (physical-right). The next capture tool accepts an explicit Y band and prints the accepted centre; it is prepared for a fourth lower-field session.

### Rejected OV9281 v8b candidate (2026-10-07)

The gated fourth lower-field session completed, so the combined v8b capture contains 288 pairs. Its strengthened audit passed: physical-left board-centre coverage was X=257.6..1125.9 px and Y=175.9..678.5 px; physical-right was X=169.3..1048.1 px and Y=163.5..664.0 px. The board-area ratio was 11.49× and the audit report SHA-256 is `a0489583d16f2ac81eab16aa1debfc2acb1b409a7863148e34773b7f7e1ba2a0`.

The non-active v8b solver used 216 train and 72 holdout pairs. It reported stereo RMS 0.378096 px, baseline 64.842164 mm versus physical 65.1 mm, holdout reprojection median/p95 0.416312/0.667915 px, and holdout rectified vertical residual median/p95 0.131134/0.298921 px. The candidate report SHA-256 is `9bba447084015a090dd19cead8fca70c86ae4126cdaa38abc29a528af7c30d77`.

Those geometry statistics again did not predict field metric performance. With 180 warm-up frames and the same matte board at stated 1.5 m, v8b produced median `z=1.441585` m, a 58.415 mm difference from the stated laser datum. An immediate active-v7 control in the same setup produced `z=1.500170` m, a 0.170 mm difference. This is sufficient comparative evidence to reject v8b as a replacement for active v7. It must not be activated, copied into runtime artifacts, or corrected by an empirical scale/offset.

## What is implemented and tested

### ROS 2 contracts and static supervised chain

- The ROS 2 contract chain has been exercised on Pi: perception contract, navigation contract, and supervisor contract.
- AR0234 observation publishes structured observations with model SHA, bbox, confidence, truncation, and target status.
- AR0234 plus OV9281 metric Measurement has produced real live range/bearing records in the 0.5 to 4.5 m conditional profile.
- Navigation and supervisor can produce recommendations and blocked states.
- The normal pipeline keeps the actuator bridge disabled.

### Existing bounded motion capability

The ESP32 interface exposes `bounded_forward_0.10_m`. Its supervised profile requires an explicit `SUPERVISED_PERSON_APPROACH_SESSION`, short evidence freshness, ESP32 READY state, no fault, active brake, no retry, and mandatory re-observation. Earlier physical checks showed about 9.38 cm travel for a requested 0.10 m step.

This capability is a tested actuator primitive. It is **not** permission to run it from the current dynamic fusion diagnostic.

### Current dynamic fusion diagnostic

Implementation:

```text
ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py
topic: /sie/diagnostics/ar0234_ov9281/dynamic_association
```

The node:

- acquires AR0234 and OV9281 frames;
- obtains the semantic target from AR0234 YOLO only;
- uses the calibrated AR0234-to-physical-left transform to build a stereo frustum;
- applies left-right disparity consistency;
- keeps only a sufficiently supported connected spatial component;
- evaluates depth clusters inside that component without trimming the full cluster to hide spread;
- reports `ASSOCIATED_DIAGNOSTIC_ONLY`, `ASSOCIATION_UNAVAILABLE`, or `TARGET_UNAVAILABLE`;
- includes fail-closed temporal association logic, but a trustworthy live `TEMPORAL_CURRENT_FRAME_MATCH` has not yet been established as a validated dynamic capability.

It never publishes a Metric Measurement, navigation decision, or motor command. `metric_measurement_authorized` remains `false`.

Earlier 42-cycle diagnostic evidence:

| Outcome | Count |
| --- | ---: |
| `ASSOCIATED_DIAGNOSTIC_ONLY` | 5 |
| `ASSOCIATION_UNAVAILABLE` | 17 |
| `TARGET_UNAVAILABLE` | 20 |

The five associations had large coherent left-right-consistent spatial components, but many other frames were refused because full selected depth-cluster MAD exceeded the current 0.05 m limit. The correct response is to investigate target surface selection and evidence quality, not silently relax MAD, calibration, or range limits.

The later dynamic diagnostic also has an opt-in shadow instance-mask worker. It records a YOLO11n-seg person mask from the same AR0234 frame and a separate masked-depth hypothesis without changing the original association decision, publishing a Metric Measurement, or accessing motors. Its `0.05 m` MAD cutoff is a team-selected software threshold for this diagnostic, not a camera specification, measured absolute error bound, or approved motion tolerance. That older shadow implementation computes live masked depth only after the original frustum range gate. The separate 2026-10-14 control-preview path below removes this diagnostic dependency; it does not expand the activated Measurement range.

### Controlled AR0234 framing and stationary semantic range gate (2026-10-05)

The earlier horizontal-camera 3.0 m test exposed a framing problem rather than a model or stereo-calibration conclusion: the AR0234 at 0.43 m above the floor was parallel to the floor, so the standing person's head and upper body were clipped by the top of the source image. The full dynamic recorder produced only one semantic target in 58 cycles even though the person remained in view.

The complete AR0234 + OV9281 rigid assembly was then pitched approximately **7° upward**, with no relative adjustment between the cameras. This is the configuration for the following narrow controlled test set:

- one stationary standing person;
- artificial lighting and a mostly uniform wall background;
- AR0234 semantic node at a 0.4 confidence threshold and 2 Hz requested rate;
- no metric Measurement, navigation, or motion authorization.

| Distance | Semantic availability | Median confidence | Interpretation |
| --- | --- | ---: | --- |
| 2.0 m | 67 / 84 `SINGLE_TARGET` (79.8%) | 0.6246 | Not a confirmed stable semantic distance. |
| 2.5 m | 86 / 86 (100%) | 0.9031 | Stable in this controlled scenario. |
| 3.0 m | 14 / 14 (100%) in full dynamic evidence cycles | 0.9223 | Semantic gate stable; stereo association still refused. |
| 3.5 m | 97 / 97 (100%) | 0.8455 | Stable semantic availability; lower-image bbox contact occurred in some frames. |
| 4.0 m | 87 / 87 (100%) | 0.9380 | Stable in this controlled scenario. |
| 4.5 m | 88 / 88 (100%) | 0.8799 | Semantic reserve only; outside the stated 4.2 m room-use extent. |

For the present room, the evidence-backed **controlled semantic reference interval is 2.5 to 4.2 m**. The 4.5 m result demonstrates reserve in this scene; it does not expand the operating room requirement. The 2.0 m result must remain outside the confirmed stable interval.

This is not a general person-detection or operational-range claim. It has not tested natural motion, varied illumination, non-uniform backgrounds, clothing variation, multiple people, occlusion, or the stability of a true 2 Hz end-to-end diagnostic recorder.

At 3.0 m after the pitch adjustment, the first full dynamic evidence recorder retained semantic targets in all 14 cycles, with approximately 7.1k to 7.4k left-right-consistent points. Every cycle refused association as `ASSOCIATION_UNAVAILABLE` with `FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE` and only 0 to 17 in-range depth samples. This was later traced to an activation-profile selection error: the run explicitly loaded the original 0.5 to 2.0 m profile, not a lack of stereo depth at 3.0 m. It must not be cited as evidence of inadequate human-surface stereo support.

The same 3.0 m full-evidence run took about 51.8 seconds for 14 cycles, roughly one stored cycle every 4 seconds. This is diagnostic evidence-recorder throughput, including capture, rectification, disparity, and evidence writes; it is not the AR0234 frame rate or a permitted runtime rate. Profile it separately before making a rate decision.

Evidence archives supplied by the operator:

| Archive | SHA-256 |
| --- | --- |
| `ar0234_3p0m_pitchup7_b_diagnostic_20261005.tar.gz` | `0d218ad8a149af67b05dc00d6c790876f3903de0babf4c068cf86c5092c0727c` |
| `ar0234_range_pitchup7_2p0_20261005.tar.gz` | `856fc424aa86e4e6b466273c2c1cf3780d07d63be194b3cd8a9144298d86b191` |
| `ar0234_range_pitchup7_2p5_20261005.tar.gz` | `5d7446f9ea03f641b75a8d3d74732c5864ff3f3914bc241a991eaf43a1a2e8b7` |
| `ar0234_range_pitchup7_3p5_4p0_20261005.tar.gz` | `3539279bcd371eaeaf2ab123ec68d1effe68a4d881e4084dd714ed3fe4f810c1` |
| `ar0234_range_pitchup7_4p5_20261005.tar.gz` | `c4eb2a340ddfdacc59e89186925b7b1a3e25f1143ca9e3501b3a4b3ec64c71f9` |
| `ar0234_depth_support_3p0_20261006.tar.gz` | `eb3c2b3d01dbbfec41b093fc02755054c095f1778aa6e9a738d7ac66f3a703be` |
| `ar0234_depth_support_3p0_profilev2_20261006.tar.gz` | `927a9846af77c3f4569d8d661dcfaa77ac9af22f936cd41a06cd0204cd2715d9` |

### 3.0 m human-surface audit with the 0.5 to 4.5 m v2 profile (2026-10-06)

The repeated stationary 3.0 m audit loaded the SHA-verified v2 extended-range profile. It retained complete evidence for 14 cycles over about 49.6 seconds. This test used the same restricted scene as the preceding semantic tests: one standing stationary person, artificial light, a mostly uniform wall, and the rigid camera block pitched about 7° upward.

| Check | Observed result |
| --- | --- |
| AR0234 semantic target | 14 / 14 cycles; confidence 0.874357 to 0.906889 |
| LR-consistent disparity support | 7,653 to 7,975 sampled points per cycle |
| Points inside the 0.5 to 4.5 m range | 7,437 to 7,845 per cycle |
| Points reprojected inside the AR0234 inner ROI | 3,413 to 3,569 per cycle |
| Spatial components | exactly one plausible connected component in every cycle |
| Depth clusters | exactly one plausible cluster in every cycle |
| `ASSOCIATED_DIAGNOSTIC_ONLY` | 8 / 14 cycles |
| `ASSOCIATION_UNAVAILABLE` | 6 / 14 cycles, all only because full-cluster depth MAD was 0.0538 to 0.0579 m, above the unchanged 0.050 m limit |

The eight associated cycles had diagnostic `z` values from 3.3024 to 3.3946 m, median 3.3245 m; their depth MAD was 0.0271 to 0.0481 m. The median projected point lay near the centre of the AR0234 torso region, and the associated bearing median was about -0.21°. This is evidence that the calibrated RGB-to-stereo path can locate a dense, connected 3D surface in the intended 2D person region under this controlled condition.

It is not a new Metric Measurement, range-accuracy validation, dynamic-fusion approval, navigation input, or actuator authorization. The label "3.0 m" is not yet an accuracy ground truth for this diagnostic `z`: the physical datum must be declared relative to the relevant camera optical frame before any difference is interpreted as bias. The six MAD refusals remain correct fail-closed outcomes. A possible transient at camera/exposure warm-up is a hypothesis only and must be tested without relaxing the MAD limit.

### Recent fusion-validation results

A later route-style audit produced **57 diagnostic cycles**. AR0234 produced a target in 12 cycles and stereo association succeeded in 10 of those 12. Representative associated depths included approximately:

- nominal 4 m -> about 4.220 m;
- nominal 3 m -> about 3.285 to 3.293 m;
- nominal 2 m -> about 2.259 m.

This run showed that the fusion chain can associate some targets over range, but it also made clear that **semantic target availability/selection was the dominant practical limiter in that run**. These measurements must not be treated as a new calibrated range profile because the route geometry, target surface, and ground-truth procedure were not equivalent to the controlled planar validation.

A later 26-cycle person test produced:

- 9 AR0234 detections with confidence about 0.91 to 0.94;
- roughly 7.4k to 8.6k left-right-consistent stereo points in detected frames;
- only 0 to 36 valid in-range depth samples in the projected target ROI in those frames;
- a later example with 8,683 LR-consistent points and 9,271 positive-disparity samples but only 56 in-range ROI samples, below the current minimum of 100.

The system correctly failed closed. This evidence is important because it shows that a globally healthy disparity field does not guarantee sufficient metric support on the projected human target surface.

### Static target-selection diagnosis

Controlled static-board runs isolated a semantic failure mode:

- `static_board_chest_1p5m` was not valid evidence for human-surface stereo quality because the detector frequently selected the curtain instead of the intended person/board target.
- The projected OV9281 ROI followed the incorrect AR0234 bbox, as it should under the current architecture.
- Therefore, the resulting stereo refusals must not be attributed to calibration or disparity until semantic selection is first proven correct on the source RGB frame.

The next controlled human test must first verify that the green AR0234 bbox continuously covers the actual person for the full observation interval before using ROI depth statistics to judge stereo support.

## Current engineering interpretation

The latest evidence requires explicit separation of two independent stereo-quality axes.

### 1. Geometric accuracy

Question:

> When a valid stereo correspondence exists, how accurately does calibrated geometry reconstruct metric depth?

The current v7 geometry agrees closely with the present 1.5 m same-setup control. The board series also shows a repeatable positive difference between v7 `z` and the current laser procedure from 2.0 m onward. Because that procedure does not yet place the board perpendicular to the optical axis or measure from the optical centre, it does not prove the physical-depth error of v7 at those ranges. It is nevertheless not sufficient to treat the v7 artifact as metrically proven across 0.5 to 4.5 m.

Relevant factors include intrinsic calibration, baseline, extrinsics, rectification, disparity precision, and physical calibration stability.

### 2. Stereo observability / target-surface support

Question:

> Does the intended target surface actually provide enough trustworthy stereo correspondence inside the projected ROI?

Relevant factors include surface texture, local contrast, illumination, exposure, motion blur, occlusion, repetitive patterns, matcher behavior, and semantic ROI placement.

The human-target evidence remains important, but it no longer justifies assuming that calibration accuracy is settled at 2.0 to 4.5 m. Good calibration cannot create disparity where the projected human surface has insufficient observable correspondence; conversely, good surface support cannot correct an unresolved range-dependent depth discrepancy.

This distinction is an engineering interpretation supported by the latest controlled tests. It does **not** yet approve a new measurement policy, a different matcher, active texture, another sensor modality, or online recalibration.

### External research note: calibration as runtime health

Recent external stereo-depth research reviewed during this work reinforced a useful hypothesis: high-precision stereo systems may need runtime calibration-health monitoring because heat, vibration, mechanical shifts, or impacts can degrade extrinsic alignment after nominal calibration.

For SIE, a future `Calibration Health Observation` is therefore a candidate concept, for example carrying epipolar residual, drift suspicion, confidence, timestamp, and state such as `VALID`, `SUSPECT`, or `BLOCKED`.

This is **not an accepted architecture rule yet**. Do not implement online recalibration or add a new canonical object solely from vendor literature. First demonstrate an actual runtime calibration-drift failure mode in SIE evidence.

## Current blockers and safety boundary

### Dynamic timing is not validated

AR0234 and OV9281 are currently captured sequentially by the host. Observed host pairing skew is about 11 to 12 ms. These timestamps are not exposure timestamps. Dynamic association can therefore be investigated only as diagnostic evidence.

Reliable dynamic engineering fusion requires proof of exposure synchronization. Repository history contains conflicting assumptions about external triggering support for this camera combination. Resolve that conflict from camera documentation and safe electrical evidence before wiring anything.

Until then:

- no dynamic Metric Measurement;
- no dynamic navigation authorization;
- no actuator command driven by dynamic fusion.

### Semantic target selection is not yet reliable enough

The chest-board diagnostic showed a high-confidence wrong-region failure mode: the RGB detector can select background structure such as a curtain, after which the calibrated projection correctly carries the wrong semantic target into stereo.

Therefore:

- no stereo-quality conclusion is valid for a cycle until the source AR0234 bbox is verified to cover the intended person;
- do not repair wrong RGB target selection by widening stereo ROI or relaxing stereo acceptance gates;
- confidence threshold changes must be justified by evidence. A higher threshold may suppress some false targets but is not a substitute for validating target identity.

The 2026-10-05 upward-pitch tests do establish continuous correct availability for one stationary person at 2.5 to 4.2 m in the stated artificial-light/plain-wall scenario. They do not resolve the general false-target risk above.

### Human-surface stereo support is not yet validated

The 2026-10-06 3.0 m v2-profile audit supersedes the earlier interpretation that the person ROI had too few in-range samples: that earlier result used the 0.5 to 2.0 m profile. With the correct 0.5 to 4.5 m profile, every cycle had thousands of in-range candidates, one connected reprojected component, and one depth cluster; eight cycles passed all current diagnostic gates.

Human-surface stereo support is still not generally validated. Six of the 14 controlled cycles failed the unchanged full-cluster MAD gate by a small margin, and the evidence does not yet distinguish camera/exposure settling, clothing/surface observability, or another source of spread. Motion, varied illumination, backgrounds, clothing, multiple people, occlusion, and exposure synchronization remain untested or unproven.

Do not attribute a failed human ROI automatically to calibration. However, the controlled planar range series independently challenges v7 at 2.0 to 3.065 m, so metric interpretation of the existing 2.5 to 4.2 m human-association diagnostics is provisional until a new candidate is validated.

### Electrical and calibration safety

- Do not connect ESP32 GPIO to camera TRG or STRB pins until pinout, signal levels, polarity, and common ground are verified with safe measurements.
- Do not tie STRB outputs together.
- No oscilloscope or logic analyser is available; a multimeter and then protected ESP32 event capture may be used only after electrical validation.
- Preserve the calibrated physical camera mount.

## OV9281 v7 relative and temporal validation (2026-10-09)

Active stereo calibration v7 remained SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`; no calibration, camera-control, Measurement, or motor policy was changed. The matte 9 x 6 inner-corner, 48 mm checkerboard was evaluated in `relative_axis_translation` mode. The front rim of the physical-left lens is a repeatable physical datum, but its offset to the optical centre is unknown. Thus the reported front-rim distance cannot establish absolute `rectified_left_optical_frame z`.

- Earlier manual 1.5 to 2.0 m axis translation: stereo delta 0.478089 m versus physical 0.500 m, relative error 4.382%, passing the *declared relative-translation* 5% screening criterion. A 1.5 to 4.0 m endpoint comparison likewise screened at about 3.41% relative error. Manual repositioning changes lateral alignment and board normal; these results neither certify all intermediate ranges nor prove 5% absolute depth accuracy. Further multi-point manual translations are deferred until a repeatable axial fixture or suitable independent metrology exists.
- In one continuously open-camera run, 10/10 frames passed each checkpoint at 0, 5, 15, and 30 minutes. Median stereo depths were respectively 1.490845, 1.466907, 1.468182, and 1.469608 m. Median disparities were 50.9471, 51.7785, 51.7335, and 51.6833 px. The 0-to-5-minute depth change was -23.94 mm while PnP board-centre z changed about -0.21 mm. Physical front-rim-to-board-plane distance was 1499 mm before and 1497 mm after the run (manual reading, declared 5 mm uncertainty). One normal-pose rejection occurred at the 5-minute checkpoint.
- After about 59 minutes disconnected from USB, a second continuously open-camera 0-to-5-minute run accepted 10/10 frames at each checkpoint without rejections. Median stereo depth went from 1.492973 to 1.461779 m (-31.19 mm), median disparity from 50.8745 to 51.9601 px (+1.0857 px), and PnP board-centre z from 1.515210 to 1.514748 m (-0.46 mm). The manually measured front-rim distance was 1497 mm before and after. Lateral board pose differed across sessions, so cross-session absolute depth differences are not attributed to camera drift.
- The within-run disparity and depth shifts greatly exceed the within-run PnP z changes and point to a stereo correspondence or relative-image-geometry effect under the existing intrinsics. PnP shares those intrinsics and is not an independent physical range measurement. It does help reject a board translation of roughly 24 to 31 mm as the sole explanation. Neither capture identifies a unique mechanism or supplies a correction coefficient.
- Historical v5 thermal evidence recorded the right module warmer than the left (ending near 37.125 versus 35.313 °C in a different experiment). The old DS18B20 sensors are currently removed. Temperature during these v7 runs was **not measured**; asymmetric heating is a hypothesis, not a demonstrated cause or a transferable v5 correction.

Source reports on the Pi: `~/.local/state/sie/evidence/ov9281_v7_relative_axis_20261008/point_1p5m_continuous_0_5_15_30min.json` and `point_1p5m_cool45_repeat_0_5min.json` in the same directory. The latter filename says `cool45`, while the operator's recorded USB-off interval was about 59 minutes. Retain the raw checkpoint and pose data with these reports. These diagnostic results do not admit stereo depth to operational Measurement.

## AR0234 instance-mask and OV9281 human-depth evidence (2026-10-11)

The immediate engineering goal remains reusable RGB semantics plus stereo geometry for general static and moving objects, with explicit target identity, evidence and refusals. The current COCO person mask is a **person-specific diagnostic backend**, not a universal object solution or an operational Measurement.

- In a saved 21-cycle person series, the original connected-component association succeeded in 5/21 cycles and refused 16/21 as multiple plausible spatial components. RGB projection showed torso and legs of one person could occupy separate 3D components. An offline SHA-bound YOLO11n-seg mask yielded one plausible depth layer in all 21 cycles, passed the unchanged 50 mm masked-depth MAD rule in 19/21 and refused two at 50.6 and 55.3 mm. A coherent layer alone does not prove absolute depth accuracy or semantic ownership in arbitrary scenes.
- The optional live shadow worker was then run on Pi for 25 cycles under `~/.local/state/sie/evidence/person_shadow_mask_20261011/6ac401b202729c7b`. It saved 22/22 masks for frames with an AR0234 person target; the other three cycles reported `NO_TARGET`. Median warm mask-inference time was approximately 0.498 s; the first frame took 2.551 s. Per-cycle status is recorded in `audit.json`, not printed to the terminal.
- All 22 original associations in that run refused `FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE` before the live masked-depth calculation. The activation used a **0.5 to 2.0 m** range while stereo depth was near 2.3 m: the first five frames had 8,150 to 8,551 LR-consistent samples, but only 13 to 52 remained in the active range. This is a configuration/applicability refusal, not evidence that the mask failed. Do not edit the activated profile to make the test pass.
- Offline replay of those frozen disparity maps and image-bound masks used an **explicit diagnostic-only 0.5 to 4.5 m range** and a whole-frame stereo grid because the live search ROI was constructed for the shorter active range. It produced coherent masked-depth diagnostics in 16/22 target frames, refused six with MAD 51.0 to 58.1 mm, and had no target in three. Median depth of the 16 passed frames was 2.290797 m, with a 0.073428 m span. Source: `~/.local/state/sie/evidence/person_shadow_mask_20261011/replay_0p5_to_4p5m.json`; analysis tool: `vision_core/tools/replay_shadow_mask_depth_range.py` (commit `b702755e1e4af78c99a0a295f4383566d9fe27d8`).
- The earlier 19/21 and newer 16/22 mask-pass rates are not controlled estimates of general reliability: the scene/subject may differ, the previous replay used a constrained ROI, and neither series provides independent ground truth of human-surface distance. Do not infer a 5% accuracy guarantee from the MAD or from these success rates. MAD describes dispersion inside selected depth samples, not bias from the real range.
- Upgrading the cameras, drive and base may reduce specific hardware failure modes, but it is not an accepted substitute for validated target association, range uncertainty, data freshness, braking distance and fail-closed behavior. Keep camera and actuator interfaces replaceable; evaluate professional hardware against the same evidence and contracts when available.

## Active development stage

SIE is in **evidence-driven AR0234-to-OV9281 target-association validation**, before validated dynamic Measurement or autonomous camera-driven motion. A separate operator-supervised bounded trial exists, as recorded in the current checkpoint above; it does not promote diagnostic camera output to canonical motion admission. Pose-aware v7 depth evidence and unresolved startup disparity shifts remain recorded above; avoid repeating manual board series without a specific new decision they can resolve. Both v8 and v8b are rejected as replacements despite strong solver metrics. Active v7 remains unchanged. A narrow stationary-person semantic gate is confirmed at 2.5 to 4.2 m for the current 7° upward rigid camera-block pose, but its metric depth values remain provisional because no optical-frame-consistent physical validation has yet established its bounds above 1.5 m.

The architecture is accepted: AR0234 provides semantics; OV9281 provides geometry. The active uncertainty has three practical parts, with reproducible startup disparity drift now an explicit part of the first:

1. what the active v7 physical-depth error envelope is when the board pose and ground-truth datum are expressed in the same optical frame, and how its startup drift is detected or bounded;
2. whether AR0234 continuously selects the intended person rather than a false background target; and
3. when it does, whether the projected OV9281 ROI contains a connected, temporally stable, metrically plausible stereo surface belonging to that person.

The central question is now:

> Can active v7 meet the stated depth-uncertainty envelope under optical-frame-consistent validation, and then does an AR0234-selected person ROI retain enough trustworthy connected stereo support to form a stable 3D target surface?

The immediate goal is to establish the validity domain and uncertainty of a 3D target measurement in real scenes, including naturally moving people. A plausible range or bearing is not an approved navigation input until this question is answered experimentally.

### Proposed staged MVP direction, not motion authorization

The intended MVP combines AR0234 semantic detection, OV9281 relative range, base odometry/velocity, and LiDAR map/localisation. The working operating hypothesis is:

- a terminal zone at or inside 1.5 m, where v7 has a strong present planar control and where final alignment remains slow, supervised, and repeatedly re-observed;
- an approach zone outside the terminal zone, where OV9281 may contribute only after a pose-aware test establishes an explicit error envelope, with an initial engineering goal of no more than 5% range error;
- LiDAR and the map constrain the robot's own pose and safe route; they do not by themselves correct an unmodelled stereo bias to a moving person or object;
- a state estimator may fuse valid measurements and their covariances, but a Kalman filter cannot manufacture a correct value from an unknown systematic camera bias. Stereo bias and uncertainty must therefore be part of the measurement model and must be gated.

This remains a proposed operational MVP. The later separate supervised trial does not change the diagnostic camera's motor-disabled boundary or approve autonomous motion. Operational bounded motion still requires a separate versioned Measurement policy, fresh physical ground truth, estimator validation, obstacle and stop behavior, and explicit authorization.

## Historical next sequence from the 2026-10-14 checkpoint

1. **Keep the demonstrated path.** Preserve v7 calibration and capture geometry,
   explicit 640x400 diagnostic matching and the single-segmentation frontend.
   Keep full-resolution/two-model paths as named references, not mandatory stages.
2. **Optimize the measured bottleneck offline.** Compare the same segmentation
   model in PyTorch and ONNX on saved AR frames; record timing, bbox/mask/selection
   agreement, hashes and refusals. Segmentation round trip is currently 632 ms.
3. **Review before switching runtime.** Preserve exact-index/cycle/RGB ownership,
   missing/ambiguous-target refusals and explicit backend provenance. Then use one
   short live check for the changed implementation, not another distance sweep.
4. **Close actual movement-input gaps.** Establish complete exposure age,
   camera-to-base/front geometry, physical depth error and fresh target ownership.
   Local shape residuals and an attractive median are not physical error bounds.
5. **Build the remaining control connection against working targets.** Reuse
   prior motor evidence; the ESP32 distance primitive does not implement a
   validated 0.15 m/s velocity setpoint or the 25 mm/0.5 s stopping bounds. Explain
   each constraint with measured deficiency or explicitly missing evidence.
6. **Preserve authorization boundaries.** Diagnostic previews cannot emit
   admitted Measurement, navigation or motor commands. Return to supervised
   camera-driven ground motion through an explicit reviewed control path.

## Rules for the next agent

- Start from the current task and this file; do not revive obsolete plans from filenames or old branches.
- Explain why a command, test, or code change is needed before asking the operator to run it.
- Use headless-safe evidence collection on Pi. Do not assume a GUI.
- Use `auto_exposure=3` for both AR0234 and OV9281 unless a new evidence-backed experiment explicitly changes it.
- Do not add a mandatory OV9281 semantic detector to the RGB + stereo architecture.
- Do not replace calibrated geometry, relax safety thresholds, widen the range, or enable motors merely because a diagnostic frame looks promising.
- Preserve raw evidence, calibration hashes, activation profiles, and reference-frame semantics.
- When diagnosing sparse person depth, first prove the RGB bbox is on the intended person, then separate stereo geometric accuracy from target-surface observability.

## Revision history

| Date | Change |
| --- | --- |
| 2026-11-26 | The owner froze SIE (Spatial Intelligence Engine, pronounced SEE) as the canonical project, architecture, and product name. SIEE remains the historical designation of the earlier experimental stage. |
| 2026-11-14 | Registered the read-only command `восстанови память SIEE` for cross-chat engineering-context recovery. The command reads current primary sources and gives a cited continuation summary; it does not update documents, regenerate Graphify, or run Pi/hardware. This change is documentation-only. |
| 2026-11-14 | Extended both equivalent operator commands (`обнови документ`, `обновить документ`) to update `PROJECT_CONTEXT.md` and `SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md` as one coordinated maintenance action. Both documents must match their respective original private Drive files; Graphify extraction remains manual. No new hardware, calibration, Measurement, or motion results claimed. |
| 2026-11-13 | Recorded the reviewed 64-cycle motor-off Fast Shadow run: 64/64 `FOLLOW_ELIGIBLE`, including 33 current-frame local-surface decisions after whole-mask MAD refusal, one uninterrupted track ID and sub-0.5 s driver-declared age. Recorded the exact conditional v2 launcher correction and removal of the unused YOLO v3 dependency from the mask-only path. Kept calibration, canonical Measurement and motor authorization unchanged; the corrected launcher still requires a new Pi verification and physical robot-front comparison. |
| 2026-11-12 | Recorded the 19:34:49 Pi fast mask-only source run: 9/9 verified bundles, 296 camera cycles below 500 ms driver age to Bridge decision, 13 coherent versus 30 MAD-refused mask depths, five track IDs broken within one person's presence, no post-exit reappearance or two-person test, and all decisions PAUSE. The prior all-mask/candidate front mismatch did not recur in this scene; active calibration, Measurement and motor boundaries unchanged. |
| 2026-11-12 | Removed mandatory ChatGPT Library synchronization at the operator's request. The repository and private Drive file are the two current copies; any Library copy is optional reading material and may be stale. Technical state and motor boundaries unchanged. |
| 2026-11-10 | Recorded the operator-reviewed fresh 120-frame two-person blind RGB check: v2/v3 presence 88/88 with 0/32 empty-frame detections, and at least two detections in 56/62 versus 60/62 two-person frames. Separated it from older 29-frame stereo material: 19 frames have two distinct box-depth medians, but ten lack disparity and none passes the 50 mm MAD gate for both boxes. Kept active v7, Measurement and Shadow motor boundaries unchanged; specified a fresh mask-owned depth and occlusion check. |
| 2026-10-31 | Recorded 14-cycle and 26-cycle Shadow depth limits, three reviewed 120-frame RGB subsets, the 230-image training pool and 120-image development validation, the 71/89 and 0/31 frozen ONNX baseline, preserved Colab v1 `best.pt`, and the prepared but unrun Colab v2 handoff. No calibration, Measurement, or motor activation changed. |
| 2026-10-29 | Added the current Shadow objective and boundaries, flashed candidate and lifted-wheel same-ID PAUSE/RESUME evidence, and 130/94/104-record dynamic camera preview findings. Kept active OV9281 v7 and unchanged MAD gate; documented missing person continuity, robot-front distance, exposure age, physical bound, and floor trial. |
| 2026-10-29 | Confirmed v1c hardware boot, idle UART STATUS 100/100 and wrong-boot rejection, then another UART 100/100 during an operator-confirmed 45 s camera-only preview (186 frames, no journal errors, all control decisions STOP). Recorded the local stale probe build-ID correction and motor-off command-path test as the next step; no motor or Measurement admission. |
| 2026-10-29 | Updated the document maintenance rule: `обнови документ` now authorizes synchronized updates of the Drive and ChatGPT Library copies; recreate the Library copy if absent. No engineering state was changed in this revision. |
| 2026-10-29 | Recorded v4 camera-loaded HTTP timeouts, the 30/30 and 100/100 motor-off UART link results, v1 boot assertion, unresolved v1b serial capture, and locally compiled but hardware-unverified v1c sketch. Updated the next step to boot and read-only UART checks with motors unpowered; retained active OV9281 v7 calibration and diagnostic-only movement scope. |
| 2026-10-28 | Recorded the user's storage rule: the current `PROJECT_CONTEXT.md` is kept locally on the laptop and as a synchronized private Google Drive copy in the SIE project folder. |
| 2026-10-28 | Clarified active OV9281 Pi v7 calibration path and SHA, historical V6 handoff, rejected v8/v8b and remaining v7 accuracy limits. Recorded the v3 0.20 m floor `SUCCESS` with leftward turn despite rightward camera bias, the normal-braking runner fix, and compiled but unflashed v4 wheel-difference controller at `d58aaec`. Superseded older next-step labels with one observed v4 floor trial after build/status verification. |
| 2026-10-26 | Updated the active handoff through `6578a71`: documented two read-only `/status` preflight timeouts with zero motion POSTs, the single 1 s then 2 s read retry, fresh-frame checks before any turn POST, and the unverified physical watchdog-v3 turn. The supervised trial remains separate from canonical Measurement and autonomous admission. |
| 2026-10-14 | Recorded shape-aware mask diagnostics, 22-cycle resolution replay, 640x400 live mode, one segmentation inference for bbox/mask and latest 25 completed Pi cycles (867 ms host median, 25/25 coherent mask-depth diagnostics). Adopted the operator-selected motion targets as implementation requirements, preserved disabled movement and unvalidated physical/exposure bounds, and prioritized offline ONNX segmentation comparison over repeated scene trials. |
| 2026-10-11 | Recorded the 21-cycle component/mask replay and 25-cycle live shadow capture, diagnosed the active 2.0 m range refusal, and added explicit 0.5 to 4.5 m offline masked-depth results (16/22 passed, six MAD refusals, three no target). Kept activation, Measurement and motors unchanged; prioritized saved-failure analysis and replaceable hardware interfaces before bounded motion. |
| 2026-10-01 | Initial consolidated project snapshot: SIE architecture, Pi ROS2 state, RGB-to-stereo direction, dynamic diagnostic evidence, and explicit safety gates. |
| 2026-10-01 | Project name changed to Spatial Intelligence Evidence Engine (SIEE); existing `sie` technical identifiers explicitly retained for compatibility. |
| 2026-10-01 | Active scope narrowed to evidence-driven precise vision: AR0234 semantic target to OV9281 3D association, reprojection validation, per-cycle evidence, and temporal validation must be proven before motion returns to scope. |
| 2026-10-02 | Added latest fusion evidence: 57-cycle route audit, fail-closed person test with globally strong disparity but sparse in-ROI depth, and semantic false-target diagnosis from static chest-board runs. |
| 2026-10-02 | Added controlled 1.50 m front-board evidence: 10-cycle mean about 1.49998 m with approximately ±5.7 mm spread, strengthening the distinction between stereo geometric accuracy and human-surface observability. |
| 2026-10-02 | Updated active stage and next sequence: first prove continuous correct person bbox, then benchmark target-surface support; added plane/surface quality metrics and a conditional future calibration-health hypothesis. |
| 2026-10-05 | Clarified accepted system scope: SIEE maintains an evidence-backed spatial state of the construction object and coordinates data and authorized actions for admitted robotic complexes, machines, and mechanisms. A mobile base is one future executor, not the boundary of SIEE. Current Vision Core safety gates are unchanged. |
| 2026-10-05 | Recorded the rigid approximately 7° upward camera-block pitch and controlled stationary-person AR0234 range evidence. Confirmed semantic availability at 2.5 to 4.2 m only for artificial light, a stationary person, and a mostly uniform wall; recorded 4.5 m as reserve, 2.0 m as not stable, persistent 3.0 m frustum-depth refusals, and the full-evidence recorder throughput constraint. |
| 2026-10-06 | Diagnosed the earlier 3.0 m stereo refusal as use of the frozen 0.5 to 2.0 m activation profile. With SHA-verified 0.5 to 4.5 m v2, 8 of 14 controlled 3.0 m cycles reached diagnostic RGB-to-stereo association; the six remaining cycles correctly failed only the unchanged 0.050 m depth-MAD gate. Added activation-profile integrity and 3.0 m MAD-stability checks to the next sequence. |
| 2026-10-06 | Added direct OV9281 planar checkerboard range series: v7 is near exact at 1.5 m but has repeatable positive depth bias at 2.0, 2.5, and 3.065 m. A 180-frame warm-up and a new matte 48 mm board reduced neither the far-range bias nor its implied disparity deficit enough to explain it away. Kept v7 and all activation profiles unchanged; prepared non-active v8 capture, audit, and candidate-solver tooling. |
| 2026-10-06 | Built and rejected the non-active v8 candidate. Although 144 raw pairs, solver RMS, physical baseline, and holdout residuals were strong, 180-frame-warm physical tests underestimated 1.5 m by 72 mm and 3.065 m by 138 mm. A same-board/same-distance v7 control ruled out a simple test-board or datum explanation. Strengthened audit requirements after identifying insufficient lower-field capture coverage; three v8b sessions now contain 216 pairs but still fail the new lower-Y gate. Prepared a gated fourth lower-field capture for the next session. |
| 2026-10-09 | Recorded the manual relative-axis screening limits and two continuous-camera v7 runs with reproducible 0-to-5-minute depth/disparity shifts of -23.94 and -31.19 mm while board PnP z shifted less than 0.5 mm. No current module temperatures were measured; retained thermal asymmetry as a hypothesis. Deferred further manual multi-range certification, prioritized fixed-board startup diagnostics, and kept v7/runtime/motor policy unchanged. |
| 2026-10-07 | Completed the gated fourth v8b capture, passed the strengthened coverage audit with 288 pairs, solved v8b, and rejected it after a same-setup 1.5 m comparative check: v8b `z=1.441585` m versus active v7 `z=1.500170` m. Recorded the new 1.5 to 4.0 m v7 matte-board screening series, explicitly reclassified it as non-certifying because the laser datum and board pose do not yet match `rectified_left_optical_frame z`, and replaced blind recapture with pose-aware v7 validation. Added the proposed staged LiDAR/odometry/stereo estimator MVP direction, while retaining the existing motor-disabled safety boundary. |


## 2026-10-16: physical-front comparison and original/current v7 geometry

The operator confirmed a 210 mm lens-rim setback and 15-degree upward block pitch. The stationary 1500 mm front-to-board comparison (9x6 inner corners, 25 mm squares) produced three nominal known terms with median 1.512272 m and frame-median span 5.49 mm, using 403–621 dense samples. This is not a physical depth error bound or offset calibration. The later approximate caliper estimate of 12.3–12.5 mm to a presumed optical centre is operator evidence only; its direction/projection and uncertainty remain unresolved.

All three corner comparisons refused: median rectified vertical residuals 2.10–2.14 full-resolution px. Offline independent coordinate recomputation and two corner detectors reproduced the mismatch. The original 15 holdout pairs, reprocessed with the byte-identical active v7, gave typical/worst pair medians 0.103/0.215 px. Source metadata and all 30 old image hashes matched. Partial spatial overlap also retained low old residuals (86 nearby old corners, median absolute residual 0.091 px). Thus current images agree with v7 substantially worse than the original images; the physical cause remains unproven.

The operator explicitly clarified that lens unscrewing/focus adjustment occurred BEFORE the 2026-09-28 source capture, not after. Do not attribute this mismatch to later lens adjustment. Next: test a small relative-rotation hypothesis offline on saved current pairs with held-out checks. Preserve active v7, thresholds, Measurement and motor boundaries. Full evidence: [v7 original/current comparison](docs/diagnostics/ov9281_v7_epipolar_comparison_20261016.md).
