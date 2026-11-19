# SIEE | New Chat Handoff: Autonomous Person Following, 2026-11-19

**Use this handoff as the practical next-chat entry point**, not as proof of current Raspberry Pi runtime without fresh operator confirmation.

## 0. Exact recovery sources (read before changing code)

1. `AI_CONTEXT.md`
2. `PROJECT_CONTEXT.md`
3. Applicable frozen Engineering Specifications and Data Contracts
4. `AGENTS.md`
5. `vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md`, for the repository change requirements (current physical Pi calibration is *v7*, not V6).
6. `docs/handoffs/WORKING_BASELINE.md`
7. `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`
8. **Latest detailed status:** `docs/diagnostics/siee_floor_follow_face_body_status_20261119.md` (operator's 2026-11-19 uploaded primary files and visual floor report).
9. Code/tests of `vision_core/person_localization/shadow_live_identity.py`, `shadow_identity_crop.py`, `shadow_face_anchored_target.py`, `vision_core/tools/shadow_live_bridge.py`, `run_shadow_uart.py`, `run_shadow_fast_preview.sh`.

Repository: `stanisked/SIE`. **Continue from the branch `codex/siee-live-identity-preview-v1-20261119`**, rather than automatically using `main` or older `codex/shadow-turtlesim-follow-20261111`. The branch includes experimental opt-in Live Identity Preview. It has not been merged to a production baseline. Do not claim that the Pi's detached worktree HEAD has been re-verified merely because GitHub advanced after the test.

Recent operator worktree on Pi: `/home/elwis/dev_ws/sie_live_identity_20261119`; runtime Python `/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python`; UART `/dev/ttyAMA0`.

## 1. Project goal and 19-Nov checkpoints

Project: SIEE, Autonomous Person Following MVP. Pi5 + AR0234 RGB, OV9281 stereo, ESP32 WROOM32 motor controller, ROS2 Jazzy, eventual LiDAR D500 fusion. Target: moving person is selected, camera depth enables bounded approach, trolley steers to the person, stops at safe standoff or loss, then resumes only after fresh valid target re-detection. Face ID + Body ReID are proposed to preserve `person_001` across temporary RGB Track ID changes.

**Verified prior checkpoints:**
- Person RGB semantic selection and Track ID retention of moving/cropped torso, diagnostic OV9281 torso range; earlier physical stereo calibration evidence stays valid within its recorded setup, but diagnostic torso depth is not canonical `Measurement`.
- 2026-11-19 motor-off 143-cycle offline identity timeline: `:1` lost after cycle 33/34; `:2` reacquired cycle 73; first saved face cycle 80; 11 source snapshots, seven face observations; OSNet positive-only similarity cross-track median 0.77758, no negative-person control. 23/23 coordinator+replay tests passed after regression fix. Identity remains candidate, never confirmed.
- 2026-11-19 live motor-off Identity Worker: 18 results; 15 visible faces; `:1` 12, `:2` 1, `:3` 5; 18/18 `LATE_DIAGNOSTIC`, 18/18 coordinator `STALE_OR_UNBOUND_RGB_MASK`, worker median 625.605 ms (range 546.61–762.86 ms). All `identity_supported=false`, `motor_authorized=false`. It has not met the 0.5 s current-age contract.
- With Identity Worker on: camera-only 112 cycles, 67 FOLLOW_ELIGIBLE, 45 PAUSE, 19 cycles driver-to-decision over 500 ms, host max 696.76 ms. Do not assert causal CPU bottleneck without profiling.
- **2026-11-19 supervised cart-floor test with Identity Worker OFF:** six forward segments (one START, five RESUME), ten accepted STEER of twelve attempts, three PAUSE, final STOP and READY, no FAULT, PWM 0/0. Applied PWM steering reported by ESP32. **The operator directly observed the cart move toward them and shift toward their left (right in camera view) when they stepped laterally.** This establishes real physical forward FOLLOW + that direction of steering, not general bidirectional follow or robust identity re-entry. Entire execution used one RGB Track ID, `single_person_reacquires=0`.
- Firmware reported from Pi UART: `v4_2_uart_shadow_v8_short_forward_completion`. The physical firmware source commit/hash has not been newly verified.

## 2. Evidence paths and hashes

- Identity live `/home/elwis/.local/state/sie/evidence/shadow_fast_20261119_180036/bridge.jsonl` and `identity-preview.jsonl`. Uploaded copy identity-preview SHA-256 `6b9f570222c98647eeffeae3daef65dfc7cebeb3b49aa7b10fe788868e7b210e`.
- Floor motion `/home/elwis/.local/state/sie/evidence/shadow_fast_20261119_181125/bridge.jsonl`. Uploaded copy `bridge(20261119-151320).jsonl`, SHA-256 `e48fa59526e2f882649156036f89537a425c943dbf1e73fbc3794615c24476e6`.
- 143-cycle offline `/home/elwis/.local/state/sie/evidence/shadow_fast_20261119_170524/bridge.jsonl`, private `identity_timeline_20261119.json` and `identity_coordinator_replay_20261119.json`. Replay JSON SHA-256 `52ee8ec2186d7d50f58603db14c7dddb7e00ac270a798c51b2a97fbffc753fb1`.
- Full metrics, test boundaries, last observed ESP32 state: `docs/diagnostics/siee_floor_follow_face_body_status_20261119.md`.

## 3. Immediate task, one test at a time

**Next testable milestone:** carefully bounded, operator-supervised **floor loss/STOP/re-entry test** on the previously functioning RGB+depth+UART flow, with `SHADOW_IDENTITY_PREVIEW=0` pending latency work. First inspect the existing `shadow_live_bridge.py` and `run_shadow_uart.py` segment, single-person reacquire and fresh input gates. Validate the floor scenario can actually be staged with available room/operator and an independent motor power cutoff. Log command statuses and final PWM 0/0. Do not loosen limits or silently auto-assign `person_001` on one RGB Track ID.

After this milestone, improve live Face/Body worker timing **motor-off** (break down YuNet, SFace, OSNet and queue age; preserve current capture rate), then separately validate negative-person identity calibration. Keep existing `FaceAnchoredTarget`, fixed-source evidence, and exact track/mask binding.

Do NOT repeat static depth calibration or raised-wheel steering tests already completed. Do not refactor for abstraction alone. No unverified progress should be declared PASS.

## 4. Proposed opening user message for new ChatGPT conversation

> Сью, продолжаем SIEE | Autonomous Person Following MVP с подтверждённой точки 19.11.2026. В репозитории `stanisked/SIE` работаем с веткой `codex/siee-live-identity-preview-v1-20261119`. Прочитай `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, `docs/handoffs/WORKING_BASELINE.md`, `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md` и `docs/diagnostics/siee_floor_follow_face_body_status_20261119.md`. На полу подтверждены движение к человеку, физическое подруливание в одну сторону и финальный STOP; журнал `shadow_fast_20261119_181125`. Live Face/Body ReID работает диагностически, но 18/18 результатов позднее 500 мс, Identity Worker ещё не участвует в моторном управлении; журнал `shadow_fast_20261119_180036`. Не повторяй уже закрытые тесты. Задача: один за другим проверить остановку при потере цели, повторное обнаружение с новым Track ID и управляемое продолжение движения, затем устранить задержки Face/Body ReID без ухудшения камер. Всегда различай ФАКТ, РЕШЕНИЕ и ГИПОТЕЗА и проверяй реальные исходники и следы команд, прежде чем менять код. Предлагай один следующий практический шаг.

## 5. Maintenance and Graphify

The same-day coordinated maintenance should update BOTH `PROJECT_CONTEXT.md` and `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`; compare and synchronize their pre-existing private Drive copies. The Drive versions were ahead of this branch with a dated 17-Nov update. Preserve it. Graphify is frozen at `db8097e`, generated 2026-11-16 (4,599 nodes, 13,658 edges, 300 communities). The new code has not been re-extracted, and no graph freshness claim can be made.
