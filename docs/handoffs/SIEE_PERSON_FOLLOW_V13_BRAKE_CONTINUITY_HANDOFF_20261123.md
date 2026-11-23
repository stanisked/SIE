# SIEE | Autonomous Person Following MVP | ESP32 v13 brake-continuity handoff | 2026-11-23

**Status:** CURRENT operational continuation record, not an ES/Data Contract, approved motor-control fix or permission for hardware operations. Follow normative order: Engineering Specifications and canonical Data Contracts, `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, this handoff, code and raw evidence.

## Repository / firmware (FACT)

- GitHub `stanisked/SIE`; active branch `codex/siee-person-follow-mvp-20261120`; last **firmware** HEAD `77067e47db98bf64d8101c4e8bbccd9822a1cc5b` before this docs-only change. v11 source is `b07fbd9c0274235473dc2db6e0abdda20986d34a`. v12 `1de64385df1f002a668c7454acfd205fd73f9651`, v13 `77067e47db98bf64d8101c4e8bbccd9822a1cc5b`.
- Pi checkout `/home/elwis/dev_ws/sie_live_identity_20261119`, physical UART `/dev/ttyAMA0` 115200 8N1; Pi GPIO14 TX → ESP32 GPIO16 RX2, Pi GPIO15 RX ← ESP32 GPIO17 TX2, common GND; CP2102 USB for laptop Arduino IDE 2.3.11.
- ESP32 flashed v13 build `v4_2_uart_shadow_v13_brake_continuity`. Physical UART GET /status and GET /fault-diagnostics returned HTTP 200 after flash, READY/PWM0/0/fault null, preflight boot ID `8B3C77062922A3EF`; subsequent test was also internally boot-consistency-guarded by the runner but exact test boot ID was not printed in summary.
- Hardware operator-confirmed: GB37-520 brushed geared motors, nominal **6 V, 200 RPM**, TA6586 drive supply **6 V**. Actual motor RPM in the three turn experiments not measured. No oscilloscope.

## Raised-wheel RIGHT 8° physical evidence (FACT)

Each of v11/v12/v13 was a separate supervised single right turn preceded by SHADOW_START 0.020 m, SHADOW_PAUSE, then one SHADOW_TURN RIGHT 8°, read-only /fault-diagnostics and terminal STOP. Target R/L 21/21 counts, hard limit 22/22 (fault on reaching/exceeding the limit), no automatic retries or fault acknowledgement.

| Firmware | Date | Git source commit | Brake request R/L | Request→write | Fault R/L | Final R/L | Additional R/L | Outcome |
|---|---|---|---:|---:|---:|---:|---:|---|
| v11 | 2026-11-20 | `b07fbd9c` | 15/14 | not measured | 26/26 | 28/27 | 13/13 | FAULT |
| v12 | 2026-11-23 | `1de64385` | 14/15 | 37 µs | 26/27 | 27/28 | 13/13 | FAULT |
| v13 | 2026-11-23 | `77067e47` | 15/13 | 38 µs | 26/25 | 27/25 | 12/12 | FAULT |

v11 brake request elapsed 73 ms; v12 74 ms; v13 70 ms. v13 guard matched the fault and `snapshot_valid=true`, `shadow_turn_brake_trigger=NEAR_TARGET`, request-to-mutex 3 µs, output calls 35 µs, total 38 µs; output-write count 15/13. Terminal `FAULT` and `FINAL_STOP_CONFIRMED: True`; final PWM 0/0. The +12/+12 counts are additional encoder progression since request, **not measured pin switching delay or independently measured yaw**. One sample/version, no demonstrated effective brake correction.

## Code, tests, diagnostics (FACT)

v12 introduced read-only `/fault-diagnostics` brake timing fields. In `firmware/esp32/traction_control_v4_2_uart_shadow_v13_brake_continuity/`, v13 removed only the preceding `stopMotors()` in `beginBoundedBraking()`, updated comments/build ID; no changes to pin wiring, initial PWM 205, target 21/21, cutoff 22/22, 400-ms active brake hold, or FAULT semantics. Pi test logs: v13 16/16, v12 8/8, v11 12/12 targeted Python PASS; laptop Arduino Verify 516175 program bytes, 72480 global bytes. Hash of flashed data verified by uploader, but complete firmware binary SHA/readback was not archived. Source review still shows intentional `Brake → Coast` after 400-ms hold in `updateBoundedBraking()`; its relevance to the fault is unproven.

## Private evidence / current safety (FACT / OPEN)

- v11 `/home/elwis/.local/state/sie/evidence/v11_right8_20261120_224844/`; historical [v11 handoff](SIEE_PERSON_FOLLOW_V11_TURN_FAULT_HANDOFF_20261120.md).
- v12 `/home/elwis/.local/state/sie/evidence/v12_right8_20261123_175655/`. Pi tar `/home/elwis/siee_v12_right8_preserved_20261123/v12_right8_evidence.tar.gz`, SHA256 `1d2b15c185bb0996412c53842f4b9e6b51e3d27cb29185fcdf27b8ca3b9a0760`. Its off-Pi copy is not confirmed.
- v13 `/home/elwis/.local/state/sie/evidence/v13_right8_20261123_205406/`, command `sie-v13-r8-c537ca8fa82e`. **Archive/SHA256 and off-Pi copy not yet confirmed.** Preserve raw logs privately, not in GitHub.
- After the latest v13 test, terminal state is FAULT/PWM0/0/STOP confirmed; **physical power-off after this test has not been confirmed**. Treat drive-power state as unknown. No further commands, reset/reflash or motor operations are authorized by this documentation update.
- Pi checkout still contains untracked `*.before_*`, `motion_recovery_coordinator.py` and tests. Do not indiscriminately stage, `git clean`, `git reset --hard`, or overwrite these. Updating GitHub does not update the local Pi worktree.

## DECISION / HYPOTHESIS / OPEN / exactly one next safe step

**DECISION:** Keep v13 as a versioned firmware checkpoint; do not claim the braking fault fixed. Do not relax 22-count hard guard, auto-clear FAULT or change PWM/voltage to force a pass. Keep earlier FAST floor-follow/steer evidence separately valid within its original bounds; identity, LiDAR, canonical Measurement, obstacle safety and unattended following remain OPEN.

**HYPOTHESIS:** residual +12/+12 after fast software brake request may be due to motor speed/inertia, actual TA6586 active braking effectiveness, later output mode transitions or a combination. `analogWrite()` completion is not independent electrical proof, PWM=0/0 does not distinguish active brake and Coast. No oscilloscope available.

**OPEN / NEXT:** Operator first reconfirms physical motor power is OFF, preserves/backs up v13 evidence with SHA256 before any reboot/ACK, then source-only reviews TA6586 output paths for **proposed**, not implemented, diagnostic v14 logging time-resolved encoder counts and software Brake/Coast writes. No v14 code/compile/flash/physical test exists. Graphify AST is still the 2026-11-16 `db8097ea4239dcf45ef7598c831a45648d2498fd` snapshot; no current extraction.

See [detailed comparison](../diagnostics/siee_esp32_brake_v11_v13_20261123.md).
