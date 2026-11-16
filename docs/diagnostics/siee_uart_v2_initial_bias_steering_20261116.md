# SIEE UART v2 initial-bias / physical steering validation — 2026-11-16

**Status:** FACT: supervised physical bilateral steering, initial bias applied by ESP32, two sign reversals during one moving-person raised-wheel Shadow session. OPEN: autonomous person-following trajectory and canonical stereo Measurement.

## Hardware and software
- Raspberry Pi 5 `piecake`, AR0234 RGB segmentation and OV9281 stereo diagnostic range; GPIO UART `/dev/ttyAMA0` to ESP32 WROOM-32.
- ESP32 firmware build `v4_2_uart_shadow_v2_initial_bias`, `shadow_api_version=v1_paused_segment`.
- Firmware copied into separate local folder `firmware/esp32/traction_control_v4_2_uart_shadow_v2_initial_bias`, with 8 UART core fields for SHADOW_START/RESUME, CRC16 unchanged, field buffer enlarged to 8 and input bytes buffer to 256. Initial `bias_pwm` is an integer in [-20,20], stored in CommandRecord and the original start/resume bias saved as `shadowLastBiasPwm` for duplicate detection.
- On START, firmware sets `startRequested` after bounded command configuration; UART `pollUart()` completes before loop's `if(startRequested)` / `startMotion()`. In DRIVING only, `updateDriving()` applies bias to wheel PWM. In RESUME it is set after `configureBoundedCommand`.
- Pi Python updated `run_shadow_uart.py` and `uart_http_transport.py`: pass initial bias with START/RESUME, expected BUILD v2, no redundant immediate STEER after DRIVING. Tests: fields START/RESUME/STEER/PAUSE/STOP PASS; Python static check PASS. These changed executable files were verified from operator terminal and **not confirmed committed to GitHub**.
- Arduino IDE 2.3.10 compile PASS (504463 bytes program, 71880 bytes globals), upload verified; post-flash UART GET /status HTTP 200, BUILD v2, READY, both PWM zero and faults false.

## Physical test matrix
| Evidence directory (under /home/elwis/.local/state/sie/evidence) | Configuration | Observed result | Final status |
|---|---|---|---|
| shadow_fast_20261116_194409 | raised wheels, 9 x 0.100m, operator left | 9 accepted START/RESUME with initial bias -20; last command 9 biased drive cycles; right/left PWM sums 2016/1215 | SESSION_ENDED, READY, PWM 0/0, no faults |
| shadow_fast_20261116_195100 | floor, one 0.100m, left | operator visually observed slight left chassis turn; last segment wheel distances 0.1198 / 0.1094m; encoder heading +0.0359 rad (~2.06°), applied bias -20, 1 biased cycle | SEGMENT_LIMIT_REACHED, READY, PWM 0/0 |
| shadow_fast_20261116_195358 | floor, one 0.100m, right | bias +20, left PWM 181 vs right 170 at brake; distances 0.1000 / 0.0994m, heading +0.0019 rad (right turn not demonstrated by this test) | SEGMENT_LIMIT_REACHED, READY, PWM 0/0 |
| shadow_fast_20261116_195714 | floor, 3 x 0.100m, right | operator visually saw rightward steering; all 3 initial biases +20, accepted; last segment wheel distances 0.0950 / 0.0964m, heading -0.0049 rad, 3 biased cycles | SEGMENT_LIMIT_REACHED, READY, PWM 0/0 |
| shadow_fast_20261116_200624 | raised wheels, moving person, 40s limit, max 20 segments / session | first session 14 accepted segments, two sign changes; track change caused STOP and NEW_SHADOW_SESSION; second session 19 segments, first/last applied bias in last command +20/-20, 17 biased cycles | both sessions SESSION_ENDED; first reason NEW_TRACK_REQUIRES_NEW_SESSION, second READY, PWM 0/0 |

## First moving-person session: exact accepted initial bias sequence
| Segment | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bias_pwm | +20 | +20 | +20 | +20 | +20 | +11 | -15 | -18 | -20 | -20 | -20 | -20 | -13 | +2 |

All HTTP 202; sign reversal at segment 7 and 14. Same Shadow command ID through these 14 segments; next different RGB track requires new Shadow session. Full RGB per-frame association continuity and exact second-session bias sequence need review of Pi source records.

## Measurement and causality boundaries
- The preview logger reports frequent `DEPTH_MAD_EXCEEDED` and `motors=DISABLED`. This node emits diagnostic-only input; the independent supervised Shadow runner issues UART motor commands. Do not interpret the diagnostic label as the final ESP32 motor state.
- `last_steer_update_seq=0` means the separate mid-segment STEER was not confirmed applied in these final records; the verified dynamic sign changes use START/RESUME initial bias.
- Final `right_distance_m`, `left_distance_m` and `heading_rad` refer to the **last segment**, not total session travel or independently measured yaw.
- Requested 0.100m motion is a bounded command, not exact displacement. In one floor trial the average last-segment encoder distance was 0.1146m.
- No canonical SIE Measurement authorization, LiDAR fusion, proven physical obstacle clearance, robust loss-of-target stop under loaded dynamic person-following, or fully autonomous navigation is claimed.

## Next reproducible check
A supervised bounded floor test with moving person, synchronized logs for RGB track ID, observed bearing, OV9281 diagnostic depth and provenance, START/RESUME bias, actual wheel odometry, independent chassis heading, stop condition and obstacle-free space. Verify both-direction control and target loss/reacquisition while preserving already demonstrated calibration and UART v2. Do not confuse Graphify static code edges with observed runtime evidence.

## Evidence
- `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_194409/bridge.jsonl`
- `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_195100/bridge.jsonl`
- `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_195358/bridge.jsonl`
- `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_195714/bridge.jsonl`
- `/home/elwis/.local/state/sie/evidence/shadow_fast_20261116_200624/bridge.jsonl` and subsequent session journal (exact filename pending readback).
