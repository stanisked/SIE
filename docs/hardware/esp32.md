# ESP32 traction-control baseline

## Firmware

Физически проверенная прошивка: `traction_control_v4_1_effective_track294`. В Git хранится sanitized-вариант: встроенные Wi-Fi credentials удалены и заменены на `#include "wifi_credentials.h"`.

Original sensitive source находится вне Git; его SHA-256 и размер зафиксированы в `baseline_manifest.json`. Sanitized SHA отличается от original только credential binding. Реальные SSID и password в документации не приводятся.

Для локальной сборки оператор должен создать header только вне коммита:

```bash
cp wifi_credentials.example.h wifi_credentials.h
# локально заменить CHANGE_ME
```

`wifi_credentials.h` включён в локальный `.gitignore` и не должен коммититься.

## Hardware and geometry

- ESP32 DevKit/WROOM-32
- ZK-5AD dual H-bridge
- 2 x JGB37-520 с quadrature encoders
- wheel diameters: right 0.0646 m, left 0.0652 m
- geometric track width: 0.290 m
- effective turn track width: 0.2941 m
- turn sustain PWM: 180

## HTTP API

Transport: HTTP over TCP port 80, hostname `sie-base`, mDNS `sie-base.local`.

| Method | Endpoint | Fixed action |
|---|---|---|
| POST | `/start` | forward 0.500 m |
| POST | `/reverse` | reverse 0.200 m |
| POST | `/turn-left` | left turn 90 degrees |
| POST | `/turn-right` | right turn 90 degrees |
| POST | `/square` | square sequence |
| POST | `/stop` | immediate stop |
| GET | `/status` | status JSON |

Motion requests return HTTP 202 when accepted, 409 when busy, and 503 when Wi-Fi is unavailable. Stop returns HTTP 200. The source has no variable distance/angle request fields and no command_id/request-id mechanism.

Status JSON includes firmware/state/direction, Wi-Fi state and IP, square state/progress/action, pose/heading/closure fields, encoder counts/distances/speeds, target distance, PWM, track-width and turn telemetry, and fault reason.

## Safety behavior found in source

The firmware stops on remote stop, Wi-Fi loss, timeout, wrong direction, encoder/stall detection, breakaway failure, and distance-limit violations. Square execution has its own timeout/failure path.

## Physical validation

Forward, reverse, left turn, right turn, STOP, and square were confirmed physically. Confirmed square runs: run 1 closure 15 mm right with 2..3 degree heading error; run 2 closure 35 mm left with about 2 degree heading error. No fault/stall occurred in those square runs.

## Current limitations and next step

The baseline exposes fixed 0.500 m forward, fixed 0.200 m reverse, and fixed 90-degree turns. It does not support variable distance or variable angle. The next separate change set should define bounded variable TURN up to 10 degrees and bounded ADVANCE up to 0.10 m, with explicit completion/status semantics and mandatory STOP handling.
