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

## v4_2 bounded-motion API: partial physical evidence, not operationally approved

`traction_control_v4_2_bounded_motion_api` is a separate sanitized firmware
candidate based on the v4_1 baseline. It preserves the v4_1 motor control,
geometry, square controller and safety logic. Bounded forward has physical
evidence, but bounded micro-turn behavior is not yet accepted.

It adds only these bounded commands for stop-and-measure integration:

| Method | Endpoint | Required query | Bound |
|---|---|---|---|
| POST | `/move-forward` | `boot_session_id`, `command_id`, `distance_m` | 0.02..0.10 m |
| POST | `/turn-left` | `boot_session_id`, `command_id`, `angle_deg` | 1.0..10.0 degrees |
| POST | `/turn-right` | `boot_session_id`, `command_id`, `angle_deg` | 1.0..10.0 degrees |
| POST | `/ack-fault` | none | clears a stopped, latched bounded-command fault |

`/turn-left` and `/turn-right` without query parameters retain the legacy
90-degree behavior. A partial, mixed or unknown bounded query is rejected.
The bounded forward and turn timeouts are 5000 ms and 4000 ms respectively.

Every bounded request uses an ASCII `command_id` of 1..64 characters from
`A-Z`, `a-z`, `0-9`, `.`, `_`, and `-`, plus the current `boot_session_id`
from `GET /status`. The session ID is a fresh 64-bit random hex value at each
boot. A mismatch returns `409 BOOT_SESSION_MISMATCH` before ledger lookup or
motor start. The motor bridge must follow this sequence:

1. call `GET /status` and read `boot_session_id`;
2. create a new command ID;
3. send one bounded request bound to that session;
4. never carry or rebind an old command into a new boot session;
5. when `boot_session_id` changes, discard the old decision and run a new
   perception and decision cycle.

The firmware records requests in a fixed in-memory ledger within one boot
session. A duplicate with identical endpoint and parameters returns its current
state without restarting motors; reuse with changed parameters returns `409
COMMAND_ID_CONFLICT`; a new ID while motion is active returns `409 BUSY`.

Every accepted bounded command stores its requested target, per-wheel encoder
target and absolute limit, predictive brake-start counts, motion kind, and
bounded timeout. The encoder guard runs on every firmware loop in STARTING,
DRIVING, BRAKING and COASTING, independently of the 100 ms PI cadence.

Для bounded `FORWARD` в текущем диапазоне `0.02..0.10 m` теперь выбирается
экспериментальный `BOUNDED_FORWARD_SHORT_STEP_MVP_V1`, policy label
`PROVISIONAL_SHORT_STEP_NOMINAL_CRAWL_MVP_V1`.
Его путь: `BREAKAWAY -> LOW_SPEED_CRAWL -> BRAKING -> SETTLING`.
Внешние `MotionState` сохранены; `BRAKING` в короткой telemetry соответствует
существующей внутренней `ACTIVE_BRAKE`.

Ramp и target-aware критерии `BREAKAWAY` остаются прежними. Initial slowdown
counts старого профиля сохраняются только как вход прежних BREAKAWAY helpers;
short-step не имеет фазы `SLOWDOWN` и не пересчитывает её границы.
При штатном завершении BREAKAWAY отдельный путь сразу ограничивает PWM
существующими ceilings `RIGHT_RUN_PWM_MIN`/`LEFT_RUN_PWM_MIN` (115/95).
После перехода PWM каждого колеса может только сохраняться или уменьшаться;
ведущее колесо не получает больше PWM, чем отстающее. Legacy sustain floor,
lagging-wheel boost и высокоскоростной APPROACH в этот путь не входят.

Brake envelope каждого колеса рассчитывается при configure прежней функцией
`boundedForwardStopEnvelopeCounts()` на **номинальной** crawl speed `0.060 m/s`.
Измеренная скорость не сдвигает эту границу. Для `0.10 m` текущая конверсия даёт
targets 102/101, envelope 9/9 и brake boundaries 93/92 counts.
Первое колесо, достигшее границы, запускает общий active brake с trigger
`SHORT_STEP_ENVELOPE`. Hard limits остаются `target + 1`; любой наблюдённый
hard-limit повышает pending reason до `BOUNDED_DISTANCE_LIMIT`, включая уже
начавшееся торможение из-за invalid speed, без перезапуска brake interval.

Перед каждой directed PWM записью в LOW_SPEED_CRAWL выполняется общий encoder
guard и проверка fresh/valid sample по текущему времени. Отсутствующий, stale
или invalid sample вызывает `BOUNDED_FORWARD_SPEED_ESTIMATE_INVALID` через
существующий active-brake path. Двухступенчатого подтверждения crawl здесь нет.
После BRAKING directed PWM не возвращается. Итоговый недобор вне неизменного
success window даёт `BOUNDED_TARGET_NOT_REACHED` с latch, превышение target
даёт `BOUNDED_DISTANCE_LIMIT` с latch. Существующие отдельные причины отказа
speed/sync/watchdog сохраняются; correction, retry и re-drive отсутствуют.

`LOW_SPEED_CRAWL` обозначает режим ограничения PWM, а не подтверждённую
фактическую скорость `0.060 m/s`. Свежий sample может показывать более высокую
скорость после BREAKAWAY. Этот номинальный envelope экспериментальный:
ни ограничение PWM, ни encoder hard-limit не доказывают mechanical containment.
Профиль может дать недобор или overshoot и не разрешает operational floor forward.

Итоговый experimental raised-wheel запуск short-step на `0.10 m` завершился
следующим evidence:

- profile: `BOUNDED_FORWARD_SHORT_STEP_MVP_V1`;
- right/left target: `102/101` counts;
- right/left crawl PWM: `95/95`;
- right/left measured speed sample: `0.2227/0.2486 m/s`;
- right/left counts при brake: `87/92`;
- первый hard limit: left `102` counts;
- right/left final: `93/102` counts;
- terminal: `BOUNDED_DISTANCE_LIMIT`, fault latch `true`, final PWM `0/0`.

Статус: `bounded_forward_0.10_m: NOT_QUALIFIED`. Floor gate не выполнялся.
Этот результат относится к ESP32 bounded-forward execution profile и не
является failure SIE planner или perception. Текущий ESP32 chassis execution
adapter не разрешён для automatic short forward advance. Один raised-wheel
trace не устанавливает hardware root cause и не служит основанием для нового
numeric tuning.

Сохранённый `BOUNDED_FORWARD_CRAWL_ENVELOPE_MVP_V1` имеет внутренние фазы:
`BREAKAWAY`, `APPROACH`, `SLOWDOWN`, `CRAWL`, `ACTIVE_BRAKE` и `SETTLING`.
Внешние `MotionState`, terminal classification и API-контракт не менялись.

Отдельная per-wheel speed estimate строится только из encoder counts и не
меняет существующую 100-ms PI cadence или legacy speed logic. Slowdown boundary
оценивает путь до provisional crawl speed, brake boundary оценивает остаточный
остановочный envelope из свежей скорости, reaction interval, braking model и
count uncertainty. Это не один фиксированный stopping margin и не калибровка.
Первые constants намеренно имеют versioned provisional label.

`SLOWDOWN` начинается по OR-семантике, когда любое колесо достигает своей
границы. В `SLOWDOWN` и `CRAWL` используется отдельный forward-only PWM path:
per-wheel PWM ceiling монотонно не растёт, legacy sustain floor и sync boost не
могут его обойти, а ведущее колесо не получает больше PWM, чем отстающее.
`CRAWL` разрешён только после заданного числа последовательных valid fresh
speed samples ниже crawl ceiling. Если speed estimate в этих фазах stale,
invalid, non-finite, negative или impossible, до следующего directed PWM write
запускается существующий active-brake path с причиной
`BOUNDED_FORWARD_SPEED_ESTIMATE_INVALID`. Hard limit проверяется раньше.

Active brake начинается по OR-семантике, когда projected final любого колеса
достигает target. После входа в `ACTIVE_BRAKE` directed PWM не возвращается.
Недопустимый sync error вызывает общий active brake без correction, retry или
re-drive. Если начальный envelope не помещается в target,
`envelope_feasible=false`: directed motion не стартует, а команда fail-closed
завершается как `BOUNDED_TARGET_NOT_REACHED`.

Targets, hard limits `target + 1`, success window, PWM mapping и nominal PWM,
active-brake hold/settle timing, FAULT/latch, routes, boot session,
idempotency, turns, square и legacy motion не менялись. Профиль может закончить
команду `BOUNDED_TARGET_NOT_REACHED` или `BOUNDED_DISTANCE_LIMIT`. Он не
доказывает physical containment и не даёт operational approval для floor
forward.

Bounded turns from 1 to 10 degrees use the separate
`BOUNDED_MICRO_TURN_V1` profile: each wheel reserves at least 6 counts before
its target, so a target of 11 counts has an initial brake-start at 5 counts.
A dynamic stop margin may increase this reserve and start braking earlier, but
may never reduce it below 6 counts. This profile applies only to bounded turns;
the legacy 90-degree turn is unchanged. Status adds `bounded_motion_profile`
and `bounded_turn_brake_reserve_counts`.

For bounded commands only, the normal stopping boundary is predictive: the
firmware starts the already documented TA6586 electrical brake before either
wheel reaches its requested target. It holds that brake, releases it, then
records final settled counts and signed count deltas versus target. Once this
BRAKING state begins, it never calls `setDrivePwm` again. The absolute
per-wheel encoder limit remains an independent emergency guard. It starts the
same active-brake/settle sequence, then completes as
`BOUNDED_DISTANCE_LIMIT` and latches the fault. The status JSON exposes target,
brake-start, counts at brake start, final settled counts, signed overshoot and
the selected stop mode (`ACTIVE_BRAKE` or `COAST`). If predictive braking
settles below either target, completion uses an explicit per-wheel lower
tolerance: `min(6, max(1, ceil(target_counts * 0.20)))`. Each settled count
must be between `target - tolerance` and `target`, inclusive. A positive count
above target remains `BOUNDED_DISTANCE_LIMIT` with a fault latch. A count below
its lower bound remains `BOUNDED_TARGET_NOT_REACHED` with a fault latch.

### Bounded-forward brake observability v1

An incident for requested `0.10 m` settled at right target/final `102/100`
and left target/final `101/110`, with `BOUNDED_DISTANCE_LIMIT`, `FAULT`, a
latched bounded fault and zero final PWM. The configured brake starts were
`66/65`; the existing count-at-brake snapshot was `75/85`; encoder average
distance was `0.1042 m`.

For bounded-forward profiles, including
`BOUNDED_FORWARD_CRAWL_ENVELOPE_MVP_V1`, `GET /status` preserves a fixed-size
`bounded_forward_brake_diagnostics` object with four nullable snapshots:
`pre_brake_guard`, `brake_command`, `first_post_brake_loop` and
`first_hard_limit_guard`. They record encoder/control timing, threshold and
limit membership, brake PWM/state, and the first hard-limit observation. The
object is `null` while idle and for non-forward commands; an individual
snapshot remains `null` until captured. No trace arrays or diagnostic strings
are retained in command history.

This instrumentation does not prove mechanical containment. It only locates
the encoder and controller timing of a future violation; target counts,
target-plus-one hard limits, PWM, brake hold, completion classification, fault
latch and routes are unchanged.

Thus distance and angle are upper physical bounds, not requirements to land on
one mathematically exact encoder count. The status also records each wheel's
completion tolerance and lower success count. The existing 100 mm floor test
settled at 98/98 counts for targets 102/101, with 0.098 m manually measured
travel, 0.0972 m encoder average, -0.0015 rad heading and `ACTIVE_BRAKE`.
This is accepted MVP mechanical evidence, not operational approval. A final
repeat floor test is still required after compile and flash.

The initial raised-wheel test of the earlier cutoff design did not validate
bounded physical behavior: a 20 mm command cut output around 20 to 22 counts
but coasted to about 60 counts. The updated candidate status is
`READY_FOR_ARDUINO_COMPILE_AND_MICRO_TURN_RETEST`. It does not claim that
mechanical coast is eliminated. That requires a fresh physical measurement
after successful compile, flash and retest.

A later raised-wheel bounded `TURN_RIGHT 4°` test exposed a separate
micro-turn problem in the inherited turn profile. It requested -0.0698 rad,
reached an encoder heading of -0.0978 rad, equivalent to 5.60 degrees, and
settled at 16/13 counts against 11/11 targets after braking configured at 8/8.
The absolute guard correctly returned `BOUNDED_DISTANCE_LIMIT`, FAULT and a
latch. This is not a physical pass. The new micro-turn profile requires an
Arduino compile followed by raised-wheel and floor retests before any approval.

With the reserve-8 profile, a subsequent raised-wheel `TURN_RIGHT 4°` test
started braking at 3/3 counts and settled at 8/6 against the 11/11 targets.
The equivalent encoder heading was 2.70°. There was no positive overshoot, but
the left wheel was below the 8-count success window, so the controller correctly
reported `BOUNDED_TARGET_NOT_REACHED`, FAULT and latch. This is safe underreach,
not physical PASS; the reserve-6 change still requires a new retest.

Breakaway uses a target-aware threshold and a target-aware PWM ceiling for
bounded commands, so a short request cannot wait for the legacy 15-count
threshold or ramp beyond its planned braking room. Legacy motion continues to
use its original breakaway and coast behavior.

This is an encoder-bounded cutoff, not a claim of mathematically zero physical
overshoot. Residual mechanical motion still requires separate physical
measurement before the bounded API can be operationally approved.

Фазовая диагностика bounded forward сохраняет отдельные nullable snapshots:
`prior_non_triggering_before_predictive_brake`, `pre_brake_guard`,
`prior_non_triggering_before_hard_limit` и `first_hard_limit_guard`. Каждый
snapshot содержит `guard_evaluation_seq`; trigger и соответствующий ему prior
sample больше не смешиваются из-за последующей перезаписи. Если первый guard
сразу срабатывает, соответствующий `prior_*` остаётся `null`. Для idle, legacy
и turn весь `bounded_forward_brake_diagnostics` остаётся `null`.

Один guard evaluation использует один локальный encoder sample и один timestamp.
`brake_command` намеренно остаётся отдельным чтением после `stopMotors()` и
`brakeMotors()`, поэтому его counts могут отличаться от trigger sample. Эта
telemetry показывает порядок encoder/control events, но не доказывает
mechanical containment и не определяет безопасный stopping envelope на полу.

Следующая проверка: один bounded-forward запуск на 0.10 m с поднятыми колёсами.
После terminal state нужно сохранить полный `/status` и остановиться без retry.
Raised-wheel evidence не даёт разрешения на floor stopping или operational
forward motion.

`GET /status` использует существующий fixed-size
`bounded_forward_crawl_envelope`: phase/sequence, speed status/reason,
per-wheel speed/timestamp/age/sequence, remaining counts, slowdown/brake
boundaries, stop-envelope counts, crawl PWM ceilings, sync error/leading wheel,
`envelope_feasible`, `slowdown_entry` и `crawl_entry` для сохранённого старого
профиля. Для short-step тот же объект компактнее: phase/sequence, speed sample,
per-wheel brake boundaries/envelopes и `right_last_applied_crawl_pwm` /
`left_last_applied_crawl_pwm`. Последние два поля хранят последний фактически
записанный crawl PWM (0 до первой записи), после brake это история;
текущий PWM читается из прежних top-level полей.

`speed_status` относится к сохранённому sample. Его исторический характер явно
показывает `speed_sample_is_historical`; `speed_usable_for_active_control`
проверяет active record, управляющую фазу, valid sample и freshness и всегда
равен false во время brake/settle и после terminal. Возраст активного sample
вычисляется при чтении status, после terminal хранится возраст на момент
последнего сохранения. Исторический `VALID` не означает пригодность для движения.
Во время active/queued legacy или square объект равен `null`, history
`CommandRecord` не стирается. В READY доступна последняя bounded history;
для turn и при отсутствии history объект `null`. Brake/settled snapshots
сохраняют прежний смысл, `brake_command` остаётся отдельным чтением после brake.

Исходные evidence для профиля: manual encoder baseline 1032/1033 counts за
пять оборотов, raised-wheel dynamic reserve 0.010 m с финалом 108/108 и
overshoot, затем fixed 0.035 m reserve с финалом 81/80 и safe underreach.
Успешный source-contract review этих фактов не заменяет compile/flash и
physical gate.

Следующий gate: ровно один raised-wheel bounded-forward запуск на `0.10 m`.
Перед командой оператор проверяет `READY`, отсутствие latch, PWM 0/0 и новый
`boot_session_id`. После единственной команды он сохраняет полный terminal
`/status`, включая crawl-envelope и phase diagnostics, targets, limits и
settled counts, затем останавливается без retry. Floor-команда на этом gate не
разрешена.

Any active bounded-command FAULT latches `bounded_fault_latched` with
`bounded_fault_reason`, including timeout, Wi-Fi loss, encoder/stall and
direction faults. While latched, all new bounded, legacy and square movement
commands return `409 BOUNDED_FAULT_LATCHED`. `POST /stop` always remains
available but does not clear the reason. Only `POST /ack-fault` can clear the
latch, and only after there is no active motion or square sequence and the
firmware has called `stopMotors`. A successful acknowledgement requires the
current state to be `FAULT` with a non-empty runtime fault reason; it then
clears only the current latch/reasons and restores controller state `READY`.
The last faulted command, its timestamps and encoder diagnostics, the boot
session and the idempotency ledger remain intact. Its response includes the
previous reason.

`GET /status` keeps all v4_1 fields and adds `motion_api_version`,
`boot_session_id`, `bounded_fault_latched`, `bounded_fault_reason`, active and
last command IDs, endpoint, state, bounded distance/angle, and
start/completion timestamps. Missing values are JSON `null`.

The current `BOUNDED_MICRO_TURN_V1` challenge disables blind correction pulses
(`bounded_max_correction_pulses = 0`). It uses one synchronized drive pulse:
base sustain PWM is 180; when the absolute encoder difference reaches 2 counts,
the leading wheel is limited to 165 and the lagging wheel receives 205. When a
wheel reaches its own brake-start, only that motor pair receives the confirmed
TA6586 active-brake combination (both inputs HIGH); the other wheel continues
until its own brake-start, after which both are braked and settled. No drive PWM
is restored after an individual brake. This challenge is symmetric for left and
right turns and does not alter bounded forward 0.10 m or the legacy 90-degree
turn.

For `BOUNDED_MICRO_TURN_V1`, terminal success also requires both wheels to
reach `ceil(target_counts * 0.50)`, the expected heading sign, and a heading
between 70% and 100% of the requested angle. The latest floor evidence is
`TURN_RIGHT` 4 degrees settled at 8/6 counts in the reserve-8 raised-wheel
evidence, with encoder heading 2.70 degrees and no positive overshoot. The
terminal result was `BOUNDED_TARGET_NOT_REACHED` with FAULT and latch because
the left wheel was below its lower completion bound. This is safe underreach,
not physical PASS; the current status is
`MICRO_TURN_SINGLE_PULSE_SYNC_CHALLENGE` and requires repeat raised-wheel and
floor tests.

The latest single-pulse synchronized floor challenge requested a 4 degree
`TURN_RIGHT` and measured 3 degrees. Both individual brake-start points were
reached (5/5 counts), the settled counts were 6/5 with one count of sync error,
and there was no positive overshoot. This result is terminal
`PARTIAL_PROGRESS`: the controller returns to `READY` without a fault latch,
preserves the actual counts and heading, and requires a fresh perception
re-observation before another turn. No automatic retry or hidden correction
pulse is allowed after this safe underreach. It is not production approval.

Final physical evidence records bounded forward 0.10 m at 0.100 m with
`SUCCESS`, `TURN_RIGHT` requested 4 degrees at about 2 degrees with one motor
activation and no positive overshoot, and `TURN_LEFT` requested 4 degrees at
about 3 degrees with one activation and no positive overshoot. Both turns are
`PARTIAL_PROGRESS`; SIE must re-observe before issuing another turn. An exact
4-degree micro-turn is not claimed.
