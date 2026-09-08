# Current bounded-motion MVP handoff

## Experimental forward crawl envelope MVP

- branch: `feature/esp32-forward-crawl-envelope-mvp-v1`;
- base: `1dea45482c2830bcb4438c4a5ed3a2adf384b5d9`;
- профиль: `BOUNDED_FORWARD_CRAWL_ENVELOPE_MVP_V1`;
- внутренние фазы bounded forward: `BREAKAWAY`, `APPROACH`, `SLOWDOWN`,
  `CRAWL`, `ACTIVE_BRAKE`, `SETTLING`;
- отдельная fixed-scalar per-wheel speed estimate не меняет 100-ms PI cadence
  и legacy speed logic;
- slowdown и projected-final brake boundaries считаются из fresh speed и
  provisional envelope model, а не из одного fixed margin;
- slowdown и brake используют per-wheel OR semantics;
- в `SLOWDOWN`/`CRAWL` PWM ceiling каждого колеса не повышается, legacy floor и
  sync boost его не обходят, ведущее колесо не получает больше PWM;
- invalid/stale speed в slowdown/crawl вызывает active brake до следующего
  directed PWM write с `BOUNDED_FORWARD_SPEED_ESTIMATE_INVALID`;
- после `ACTIVE_BRAKE` directed PWM не возвращается;
- при `envelope_feasible=false` directed motion не стартует, результат
  fail-closed `BOUNDED_TARGET_NOT_REACHED`;
- targets, hard limits `target + 1`, success window, brake/settle timing,
  FAULT/latch, API, ledger, turns, square и legacy motion не менялись.

Исходные evidence: manual encoder baseline 1032/1033 counts за 5 оборотов;
dynamic 0.010 m reserve завершился overshoot 108/108; fixed 0.035 m reserve
завершился safe underreach 81/80. Наблюдавшаяся guard cadence около 2 ms и
отсутствие directed PWM после `BRAKING` относятся к этим отдельным physical
traces, а не доказывают новый профиль.

Constants и envelope model имеют явный provisional label и не считаются
калиброванными. Change set не доказывает mechanical containment и не даёт
operational approval для floor forward.

Offline-проверки change set:

- `.ino` SHA-256:
  `ee82c04eb86ecf885dd7d20cc41e949ec019c5aaddc2011a4df5aca3312aea4e`;
- три focused source-contract tests: `3 passed, 47 deselected`;
- manifest JSON parse: PASS;
- credential scan tracked firmware/docs: PASS, совпадений нет;
- `git diff --check`: PASS;
- Arduino compile/upload и hardware run не выполнялись.

Следующий gate после отдельного compile/flash пользователем: ровно один
raised-wheel bounded-forward запуск на `0.10 m`. До команды проверить `READY`,
`bounded_fault_latched=false`, PWM 0/0 и новый `boot_session_id`. После команды
сохранить полный terminal `/status` с phase/envelope telemetry и остановиться
без retry, `/ack-fault` или floor-команды.

## Experimental forward conservative brake MVP

- branch: `feature/esp32-forward-conservative-brake-mvp-v1`;
- профиль: `BOUNDED_FORWARD_CONSERVATIVE_MVP_V1`;
- только bounded forward получает initial per-wheel brake-start от
  `FORWARD_MAX_STOP_MARGIN_M = 0.035 m`;
- dynamic brake-start может сдвинуться раньше, но не позже этой границы;
- targets, hard limits, PWM, brake/settle timing, completion window,
  FAULT/latch, HTTP routes, boot session, idempotency, legacy motion и turns не
  менялись;
- ранний brake может закончиться fail-closed
  `BOUNDED_TARGET_NOT_REACHED`;
- профиль экспериментальный: mechanical containment и operational approval
  floor forward не заявляются.

Следующий gate после offline review и отдельного compile/flash пользователем:
ровно один raised-wheel bounded-forward запуск на `0.10 m`, полный terminal
`/status`, затем stop без retry. Этот change set сам hardware не запускает.

## Цель

Зафиксировать экспериментальный безопасный MVP bounded ESP32 motion API v4.2
для stop-and-measure интеграции. Это не operational motion approval.

## Revision

- branch: `feature/esp32-bounded-motion-api-v1`
- final HEAD before this handoff commit: `e6c3806fb9da372d0a188c1aa60e4d4848963545`
- firmware SHA-256: `19d2bac0f8f46f53e34fcd46f78ea8ca3aadd0985c3ef1de95a5bc7ecb06c28a`

## Зафиксированная семантика

- bounded forward: `0.02..0.10 m`;
- bounded turn: `1..10°`;
- положительный encoder overshoot: `BOUNDED_DISTANCE_LIMIT`, `FAULT`, fault latch;
- micro-turn correction pulses и автоматические retries отключены;
- безопасный недобор может завершиться `PARTIAL_PROGRESS` только после
  остановки моторов и settle;
- `PARTIAL_PROGRESS` возвращает controller в `READY`, не устанавливает latch,
  сохраняет counts/heading и требует нового perception/decision cycle;
- `SUCCESS` не используется для явно недовыполненного поворота;
- `/stop`, `/ack-fault`, `boot_session_id` и idempotency сохраняют прежние
  safety guarantees.

## Физически подтверждено

- bounded forward `0.10 m`: measured `0.100 m`, `SUCCESS`, без overshoot;
- latest `TURN_RIGHT` requested `4°`: один synchronized pulse, около `2°`,
  `PARTIAL_PROGRESS`, `reobserve_required=true`, без overshoot;
- latest `TURN_LEFT` requested `4°`: один synchronized pulse, около `3°`,
  `PARTIAL_PROGRESS`, `reobserve_required=true`, без overshoot;
- clean reboot: `READY`, PWM `0/0`, counts `0/0`, без fault/latch;
- exact 4° micro-turn не заявляется.

Forward evidence относится к более раннему floor test и не объявляется
отдельной validation текущего firmware SHA, если forward-path после теста
изменялся.

## Не подтверждено и ограничения

- operational accuracy bounded micro-turn не подтверждена;
- механическое поведение требует отдельного Arduino compile и финального
  физического retest;
- `PARTIAL_PROGRESS` требует повторного perception measurement до следующей
  команды движения;
- реальные Wi-Fi credentials не входят в Git.

## Следующий безопасный этап

Отдельный change set для интеграции person-approach decision с bounded API,
без автоматического запуска моторов и без нового tuning micro-turn.
