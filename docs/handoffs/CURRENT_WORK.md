# Current bounded-motion MVP handoff

## Следующий этап: dry-run bridge

Эта branch-local рабочая копия содержит dry-run bridge между `person_approach`
decision layer и bounded v4.2 API. Bridge формирует только JSON-safe план или
явный block result. Сетевые вызовы и motor commands не добавлены.

- branch: `feature/person-approach-bounded-bridge-dry-run-v1`
- base HEAD: `da3273a8fbd553483293c0af0b5468aaee23315c`
- статус: offline implementation complete, профильные проверки пройдены
- production integration: запрещена до отдельного review и физического approval

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

## Contracts, которые bridge обязан соблюдать

- `person_approach` принимает JSON-safe cycle records и выдаёт только
  `PersonApproachDecision` с явными единицами и `reference_frame`;
- окно решения: последние 5 циклов, latest `SUCCESS`, минимум 4/5 success,
  без `MULTIPLE_PERSONS`, монотонные timestamps, age последнего measurement
  не более 1 секунды;
- `BLOCKED_*` и `HOLD_*` никогда не содержат turn или forward step;
- `ADVANCE` ограничен максимумом `0.10 m`, turn максимумом `10°`;
- bounded API требует свежий `boot_session_id` и новый `command_id`, сохраняет
  idempotency и fault-latch semantics;
- `PARTIAL_PROGRESS` означает остановленные моторы, `READY`, latch false и
  обязательное новое perception/decision cycle до следующей команды;
- bridge остаётся dry-run: не импортирует ROS/Wi-Fi/ESP32 motor control и не
  отправляет сетевые запросы.

## Реализованный dry-run bridge

- planner: `vision_core/person_approach/bounded_bridge.py`;
- локальный CLI: `vision_core/tools/run_person_approach_bounded_bridge_dry_run.py`;
- вход: одно decision, ровно пять live cycle records, обязательный
  `boot_session_id`, опциональный previous terminal outcome;
- выход: только `PLANNED_BOUNDED_COMMAND` или `BLOCKED_NO_COMMAND`, всегда с
  `network_performed=false`;
- `command_id` детерминирован из decision ID, endpoint, canonical parameter и
  boot session. Формат: `pa-` + первые 61 hex-символа SHA-256, ровно 64 ASCII
  символа, что соответствует v4.2 contract `1..64`;
- `PARTIAL_PROGRESS` с `reobserve_required=true` блокирует следующий план;
- firmware, perception, stereo, ROS и live runtime не изменялись.

Локальный Blocker/High review завершён. Найден и закрыт один High: bridge теперь
проверяет соответствие знака `turn_angle_deg` направлению `TURN_LEFT` или
`TURN_RIGHT` до преобразования величины в положительный API-параметр.

Отдельный corrective High закрыт: generation `command_id` ограничен длиной
ровно 64 символа, а unit test проверяет prefix, фактическую длину, ASCII charset
и соответствие firmware pattern.

Пример planned command:

```json
{"result":"PLANNED_BOUNDED_COMMAND","method":"POST","endpoint":"/move-forward","query":{"boot_session_id":"0123456789ABCDEF","command_id":"pa-5db5618d3f15a65991d8f0cc504b46f7e6beec9498a5aac7c7d42f532df6f","distance_m":"0.1"},"network_performed":false}
```

Проверено offline:

- профильные bridge и decision tests: `29 passed`;
- `py_compile` для всех изменённых Python-файлов;
- `git diff --check`;
- JSON serialization с `allow_nan=false`;
- source и credential scan;
- firmware v4.2 не изменён, SHA-256 `.ino` остался
  `19d2bac0f8f46f53e34fcd46f78ea8ca3aadd0985c3ef1de95a5bc7ecb06c28a`.

Следующий gate: отдельный review production integration и физическое approval.
До него bridge нельзя превращать в HTTP client или executor.
