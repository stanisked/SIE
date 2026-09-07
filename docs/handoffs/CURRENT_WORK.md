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

## SIE MVP demo v0: supervised composition

Ветка `feature/sie-supervised-person-approach-demo-v0` добавляет только
supervised demo runner поверх существующих person-depth live runtime,
`PersonApproachDecisionEngine` и bounded dry-run planner. Он не дублирует
perception, temporal stabilization или command planner.

- live runtime открывается только при явном пользовательском запуске CLI;
- runner формирует один JSONL demo record без raw pixels;
- `boot_session_id` передаётся аргументом и всегда содержит
  `boot_session_freshness_verified=false`;
- demo v0 допускает только `ADVANCE`; оба turn status дают
  `TURN_NOT_ENABLED_IN_SUPERVISED_DEMO_V0`;
- первый planned `ADVANCE` переводит runner в
  `AWAITING_OPERATOR_CONFIRMATION`; новых plans и retries нет;
- `network_performed=false` и `motor_command_performed=false` в каждом record;
- каждый финальный record содержит `evidence_window_summary` ровно из пяти
  JSON-safe entries с `cycle_id`, `cycle_status`, `person_status` и
  `measurement_status`, без raw pixels и числовой depth-геометрии;
- HTTP client, socket, executor и firmware changes не добавлены.

Это демонстрационный supervised слой, не operational approval. Следующий gate:
review demo evidence и отдельное явное решение о любой future execution layer.

## Far-field alignment v1

Добавлена offline-only 2D policy для одного человека в
`ar0234_image_frame`. Она принимает explicit `center_tolerance_px`, вычисляет
только image-space offset относительно explicit optical axis и возвращает
`ALIGN_TOWARD_IMAGE_LEFT`, `ALIGN_TOWARD_IMAGE_RIGHT` или
`READY_FOR_RANGE_ACQUISITION`. Zero/multiple/invalid/non-finite/frame-unit
cases дают явный `BLOCKED_NO_ALIGNMENT`.

Policy не содержит depth, метров, base-turn mapping, HTTP, сети, ESP32 или
моторов. Image-left/right остаются семантикой изображения.

## AR0234 alignment preview для yaw-mapping gate

`vision_core/tools/run_ar0234_alignment_preview.py` открывает только approved
AR0234 checked capture в режиме `1920x1200`, `30 FPS`, `MJPG`, buffer `1` при
явном запуске пользователя. Preview валидирует AR intrinsic `camera_matrix`,
principal point и resolution, рисует overlay только на копии кадра и не пишет
frames. `IMAGE_LEFT`, `CENTER_BAND` и `IMAGE_RIGHT` описывают только положение
в image frame, не направление поворота базы. При `PERSON_LOST` или
`MULTIPLE_PERSONS` rolling median очищается и старый bbox не остаётся актуальным.
Для validated OpenCV 4.14 headless runtime окно использует tkinter fallback
с raw PPM byte data;
отсутствие и HighGUI, и tkinter/display даёт явную fail-closed GUI error.

### YAW_MAPPING_RIGHT: PASS

Зафиксирован один физический right-side gate:

- image semantic: `IMAGE_RIGHT`;
- firmware endpoint: `POST /turn-right`;
- requested angle: `4°`;
- terminal state: `PARTIAL_PROGRESS`;
- firmware state after terminal: `READY`;
- `bounded_fault_latched`: `false`;
- `reobserve_required`: `true`;
- motor PWM after terminal: `0`;
- AR0234 optical axis: `cx=997.365537 px`;
- BEFORE rolling median `center_x`: `1353.0 px`, offset `+355.6 px`;
- AFTER rolling median `center_x`: `1256.5 px`, offset `+259.1 px`;
- observed delta: `-96.5 px`.

Conclusion: `IMAGE_RIGHT → firmware TURN_RIGHT` moves the detected person
toward the optical axis.

Evidence limits: this is one physical right-side gate; exact `4°` accuracy was
not confirmed; `IMAGE_LEFT` mapping remains unconfirmed. Policy is not yet
linked to real endpoints. Forward, HTTP executor and autonomous motion remain
absent.

### YAW_MAPPING_BILATERAL: PASS

Physical calibration now confirms bilateral image-to-firmware yaw endpoint
mapping:

RIGHT:

- `IMAGE_RIGHT → POST /turn-right`;
- requested angle: `4°`;
- rolling median `center_x`: `1353.0 → 1256.5 px`;
- delta: `-96.5 px`, toward the optical axis;
- terminal: `PARTIAL_PROGRESS`, then `READY`, `latch=false`, `PWM=0`.

LEFT:

- `IMAGE_LEFT → POST /turn-left`;
- requested angle: `4°`;
- rolling median `center_x`: `608.0 → 721.5 px`;
- delta: `+113.5 px`, toward the optical axis;
- terminal: `PARTIAL_PROGRESS`, then `READY`, `latch=false`, `PWM=0`.

Interpretation: image-frame semantics now have confirmed firmware yaw endpoint
mapping. Every turn still requires a complete stop and re-observation. Exact
angle accuracy is not confirmed. No forward, HTTP executor or autonomous motor
loop is approved.

## Temporal 2D yaw-alignment dry-run v1

`YAW_MAPPING_BILATERAL: PASS` is the physical sign-mapping evidence for this
planner: `IMAGE_RIGHT → /turn-right` and `IMAGE_LEFT → /turn-left`. The new
planner consumes only a local JSONL window of up to five AR0234 image-frame
observations, requires at least four valid `SINGLE_PERSON` records, and uses
the robust median image offset plus MAD. Invalid, multiple, frame/unit mismatch
or unstable windows fail closed with `BLOCKED_NO_TURN`.

Rule clarification from physical evidence, not a threshold change: one
intermediate `PERSON_LOST` is an allowed missing sample when the latest
observation is `SINGLE_PERSON` and at least four valid single-person samples
remain. More than one `PERSON_LOST`, any `MULTIPLE_PERSONS`, or latest
`PERSON_LOST` remains `BLOCKED_NO_TURN`. Median and MAD use only valid
single-person samples; the result exposes their counts, latest status, used
evidence IDs and complete window evidence IDs.

The planner emits only a dry-run semantic `POST` endpoint and `angle_deg=4`.
After a planned turn it enters `AWAIT_REOBSERVATION` and refuses a repeat of
the same evidence window. This does not claim exact `4°` accuracy and adds no
HTTP executor, forward motion, autonomous motor loop, network or device access.

Next physical gate: observe a person at several natural left, center and right
positions. For each position, use only one manually confirmed turn, complete
stop, then a new observation before any further decision.

## AR0234 localization-to-yaw observation adapter

`prepare_ar0234_yaw_observations.py` converts local JSONL from
`run_person_localization_ar0234.py` into the temporal planner input. It derives
the optical axis from AR intrinsic `K` (or the existing `camera_matrix` form),
derives deterministic evidence IDs when absent, and emits no offset for lost,
multiple or invalid-bbox records. Output creation is exclusive by default;
`--overwrite` is explicit.

Physical gate:

1. Record five raw AR0234 frames with a person in a natural position.
2. Run the adapter.
3. Run the temporal yaw dry-run planner.
4. Only for one planned turn, manually verify ESP32 status and manually issue
   exactly that bounded turn.
5. Confirm complete stop, `READY` and `PWM=0`.
6. Record five new frames and repeat observation then planning.
7. No retries, forward motion, correction loop or executor.

### TEMPORAL_YAW_ALIGNMENT_SUPERVISED_CYCLE_001: PASS

One supervised observe-plan-turn-reobserve cycle produced this physical
evidence:

- initial normalized window: four valid `SINGLE_PERSON`, one intermediate
  `PERSON_LOST`, latest `SINGLE_PERSON`;
- initial robust median offset: `-364.115537 px`; MAD: `1.25 px`;
- dry planner selected `POST /turn-left`, `angle_deg=4`,
  `AWAIT_REOBSERVATION`;
- manual bounded command ID: `tempyaw-left-20260907-02`;
- terminal firmware state: `PARTIAL_PROGRESS`, then `READY`,
  `bounded_fault_latched=false`, `PWM=0`, `reobserve_required=true`;
- firmware heading: `+0.0304 rad`, approximately `+1.74 degrees`; exact
  `4-degree` accuracy is not claimed;
- after window: five valid `SINGLE_PERSON`, median offset `-343.365537 px`,
  MAD `12.5 px`;
- observed delta: `+20.75 px`, toward the optical axis;
- the planner still selected `/turn-left`, correctly, because the person
  remained left of center.

`PASS` is only one supervised observe-plan-turn-reobserve sign/effect cycle.
It is not yaw-gain calibration, operational approval, an automatic correction
loop, forward approval or executor approval. The participant may naturally
move, so this is evidence rather than a fixed-pose benchmark.

Next gate: collect bounded evidence from several natural left/right trials
before considering any automatic execution layer.

### TEMPORAL_YAW_ALIGNMENT_SUPERVISED_CYCLE_002_RIGHT: PASS

One supervised right-side observe-plan-single-turn-reobserve cycle produced:

- before window: `5/5 SINGLE_PERSON`, median offset `+249.1344628 px`, MAD
  `2.0 px`;
- dry plan: `POST /turn-right`, `angle_deg=4`, `AWAIT_REOBSERVATION`;
- manual bounded command ID: `tempyaw-right-20260907-01`;
- terminal: `PARTIAL_PROGRESS`, then `READY`,
  `bounded_fault_latched=false`, active command `null`, `PWM=0`,
  `reobserve_required=true`;
- firmware heading: `-0.0405 rad`, approximately `-2.32 degrees`;
- after window: `5/5 SINGLE_PERSON`, median offset `+195.1344628 px`, MAD
  `4.5 px`;
- observed delta: `-54.0 px`, toward the optical axis.

### TEMPORAL_YAW_ALIGNMENT_BILATERAL_SUPERVISED: PASS

Left and right full observe-plan-single-turn-reobserve cycles both produced
image displacement toward center. There was no `FAULT`, no latch and no
nonzero final PWM. Exact yaw gain and `4-degree` accuracy are not calibrated.
There is no auto-loop, forward motion, executor or operational approval. The
current planner correctly keeps proposing a turn while the person remains
outside the `±40 px` center band.

Next software gate: a separate dry-run supervised state machine that joins
far-field 2D alignment to `RANGE_ACQUISITION_REQUIRED` when centered, then to
the existing stereo-depth decision. This documentation commit adds no executor,
HTTP, ESP32 or forward behavior.

## Supervised acquire-range dry-run v1

`TEMPORAL_YAW_ALIGNMENT_BILATERAL_SUPERVISED: PASS` remains the physical
alignment sign/effect evidence. The new offline-only coordinator orchestrates
stages only: existing temporal planned turns become
`AWAIT_OPERATOR_TURN_AND_REOBSERVATION`; centered observations become
`RANGE_ACQUISITION_REQUIRED` until a current valid person-depth record is
available, then `DEPTH_APPROACH_DECISION_REQUIRED` for the existing decision
layer. It does not create a turn, forward command, executor, HTTP request or
device action.

Far-field forward without valid depth provenance remains intentionally
unsupported. Next physical gate: manually align a person to center, then
observe whether Stereo V6 depth becomes valid before any advance planning.

### Shared live-cycle yaw/depth evidence clarification

A sequential depth capture followed by a separate AR capture was correctly
blocked by the coordinator: the latest depth cycle at
`2026-09-07T14:52:26.539039+00:00` was `6.40 s` older than the temporal result
at `2026-09-07T14:52:32.935537+00:00`. The accepted fix is shared
`sie.person_depth_live_cycle.v1` evidence through the yaw adapter, not a
relaxation of freshness policy.

Next physical gate: run one five-cycle person-depth-live capture, adapt it with
`--source-kind person-depth-live-cycle`, run the temporal planner, then pass
the latest depth record and temporal result to the coordinator. This adds no
executor, network, hardware or forward behavior.

## Unified supervised target demo dry-run v1

The runner keeps one existing person-depth live runtime for a single shared
five-cycle window, derives temporal 2D observations through the shared adapter,
then routes only existing temporal, coordinator and approach-decision outputs.
Core stages and records use `target`, `entity`, `observation` and
`measurement`; the current source adapter explicitly declares
`entity_type="person"`. It does not claim arbitrary-object recognition.

Existing centered live-decision evidence is: median `z=2.23577 m`, range
`2.24435 m`, `x=0.02854 m`, with result `ADVANCE 0.10 m`. This remains dry-run
evidence only; no motor command was performed.

Next physical gate after commit: collect one integrated runner output for each
of off-center turn, centered range-required, centered advance and centered
hold cases. Review a separate executor proposal before any physical forward
command.
