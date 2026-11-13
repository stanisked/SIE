# SIEE / Fast Shadow / Graphify: технический контекст и журнал продолжения

**Версия документа:** 1.0  
**Срез знаний:** 2026-11-12  
**Проект:** Spatial Intelligence Evidence Engine (SIEE; технические идентификаторы `sie` сохраняются)  
**Основная инженерная ветка для этого handoff:** `codex/shadow-turtlesim-follow-20261111`  
**Назначение:** передача накопленного инженерного контекста между ChatGPT, Codex и другими LLM; сохранение истории проверенных решений и экспериментов; использование Graphify для ускорения разработки мобильного MVP.  
**Статус:** рабочий handoff, **не** Engineering Specification, не разрешение на физическое движение и не замена `PROJECT_CONTEXT.md`.  
**Репозиторий:** `https://github.com/stanisked/SIE`  
**Связанный основной чат:** `https://chatgpt.com/share/6af5d654-c47c-83ed-8615-e224c3f8ae10` (в момент подготовки ссылка не открывалась; содержание этого чата не считалось независимо прочитанным).

**Продолжение от 13 ноября:** результаты текущего Fast Shadow и границы успеха добавлены в §11. Пункты §0 и §13 сохраняют исторический срез 12 ноября.

> **Принцип документа:** писать только то, что подтверждено конкретным исходником, тестом, журналом или сообщением оператора. Разделять **ФАКТ**, **ГИПОТЕЗА**, **РЕШЕНИЕ** и **НЕ ПРОВЕРЕНО**. Указывать источник и состояние кода, к которому относится утверждение. Не переписывать прошлое задним числом: новые результаты добавлять в журнал изменений.

---

## 0. Прочитать за 2 минуты: где остановились

1. **Цель пользователя:** работающий мобильный MVP SIEE, а не бесконечный архитектурный аудит. Текущий ближайший сценарий: RGB-выбор человека, stereo-геометрия, актуальное сопоставление цели, решение `FOLLOW_ELIGIBLE`/`PAUSE`, затем отдельная проверка связи с базой. Новые ограничения и бюрократические барьеры без измеримой пользы пользователь не поддержал. Не отменять уже существующие аппаратные защиты и отказ при потере цели.
2. **Актуальная рабочая реализация для продолжения:** Fast Shadow из `sie_shadow_follow_tools`, а не октябрьская V6 и не медленный полноразмерный Dynamic Audit. Используется AR0234 RGB, OV9281 stereo, однокадровая сегментация COCO, детектор верхней части тела v3, exact-cycle binding, front geometry diagnostics и Shadow adapter. `run_shadow_fast_preview.sh` предназначен для camera-only диагностики, запуск двигателей в нём исключён.
3. **Последний известный Pi-полевой журнал:** `shadow_fast_20261112_122628/bridge.jsonl`. Описан в `docs/diagnostics/shadow_uart_candidate_20261029.md`. **117 camera cycles** за ~76 секунд после запуска, **1672** no-new-frame polls, двигатели не запускались; 0 dropped journal records и 0 write errors по отчёту.
4. **Цель была выбрана** в 87/117 циклах (три серии 6, 25, 56; между ними два интервала по 15 `TARGET_UNAVAILABLE`). В 60/87 выбранных циклов масочная глубина проходила диагностическую проверку; 27/87 отказов по MAD.
5. **Блокер №1:** во **всех 87** циклах с выбранной целью YOLO v3 выдал **две рамки выше порога**, тогда как COCO instance selector дал **одного пригодного человека**. Текущий `v3_mask_binding` отказал корректно по своей логике. Не известно, это дублирующая рамка, фон или второй реальный человек. **Все 117 решений Bridge были `PAUSE`; `FOLLOW_ELIGIBLE` не получен.** Исчезновение/возврат выбора маски ещё не означает доказанный reacquisition Shadow-трека.
6. **Блокер №2:** для новой связки v3 + mask возраст от driver-declared SOE до решения превышал **500 мс на всех циклах**: AR median/max **668/757 мс**, OV **668/769 мс**. Host-processing selected-target median **676 мс** (594–771), no-target median **567 мс** (507–658). Это **другая конфигурация**, не опровержение прежних sub-500ms результатов оптимизированного preview.
7. **Ближайший полезный шаг:** взять **этот** последний Fast Shadow pipeline, сохранять в каждом цикле compact v3 boxes/confidence и v3 inference duration, различить duplicate/false positive/second person, затем измерить вклад v3 и выбрать **одно** изменение, снижающее возраст решения. Не повторять механически 30-секундный Dynamic Audit, который уже дал ответ о медленной реализации. Не включать физическое следование на основании журнала с 117 `PAUSE`.
8. **Graphify уже поставлен и проверен локально:** GitHub-граф 4500 nodes / 13379 edges / 290 communities; Pi V6-граф 4286 / 12706 / 257. Это **граф исходного кода**, а не канонический SIEE Evidence Graph.

**Перед следующим экспериментом:** сверить `git status`, актуальный commit, наличие v3 ONNX с нужным SHA, реальный launcher и текущие журналы на Pi. Информация ниже относится к срезу 12 ноября и не доказывает, что в более поздней сборке всё неизменно.

---

## 1. Приоритеты, полномочия и ограничения источников

### 1.1. Иерархия для LLM

Сначала читать:

1. `AI_CONTEXT.md` (version 0.2), прежде всего иерархию источников.
2. `PROJECT_CONTEXT.md` (официальный cross-project snapshot; на 12 ноября техническая сводка охватывала состояние до 10 ноября, с последующим уточнением порядка хранения).
3. Релевантные документы `docs/architecture/` (нормативный статус определять **по заголовку**, а не по имени файла).
4. `AGENTS.md` (операционные правила, не переписывать генератором инструментов).
5. `docs/diagnostics/shadow_uart_candidate_20261029.md` и текущие handoff/журналы для Shadow.
6. Точные файлы реализации, тесты, runtime evidence и выводы оператора.
7. Настоящий Graphify handoff как **навигационную карту, индекс проверок и хронологию**, но не самостоятельное доказательство runtime-состояния.

**Важно:** `PROJECT_CONTEXT.md` обновлять **только по явной команде** пользователя `обнови документ` или `обновить документ`. Изменять одновременно локальную authoritative-копию и текущую приватную копию в Drive. Старые Library-копии не являются актуальным источником автоматически. Создание/обновление **этого** handoff не даёт разрешения менять `PROJECT_CONTEXT.md` или активные калибровки.

### 1.2. Нормативные ориентиры

- `docs/architecture/DOCUMENTATION_STANDARD.md`: ES-000, Approved.
- `docs/architecture/SIE_ARCHITECTURE.md`: конституция архитектуры; в просмотренном файле заголовок `Architecture Foundation (Draft)`; не переименовывать этот статус по предположению.
- `docs/architecture/SIE_RULES.md`: ES-002, Approved; в частности `ARCH-001` (Data Contracts), `ARCH-002` (Vision Core не принимает engineering decisions), `ARCH-006` (Decision Engine использует World State, Knowledge Engine, Task Evaluator), `ARCH-013` (traceability).
- `docs/architecture/SIE_ENGINEERING_PRINCIPLES.md`: ES-003, Approved.
- `docs/architecture/SIE_DATA_MODEL.md`: ES-004, Approved.
- `docs/architecture/SIE_SPATIAL_MODEL.md`: ES-005, Approved.
- `docs/architecture/SIE_EVIDENCE_GRAPH.md`: ES-006, Approved.
- `docs/architecture/data_contracts.md` в проверенном снимке является короткой заглушкой, не считать её исчерпывающим каталогом контрактов.

Канонический поток: `Raw Observation → Observation → Measurement → Interpretation → Fact → World State → Decision → Action`. Экспериментальные Shadow данные **не выдавать** за admitted `Measurement` или автономное решение SIEE. При этом **отдельные операторские эксперименты уже существуют**; не делать из одного `execution_authorized=False` утверждение, будто любой независимо инициированный оператором опыт принципиально запрещён.

### 1.3. Что именно не удалось получить

Ссылка на основной ChatGPT shared thread `6af5d654-c47c-83ed-8615-e224c3f8ae10` **не была доступна для чтения** при подготовке этого файла. Контекст восстановлен по текущей ветке обсуждения, GitHub-снимкам и указанным инженерным handoff. **Не представлять этот документ как полную стенограмму основного чата.** При появлении нового evidence дополнять журнал, а не додумывать пропуски.

---

## 2. Источники кода: три разных состояния, не смешивать

| Метка | Где | Подтверждённое состояние на 2026-11-12 | Роль |
|---|---|---|---|
| `GITHUB_GRAPH` | Ноутбук: `~/dev_ws/sie_graphify_audit` | Ветка `codex/shadow-turtlesim-follow-20261111`, commit `60eeade6472d744868fbce24b2b885586d5034f2`; чистая отдельная копия при создании | Актуальный на момент проверки GitHub-снимок для AST-аудита |
| `PI_V6_SOURCE` | Pi: `/home/elwis/dev_ws/sie_v6_temperature_disabled_mvp` | `codex/sie-ros2-integration-v1`, `591c1e9`, отставание от remote на 6 коммитов; есть локально изменённые и неотслеживаемые файлы | Историческая октябрьская реализация, существующий `install/sie_ros2`; **не** весь ноябрьский Fast Shadow |
| `PI_SHADOW_TOOLS` | Pi: `/home/elwis/dev_ws/sie_shadow_follow_tools` | `codex/shadow-turtlesim-follow-20261111`, `f630eb1` 12 ноября, чистая при проверке | Ноябрьские Shadow-скрипты, launcher, тесты и handoff для MVP |
| `PI_BACKUP_TOOLS` | Pi: `/home/elwis/dev_ws/sie_backup_tools` | `codex/shadow-rgb-detector-20261030`, `dca064f` 11 ноября | Вспомогательная snapshot-копия; не считать активной по названию |

Код на Pi не обязан совпадать с GitHub `60eeade`: на Pi был указан `f630eb1`. Перед сравнением смотреть commit и `git status`, а для локальных незакоммиченных файлов сравнивать **содержимое**, не только commit SHA. `install/sie_ros2` в V6 доказывает наличие сборки, но **не** версию реально запущенного процесса. Во время предварительной диагностики SIEE/ROS процессов в `ps` не было; это не доказывает, что их не запускали раньше.

### 2.1. Скопированные Pi-исходники на ноутбуке

Локально создана отдельная read-only по отношению к Pi копия:

`~/graphify_pi_sources/sie_v6_temperature_disabled_mvp/`

Скопировано `rsync` около **10 480 877 байт** из исходной Pi V6 директории. Перед копированием проверили имена файлов: обнаружили 8 `wifi_credentials.example.h`; все `wifi_credentials*.h` исключены вместе с `.git`, `build`, `install`, `log`, `runtime_artifacts`, `.venv`, `__pycache__`, `.env*`, ключами и бинарниками. **Это не является полным секрет-сканированием содержимого**. Копию не публиковать и не отправлять в Graphify Cloud без отдельной проверки.

---

## 3. Graphify: зачем и что реально сделали

### 3.1. Инструмент

- Установлен `uv` 0.13.7, исполняемый файл `~/.local/bin/uv`.
- Установлен пакет **`graphifyy`**, CLI `~/.local/bin/graphify` и `~/.local/bin/graphify-mcp`.
- Проверенная версия CLI Graphify: **0.9.107**.
- Использован **локальный AST extraction `--code-only`**, без LLM extraction и без Graphify Cloud.
- Не выполнялся `graphify install`, поскольку он может менять агентские инструкции/`AGENTS.md`.

### 3.2. Команды, уже выполненные на ноутбуке

```bash
~/.local/bin/graphify extract \
  "$HOME/dev_ws/sie_graphify_audit" \
  --code-only --out "$HOME/graphify_sie_audit" --max-workers 2
~/.local/bin/graphify cluster-only "$HOME/graphify_sie_audit" --no-label

~/.local/bin/graphify extract \
  "$HOME/graphify_pi_sources/sie_v6_temperature_disabled_mvp" \
  --code-only --out "$HOME/graphify_pi_v6_audit" --max-workers 2
~/.local/bin/graphify cluster-only "$HOME/graphify_pi_v6_audit" --no-label
```

Артефакты:

```text
~/graphify_sie_audit/graphify-out/graph.json
~/graphify_sie_audit/graphify-out/GRAPH_REPORT.md
~/graphify_sie_audit/graphify-out/graph.html

~/graphify_pi_v6_audit/graphify-out/graph.json
~/graphify_pi_v6_audit/graphify-out/GRAPH_REPORT.md
~/graphify_pi_v6_audit/graphify-out/graph.html
```

**Важно:** `graph.html` локальный обзор кода; `GRAPH_REPORT.md` с кластеризацией не заменяет ручную проверку архитектурных инвариантов, runtime-тестов и исходников.

### 3.3. Численные результаты

| Метрика | GitHub graph (`60eeade`) | Pi V6 snapshot (`591c1e9` + local) |
|---|---:|---:|
| Nodes | **4500** | **4286** |
| Edges | **13379** | **12706** |
| Communities | **290** | **257** |
| Поддержанные файлы Pi V6 | — | **489 code files** |
| Неклассифицированные Pi V6 | — | **57**, включая `package.xml`, `setup.cfg`, `.patch` |
| Code files без символов Pi V6 | — | **57** |
| Частично разобранные `remote_stop_observer.h` | **8** | **8** |

Ошибки AST затрагивают `firmware/esp32/.../remote_stop_observer.h` (в том числе строку 19). **Не выводить из отсутствия Graphify-ребра отсутствие C++-поведения или аппаратной защиты.** Парсер мог пропустить часть firmware. Разница 214 узлов/673 связей не равна перечню отсутствующих функций и не показывает качество MVP.

### 3.4. Проанализированный узел и конкретный результат

Проверен узел `Ar0234Ov9281DynamicAssociationAuditNode`:

```bash
~/.local/bin/graphify explain \
  "Ar0234Ov9281DynamicAssociationAuditNode" --all \
  --graph "$HOME/graphify_sie_audit/graphify-out/graph.json"
~/.local/bin/graphify explain \
  "Ar0234Ov9281DynamicAssociationAuditNode" --all \
  --graph "$HOME/graphify_pi_v6_audit/graphify-out/graph.json"
```

- GitHub: **46** непосредственных связей.
- Pi V6: **44** непосредственных связи.
- GitHub-снимок добавил два видимых метода относительно Pi V6: `_association_range()` и `_prepare_stereo_pair()`.
- Сравнение исходников `git diff --no-index` подтвердило детали, не только графовые рёбра.

**Что изменилось в коде:**

1. `shadow_association_range_mode` выбирает `activated` или `diagnostic` для диагностической ассоциации. В `diagnostic` используется отдельный `shadow_diagnostic_range` (в wrapper 0.5–4.5 м), **не меняя** activated Measurement range.
2. `_prepare_stereo_pair()` сохраняет rectified stereo/disparity на opt-in evidence-cycles **до** RGB target selection, поэтому при отказе выбора цели остаётся stereo-evidence.
3. При успешной цели заранее вычисленные disparity и LR-consistency переиспользуются, без второго matching в том же цикле.
4. Добавлена обработка `ExternalShutdownException` при завершении ROS.
5. Старый комментарий о frustum `0.5–4.5 m` остаётся статичным при параметризованном диапазоне: комментарий не использовать как runtime-политику.

**ФАКТ, выявленный через Graphify + исходники:**

```text
Ar0234Ov9281MetricMeasurementNode
    -> Ar0234Ov9281DynamicAssociationAuditNode
        -> Ar0234Ov9281ControlPreviewNode
```

Базовый конструктор создаёт `measurement_publisher` на `/sie/perception/raw_measurement` и `observation_publisher`; диагностический дочерний класс создаёт `audit_publisher`, а `ControlPreviewNode` создаёт `preview_publisher`. В просмотренных переопределённых диагностических `_cycle()` нет вызова `_publish_success()`/`_publish_refusal()` базового класса и не обнаружена прямая команда ESP32. **Наличие унаследованного publisher само по себе не доказывает фактическую публикацию Measurement.**

**Тестовые границы:**

- `vision_core/tests/test_shadow_stereo_before_target.py`: AST-извлечение `_cycle`, `_prepare_stereo_pair`, `_frustum_associate`; проверены сохранение stereo до отказа цели и отсутствие повторного matching.
- `vision_core/tests/test_live_shadow_local_surfaces.py`: AST-извлечение отдельных методов, diagnostic-only flags, timing/refusals.
- Проверка этих методов **не** эквивалентна полной инициализации настоящего ROS Node; в просмотренных файлах нет отдельного интеграционного теста наличия/отсутствия publisher на реальном ROS graph. Это **возможный пробел**, не текущий главный MVP-блокер.

### 3.5. Как использовать Graphify дальше, а как нельзя

**Использовать:** поиск зависимостей и потребителей, impact analysis перед точечным ускорением pipeline, выделение циклических зависимостей, проверка мест повторного вычисления disparity/segmentation, сравнение исходников разных коммитов.

**Не использовать:** как источник истины о работающей версии на Pi, качестве калибровки, безопасности ESP32, количестве ROS subscribers в runtime или канонической SIEE Evidence Graph. Статический граф не показывает динамические строки топиков, shell launch, UART POST, ROS runtime без дополнительного чтения кода и журналов.

**Новый полезный аудит в будущем:** построить **третий** граф именно по checkout `PI_SHADOW_TOOLS` (после выяснения commit/status), затем связать изменившиеся области со свежими Fast Shadow runtime-journals. Уже построенные два графа не надо строить заново без новой причины.

---

## 4. Реальная архитектура Fast Shadow MVP на 12 ноября

### 4.1. Camera-only передний контур

```text
AR0234 RGB (1920×1200)
    | current frame / timestamp / provenance
    +--> segmentation COCO YOLO11n-seg (bbox + actual instance mask)
    +--> independently pinned YOLO11 upper-body v3 boxes
              |
              +--> v3-to-instance binding (unique match, same RGB/cycle)
                             |
OV9281 stereo (combined 2560×800, physical right first)
    --> calibrated physical-left/right rectification
    --> disparity + LR consistency / mask-owned depth
                             |
              exact-cycle stereo + RGB/mask binding
                             |
              front geometry / freshness / quality diagnostic
                             |
              Shadow adapter -> FOLLOW_ELIGIBLE or PAUSE (diagnostic)
                             |
              camera-only journal (no UART/motor commands)
```

**AR0234** единственный semantic target source для нового pipeline. **OV9281** отвечает за геометрию. Нельзя подменять физическую маску прямоугольником детектора. При двух пригодных людях или неоднозначном сопоставлении адаптер отказывает. Цель MVP сейчас: получить стабильное владение целью, пригодную глубину и своевременное решение; другие классы объектов добавляются сменным адаптером позже.

### 4.2. Основные файлы

| Назначение | Файл / модуль |
|---|---|
| Полный diagnostic audit | `ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py` |
| Оптимизированный preview | `ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py` |
| Camera-only Fast Shadow launcher | `vision_core/tools/run_shadow_fast_preview.sh` |
| Bridge из diagnostic topics | `vision_core/tools/shadow_live_bridge.py` |
| Связка Fast Shadow → experimental observation | `vision_core/tools/shadow_fast_adapter.py` |
| Сводка JSONL | `vision_core/tools/summarize_shadow_fast_preview.py` |
| Проверка свежести/дальности цели | `vision_core/tools/shadow_range_gate.py` |
| Сегментный state machine | `vision_core/tools/shadow_supervisor.py` |
| Отдельный экспериментальный UART runner | `vision_core/tools/run_shadow_uart.py` |
| UART транспорт к ESP32 | `vision_core/tools/uart_http_transport.py` |
| Full-resolution Shadow bridge launcher | `vision_core/tools/run_shadow_live_follow.sh` |
| Тесты точного связывания | `vision_core/tests/test_shadow_fast_adapter.py` |
| Тесты live bridge | `vision_core/tests/test_shadow_live_bridge.py` |
| Тесты stereo-before-target | `vision_core/tests/test_shadow_stereo_before_target.py` |
| Основной экспериментальный handoff | `docs/diagnostics/shadow_uart_candidate_20261029.md` |

В `run_shadow_fast_preview.sh` используются native V4L2 latest-frame capture, half-resolution stereo `640x400`, target crop, single-segmentation backend, v3 match или segmentation semantic-source, ONNX Runtime, deferred/local diagnostic options и bridge-решения. Shell wrapper **отвергает** `--execute`; сам bridge для `--source fast_mask --execute` также завершает работу с отказом. Эти проверки подтверждены просмотренным кодом, но не являются результатом аппаратного измерения.

### 4.3. Исполнительный контур: отделять от camera-only измерений

Отдельный `run_shadow_live_follow.sh` запускает `DynamicAssociationAuditNode` и `shadow_live_bridge.py`, передавая `"$@"`. По умолчанию Bridge только выводит диагностические решения, **без** UART. В режиме `--execute` и `--source dynamic_audit` программный путь может открыть UART и передать сессию в `run_shadow_uart.py`; доступность этого пути **не** равна факту физического запуска, валидности человека или допуска canonical Measurement. `max_segments` там необязателен, а в отличие от V7 One Step одноразовый interactive token не запрашивается. Это различие исследовано, но пользователь отверг предложение отвлекаться сейчас на новые бюрократические ограничения.

Отдельный операторский `run_v7_supervised_one_step.py` использует `control_preview`, требует явный `--execute` плюс token оператора, заявленный front clearance, повторную проверку 500-ms age, один 0.1-m request и stop attempt. **Не смешивать** с Shadow Fast pipeline и не переносить его показатели на другой эксперимент.

Обязательное различение: **канонический** профиль `runtime_artifacts/ar0234_ov9281_decision_chain_v1/.../activation.json` на Pi прочитан оператором и содержит:

```json
{
  "status": "ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY",
  "authorization": {
    "execution_authorized": false,
    "actuator_bridge": "DISABLED_PHASE_1",
    "metric_measurement_range_m": [0.5, 2.0]
  }
}
```

Это **выжимка прочитанных полей**, не полный JSON. Canonical Measurement/execution profile **не расширен** диагностическим диапазоном 0.5–4.5 м. Отдельный операторский trial не следует автоматически объявлять нарушением только на основании этого поля. Текущее направление работы: **не превращать Graphify в кампанию за новые запреты**, а измерять реальный pipeline, сохраняя уже имеющиеся fail-closed условия.

---

## 5. Проверенная калибровка и аппаратура

| Объект | Подтверждённая привязка |
|---|---|
| Хост | Raspberry Pi 5, hostname `piecake`, ROS 2 Jazzy, headless |
| RGB | AR0234 UVC 1920×1200; AR0234 v5 intrinsic SHA-256 `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` |
| Stereo | Dual OV9281 UVC combined 2560×800; активная Pi v7 SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52` |
| Экстинсика | AR0234 → OV9281 physical-left SHA-256 `10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc` |
| Семантика потоков OV9281 | Первая половина combined-frame = **physical right**, вторая = **physical left** |
| Источник семантики | AR0234, а не отдельный MediaPipe на OV9281 |
| Диапазон v1 Measurement на Pi | **0.5–2.0 м**, `execution_authorized=False` |
| Расширенный Shadow frustum | **0.5–4.5 м**, диагностический; не автоматическая активация Measurement |
| Front geometry | В Shadow-пробах использованы операторский pitch +15°, setback 0.210 м и отдельный оценочный optical-to-rim компонент ~0.0123–0.0125 м. Эти значения не образуют доказанной физической границы ошибки |
| ESP32 | Отдельная firmware `v4_2_uart_shadow_v1_candidate`, UART HTTP facade, bounded segment/PAUSE/RESUME/STOP; физические испытания подтверждают отдельные команды, **не** законченный person-following loop |

V8 и v8b OV9281 были отвергнуты как замена активной v7. Не подменять калибровку и не вводить эмпирический масштаб только ради красивого числа глубины. Термодрейф disparity на старте наблюдался ранее и не устранён простой сменой модели.

Контекст также включает планируемый LiDAR D500 (STL-19P) для локализации/пространственных ограничений и позднее IMU/VIO. **Наличие LiDAR в планах не означает работающий fused runtime.**

---

## 6. Хронология подтверждённых экспериментов, важная для MVP

### 6.1. До ноября: база для сравнения

- **2026-10-09 — 2026-10-11:** stereo v7 относительная/временная проверка и отдельные 25 live shadow mask cycles; 22/22 масок сохранены для кадров с целью, три цикла `NO_TARGET`. При active 0.5–2.0 м все 22 первоначальных ассоциации отказали по range при глубине около 2.3 м; offline diagnostic 0.5–4.5 м дал **16/22** coherent masked-depth, 6 MAD refusals. Это не доказательство физической точности 5%.
- **2026-10-14/15:** создан оптимизированный control-input preview с half-resolution stereo, segmentation-mask depth и native capture. Отдельные прежние замеры fast path в документации: run `20261015_190656` driver-to-result median/max AR **266.59/275.70 мс**, OV **266.79/287.59 мс**; host cycle **263.54/299.43 мс**. Более поздний 99-cycle control preview: **98/98** steady coherent driver-age до 250 мс, host median/max **195.3/242.2 мс**. **Важно:** эти показатели получены **до** текущего дополнительного v3-to-mask binding и не переносить их на финальный Fast Shadow.
- **2026-10-29:** flashed Shadow UART candidate и lifted-wheel PAUSE/RESUME, завершающий STOP. Проверены отдельные механизмы ограниченного движения и остановки; не доказано сопровождение человека.
- **2026-11-10:** independent reviewed sampled RGB dataset: **120** AR0234 frames (32 empty, 26 single, 62 two-person; 150 labelled persons), v2/v3 обнаружили человека на всех **88** positive и 0 из 32 empty. На 62 two-person кадрах v2 дала ≥2 boxes на **56/62**, v3 на **60/62**. Это **другая** серия, не 117-cycle live test. Отдельная двухчеловеческая stereo-серия: 29 RGB frames с двумя v3 detections, из них 19 имеют disparity; нельзя утверждать стабильную отдельную depth surface для обоих людей по всей серии.

### 6.2. 11 ноября: Shadow UART и медленный Bridge

- **Операторский короткий floor step:** firmware `v4_2_uart_shadow_v1_candidate`, одна команда `SHADOW_START` на **0.020 м**; encoder наблюдал приблизительно **0.0179 м left / 0.0148 м right** при остановке, PWM=0 и terminal STOP; это не независимая калибровка stopping distance.
- **Первый full-resolution Dynamic Audit → Bridge:** 13 циклов примерно за 30 с; все `ASSOCIATION_UNAVAILABLE`, processing около **1.5–1.8 с**, parallel matching около **1.0–1.2 с**; UART не открывался. Первоначальное отображение Bridge скрывало исходную причину отказа.
- **Четырёхцикловая диагностика range:** текущий RGB-объект есть; 5 183–5 700 LR-consistent samples, но ни одного sample внутри **activated 0.5–2.0 м**; stereo central depth ~3.2–3.7 м. Причина `FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE`, не отсутствие RGB или disparity. Capture-to-bridge возраст **1.39–1.66 с**.
- **Отдельный 0.5–4.5-м диагностический range:** 4 текущие RGB-цели, одна plausible spatial component и один depth cluster; **1/4** `ASSOCIATED_DIAGNOSTIC_ONLY`, остальные три отказа MAD **0.0799 / 0.0623 / 0.0561 м**. Bridge `PAUSE` даже при одной ассоциации из-за возраста **1.29–1.37 с**. Исполнение не запускалось.
- **Из этого следует:** не повторять медленный full-resolution bridge в надежде получить <500 мс без изменения вычислительного пути. Уже существующий optimized Fast Preview должен быть точкой продолжения.

### 6.3. 12 ноября: первый Fast Shadow Pi camera-only field journal

**Сводка из `docs/diagnostics/shadow_uart_candidate_20261029.md`, привязанная к операторскому файлу:**

`/home/elwis/.local/state/sie/evidence/shadow_fast_20261112_122628/bridge.jsonl`

| Показатель | Подтверждённый результат |
|---|---:|
| Camera cycles | **117** |
| Длительность после startup | **~76 с** |
| Polls без нового camera cycle | **1672** |
| Сохранённых target-selected cycles | **87** |
| Три target-selected серии | **6, 25, 56** |
| Два интервала `TARGET_UNAVAILABLE` | **15 + 15** циклов |
| Mask-depth coherent из выбранных | **60/87** |
| Mask-depth MAD refusal | **27/87** |
| V3 detections при выбранной цели | **2 boxes на каждом из 87** |
| COCO eligible person instances | **1** в тех же 87 циклах |
| Bridge `FOLLOW_ELIGIBLE` | **0** |
| Bridge `PAUSE` | **117** |
| AR declared SOE-to-decision median/max | **668 / 757 мс** |
| OV declared SOE-to-decision median/max | **668 / 769 мс** |
| Host processing selected median (range) | **676 мс (594–771)** |
| Host processing no-target median (range) | **567 мс (507–658)** |
| Upstream journal drops / write errors | **0 / 0** |
| Motor execution | **Нет** |

**Интерпретация:** segmentation вновь находила цель после пропаданий, но Shadow target ownership/reacquisition не подтверждён: v3-to-COCO binding отказал во всех выбранных циклах, и последующая логика не сформировала `FOLLOW_ELIGIBLE`. Нельзя автоматически записывать три серии как один успешный трек.

**Неизвестно:** почему v3 выдал две рамки при одном COCO instance; в сохранённом Bridge JSONL **нет** достаточных координат и confidence каждой v3 рамки для такого вывода. Возможно, это duplicate detection, background false positive или второй человек, но **ни один вариант не подтверждён**. Операторские фазы видео/сцен могут быть нужны для интерпретации.

**Тайминг:** 500 мс это экспериментальный бюджет driver-declared SOE→Bridge decision, не физически измеренная exposure→ESP32 acceptance. Его превышение в этом прогоне подтверждено, но нельзя распространять его на прежние fast-preview конфигурации без v3.

---

## 7. Что мы узнали в ходе Graphify-расследования потребителей топиков

Статический граф наследования не показал всю runtime-цепочку, поэтому проверили строки ROS-топиков и shell-wrapper.

**Публикуемые диагностические темы:**

- `/sie/diagnostics/ar0234_ov9281/dynamic_association`
- `/sie/diagnostics/ar0234_ov9281/control_preview`
- опционально `/sie/diagnostics/ar0234_ov9281/local_surface_async`

**Потребители `control_preview`:**

- `vision_core/tools/run_v7_supervised_one_step.py`
- `vision_core/tools/shadow_live_bridge.py` при `--source fast_mask` (но engine execution для fast_mask явно заблокирован).

**Потребитель `dynamic_association`:** `vision_core/tools/shadow_live_bridge.py` при `--source dynamic_audit`.

**Скрипты запуска:** `run_shadow_fast_preview.sh` и `run_shadow_live_follow.sh` являются **разными** окружениями и путями. Упоминание `--execute` в Bridge не означает, что fast-preview launcher его пропускает. А `run_v7_supervised_one_step.py` использует отдельное операторское подтверждение и отдельный сценарий.

**Наблюдение аудита:** `DynamicAssociationAuditNode` и `ControlPreviewNode` технически наследуют base Measurement publisher. В просмотренном коде они публикуют диагностические объекты; факт запрещённого Measurement не установлен. В конце аудита обнаружили возможность отдельного экспериментального Shadow `--execute`, но сделали важную корректировку: каноническое `execution_authorized=False` и существование независимо авторизованного операторского trial не являются автоматически противоречием. Новых административных барьеров без доказанной пользы для MVP не вводить.

**Подтверждение на Pi:** wrapper `run_shadow_live_follow.sh` существует в `PI_SHADOW_TOOLS`, но не найден `shadow_live_bridge.py` в `PI_V6_SOURCE`. В обоих присутствует `run_v7_supervised_one_step.py`. Сверять с конкретным checkout, а не с общим названием `SIE`.

---

## 8. Текущие блокеры и приоритетная последовательность

### P0 — V3 duplicate/ambiguous detections

**ФАКТ:** у single COCO eligible instance было две v3 boxes в каждом из 87 selected-target циклов; Bridge отказал по binding. В `vision_core/tools/summarize_shadow_fast_preview.py` уже предусмотрен подсчёт `v3_detection_counts`, `v3_raw_detection_counts`, `v3_inference`, `driver_age_to_decision_ms`. В `shadow_live_bridge.py` имеются поля `v3_detections` и `v3_inference_s` для preview row, но журнал рассмотренного прогона не содержал нужных box coordinates/confidence.

**Следующее действие:** проверить на текущем Pi checkout, записываются ли полные compact boxes и runtime в **каждую** строку нового JSONL, не включая двигатели. Сравнить эти рамки с COCO instance bbox и операторскими интервалами (один человек / ушёл / вернулся / двое). Результат классифицировать:

- `DUPLICATE_BOX_OF_SAME_PERSON` — если пересечение/координаты и видео подтверждают одну физическую цель;
- `FALSE_POSITIVE_BACKGROUND` — если подтверждён фон;
- `SECOND_PHYSICAL_PERSON` — если есть второй человек;
- `UNDETERMINED` — если данных недостаточно.

**Не предлагать вслепую:** ослабить правило двух лиц, автоматически отбросить вторую рамку, поменять NMS/threshold или считать единственную COCO-маску доказательством отсутствия второго человека. Сначала измерить причину на сырых boxes.

### P0 — Latency after adding V3

**ФАКТ:** median declared SOE-to-decision 668 мс, host selected processing 676 мс; все циклы превысили 500 мс. Но предыдущий pipeline без нового v3 сочетания был быстрее.

**Следующее действие:** получить per-cycle `v3_inference_s`, segmenter/RGB/decode/stereo timing и сопоставить с текущим timeline; построить распределения по target/no-target. Потом проверить **одно** изменение с измеримым прогнозом: параллельность/повторное использование декодированного RGB, inference backend, постпроцессинг или устранение лишнего синхронного ожидания. Нельзя заранее назначать bottleneck без профиля.

### P1 — Target ownership и reacquisition

Подтверждение выбора сегментационной цели после исчезновения **не равно** связному Shadow track. После устранения ambiguous v3 binding проверить exact same-cycle ownership и причины переходов `FOLLOW_ELIGIBLE ↔ PAUSE`, отдельно по сценам с одним и двумя людьми и реальным re-entry.

### P1 — Stereo surfaces and front distance

Сохранить отдельное рассмотрение MAD refusals **27/87**, connected components и depth-layer ownership. Не интерпретировать MAD как абсолютную ошибку дальности. Не считать операторские геометрические offsets полностью измеренными. Для мобильного сценария понадобится LiDAR/odometry fusion, но это не повод останавливать диагностику RGB/Stereo.

### Не P0 на данном этапе

- Создание дополнительных ADR ради уже известного кода.
- Перестройка наследования MeasurementNode без установленного runtime regression.
- Повторение старых 1.5–4.0 м checkerboard sweep без нового проверяемого вопроса.
- Установка Graphify Cloud, автоматическое изменение `AGENTS.md`.
- Полный рефакторинг runtime до получения работающего `FOLLOW/PAUSE` на камерах.

---

## 9. Рекомендуемый цикл работы с кодом и evidence

1. **Одна проблема за раз.** Сначала коротко формулировать измеримую гипотезу и какой результат отличит её от альтернативы.
2. **Сверить код и оборудование.** `git status --short --branch`, commit SHA, используемый launcher, exact ONNX SHA, calibrated artifact SHA, ROS/Pi окружение, текущие devices. Не делать `git pull`, rebuild или change active Pi silently.
3. **Read-only анализ перед тестом.** Graphify (`explain`, `affected`, source diff), `rg`, чтение существующего JSONL, тестов и prior evidence. Не просить пользователя повторить уже выполненный эксперимент.
4. **Минимальный тест без движений.** Сохранить raw audit и компактный Bridge journal, frame ids/cycle ids/hashes. Не смешивать данные с разных конфигураций.
5. **Сравнение до/после.** Отдельно upstream camera processing, driver-to-decision age, v3 stage, segmentation mask, binding refusals, stereo depth support, target continuity, `FOLLOW/PAUSE`.
6. **Запись в этот handoff.** Append new journal record, источники, конфигурацию, подлинный commit и решение. Не формулировать гипотезу как принятую политику.
7. **Не менять `PROJECT_CONTEXT.md` без пользовательской команды.** Если изменение реально затронуло approved contract, обсуждать отдельно; этот файл сам по себе не меняет статус архитектуры.

### Метрики, которые сравнивать по каждому будущему журналу

```text
session_id / exact source journal / commit / launcher / model SHA
camera_cycles / elapsed_s / no_new_frame_ticks
selected_eligible_count / single / multiple / unavailable
v3_count_distribution / v3 box coords+scores / match status
mask-depth coherent / refused / reason histogram
AR and OV driver-SOE -> Bridge decision p50/p95/max
host processing p50/p95/max; v3 inference p50/p95/max
FOLLOW_ELIGIBLE / PAUSE / other decisions
track IDs and changes across loss/reacquisition
journal drops/write_errors / controller_enabled / motor command count
```

**Никогда** не считать два значения age эквивалентными, если одно `driver-declared SOE→bridge decision`, а другое `host start→result ready` или `exposure→ESP32`. В документах указывать измерительную точку и конфигурацию.

---

## 10. Как другие LLM должны пользоваться и пополнять этот документ

### 10.1. Инструкция при открытии новой сессии

> Ты продолжаешь SIEE Fast Shadow MVP. Прочитай `AI_CONTEXT.md`, `PROJECT_CONTEXT.md`, `AGENTS.md`, затем этот `SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md` и свежий `docs/diagnostics/shadow_uart_candidate_20261029.md`. Проверь актуальный branch/commit на Pi и соответствие источников. Не повторяй уже выполненный 117-cycle fast test. Главная задача: выяснить, почему YOLO v3 выдаёт две рамки при одном COCO eligible instance, измерить добавленную v3 latency, довести camera-only `FOLLOW_ELIGIBLE` / `PAUSE` до проверяемого поведения. Работай по одному шагу, ожидай вывода оператора. Все факты связывай с первичными журналами, файлами и commit. Не меняй `PROJECT_CONTEXT.md` без команды `обнови документ` / `обновить документ`. Не принимай Graphify graph за SIEE Evidence Graph. Не добавляй новые бюрократические ограничения вместо проверки MVP.

### 10.2. Где хранить и как сохранять историю

**Рекомендуемое место в repo:** `docs/handoffs/SIEE_GRAPHIFY_MVP_HANDOFF_20261112.md`. При последующих обновлениях сохранять **одно и то же имя/ID**, не плодить разрозненные `FINAL_v2` и `latest2`. При долгом развитии можно переименовать в нейтральное `SIEE_GRAPHIFY_MVP_HANDOFF.md` **явным отдельным изменением**, оставив историю Git.

В конце файла поддерживать **append-only журнал**. Основные sections 0/8 допускается обновлять как текущий digest, **но** сначала добавить событие с точным evidence и не удалять старые результаты. Фиксировать commit SHA файла и все source SHA при доступности. Текст в markdown должен оставаться читаемым без Graphify, ChatGPT или внешнего сервиса.

**Правило неопределённости:** если нет оригинального JSONL, написано `по сводке ...`, а не `самостоятельно перепроверен raw journal`. Если модель не может прочитать первичный файл, явно отмечать, что результат взят из проверенного handoff. Не выдумывать отсутствующие метрики, timestamp и filenames.

### 10.3. Шаблон одной записи журнала

```markdown
### YYYY-MM-DD HH:MM [Europe/Moscow] — Короткий заголовок
- **Цель:** ...
- **Источник:** repo/branch/commit; Pi checkout; launcher/args; source journal path + SHA-256, если известен.
- **Проверено:** ...
- **Метрики:** denominator, count, p50/p95/max, exact timing definition.
- **ФАКТ:** ...
- **ГИПОТЕЗА:** ...
- **РЕШЕНИЕ оператора:** ... / не принято.
- **Оставшийся вопрос:** ...
- **Следующий один проверяемый шаг:** ...
- **Изменены файлы:** ... / нет.
- **Влияние на калибровку/Measurement/motors:** ... / без изменений.
```

### 10.4. Правило ответственности за обновление

LLM должна предлагать запись после завершения осмысленного эксперимента и **вносить её при явном запросе пользователя на обновление этого handoff**. Обычная переписка сама по себе **не** гарантирует, что Markdown-файл в GitHub/Drive обновился. В последующих чатах нужен доступ к самому файлу или прикреплённая актуальная версия. Фраза `используй handoff` не равнозначна автоматической синхронизации между чатами.

---

## 11. Журнал развития этого handoff (append-only)

### 2026-11-12 — Первая фиксация Graphify-аудита и состояния Fast Shadow MVP

- **Создан документ** для передачи контекста между LLM в основной ветке `codex/shadow-turtlesim-follow-20261111`.
- **ФАКТ:** локальная установка Graphify 0.9.107, два построенных AST-графа: GitHub 4500/13379/290 и Pi V6 4286/12706/257.
- **ФАКТ:** сравнение `Ar0234Ov9281DynamicAssociationAuditNode` выявило `_association_range` и `_prepare_stereo_pair`, разделение diagnostic/activated диапазонов и сохранение stereo evidence при RGB refusal.
- **ФАКТ:** исходники показывают наследование MetricMeasurement → DynamicAudit → ControlPreview и отдельные скрипты-потребители диагностических ROS topics. Из этого **не** следует, что запрещённый Measurement реально публиковался.
- **ФАКТ:** на Pi профиле 0.5–2.0 м `execution_authorized=False`, `actuator_bridge=DISABLED_PHASE_1`; экспериментальный Shadow bridge отдельный путь. Пользователь отклонил предложения превращать MVP-работу в дополнительные барьеры и поддержал фокус на pipeline/timing/real experiments.
- **ФАКТ по инженерному журналу:** 117-cycle Fast Shadow camera-only test показал 87 selected-target frames, 60 coherent mask-depth, 117 `PAUSE`, конфликт 2 v3 boxes vs 1 COCO eligible instance и >500-ms decision age.
- **ГИПОТЕЗЫ:** дублирующие v3 boxes либо false positives/second person; дополнительное v3 inference или синхронное ожидание создаёт latency. Ни одна не принята без raw per-cycle evidence.
- **Следующий шаг:** собрать компактные v3 boxes/confidence и per-cycle `v3_inference_s`, локализовать конкретное узкое место Fast Shadow, измерить одно улучшение.
- **Ограничение источника:** исходная shared ChatGPT conversation по URL не была прочитана; документ не заменяет её стенограмму. Raw `shadow_fast_20261112_122628/bridge.jsonl` в текущем рабочем контексте не был заново перечитан; числа получены из `docs/diagnostics/shadow_uart_candidate_20261029.md`.

### 2026-11-13 — Локальная поверхность проходит решение текущего кадра

- **ФАКТ, источник:** Pi evidence `/home/elwis/.local/state/sie/evidence/shadow_fast_20261113_181718`, архив оператора `shadow_fast_20261113_181718_review.tar.gz` (Library `libfile_47c6587477708191a824fbb039ae0067`) с `bridge.jsonl` и 64 точными audit JSON. Прогон `MOTOR_OFF_DIAGNOSTIC`, без команды моторам.
- **ФАКТ:** все 64 camera cycles дали `FOLLOW_ELIGIBLE`: 31 `CURRENT_FAST_MASK_OBSERVATION` при когерентном общем MAD и 33 `CURRENT_STABLE_LOCAL_PREVIEW` при `DEPTH_MAD_EXCEEDED`. Во всех 33 последних audit содержит текущий `shadow_target_bearing_deg` и однозначный совпавший RGB track. Один track ID на все 64 цикла; ни исчезновения цели, ни сцены с двумя людьми в этом прогоне нет.
- **ФАКТ:** максимальный возраст по driver timestamp до решения: AR0234 337.188 мс, OV9281 333.264 мс; физическое время экспозиции независимо не проверено. Локальный stable-front имел 2619–4008 точек, оценка расстояния от фронта в Bridge 2.174–2.613 м. Это диагностическая оценка, а не измеренный допуск физической поверхности человека.
- **РЕШЕНИЕ:** этап MVP «решение по текущему кадру с локальной поверхностью вместо общего MAD как единственного блокирующего признака» признан успешным для motor-off Fast Shadow. Это не допуск `Measurement` и не разрешение на движение.
- **Уточнение диапазона:** расширенный условный supervised-профиль v2 имеет `[0.5, 4.5]` м, SHA-256 `c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9`; он уже зафиксирован в `PROJECT_CONTEXT.md` и не разрешает динамическое движение сам по себе. Однако Fast Shadow launcher в этом прогоне загрузил старый v1 с `[0.5, 2.0]` м, а диапазон preview установил `[0.5, 4.5]` м. Поэтому `activated_measurement_range_m` ошибочно показывал `[0.5, 4.5]`, тогда как сохранённый `activated_depth_range_m` равен `[0.5, 2.0]`. Все 64 оценки 2.174–2.613 м лежат внутри диапазона v2 и вне загруженного v1. Коммит `f25a8aba6ffceff8556c2dcfc6d4a2a549f39e44` исправляет только запись первого поля, не меняя профиль. Предыдущий коммит `04ada455c5d57fbb2454abfe1194e88e3bad1022` обеспечил вычисление угла на MAD-кадрах. Новое поле после `f25a8aba` на Pi ещё не проверялось.
- **Следующий шаг MVP:** сопоставить локальную глубину с реальным зазором от фронта тележки на рабочем расстоянии около 2.5–2.65 м, затем явно увязать Fast Shadow с условным v2 диапазоном 0.5–4.5 м и отдельно решить применимость профиля к динамическому следованию. Не возвращаться к подбору общего MAD или повторной проверке скорости уже работающего motor-off контура.

### 2026-11-13 — Mask-only launcher больше не зависит от неиспользуемой YOLO v3

- **Источник:** branch `codex/shadow-turtlesim-follow-20261111`, implementation commit `fcb1959fc637e6cfdbec3659edb1d1d6a650c0b2`; исходный отказ оператора при подготовке v2-прогона: проверенная v3 с SHA-256 `fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5` отсутствовала на Pi, а неаутентифицированная Drive-загрузка вернула 920419 байт с SHA-256 `efecc7c072e7b9fd598cd44099c078e7f641859c8e882de35cdf9fb845dc9588` и была корректно отвергнута до установки.
- **ФАКТ:** прогон `shadow_fast_20261113_181718` использовал `semantic_source=segmentation_mask`; во всех 64 camera rows поля v3 отсутствуют, `rgb_detector_s=0`, семантика и track получены из текущей COCO instance mask с SHA-256 модели `2f6ce726f47b7d0eb276d29e11df9fc181758a8f8eb2fb38d13558f64b62bdbf`.
- **Изменение:** диагностический mask-only launcher явно отключает базовый AR v3 detector и больше не требует `SHADOW_RGB_MODEL_PATH`. Строгий `v3_mask_binding` путь по-прежнему требует модель v3 и проверяет точный SHA-256. Базовый MeasurementNode и DynamicAudit сохраняют detector включённым по умолчанию; отключение разрешено только явно обозначенному diagnostic subclass.
- **Проверка:** `py_compile` двух изменённых ROS-модулей, `bash -n` обоих Fast Shadow launcher, две новые launcher contract-проверки и `git diff --check` прошли. Полный pytest в рабочей среде не запускался: модуль `pytest` отсутствует.
- **Влияние:** калибровка, v2 activation profile, mask/stereo алгоритм, решения `FOLLOW_ELIGIBLE`/`PAUSE`, Measurement и motor boundary не изменены. Следующий Pi-прогон остаётся motor-off и проверяет профиль v2 на уже работающем segmentation-mask пути без фиктивной зависимости от v3.

---

## 12. Первичные указатели, проверенные ссылки и provenance

### Source-of-truth и нормативные документы

- `AI_CONTEXT.md` (version 0.2): `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/AI_CONTEXT.md`
- `PROJECT_CONTEXT.md`: `https://github.com/stanisked/SIE/blob/f630eb1/PROJECT_CONTEXT.md`
- Приватная текущая Drive-копия `PROJECT_CONTEXT.md`: `https://drive.google.com/file/d/1mRj9UluoMptShOMPMz4h_MbCFczlVn0o/view`
- `AGENTS.md`: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/AGENTS.md`
- ES-002 `SIE_RULES.md`: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/docs/architecture/SIE_RULES.md`
- ES-006 `SIE_EVIDENCE_GRAPH.md`: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/docs/architecture/SIE_EVIDENCE_GRAPH.md`

### Shadow и recent experiment

- Shadow handoff `docs/diagnostics/shadow_uart_candidate_20261029.md`: `https://github.com/stanisked/SIE/blob/f630eb1/docs/diagnostics/shadow_uart_candidate_20261029.md`
- `run_shadow_fast_preview.sh`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tools/run_shadow_fast_preview.sh`
- `shadow_live_bridge.py`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tools/shadow_live_bridge.py`
- `shadow_fast_adapter.py`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tools/shadow_fast_adapter.py`
- `run_shadow_uart.py`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tools/run_shadow_uart.py`
- `summarize_shadow_fast_preview.py`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tools/summarize_shadow_fast_preview.py`
- `test_shadow_fast_adapter.py`: `https://github.com/stanisked/SIE/blob/f630eb1/vision_core/tests/test_shadow_fast_adapter.py`
- Raw Pi journal, **не загружен сюда**: `/home/elwis/.local/state/sie/evidence/shadow_fast_20261112_122628/bridge.jsonl`

### Graphify investigations

- Base metric node: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/ros2/sie_ros2/sie_ros2/ar0234_ov9281_metric_measurement_node.py`
- Dynamic audit: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py`
- Control preview: `https://github.com/stanisked/SIE/blob/60eeade6472d744868fbce24b2b885586d5034f2/ros2/sie_ros2/sie_ros2/ar0234_ov9281_control_preview_node.py`
- ROS integration tests: `vision_core/tests/test_shadow_stereo_before_target.py`, `vision_core/tests/test_live_shadow_local_surfaces.py`, `vision_core/tests/test_shadow_live_bridge.py`.
- Исходный пользовательский shared chat (недоступен при составлении): `https://chatgpt.com/share/6af5d654-c47c-83ed-8615-e224c3f8ae10`.

**Проверка источника перед использованием:** ссылки с фиксированными commit идентификаторами не меняются при дальнейших коммитах. `f630eb1` и `60eeade...` являются разными срезами. Если новый checkout имеет другой hash, нужно читать новый код и добавлять отдельную запись, не подменять старое доказательство.

---

## 13. Критерий полезного следующего результата

Этот документ не требует нового фреймворка или рефакторинга ради самого рефакторинга. Следующий реальный прирост для MVP будет, если новый **camera-only** журнал покажет:

1. **Почему** на single-person сцене возникает `v3_count=2` и какие именно boxes/score отличают дубликат от второго человека.
2. На каких стадиях теряются ~**168 мс сверх** экспериментального бюджета 500 мс по медиане driver-to-decision (668 − 500); учитывать, что это арифметическая разница двух метрик одной конфигурации, не готовое объяснение причины.
3. После точечного исправления: измеренные доли успешного exact-cycle binding, своевременного `FOLLOW_ELIGIBLE` и корректного `PAUSE` при отсутствии/неоднозначности цели, с сохранением подлинных cycle IDs и полных таймингов.

**На этом этапе не требуются новые P0-процедуры согласования. Требуется работающий, измеримый MVP.**
