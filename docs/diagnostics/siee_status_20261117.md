# SIEE — Итоговый инженерный статус на 17 ноября 2026

**Тип:** точка восстановления / проверяемые результаты, не Engineering Specification.
**Источник:** дневной журнал испытаний оператора (включая `bridge(3).jsonl`, 157 JSONL записей), наблюдения оператора, код и тесты GitHub, UART /status и результаты Arduino IDE.
**Принцип:** ФАКТ / РЕШЕНИЕ / ГИПОТЕЗА / OPEN; не переносить новые диагностические значения в утверждённые Data Contracts и Calibration без отдельного решения.

## 1. Рабочие ветки и способ восстановления
- Базовая утверждённая точка исходного контекста: `codex/shadow-turtlesim-follow-20261111`; снимок Graphify 2026-11-16 построен по `db8097ea4239dcf45ef7598c831a45648d2498fd`.
- Экспериментальный RGB Edge Recovery / асинхронный FOLLOW: `codex/shadow-rgb-edge-recovery-20261117`.
- **Текущая ветка исполняемых изменений 17 ноября:** `codex/shadow-short-segment-brake-v7-20261117`, содержит v7 ESP32 и совместимый Python UART runner. Она *не* означает, что новый код слит в базовую ветку 20261111.
- Pi checkout (оператор): `/home/elwis/dev_ws/sie_shadow_follow_tools`; Python: `/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python`; ESP32 UART: `/dev/ttyAMA0`.
- Компиляция и прошивка ESP32: на ноутбуке Ubuntu, Arduino IDE 2.3.11, `ESP32 Dev Module`, USB `/dev/ttyUSB0`; обычно копировать полную папку скетча с Pi на ноутбук по `scp`, открыть одноимённый `.ino` и использовать Verify/Upload в IDE. Не подменять этот проверенный маршрут новым CLI.
- Актуальная физически прошитая ESP32: `v4_2_uart_shadow_v7_short_forward_brake`, подтверждена UART `HTTP 200`.

## 2. RGB/Depth → FOLLOW → STEER: физическое испытание (ФАКТ)
- Реальное напольное испытание с `SHADOW_FAST_SEMANTIC_SOURCE=segmentation_mask`, `--execute --supervised-experimental --continuous --edge-turn`, `--front-rim-forward-of-optical-m 0.0125`, UART `/dev/ttyAMA0`, `--segment-m 0.40`, `--minimum-confidence 0.40`.
- В разобранном `bridge(3).jsonl`: 6 сегментов движения вперёд, 4 принятых `STEER`, один RGB `track_id=ar0234-audit-track:8144989e0438cf10:1` во всех зарегистрированных FOLLOW-eligible измерениях. Дистанции в метрах последовательно: 1.7403, 1.5521, 1.3520, 1.1658, 0.9113, 0.7320; это диагностическая глубина, не независимый абсолютный замер фронтального зазора.
- Оператор **непосредственно наблюдал** физическую смену ориентации тележки, когда человек был у края RGB кадра и движение вперёд уже прекратилось. Однако в UART журнале **отсутствует** `POST /shadow-turn`; `edge_turns_started=0`, `shadow_turn_result=NONE`. Это не подтверждённый отдельный RGB Edge Recovery pivot; возможны дифференциальное подруливание и остаточная механическая инерция. Не утверждать ложный результат `SHADOW_TURN SUCCESS`.
- Последний короткий `SHADOW_RESUME` запрашивал **0.042 м**. Торможение началось при **0.0466 м**, конечный пробег **0.0635 м**, после пересечения прежнего абсолютного центра **0.062 м** сработал `STEERED_CENTER_LIMIT`; терминально `FAULT`, PWM 0/0. Это не доказательство физически достигнутых 0.5 м от переднего края.

## 3. Edge Recovery software (ФАКТ)
- Реализована bounded same-track корректировка ориентации до 8° в состоянии PAUSED; новая связанная RGB/Depth пара обязательна перед возобновлением движения FORWARD. Лимит возраста последней проверенной глубины для **одного поворота**, но не для движения вперёд, расширен с 2.5 до 3.0 с по реальным задержкам UART; новые разные Track ID запрещают использование сохранённой edge-подсказки.
- После исправления двух тестовых фикстур 13/13 targeted regression tests PASS, включая 2.716-секундную задержку, правило другого track ID, partial turn refusal, и resume только со свежей глубиной. Физический `SHADOW_TURN` и post-turn re-entry остаются OPEN.

## 4. ESP32 v7: раннее торможение (ФАКТ)
- Новая `firmware/esp32/traction_control_v4_2_uart_shadow_v7_short_forward_brake/` на базе v5. Отдельная задача запрашивает ранний brake для steered-forward сегментов до 0.100 м *во время STARTING*, без отключения существующих hard wheel/center/heading limits. `run_shadow_uart.py` принимает build v7; 4/4 новых теста PASS, но это не доказательство аппаратной динамики.
- Arduino IDE 2.3.11: компиляция 508199 B (38% flash), глобальные 72320 B (22% RAM); запись 508352 B и `Hash of data verified`; ESP32 перезапущена.
- UART readback: `HTTP 200`, `firmware_build_id=v4_2_uart_shadow_v7_short_forward_brake`, `READY`, `fault=None`, `bounded_fault_latched=False`, PWM 0/0.
- Изолированный сегмент **42 мм**, при поднятых колёсах: `short_forward_brake_requested=True`, trigger=26 counts при threshold=26; конечный пробег **19.3 мм**, `PARTIAL_PROGRESS`, `READY`, FAULT нет.
- Повтор **42 мм на полу**: trigger=27 counts при threshold=26; конечный пробег **22.8 мм**, `PARTIAL_PROGRESS`, `READY`, FAULT нет. Недоход от команды 42 мм: **19.2 мм**. Раннее торможение подтверждено, стабильное полное исполнение сегмента **не** подтверждено.
- Значение, **реально прошитое в v7**: `STEERED_SHORT_BRAKE_RESERVE_M = 0.030f` (30 мм). Предложение снизить до **0.017f** (17 мм) **ещё не внедрено, не скомпилировано, не прошито, не испытывалось**. Прогноз остановки около 35 мм — лишь гипотеза по одному напольному замеру.

## 5. Архитектурные границы и ещё открытые вопросы
- RGB удерживает текущую цель в проверенных интервалах; OV9281 обеспечивает диагностическую глубину; ESP32 принимает и физически исполняет FOLLOW/STEER. Рабочий диапазон диагностики 0.5–4.5 м, целевая дистанция прекращения FOLLOW **0.5 м от переднего края**. Точность динамического torso-front и дистанция при остановке на физической земле независимым эталоном не подтверждены; LiDAR D500 / fusion пока не интегрированы.
- `--supervised-experimental` не равен canonical `Measurement` admission. Не считать эпизод физического поворота успешным `SHADOW_TURN` без его POST и результата от ESP32.
- `PARTIAL_PROGRESS` нельзя обозначать как `SUCCESS`; остановка по fault — действие защиты, а не успешный контроль расстояния.
- Интеграция ROS2 + Vision Core + LiDAR и удержание цели после отдельного edge TURN на полу остаются в списке проверок.
- Graphify folder `Graphify_AST_db8097e_20261116` содержит `graph.json`, `graph.html`, `GRAPH_REPORT.md`, построенные по исходникам 16 ноября: 4599 узлов / 13658 рёбер / 300 сообществ. **Не пересчитывались** по ветке v7; граф исторический, нельзя выдавать его за текущий анализ 17 ноября.

## 6. Следующий измеримый шаг
- Сохранить v7 как подтверждённый brake-guard baseline. Если принимается настройка 17 мм, сделать **отдельную ревизию** с текущими hard limits неизменными; compile/flash через Arduino IDE на ноутбуке; проверить один bounded 42-мм сегмент на вывешенных колёсах и полу, записав trigger counts, brake-start, final encoder distance, `last_command_state`, `fault`, PWM 0/0. Без повторного целевого FOLLOW, пока короткий сегмент не покажет допустимое завершение без аварии.
- Затем отдельный supervised опыт физического edge `SHADOW_TURN` → fresh associated RGB/Depth → `RESUME`, строго по UART-журналу; не менять калибровку и не подменять физическую проверку кодовыми тестами.

## Источники / ссылки
- Baseline: https://github.com/stanisked/SIE/tree/codex/shadow-turtlesim-follow-20261111
- Edge: https://github.com/stanisked/SIE/tree/codex/shadow-rgb-edge-recovery-20261117
- v7: https://github.com/stanisked/SIE/tree/codex/shadow-short-segment-brake-v7-20261117
- Graphify snapshot: https://drive.google.com/drive/folders/1R_oNgCjA1w4fv_zdf_AGYyYG8QzRAR1t
- PROJECT_CONTEXT.md (Drive): https://drive.google.com/file/d/1mRj9UluoMptShOMPMz4h_MbCFczlVn0o/view
