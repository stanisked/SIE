# Каталог регрессий supervised static-target MVP

Этот каталог фиксирует поведение одной supervised петли: observation,
measurement, decision, один подтверждённый bounded command и обязательное
re-observation. Он не даёт autonomous approval и не меняет qualification
исполнительного адаптера.

| Сценарий / ранее найденная проблема | Ожидаемый результат | Регрессия | Что нельзя менять без новых evidence и review |
| --- | --- | --- | --- |
| YOLO даёт `SINGLE_TARGET`, а legacy MP-PersonDet даёт `MULTIPLE_PERSONS` | Для stereo metric depth используется bbox и provenance YOLO. Legacy detector не выбирает другой ROI. | `test_yolo_single_target_overrides_legacy_multiple_for_metric_roi` | Приоритет single YOLO для metric ROI и связь measurement с `person_evidence_id` YOLO. |
| YOLO даёт несколько eligible targets | Bbox не выбирается, measurement не создаётся, command отсутствует. | `test_ambiguous_yolo_blocks_without_selecting_bbox_or_planning` | Запрещено выбирать самый уверенный или самый большой bbox вместо fail-closed результата. |
| YOLO даёт `NO_TARGET` | Для одного цикла сохраняется MP-PersonDet fallback с его собственным ROI provenance. | `test_yolo_no_target_keeps_legacy_persondet_metric_roi_fallback` | Fallback допустим только для отсутствующего YOLO target, не для ambiguous YOLO. |
| Два новых five-cycle окна не дают valid metric depth | Итог `RANGE_ACQUISITION_REQUIRED`; HTTP, command и motor action отсутствуют. | `test_two_zero_metric_windows_require_range_without_http_or_motor` | Нельзя смешивать старые и новые окна или планировать движение без current metric decision. |
| Первый preflight `GET /status` временно недоступен | Разрешены только повторные read-only `GET`; единственный `POST` идёт после валидного `READY`. | `test_preflight_retries_get_then_posts_only_once_after_ready` | Нельзя повторять `POST`, создавать replacement command или выполнять corrective action. |
| Evidence stale до сетевого preflight | Bridge блокирует plan до любого HTTP действия. | `test_stale_evidence_blocks_before_any_http_preflight` | Нельзя ослаблять freshness policy или переносить её проверку после preflight. |
| Независимые execute runs получают один и тот же bridge plan | Каждый run получает новый криптографически случайный `command_id`; внутри run ID не меняется, `POST` ровно один. | `test_each_execute_run_gets_new_command_id_and_one_post_keeps_it` | Нельзя переиспользовать ID как реакцию на timeout, duplicate или terminal-read failure. |

## Набор 2026-09-17

Реальный запуск показал пять centered `SINGLE_TARGET` YOLO observations при
`--yolo-confidence-threshold 0.70`, но legacy MP-PersonDet в части циклов
вернул `MULTIPLE_PERSONS`, а две отдельные metric попытки дали только 2 и 3
valid measurement. Ожидаемый safe итог был `RANGE_ACQUISITION_REQUIRED` с
`command_id=null`, `network_performed=false` и
`motor_command_performed=false`.

Фикстуры в regression suite не содержат raw frames, model inference,
камеры, ESP32 или сетевые вызовы. Они проверяют только зафиксированные
контракты и provenance.
