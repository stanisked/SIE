# AR0234 close-range person alignment v1

## Назначение

Набор готовится для будущего fine-tune единственного класса
`person_upper_body`. Это generic person, а не распознавание конкретного
человека и не face detector. До обучения модель отсутствует; эти кадры не
подключены к yaw alignment, turn или движению.

## Разметка

Разметка выполняется локально в LabelMe, затем детерминированно конвертируется
в YOLO. Единственный допустимый класс: `person_upper_body`. LabelMe JSON
хранится отдельно в `annotations_labelme/`; в JSON запрещён `imageData`.
`raw/` остаётся canonical lossless source, а `images/` содержит hardlink на
тот же PNG и является входом LabelMe. YOLO `.txt` создаются только converter
в `labels/`, который никогда не заменяет существующий label без `--overwrite`.

Для positive кадра требуется ровно один rectangle. Для negative кадра допустим
JSON без `shapes` либо отсутствие JSON; его YOLO file не содержит bbox.
До полного audit не создавай `train/val/test` split.

## Сбор

Цель: 500-800 разнородных кадров. Снимай отдельными session: фронтально,
профиль, сидя, стоя, слева, по центру, справа, на 1.5-3.5 m, при дневном и
искусственном освещении. Обязательно добавь hard negatives: игрушку, одежду,
диван, колени и пустую сцену. Каждый session получает честные human-entered
tags: дистанция, pose, положение, свет, тип сцены и `contains_person`.

Не используй соседние кадры одной session в разных выборках. `train/val/test`
split создаётся только после полной ручной разметки и только по `session_id`.

## Запуск LabelMe

Открывай только `images/`; LabelMe JSON сохраняй только в
`annotations_labelme/`. Не открывай и не меняй `raw/`. Конфиг ограничивает
метку `person_upper_body`, exact validation, rectangle workflow и выключает
embedding исходного изображения:

```bash
labelme \
  /home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1/images \
  --config /home/stanislav/dev_ws/sie_v6_temperature_disabled_mvp/docs/datasets/ar0234_close_range_person_alignment_v1/labelme_person_upper_body_config.yaml \
  --output /home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1/annotations_labelme
```

После разметки сначала выполни `--mode dry-run`, затем только при чистом
отчёте `--mode apply`; validation не создаёт split.
