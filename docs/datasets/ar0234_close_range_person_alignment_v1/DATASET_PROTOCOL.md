# AR0234 close-range person alignment v1

## Назначение

Набор готовится для будущего fine-tune единственного класса
`person_upper_body`. Это generic person, а не распознавание конкретного
человека и не face detector. До обучения модель отсутствует; эти кадры не
подключены к yaw alignment, turn или движению.

## Разметка

Используй LabelImg в YOLO format. `classes.txt` содержит ровно одну строку:
`person_upper_body`. Один bbox должен включать голову, плечи и торс до нижней
границы кадра. Лицо отдельно не размечается. Для пустой сцены LabelImg должен
оставить корректный пустой `.txt` с тем же stem, что и PNG.

Canonical lossless source хранится в `raw/`; `images/` содержит hardlink на
тот же PNG и служит входом LabelImg. `labels/` содержит соответствующие YOLO
файлы. Перед split запускай validator, который требует class id `0`, пять
полей, конечные normalized значения в `(0, 1]` и bbox внутри кадра.

## Сбор

Цель: 500-800 разнородных кадров. Снимай отдельными session: фронтально,
профиль, сидя, стоя, слева, по центру, справа, на 1.5-3.5 m, при дневном и
искусственном освещении. Обязательно добавь hard negatives: игрушку, одежду,
диван, колени и пустую сцену. Каждый session получает честные human-entered
tags: дистанция, pose, положение, свет, тип сцены и `contains_person`.

Не используй соседние кадры одной session в разных выборках. `train/val/test`
split создаётся только после полной ручной разметки и только по `session_id`.

## Запуск LabelImg

На capture host `labelImg` пока не установлен. После локальной установки
запусти ровно так:

```bash
labelImg \
  /home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1/images \
  /home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1/classes.txt \
  /home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1/labels
```

В LabelImg выбери формат `YOLO`. Открывай только `images/`, а save directory
оставь `labels/`; `raw/` не открывай и не изменяй. Перед split запускай
`manage_ar0234_close_range_person_alignment_dataset.py --validate-labels`.
