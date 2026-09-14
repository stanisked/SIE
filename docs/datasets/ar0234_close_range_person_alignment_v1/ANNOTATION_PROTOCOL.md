# Annotation protocol: AR0234 close-range person alignment v1

## Область

Это локальная ручная разметка единственного класса `person_upper_body` для
будущей модели. Она не включает face labels, yaw alignment, управление
движением или qualification.

## Один допустимый bbox

Для кадра с одним человеком создай ровно один LabelMe `rectangle` с exact
label `person_upper_body`. Рамка включает голову, плечи, руки и видимую часть
торса. Если тело обрезано нижней границей изображения, рамка заканчивается
ровно на границе изображения. Не размечай отдельно лицо.

Игрушки, одежду, диван, колени и прочие предметы не размечай. Если в кадре
два человека, отложи его в review: не превращай такой кадр в single target.
Для negative session оставь JSON без `shapes` или не создавай JSON вовсе.

## Порядок работы

1. Открой `images/` в LabelMe с versioned config, JSON сохраняй только в
   `annotations_labelme/`.
2. Не меняй `raw/`, capture manifests, inventory или оригинальный PNG.
3. Выполни converter сначала в режиме `dry-run`. Он проверит exact label,
   rectangle, imagePath, размер `1920x1200`, frozen inventory и правило
   ровно одного bbox для positive изображения.
4. Только если dry-run не сообщает failures, запусти `apply`. Существующие
   `labels/*.txt` можно заменить только с явным `--overwrite`.
5. Запусти `validate`; `train/val/test` split на этом этапе запрещён.
