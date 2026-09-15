# AR0234 `person_upper_body` ONNX preview

Этот preview является отдельным visual-only инструментом. Он не меняет
проверенный SIE runtime, не использует Haar locator, не пишет кадры, не создаёт
yaw plan и не обращается к ESP32, HTTP или сети.

Перед первым запуском проверь модель:

```bash
sha256sum /home/stanislav/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx
```

Ожидаемый SHA-256:

```text
dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c
```

Создай отдельное окружение, не меняя validated runtime:

```bash
python3 -m venv /home/stanislav/.venvs/sie-ar0234-yolo11-preview
/home/stanislav/.venvs/sie-ar0234-yolo11-preview/bin/python -m pip install --upgrade pip
/home/stanislav/.venvs/sie-ar0234-yolo11-preview/bin/python -m pip install onnxruntime numpy opencv-python
```

Запуск выполняет UVC read только через approved AR0234 stable by-id path,
показывает overlay и печатает JSONL без raw pixels. `Q` или `Esc` закрывает
окно.

```bash
cd /home/stanislav/dev_ws/sie_v6_temperature_disabled_mvp
PYTHONPATH=. /home/stanislav/.venvs/sie-ar0234-yolo11-preview/bin/python \
  vision_core/tools/run_ar0234_person_upper_body_onnx_preview.py \
  --model /home/stanislav/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx \
  --confidence-threshold 0.40
```

Результат не квалифицирует yaw alignment, turn, forward motion или любой
исполнительный слой. До такого использования нужны отдельные evidence и review.

## Companion runtime для supervised static-target demo

Проверенное окружение MP-PersonDet
`/home/stanislav/dev_ws/runtime_envs/sie_mp_persondet_cv414` не меняется.
Для совместного supervised demo создай отдельный sibling venv с OpenCV 4.14 и
ONNX Runtime:

```bash
python3 -m venv /home/stanislav/dev_ws/runtime_envs/sie_static_target_yolo_onnx_cv414
/home/stanislav/dev_ws/runtime_envs/sie_static_target_yolo_onnx_cv414/bin/python -m pip install --upgrade pip
/home/stanislav/dev_ws/runtime_envs/sie_static_target_yolo_onnx_cv414/bin/python -m pip install \
  "opencv-python>=4.14,<4.15" onnxruntime numpy
```

Этот companion runtime нужен только потому, что actual supervised runner
использует сохранённый MP-PersonDet для stereo ROI и локальный YOLO ONNX для
primary AR0234 image evidence. Он не меняет validated MP-PersonDet venv,
model artifact или V6 stereo policy. Перед физическим запуском отдельно
проверь `--help`; камера и ESP32 при этой проверке не открываются.
