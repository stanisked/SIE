#!/usr/bin/env bash
# New overlay venv; existing ROS/segmentation environments are read-only inputs.
set -eo pipefail
base=/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python
folder=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark
"$base" -c 'import torch, ultralytics, scipy, cv2; print("Existing segmentation dependencies ready")'
site_path=$("$base" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
if [ ! -x "$folder/bin/python" ]; then
  "$base" -m venv "$folder"
fi
"$folder/bin/python" - "$site_path" <<'PY'
from pathlib import Path
import sys, sysconfig
source=Path(sys.argv[1]); target=Path(sysconfig.get_paths()['purelib'])/'sie_segmentation_base.pth'
if not source.is_dir(): raise SystemExit('segmentation base site-packages missing')
text=str(source)+'\n'
if target.exists() and target.read_text()!=text:
    raise SystemExit('overlay points to another base; inspect before replacing')
target.write_text(text)
PY
"$base" -m pip freeze > "$folder/base-packages-before.txt"
"$folder/bin/python" -m pip install --only-binary=:all: onnx onnxruntime onnxscript
"$base" -m pip freeze > "$folder/base-packages-after.txt"
cmp "$folder/base-packages-before.txt" "$folder/base-packages-after.txt"
"$folder/bin/python" -m pip freeze > "$folder/resolved-packages.txt"
"$folder/bin/python" - <<'PY'
import torch, ultralytics, onnx, onnxruntime, onnxscript, scipy, cv2
print('ONNX benchmark ready:', 'torch',torch.__version__,'ultralytics',ultralytics.__version__,
      'onnx',onnx.__version__,'ORT',onnxruntime.__version__,'OpenCV',cv2.__version__)
print('Providers:',onnxruntime.get_available_providers())
PY
