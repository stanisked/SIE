#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export PYTHONPATH="$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
export SIE_PREVIEW_STEREO_MODE=half_640x400
export SIE_PREVIEW_RGB_BACKEND=single_segmentation
export SIE_PREVIEW_SEGMENTATION_ENGINE=onnxruntime
export SIE_PREVIEW_SEGMENTER_PYTHON=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
export SIE_PREVIEW_ONNX_REPORT=/home/elwis/.local/state/sie/evidence/segmentation_onnx_comparison_20261015_105936/comparison.json
export SIE_PREVIEW_SEGMENTER_MODEL=/home/elwis/.local/state/sie/evidence/segmentation_onnx_comparison_20261015_105936/source_checkpoint.onnx
export SIE_PREVIEW_ORT_THREADS=2
export SIE_PREVIEW_DEFER_LOCAL=true
export SIE_PREVIEW_LOCAL_EVERY_N=5
"$SIE_PREVIEW_SEGMENTER_PYTHON" - "$SIE_PREVIEW_SEGMENTER_MODEL" "$SIE_PREVIEW_ONNX_REPORT" <<'PY'
import sys
from onnx_segmentation_backend import verify_report
report,shape=verify_report(sys.argv[1],sys.argv[2],2)
print('Reviewed ONNX export verified:',shape,'CPU threads=2')
PY
bash vision_core/tools/run_control_input_preview.sh
