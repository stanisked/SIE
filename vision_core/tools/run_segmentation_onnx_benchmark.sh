#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
runtime=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
if [ ! -x "$runtime" ]; then
  echo 'Run bash vision_core/tools/setup_segmentation_onnx_benchmark.sh first' >&2
  exit 2
fi
output="/home/elwis/.local/state/sie/evidence/segmentation_onnx_comparison_$(date +%Y%m%d_%H%M%S)"
echo "Offline report root: $output"
"$runtime" vision_core/tools/benchmark_segmentation_onnx.py \
  --model /home/elwis/dev_ws/model_artifacts/yolo11n_seg_coco/yolo11n-seg.pt \
  --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_201809 \
  --output-dir "$output" --ort-threads 1 2 4 --repeats 2
cp /home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/resolved-packages.txt "$output/resolved-packages.txt"
