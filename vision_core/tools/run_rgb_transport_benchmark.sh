#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python vision_core/tools/benchmark_concurrent_segmentation_threads.py --experiment rgb_transport \
 --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_201809 \
 --model /home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx \
 --report /home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/comparison.json \
 --output "/home/elwis/.local/state/sie/evidence/rgb_transport_comparison_$(date +%Y%m%d_%H%M%S).json"
