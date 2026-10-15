#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
runtime=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
review=/home/elwis/.local/state/sie/evidence/segmentation_onnx_comparison_20261015_105936
output="/home/elwis/.local/state/sie/evidence/segmentation_input_size_$(date +%Y%m%d_%H%M%S)"
echo "Offline report root: $output"
"$runtime" vision_core/tools/benchmark_segmentation_input_size.py \
  --reference-model "$review/source_checkpoint.onnx" \
  --reference-report "$review/comparison.json" \
  --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_201809 \
  --ar-intrinsic /home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  --stereo-calibration /home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  --activation /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  --output-dir "$output" --sizes 320x512 256x384 --repeats 2
