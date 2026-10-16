#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export PYTHONPATH="$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python vision_core/tools/benchmark_jpeg_rgb_transport.py \
 --evidence-root /home/elwis/.local/state/sie/evidence/person_live_local_surfaces_20261014_201809 \
 --model /home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx \
 --report /home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/comparison.json \
 --ar-intrinsic /home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
 --stereo-calibration /home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
 --activation /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
 --output "/home/elwis/.local/state/sie/evidence/jpeg_rgb_transport_$(date +%Y%m%d_%H%M%S).json"
