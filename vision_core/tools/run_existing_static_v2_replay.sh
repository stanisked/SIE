#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
python3 vision_core/tools/replay_existing_static_profile_v2.py \
  --journal-root /home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261021_113225 \
  --profile /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json \
  --output "/home/elwis/.local/state/sie/evidence/existing_static_v2_replay_$(date +%Y%m%d_%H%M%S).json"
