#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
python3 vision_core/tools/analyze_control_depth_quality_shadow.py \
 --journal-root /home/elwis/.local/state/sie/evidence/control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_141536 \
 --output "/home/elwis/.local/state/sie/evidence/depth_admission_draft_$(date +%Y%m%d_%H%M%S).json"
