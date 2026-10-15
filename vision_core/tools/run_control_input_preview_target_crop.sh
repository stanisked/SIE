#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_TARGET_CROP=true
bash vision_core/tools/run_control_input_preview_640x400_onnx_256x384.sh
