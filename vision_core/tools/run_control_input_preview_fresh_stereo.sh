#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_FRESH_STEREO_DURING_RGB=true
bash vision_core/tools/run_control_input_preview_original_jpeg.sh
