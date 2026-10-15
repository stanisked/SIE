#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_OVERLAP_STEREO_DECODE=true
bash vision_core/tools/run_control_input_preview_new_rgb.sh
