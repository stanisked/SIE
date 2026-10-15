#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_RGB_TRANSPORT=mmap
bash vision_core/tools/run_control_input_preview_target_crop.sh
