#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_CROP_LOOKUP_BACKEND=blocked
bash vision_core/tools/run_control_input_preview_target_crop_mmap_no_local.sh
