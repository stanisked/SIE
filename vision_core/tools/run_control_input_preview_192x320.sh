#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_REVIEWED_INPUT_MODE=192x320
bash vision_core/tools/run_control_input_preview_fresh_stereo.sh
