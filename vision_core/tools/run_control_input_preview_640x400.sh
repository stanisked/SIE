#!/usr/bin/env bash
set -eo pipefail
export SIE_PREVIEW_STEREO_MODE=half_640x400
bash /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp/vision_core/tools/run_control_input_preview.sh
