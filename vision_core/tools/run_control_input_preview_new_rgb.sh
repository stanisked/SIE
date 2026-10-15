#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_NEW_RGB_TRIGGER=true
bash vision_core/tools/run_control_input_preview_latest_capture.sh
