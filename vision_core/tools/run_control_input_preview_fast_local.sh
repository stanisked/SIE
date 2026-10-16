#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_FAST_LOCAL_CONSISTENCY=true
bash vision_core/tools/run_control_input_preview_192x320.sh
