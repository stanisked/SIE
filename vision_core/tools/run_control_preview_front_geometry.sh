#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_FRONT_MOUNT_PROFILE="$PWD/vision_core/config/diagnostics/robot_front_mount_operator_v1.json"
bash vision_core/tools/run_control_input_preview_fast_local.sh
