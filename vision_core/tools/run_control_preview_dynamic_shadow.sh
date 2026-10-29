#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
export SIE_PREVIEW_EXISTING_STATIC_V2=true
export SIE_PREVIEW_DYNAMIC_SHADOW_DIAGNOSTIC=true
bash vision_core/tools/run_control_preview_front_geometry.sh
