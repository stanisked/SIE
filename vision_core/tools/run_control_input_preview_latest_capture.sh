#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
native_dir=$(mktemp -d /tmp/sie-native-capture-XXXXXX)
trap 'rm -rf -- "$native_dir"' EXIT
cc -std=c11 -O2 -Wall -Wextra -Werror vision_core/tools/stream_v4l2_mjpeg.c -o "$native_dir/capture"
export SIE_PREVIEW_NATIVE_CAPTURE_EXECUTABLE="$native_dir/capture"
export SIE_PREVIEW_CAPTURE_BACKEND=latest_v4l2
bash vision_core/tools/run_control_input_preview_target_crop_blocked.sh
