#!/usr/bin/env bash
# One terminal, current checkout, established fast capture/mask-depth path.
# This launcher intentionally has no motor execution route.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
segmenter_python=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
model=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx
report=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/comparison.json
v3="${SHADOW_RGB_MODEL_PATH:-}"
[[ -n "$v3" ]] || { echo 'Set SHADOW_RGB_MODEL_PATH to reviewed v3 ONNX' >&2; exit 2; }
for required in "$runtime" "$segmenter_python" "$model" "$report" "$v3"; do
  [[ -f "$required" ]] || { echo "Missing reviewed input: $required" >&2; exit 2; }
done
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
native_dir=$(mktemp -d /tmp/sie-shadow-native-XXXXXX)
cc -std=c11 -O2 -Wall -Wextra -Werror vision_core/tools/stream_v4l2_mjpeg.c -o "$native_dir/capture"
evidence="/home/elwis/.local/state/sie/evidence/shadow_fast_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$evidence"
"$runtime" -c 'from sie_ros2.ar0234_ov9281_control_preview_node import main; main()' \
  --ros-args -p project_root:="$PWD" -p model_path:="$v3" -p model_version:=v3 \
  -p confidence_threshold:=0.40 -p preview_shadow_v3_mask_match:=true \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  -p shadow_segmenter_python:="$segmenter_python" -p shadow_segmenter_model:="$model" \
  -p preview_segmentation_engine:=onnxruntime -p preview_onnx_comparison_report:="$report" \
  -p preview_onnx_threads:=2 -p preview_rgb_backend:=single_segmentation \
  -p preview_rgb_transport:=mmap -p preview_local_surfaces_enabled:=false \
  -p preview_defer_local_surfaces:=true -p preview_capture_backend:=latest_v4l2 \
  -p preview_native_capture_executable:="$native_dir/capture" \
  -p preview_native_rgb_trigger:=true -p preview_target_crop:=true \
  -p preview_crop_lookup_backend:=blocked -p preview_stereo_mode:=half_640x400 \
  -p preview_front_mount_profile:="$PWD/vision_core/config/diagnostics/robot_front_mount_operator_v1.json" \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 \
  -p frame_rate_hz:=10.0 -p evidence_dir:="$evidence" &
camera_pid=$!
trap 'kill -INT "$camera_pid" 2>/dev/null || true; wait "$camera_pid" 2>/dev/null || true; rm -rf -- "$native_dir"' EXIT
"$runtime" vision_core/tools/shadow_live_bridge.py --source fast_mask \
  --segmentation-model-sha256 "$(sha256sum "$model" | cut -d ' ' -f 1)" \
  --journal "$evidence/bridge.jsonl" "$@"
