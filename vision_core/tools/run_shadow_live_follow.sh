#!/usr/bin/env bash
# One-terminal RGB + stereo + Shadow UART experiment. Preview is the default.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD${PYTHONPATH:+:$PYTHONPATH}"

model_version="${SHADOW_RGB_MODEL_VERSION:-v2}"
model_path="${SHADOW_RGB_MODEL_PATH:-/home/elwis/best.onnx}"
case "$model_version" in v2|v3) ;; *) echo 'Model version must be v2 or v3' >&2; exit 2 ;; esac
[[ -f "$model_path" ]] || { echo "Model missing: $model_path" >&2; exit 2; }

"$runtime" -c 'from sie_ros2.ar0234_ov9281_dynamic_association_audit_node import main; main()' \
  --ros-args \
  -p project_root:="$PWD" -p model_path:="$model_path" \
  -p model_version:="$model_version" -p confidence_threshold:=0.40 \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 \
  -p shadow_instance_mask_enabled:=false -p evidence_capture_enabled:=false &
camera_pid=$!
trap 'kill -INT "$camera_pid" 2>/dev/null || true; wait "$camera_pid" 2>/dev/null || true' EXIT

"$runtime" vision_core/tools/shadow_live_bridge.py "$@"
