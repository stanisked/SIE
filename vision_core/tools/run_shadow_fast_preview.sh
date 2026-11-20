#!/usr/bin/env bash
# One terminal, current checkout, established fast capture/mask-depth path.
# This launcher intentionally has no motor execution route.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
segmenter_python=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
case "${SHADOW_FAST_RUNTIME_PROFILE:-baseline}" in
  baseline)
    model=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx
    report=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/comparison.json
    export SIE_V4L2_BUFFER_COUNT=1
    ;;
  fast_192x320_4buf)
    model=/home/elwis/.local/state/sie/evidence/segmentation_input_192x320_20261016_130842/input_192x320/checkpoint.onnx
    report=/home/elwis/.local/state/sie/evidence/segmentation_input_192x320_20261016_130842/comparison.json
    export SIE_V4L2_BUFFER_COUNT=4
    ;;
  *)
    echo "Unsupported SHADOW_FAST_RUNTIME_PROFILE" >&2
    exit 2
    ;;
esac
activation_profile=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json
activation_profile_sha256=c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9
v3="${SHADOW_RGB_MODEL_PATH:-}"
semantic_source="${SHADOW_FAST_SEMANTIC_SOURCE:-v3_mask_binding}"
detector_ros_args=()
case "$semantic_source" in
  v3_mask_binding)
    shadow_parameter='preview_shadow_v3_mask_match:=true'
    ar_detector_enabled=true
    detector_ros_args=(-p "model_path:=$v3" -p 'model_version:=v3')
    ;;
  segmentation_mask)
    shadow_parameter='preview_shadow_segmentation_input:=true'
    ar_detector_enabled=false
    ;;
  *) echo "Unsupported Shadow semantic source: $semantic_source" >&2; exit 2 ;;
esac
if [[ "$semantic_source" = v3_mask_binding ]]; then
  [[ -n "$v3" ]] || { echo 'Set SHADOW_RGB_MODEL_PATH to reviewed v3 ONNX' >&2; exit 2; }
fi
for arg in "$@"; do
  case "$arg" in
    --source|--source=*|--journal|--journal=*|--semantic-source|--semantic-source=*|--segmentation-model-sha256|--segmentation-model-sha256=*)
      echo "Unsupported preview argument: $arg" >&2; exit 2 ;;
  esac
done
if [[ " $* " == *" --execute "* ]] &&
   [[ " $* " != *" --supervised-experimental "* ]]; then
  echo "Follow Person requires --supervised-experimental with --execute" >&2
  exit 2
fi
required_inputs=("$runtime" "$segmenter_python" "$model" "$report" "$activation_profile")
if [[ "$semantic_source" = v3_mask_binding ]]; then
  required_inputs+=("$v3")
fi
# Opt-in Face/Body evidence remains a diagnostic side stream. It cannot
# authorize a track transfer or movement in the existing Shadow bridge.
identity_ros_args=()
case "${SHADOW_IDENTITY_PREVIEW:-0}" in
  0) ;;
  1)
    face_dir=/home/elwis/dev_ws/runtime_artifacts/sie_face_yunet_sface_v1
    body_dir=/home/elwis/dev_ws/runtime_artifacts/sie_body_osnet_x0_25
    face_private=/home/elwis/.local/state/sie/private/face_profiles
    identity_inputs=(
      "$face_dir/face_detection_yunet_2023mar.onnx"
      "$face_dir/face_recognition_sface_2021dec.onnx"
      "$face_private/person_001_face_enrollment_v1.npz"
      "$body_dir/osnet_x0_25_msmt17.onnx"
    )
    for identity_input in "${identity_inputs[@]}"; do
      [[ -f "$identity_input" ]] || { echo "Missing identity input: $identity_input" >&2; exit 2; }
    done
    identity_ros_args=(
      -p 'preview_identity_diagnostic_enabled:=true'
      -p 'preview_identity_every_n_cycles:=5'
      -p "preview_identity_yunet_model:=${identity_inputs[0]}"
      -p "preview_identity_sface_model:=${identity_inputs[1]}"
      -p "preview_identity_face_profile:=${identity_inputs[2]}"
      -p "preview_identity_osnet_model:=${identity_inputs[3]}"
    )
    ;;
  *) echo 'SHADOW_IDENTITY_PREVIEW must be 0 or 1' >&2; exit 2 ;;
esac
for required in "${required_inputs[@]}"; do
  [[ -f "$required" ]] || { echo "Missing reviewed input: $required" >&2; exit 2; }
done
if [[ "$semantic_source" = v3_mask_binding ]]; then
  v3_sha=$(sha256sum "$v3" | cut -d ' ' -f 1)
  [[ "$v3_sha" = fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5 ]] || {
    echo "Shadow v3 ONNX SHA-256 mismatch: $v3_sha" >&2; exit 2;
  }
fi
actual_activation_profile_sha256=$(sha256sum "$activation_profile" | cut -d ' ' -f 1)
[[ "$actual_activation_profile_sha256" = "$activation_profile_sha256" ]] || {
  echo "Activation profile SHA-256 mismatch: $actual_activation_profile_sha256" >&2; exit 2;
}
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
native_dir=$(mktemp -d /tmp/sie-shadow-native-XXXXXX)
cc -std=c11 -O2 -Wall -Wextra -Werror vision_core/tools/stream_v4l2_mjpeg.c -o "$native_dir/capture"
evidence="/home/elwis/.local/state/sie/evidence/shadow_fast_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$evidence"
case "${SHADOW_CAPTURE_SNAPSHOTS:-0}" in
  0) unset SIE_SHADOW_SOURCE_SNAPSHOTS ;;
  1) export SIE_SHADOW_SOURCE_SNAPSHOTS=1 ;;
  *) echo 'SHADOW_CAPTURE_SNAPSHOTS must be 0 or 1' >&2; exit 2 ;;
esac
"$runtime" -c 'from sie_ros2.ar0234_ov9281_control_preview_node import main; main()' \
  --ros-args -p project_root:="$PWD" "${detector_ros_args[@]}" \
  -p ar_person_detector_enabled:="$ar_detector_enabled" \
  -p confidence_threshold:=0.40 -p "$shadow_parameter" \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:="$activation_profile" \
  -p shadow_segmenter_python:="$segmenter_python" -p shadow_segmenter_model:="$model" \
  -p preview_segmentation_engine:=onnxruntime -p preview_onnx_comparison_report:="$report" \
  -p preview_onnx_threads:=2 -p preview_rgb_backend:=single_segmentation \
  -p preview_rgb_transport:=mmap -p preview_local_surfaces_enabled:=false \
  -p preview_defer_local_surfaces:=true -p preview_fast_local_consistency:=true \
  -p preview_capture_backend:=latest_v4l2 \
  -p preview_overlap_stereo_decode:=true \
  -p preview_native_capture_executable:="$native_dir/capture" \
  -p preview_native_rgb_trigger:=true -p preview_target_crop:=true \
  -p preview_crop_lookup_backend:=blocked -p preview_stereo_mode:=half_640x400 \
  -p preview_front_mount_profile:="$PWD/vision_core/config/diagnostics/robot_front_mount_operator_v1.json" \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 \
  -p frame_rate_hz:=10.0 -p evidence_dir:="$evidence" "${identity_ros_args[@]}" &
camera_pid=$!
trap 'kill -INT "$camera_pid" 2>/dev/null || true; wait "$camera_pid" 2>/dev/null || true; rm -rf -- "$native_dir"' EXIT
"$runtime" vision_core/tools/shadow_live_bridge.py --source fast_mask \
  --semantic-source "$semantic_source" \
  --segmentation-model-sha256 "$(sha256sum "$model" | cut -d ' ' -f 1)" \
  --journal "$evidence/bridge.jsonl" "$@"
kill -INT "$camera_pid" 2>/dev/null || true
wait "$camera_pid" 2>/dev/null || true
if [[ " $* " == *" --execute "* ]]; then
  echo "Follow Person execution journal: $evidence/bridge.jsonl"
else
  "$runtime" vision_core/tools/summarize_shadow_fast_preview.py "$evidence/bridge.jsonl"
fi
echo "Evidence: $evidence"
if [[ "${SHADOW_IDENTITY_PREVIEW:-0}" = 1 ]]; then
  echo "Identity diagnostic: $evidence/identity-preview.jsonl (motor authorization unchanged)"
fi
if [[ "${SHADOW_CAPTURE_SNAPSHOTS:-0}" = 1 ]]; then
  "$runtime" vision_core/tools/summarize_shadow_source_snapshots.py \
    "$evidence/bridge.jsonl" "$evidence/shadow-source-snapshots"
fi
