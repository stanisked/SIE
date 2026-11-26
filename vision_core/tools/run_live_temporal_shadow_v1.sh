#!/usr/bin/env bash
# Sensor-only AR0234/OV9281 preview and metadata subscriber for an existing LD19 topic.
set -eo pipefail
if [[ $# -eq 1 && "$1" == --help ]]; then
  echo "Usage: run_live_temporal_shadow_v1.sh"
  echo "SIE_TEMPORAL_DURATION_S defaults to 30 (range 1 to 300)."
  echo "SIE_TEMPORAL_CAMERA_SELECTION defaults to latest_pair. Also accepts fresh_stereo_during_rgb."
  echo "SIE_TEMPORAL_LD19_CLOCK_EVIDENCE optionally identifies the current publisher's same-host realtime evidence."
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "This launcher accepts no positional or passthrough arguments." >&2
  exit 2
fi
duration=${SIE_TEMPORAL_DURATION_S:-30}
if [[ ! "$duration" =~ ^[1-9][0-9]{0,2}$ ]] || (( duration > 300 )); then
  echo "SIE_TEMPORAL_DURATION_S must be an integer from 1 to 300." >&2
  exit 2
fi
selection_args=()
case "${SIE_TEMPORAL_CAMERA_SELECTION:-latest_pair}" in
  latest_pair) ;;
  fresh_stereo_during_rgb)
    selection_args=(-p preview_native_rgb_jpeg:=true -p preview_fresh_stereo_during_rgb:=true)
    ;;
  *) echo "Unsupported SIE_TEMPORAL_CAMERA_SELECTION" >&2; exit 2 ;;
esac
clock_args=()
if [[ -n "${SIE_TEMPORAL_LD19_CLOCK_EVIDENCE:-}" ]]; then
  clock_args=(-p "preview_temporal_ld19_clock_evidence:=${SIE_TEMPORAL_LD19_CLOCK_EVIDENCE}")
fi

cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/ros2/sie_ros2:$PWD/vision_core/tools:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false SIE_V4L2_BUFFER_COUNT=1
unset SIE_SHADOW_SOURCE_SNAPSHOTS
umask 077

runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
segmenter_python=/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark/bin/python
model_dir=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384
model="$model_dir/checkpoint.onnx"
report="$model_dir/comparison.json"
artifacts=/home/elwis/dev_ws/runtime_artifacts
activation="$artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json"
for input in "$runtime" "$segmenter_python" "$model" "$report" "$activation"; do
  [[ -f "$input" ]] || { echo "Missing reviewed input: $input" >&2; exit 2; }
done
activation_sha256=$(sha256sum "$activation" | cut -d ' ' -f 1)
[[ "$activation_sha256" == c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9 ]] || {
  echo "Activation profile SHA-256 mismatch: $activation_sha256" >&2; exit 2;
}

native_dir=$(mktemp -d /tmp/sie-temporal-native-XXXXXX)
trap 'rm -rf -- "$native_dir"' EXIT
cc -std=c11 -O2 -Wall -Wextra -Werror vision_core/tools/stream_v4l2_mjpeg.c -o "$native_dir/capture"
evidence_root="${XDG_STATE_HOME:-$HOME/.local/state}/sie/evidence"
mkdir -p "$evidence_root"
evidence=$(mktemp -d "$evidence_root/live_temporal_shadow_XXXXXXXX")
echo "Evidence: $evidence"

exit_code=0
timeout --signal=INT --kill-after=20s "${duration}s" "$runtime" -c \
  'from sie_ros2.ar0234_ov9281_control_preview_node import main; main()' \
  --ros-args -p project_root:="$PWD" -p evidence_dir:="$evidence" \
  -p control_preview_topic:=/sie/diagnostics/ar0234_ov9281/temporal_shadow_preview \
  -p ar_person_detector_enabled:=false -p confidence_threshold:=0.40 \
  -p preview_shadow_segmentation_input:=true \
  -p ar_intrinsic:="$artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json" \
  -p stereo_calibration:="$artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz" \
  -p activation_profile:="$activation" \
  -p shadow_segmenter_python:="$segmenter_python" -p shadow_segmenter_model:="$model" \
  -p preview_segmentation_engine:=onnxruntime -p preview_onnx_comparison_report:="$report" \
  -p preview_onnx_threads:=2 -p preview_rgb_backend:=single_segmentation \
  -p preview_rgb_transport:=mmap -p preview_local_surfaces_enabled:=false \
  -p preview_defer_local_surfaces:=true -p preview_fast_local_consistency:=true \
  -p preview_capture_backend:=latest_v4l2 -p preview_native_capture_executable:="$native_dir/capture" \
  -p preview_native_rgb_trigger:=true -p preview_overlap_stereo_decode:=true \
  -p preview_target_crop:=true -p preview_crop_lookup_backend:=blocked \
  -p preview_stereo_mode:=half_640x400 \
  -p preview_front_mount_profile:="$PWD/vision_core/config/diagnostics/robot_front_mount_operator_v1.json" \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 -p frame_rate_hz:=10.0 \
  -p preview_temporal_shadow_enabled:=true "${selection_args[@]}" "${clock_args[@]}" \
  || exit_code=$?
case "$exit_code" in
  0|124|130) ;;
  *) exit "$exit_code" ;;
esac
