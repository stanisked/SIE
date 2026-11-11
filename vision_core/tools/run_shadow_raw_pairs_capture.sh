#!/usr/bin/env bash
# Camera-only, full-resolution evidence capture for mask-depth refusals.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

companion=/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
rgb_model="${SHADOW_RGB_MODEL_PATH:-/home/elwis/best.onnx}"
rgb_model_version="${SHADOW_RGB_MODEL_VERSION:-v2}"
case "$rgb_model_version" in
  v2) rgb_model_sha=5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b ;;
  v3) rgb_model_sha=fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5 ;;
  *) echo "SHADOW_RGB_MODEL_VERSION must be v2 or v3" >&2; exit 2 ;;
esac
if [[ ! -f "$rgb_model" ]] || [[ "$(sha256sum "$rgb_model" | cut -d ' ' -f1)" != "$rgb_model_sha" ]]; then
  echo "Shadow RGB $rgb_model_version model missing or SHA mismatch: $rgb_model" >&2
  exit 2
fi
default_evidence_root="/home/elwis/.local/state/sie/evidence/shadow_raw_pairs_$(date +%Y%m%d_%H%M%S)"
evidence_root="${SHADOW_RAW_EVIDENCE_ROOT:-$default_evidence_root}"
duration_s="${SHADOW_CAPTURE_DURATION_S:-35}"
if [[ ! "$duration_s" =~ ^(0|[1-9][0-9]*)$ ]] || (( duration_s < 10 || duration_s > 120 )); then
  echo "SHADOW_CAPTURE_DURATION_S must be an integer from 10 to 120" >&2
  exit 2
fi
if [[ -e "$evidence_root" ]]; then
  echo "Evidence root already exists: $evidence_root" >&2
  exit 2
fi
echo "Evidence root: $evidence_root"
exit_code=0
timeout --signal=INT --kill-after=10s "${duration_s}s" "$runtime" \
  install/sie_ros2/lib/sie_ros2/ar0234_ov9281_dynamic_association_audit_node \
  --ros-args \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 \
  -p project_root:="$PWD" \
  -p model_path:="$rgb_model" -p model_version:="$rgb_model_version" -p confidence_threshold:=0.40 \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  -p evidence_capture_enabled:=true -p evidence_dir:="$evidence_root" \
  -p shadow_instance_mask_enabled:=true \
  -p shadow_segmenter_python:="$companion" \
  -p shadow_segmenter_model:=/home/elwis/dev_ws/model_artifacts/yolo11n_seg_coco/yolo11n-seg.pt \
  -p shadow_local_surfaces_enabled:=false \
  -p shadow_diagnostic_range_min_m:=0.5 -p shadow_diagnostic_range_max_m:=4.5 \
  || exit_code=$?
if [ "$exit_code" -ne 0 ] && [ "$exit_code" -ne 124 ] && [ "$exit_code" -ne 130 ]; then
  exit "$exit_code"
fi
"$runtime" vision_core/tools/summarize_shadow_raw_pairs.py "$evidence_root"
