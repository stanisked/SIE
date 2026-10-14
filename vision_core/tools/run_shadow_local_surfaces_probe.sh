#!/usr/bin/env bash
# One supervised diagnostic capture. No Measurement or motion publisher.
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
companion=/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
"$companion" -c 'import scipy, cv2, ultralytics; print("Companion SciPy:", scipy.__version__, "OpenCV:", cv2.__version__, flush=True)'
"$runtime" -c 'import rclpy, onnxruntime, sie_ros2; print("ROS / ONNX ready", flush=True)'
evidence_root="/home/elwis/.local/state/sie/evidence/person_live_local_surfaces_$(date +%Y%m%d_%H%M%S)"
echo "Evidence root: $evidence_root"
exit_code=0
timeout --signal=INT --kill-after=10s 60s "$runtime" \
  install/sie_ros2/lib/sie_ros2/ar0234_ov9281_dynamic_association_audit_node \
  --ros-args \
  -p project_root:="$PWD" \
  -p model_path:=/home/elwis/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  -p evidence_capture_enabled:=true -p evidence_dir:="$evidence_root" \
  -p shadow_instance_mask_enabled:=true \
  -p shadow_segmenter_python:="$companion" \
  -p shadow_segmenter_model:=/home/elwis/dev_ws/model_artifacts/yolo11n_seg_coco/yolo11n-seg.pt \
  -p shadow_local_surfaces_enabled:=true \
  -p shadow_diagnostic_range_min_m:=0.5 -p shadow_diagnostic_range_max_m:=4.5 \
  || exit_code=$?
if [ "$exit_code" -ne 0 ] && [ "$exit_code" -ne 124 ] && [ "$exit_code" -ne 130 ]; then
  exit "$exit_code"
fi
echo "Saved evidence root: $evidence_root"
"$runtime" vision_core/tools/summarize_shadow_stage_timing.py "$evidence_root"
