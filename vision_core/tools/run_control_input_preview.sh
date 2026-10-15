#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
mode="${SIE_PREVIEW_STEREO_MODE:-full_1280x800}"
backend="${SIE_PREVIEW_RGB_BACKEND:-two_model_reference}"
engine="${SIE_PREVIEW_SEGMENTATION_ENGINE:-pytorch}"
segmenter_python="${SIE_PREVIEW_SEGMENTER_PYTHON:-/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python}"
segmenter_model="${SIE_PREVIEW_SEGMENTER_MODEL:-/home/elwis/dev_ws/model_artifacts/yolo11n_seg_coco/yolo11n-seg.pt}"
engine_args=(-p preview_segmentation_engine:="$engine")
case "$engine" in
  pytorch) ;;
  onnxruntime)
    : "${SIE_PREVIEW_ONNX_REPORT:?Set exact reviewed comparison report}"
    engine_args+=(-p preview_onnx_comparison_report:="$SIE_PREVIEW_ONNX_REPORT" -p preview_onnx_threads:="${SIE_PREVIEW_ORT_THREADS:-2}") ;;
  *) echo "Unsupported segmentation engine: $engine" >&2; exit 2 ;;
esac
case "$mode" in full_1280x800|half_640x400) ;; *) echo "Unsupported diagnostic mode: $mode" >&2; exit 2 ;; esac
directory="/home/elwis/.local/state/sie/evidence/control_input_preview_${mode}_${backend}_${engine}_$(date +%Y%m%d_%H%M%S)"
echo "Journal root: $directory"
exit_code=0
timeout --signal=INT --kill-after=10s 30s "$runtime" -c \
  'from sie_ros2.ar0234_ov9281_control_preview_node import main; main()' \
  --ros-args \
  -p project_root:="$PWD" \
  -p model_path:=/home/elwis/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:=/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  -p shadow_segmenter_python:="$segmenter_python" \
  -p shadow_segmenter_model:="$segmenter_model" \
  "${engine_args[@]}" \
  -p preview_local_surface_every_n_cycles:="${SIE_PREVIEW_LOCAL_EVERY_N:-1}" \
  -p preview_defer_local_surfaces:="${SIE_PREVIEW_DEFER_LOCAL:-false}" \
  -p preview_rgb_backend:="$backend" \
  -p preview_stereo_mode:="$mode" \
  -p stereo_match_execution:=parallel -p stereo_opencv_threads:=1 \
  -p frame_rate_hz:=10.0 -p evidence_dir:="$directory" \
  || exit_code=$?
if [ "$exit_code" -ne 0 ] && [ "$exit_code" -ne 124 ] && [ "$exit_code" -ne 130 ]; then
  exit "$exit_code"
fi
"$runtime" - "$directory" <<'PY'
from pathlib import Path
from collections import Counter
import json, statistics, sys
files = sorted(Path(sys.argv[1]).rglob('ar0234-ov9281-control-preview-*.json'))
rows = [json.loads(p.read_text()) for p in files]
times = [r['processing_timing']['host_capture_start_to_result_ready_s'] for r in rows[1:]
         if r['status'] != 'CANCELLED_SHUTDOWN']
print('SHAPE:', dict(Counter(r.get('shape_diagnostic',{}).get('status') for r in rows)))
print('MASK_DEPTH:', dict(Counter(r.get('shape_diagnostic',{}).get('mask_depth',{}).get('status') for r in rows)))
for key in ('rgb_detector_s','rgb_segmentation_round_trip_s','stereo_worker_s'):
    values=[r[key] for r in rows[1:] if r.get(key) is not None and r['status']!='CANCELLED_SHUTDOWN']
    if values: print(key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
print('CONTROL_PREVIEW_SUMMARY', 'records=', len(rows), 'statuses=', dict(Counter(r['status'] for r in rows)))
if times:
    print('host_median_s=', round(statistics.median(times), 4), 'host_max_s=', round(max(times), 4),
          'within_250ms=', sum(t <= .25 for t in times), '/', len(times))
for key in ('mask_s','mask_depth_prepare_s','surface_s','round_trip_s'):
    values=[r.get('shape_diagnostic',{}).get(key) for r in rows[1:] if r['status']!='CANCELLED_SHUTDOWN']
    values=[v for v in values if v is not None]
    if values: print(key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
depths=[r.get('shape_diagnostic',{}).get('mask_depth',{}).get('candidate_depth_m') for r in rows if r['status']!='CANCELLED_SHUTDOWN']
depths=[d for d in depths if d is not None]
if depths: print('MASK_CANDIDATE_DEPTH median_m=',round(statistics.median(depths),4),'span_m=',round(max(depths)-min(depths),4))
local_files=sorted(Path(sys.argv[1]).rglob('local-surface-*.json'))
local_rows=[json.loads(p.read_text()) for p in local_files]
print('LOCAL_SAMPLING:',dict(Counter(r.get('local_surface_async',{}).get('status') for r in rows)))
print('LOCAL_SURFACE_ASYNC records=',len(local_rows),'statuses=',dict(Counter(r['status'] for r in local_rows)))
for key in ('queue_wait_s','local_processing_s','submit_to_result_s'):
    values=[r[key] for r in local_rows if r.get(key) is not None]
    if values:print('LOCAL',key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
print('Journal root:', sys.argv[1])
PY
