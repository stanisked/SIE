#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
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
activation_profile="/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json"
profile_args=()
if [ "${SIE_PREVIEW_EXISTING_STATIC_V2:-false}" = true ]; then
  activation_profile="/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json"
  "$runtime" - "$activation_profile" <<'CHECK_PROFILE'
import sys
from vision_core.person_localization.existing_static_profile_preview import load_existing_static_profile
load_existing_static_profile(sys.argv[1])
print('STATIC_V2_PROFILE_VERIFIED: existing profile; static checks before stereo; motors disabled')
CHECK_PROFILE
  profile_args+=(-p preview_existing_static_profile_v2:=true)
fi
front_args=()
if [ -n "${SIE_PREVIEW_FRONT_MOUNT_PROFILE:-}" ]; then
  front_args+=(-p preview_front_mount_profile:="$SIE_PREVIEW_FRONT_MOUNT_PROFILE")
fi
case "$mode" in full_1280x800|half_640x400) ;; *) echo "Unsupported diagnostic mode: $mode" >&2; exit 2 ;; esac
directory="/home/elwis/.local/state/sie/evidence/control_input_preview_${mode}_${backend}_${engine}_$(date +%Y%m%d_%H%M%S)"
echo "Journal root: $directory"
exit_code=0
duration="${SIE_PREVIEW_DURATION_S:-30}"
case "$duration" in 30|120) ;; *) echo "Unsupported preview duration" >&2; exit 2 ;; esac
timeout_mode=()
# The approach runner owns a fresh process group and must be able to stop the
# entire preview tree. GNU timeout otherwise moves its child into another
# process group, leaving camera output alive after the runner has exited.
if [ "$duration" = 120 ]; then timeout_mode=(--foreground); fi
timeout "${timeout_mode[@]}" --signal=INT --kill-after=10s "${duration}s" "$runtime" -c \
  'from sie_ros2.ar0234_ov9281_control_preview_node import main; main()' \
  --ros-args \
  -p project_root:="$PWD" \
  -p model_path:=/home/elwis/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx \
  -p ar_intrinsic:=/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  -p stereo_calibration:=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  -p activation_profile:="$activation_profile" "${profile_args[@]}" \
  -p shadow_segmenter_python:="$segmenter_python" \
  -p shadow_segmenter_model:="$segmenter_model" \
  "${engine_args[@]}" "${front_args[@]}" \
  -p preview_local_surfaces_enabled:="${SIE_PREVIEW_LOCAL_ENABLED:-true}" \
  -p preview_local_surface_every_n_cycles:="${SIE_PREVIEW_LOCAL_EVERY_N:-1}" \
  -p preview_defer_local_surfaces:="${SIE_PREVIEW_DEFER_LOCAL:-false}" \
  -p preview_rgb_backend:="$backend" \
  -p preview_rgb_transport:="${SIE_PREVIEW_RGB_TRANSPORT:-npz}" \
  -p preview_fast_local_consistency:="${SIE_PREVIEW_FAST_LOCAL_CONSISTENCY:-false}" \
  -p preview_fresh_stereo_during_rgb:="${SIE_PREVIEW_FRESH_STEREO_DURING_RGB:-false}" \
  -p preview_native_rgb_jpeg:="${SIE_PREVIEW_NATIVE_RGB_JPEG:-false}" \
  -p preview_overlap_stereo_decode:="${SIE_PREVIEW_OVERLAP_STEREO_DECODE:-false}" \
  -p preview_native_rgb_trigger:="${SIE_PREVIEW_NEW_RGB_TRIGGER:-false}" \
  -p preview_capture_backend:="${SIE_PREVIEW_CAPTURE_BACKEND:-opencv}" \
  -p preview_native_capture_executable:="${SIE_PREVIEW_NATIVE_CAPTURE_EXECUTABLE:-}" \
  -p preview_crop_lookup_backend:="${SIE_PREVIEW_CROP_LOOKUP_BACKEND:-reference}" \
  -p preview_target_crop:="${SIE_PREVIEW_TARGET_CROP:-false}" \
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
print('RGB_TRANSPORT:',dict(Counter(r.get('segmentation_observation',{}).get('input_transport') for r in rows)))
print('SHAPE:', dict(Counter(r.get('shape_diagnostic',{}).get('status') for r in rows)))
print('MASK_DEPTH:', dict(Counter(r.get('shape_diagnostic',{}).get('mask_depth',{}).get('status') for r in rows)))
for key in ('rgb_detector_s','rgb_segmentation_round_trip_s','stereo_worker_s'):
    values=[r[key] for r in rows[1:] if r.get(key) is not None and r['status']!='CANCELLED_SHUTDOWN']
    if values: print(key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
print('CONTROL_PREVIEW_SUMMARY', 'records=', len(rows), 'statuses=', dict(Counter(r['status'] for r in rows)))
if times:
    print('host_median_s=', round(statistics.median(times), 4), 'host_max_s=', round(max(times), 4),
          'within_250ms=', sum(t <= .25 for t in times), '/', len(times),
          'within_500ms_experimental=', sum(t <= .5 for t in times), '/', len(times))
for key in ('mask_s','mask_depth_prepare_s','surface_s','round_trip_s'):
    values=[r.get('shape_diagnostic',{}).get(key) for r in rows[1:] if r['status']!='CANCELLED_SHUTDOWN']
    values=[v for v in values if v is not None]
    if values: print(key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
depths=[r.get('shape_diagnostic',{}).get('mask_depth',{}).get('candidate_depth_m') for r in rows if r['status']!='CANCELLED_SHUTDOWN']
depths=[d for d in depths if d is not None]
if depths: print('MASK_CANDIDATE_DEPTH median_m=',round(statistics.median(depths),4),'span_m=',round(max(depths)-min(depths),4))
fast=[r.get('shape_diagnostic',{}).get('fast_local_consistency') for r in rows[1:] if r['status']!='CANCELLED_SHUTDOWN']
fast=[f for f in fast if f is not None]
print('FAST_LOCAL:',dict(Counter(f.get('status') for f in fast)))
for label,values in (
    ('fit_ms',[f['fit_s']*1000 for f in fast]),
    ('residual_MAD_mm',[f['pooled_held_out_axial_residual']['mad_m']*1000 for f in fast if 'mad_m' in f.get('pooled_held_out_axial_residual',{})]),
    ('residual_p90_mm',[f['pooled_held_out_axial_residual']['absolute_p90_m']*1000 for f in fast if 'absolute_p90_m' in f.get('pooled_held_out_axial_residual',{})]),
    ('fitted_fraction',[f['fitted_sample_fraction'] for f in fast])):
    if values:print('FAST_LOCAL',label,'n=',len(values),'median=',round(statistics.median(values),4),'max=',round(max(values),4),'min=',round(min(values),4))
local_files=sorted(Path(sys.argv[1]).rglob('local-surface-*.json'))
local_rows=[json.loads(p.read_text()) for p in local_files]
print('LOCAL_SAMPLING:',dict(Counter(r.get('local_surface_async',{}).get('status') for r in rows)))
print('LOCAL_SURFACE_ASYNC records=',len(local_rows),'statuses=',dict(Counter(r['status'] for r in local_rows)))
for key in ('queue_wait_s','local_processing_s','submit_to_result_s'):
    values=[r[key] for r in local_rows if r.get(key) is not None]
    if values:print('LOCAL',key,'median_s=',round(statistics.median(values),4),'max_s=',round(max(values),4))
steady=[r for r in rows[1:] if r['status']!='CANCELLED_SHUTDOWN']
def show_stages(prefix, mappings):
    keys=sorted({key for mapping in mappings for key in mapping})
    for key in keys:
        values=[mapping[key] for mapping in mappings if isinstance(mapping.get(key),(int,float)) and not isinstance(mapping.get(key),bool)]
        if values:print(prefix,key,'n=',len(values),'median_ms=',round(statistics.median(values)*1000,2),'max_ms=',round(max(values)*1000,2))
show_stages('FOREGROUND',[r.get('foreground_timing_s',{}) for r in steady])
show_stages('RGB_RPC',[r.get('segmentation_observation',{}).get('rpc_timing_s',{}) for r in steady])
show_stages('FUSION_RPC',[r.get('shape_diagnostic',{}).get('rpc_timing_s',{}) for r in steady])
show_stages('TIMELINE',[{k:v for k,v in r.get('perception_timeline_s',{}).items() if k in ('stereo_queue_s','rgb_stereo_wall_interval_overlap_s','rgb_stereo_decode_wall_interval_overlap_s')} for r in steady])
show_stages('RGB_WORKER',[{k:r.get('segmentation_observation',{}).get(k) for k in ('worker_input_open_verify_s','worker_rgb_load_s','worker_jpeg_decode_s','prediction_extract_s')} for r in steady])
if steady:
    print('ACTIVE_ORT_SETTINGS:',steady[0].get('segmentation_observation',{}).get('segmentation_engine',{}).get('active_ort_session'))
from sie_ros2.latest_v4l2_capture import diagnostic_freshness_groups
coherent=[r for r in steady if r.get('status') in ('MASKED_DEPTH_SHADOW_DIAGNOSTIC_ONLY','SHAPE_AWARE_SHADOW_DIAGNOSTIC_ONLY') and r.get('shape_diagnostic',{}).get('mask_depth',{}).get('status')=='COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY']
show_stages('FRAME_AGE_COHERENT',[r.get('frame_freshness',{}).get('driver_declared_soe_to_result_s',{}) for r in coherent])
show_stages('FRAME_AGE_AT_SELECTION',[r.get('frame_freshness',{}).get('driver_declared_soe_to_pair_selection_s',{}) for r in coherent])
show_stages('FRAME_SELECTION_TO_RESULT',[r.get('frame_freshness',{}).get('pair_selection_to_result_s',{}) for r in coherent])
show_stages('FRAME_PAIR_COHERENT',[{'driver_skew_s':r.get('frame_freshness',{}).get('driver_declared_pair_skew_s')} for r in coherent])
for label,group in diagnostic_freshness_groups(steady).items():
    print('RESULT_FRESHNESS',label,json.dumps(group))
print('FRESHNESS_NOTE: first/cancelled excluded; coherent diagnostics are not admitted Measurement; driver SOE not independently verified.')
print('TIMING_NOTE: FOREGROUND durations exclusive per cycle; RPC/worker/timeline are nested or overlapping; do not sum across groups.')
front=[r.get('shape_diagnostic',{}).get('front_geometry') for r in rows]
front=[f for f in front if isinstance(f,dict)]
if front:
    print('FRONT_GEOMETRY:',dict(Counter(f.get('status') for f in front)))
    terms=[f['known_term_m']['median'] for f in front if f.get('known_term_m')]
    if terms:print('FRONT_KNOWN_TERM median_m=',round(statistics.median(terms),4),'optical_datum_offset=UNKNOWN')
    timing=[f['processing_s'] for f in front if 'processing_s' in f]
    if timing:print('FRONT_GEOMETRY processing_median_ms=',round(1000*statistics.median(timing),3))
print('STATIC_PROFILE_GATE:',dict(Counter(r.get('existing_profile_static_gate',{}).get('status','NOT_EVALUATED') for r in rows)))
print('CONTROL_DECISION_PREVIEW:',dict(Counter(r.get('decision_preview',{}).get('status') for r in rows)))
print('CONTROL_FAILED_CHECKS:',dict(Counter(x for r in rows for x in r.get('decision_preview',{}).get('failed_checks',[]))))
print('CONTROL_MISSING_CHECKS:',dict(Counter(x for r in rows for x in r.get('decision_preview',{}).get('missing_checks',[]))))
assessment=[r.get('control_input_shadow',{}).get('assessment_processing_s') for r in rows[1:]]
assessment=[v for v in assessment if isinstance(v,(int,float))]
if assessment:print('CONTROL_ASSESSMENT_MS median/max:',round(1000*statistics.median(assessment),3),round(1000*max(assessment),3))
print('Journal root:', sys.argv[1])
PY
