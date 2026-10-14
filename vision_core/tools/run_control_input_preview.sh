#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
directory="/home/elwis/.local/state/sie/evidence/control_input_preview_$(date +%Y%m%d_%H%M%S)"
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
times = [r['processing_timing']['host_capture_start_to_result_ready_s'] for r in rows[1:]]
print('CONTROL_PREVIEW_SUMMARY', 'records=', len(rows), 'statuses=', dict(Counter(r['status'] for r in rows)))
if times:
    print('host_median_s=', round(statistics.median(times), 4), 'host_max_s=', round(max(times), 4),
          'within_250ms=', sum(t <= .25 for t in times), '/', len(times))
print('Journal root:', sys.argv[1])
PY
