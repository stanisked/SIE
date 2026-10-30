#!/usr/bin/env bash
# Camera-only moving-person evidence, then frozen-mask depth replay and timeline.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
evidence_root="/home/elwis/.local/state/sie/evidence/shadow_moving_person_$(date +%Y%m%d_%H%M%S)"
export SHADOW_RAW_EVIDENCE_ROOT="$evidence_root"
export SHADOW_CAPTURE_DURATION_S="${SHADOW_CAPTURE_DURATION_S:-60}"
scenario="${SHADOW_OPERATOR_SCENARIO:-moving_person_operator_observation_no_range_truth}"

# Existing capture wrapper uses the deployed ROS build and the active v7 files.
bash "$repo_root/vision_core/tools/run_shadow_raw_pairs_capture.sh"
cd "$repo_root"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
"$runtime" vision_core/tools/replay_shadow_mask_depth_range.py \
  --evidence-root "$evidence_root" \
  --ar-intrinsic /home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  --stereo-calibration /home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  --activation /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  --diagnostic-range-m 0.5 4.5 \
  --output "$evidence_root/shadow_mask_replay.json"
"$runtime" vision_core/tools/analyze_shadow_temporal_depth.py \
  --evidence-root "$evidence_root" \
  --replay-report "$evidence_root/shadow_mask_replay.json" \
  --operator-scenario "$scenario" \
  --output "$evidence_root/shadow_temporal_depth_report.json"
