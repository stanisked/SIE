# Shadow instance-mask depth: offline median 3x3 comparison

Date: 2026-10-14. Status: diagnostic implementation tested; 25-cycle Pi results pending.

## Purpose and evidence

Compare the frozen 2026-10-11 raw masked-depth replay with a small spatial
median, using exactly the same calibration, diagnostic range, stereo grid,
RGB masks, 100-point minimum and 0.050 m MAD cutoff. The baseline has 25
cycles: 16 coherent masked-depth hypotheses, six MAD refusals, three no target.
The 0.050 m cutoff is an existing diagnostic software choice, not an independently
validated depth-accuracy or motion tolerance. No runtime or activation changes.

Tool: `vision_core/tools/compare_shadow_mask_depth_median.py`.
Filter: `vision_core/person_localization/diagnostic_depth_median.py`.

## Method

1. Verify activation/calibration hashes, per-cycle image/mask/model binding and
   exact reproduction of the baseline depth hypotheses. Refuse inconsistent
   cycles or changed baseline results before creating comparison output.
2. Build full-resolution rectified-left depth from original LR-consistent
   disparity, within the explicitly declared 0.5 to 4.5 m diagnostic interval.
3. Project original points to AR0234. Eligibility requires the original inner
   ROI and a two-pixel-eroded person instance mask. A center is replaced by
   its spatial 3x3 median only when all nine original pixels are eligible.
   No image padding, hole filling, point deletion, temporal averaging or
   in-place source edits. Windows are evaluated once, not iteratively.
4. Reproject the filtered depth and repeat masked-depth evaluation on the
   unchanged stride-4 stereo grid. Also record a fixed-original-support result
   to distinguish smoothing from changes in RGB mask membership.
5. Record raw/filtered statuses, depths, MAD, layer/sample counts, mask-entry
   and exit counts, and full-resolution/grid correction statistics.

## Run on Pi

No camera capture, segmentation inference, ROS sourcing or colcon build is needed.
Use the existing companion venv containing OpenCV and NumPy:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
/home/elwis/dev_ws/runtime_envs/sie_seg_offline/bin/python \
  vision_core/tools/compare_shadow_mask_depth_median.py \
  --baseline-report /home/elwis/.local/state/sie/evidence/person_shadow_mask_20261011/replay_0p5_to_4p5m.json \
  --ar-intrinsic /home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json \
  --stereo-calibration /home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz \
  --activation /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json \
  --diagnostic-range-m 0.5 4.5 \
  --output /home/elwis/.local/state/sie/evidence/person_shadow_mask_20261011/median3x3_comparison.json
```

Output is exclusive: choose a new filename if the report or its evidence
directory exists. The companion evidence directory contains changed stereo
pixel coordinates and raw/filtered metric depths in compressed NPZ files.
When a rectified-left source image is present, it also contains overlays:
green outlines show eligible regions, yellow marks corrected pixels, red marks
corrections exceeding 50 mm. Red is only a visualization threshold.

## Interpretation and next step

Review all 22 target frames, including the six original refusals and any
previously coherent frame that becomes refused. Compare depth shifts, surviving
sample support, layer changes and correction overlays, not only the pass count.
Thin structures and internal depth discontinuities may be smoothed even when
the instance boundary is protected. Median smoothing can reduce MAD for genuine
surface variation and is not proof of improved metric accuracy. Filtered values
are derived estimates, not newly LR-validated stereo correspondences.

Keep the original live decisions, activated 2.0 m limit, calibration and disabled
Measurement/navigation/motor outputs. Review the replay results before considering
any live shadow integration. No new human or board capture is required for this
comparison. The earlier manual board series remains deferred.

## Validation performed

- Python syntax checks passed for the filter, comparison tool and tests.
- Seven unittest cases passed, including full synthetic geometric CLI replay,
  source-file preservation, no-target handling and refusal of a tampered baseline.
- Tests cover isolated spikes, invalid neighbors, ownership boundaries, a
  straight depth edge and a slope. A dedicated thin-surface test demonstrates
  the filter's known destructive behavior rather than assuming edge preservation.
- An algorithm-only smoke check on the two locally available earlier 800x1280
  disparity maps preserved invalid pixels and source files. It does not evaluate
  person masks or the six target refusals and is not a Pi performance benchmark.
- Full comparison on the 25-cycle Pi evidence is pending the command above.
