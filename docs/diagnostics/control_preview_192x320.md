# Reviewed 192x320 live diagnostic preview

2026-10-16. Explicit diagnostic rollout authorized by operator. No Measurement,
navigation or motor commands. Default reviewed input remains 256x384.

## Reviewed offline evidence

`/home/elwis/.local/state/sie/evidence/segmentation_input_192x320_20261016_130842/comparison.json`:
44 comparisons on 22 saved cycles (two passes), inputs unchanged. Segmentation
median 96.677ms reference versus 79.019ms candidate (17.658ms saving).
Mask IoU median/min 0.9721894940444458/0.9652224742582682. No target-selection,
selected-correspondence or geometry-status changes. All 44 depth statuses remain
COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY. Absolute median-depth delta median 0,
maximum 0.010505437850952148m. This delta is a difference between algorithms, not
measured physical error.

Four unmatched instances occur in cycles 00000014 and 00000020, each twice. The
candidate detects two instances instead of one, but both configurations retain
one eligible target, index 0, with unambiguous selected correspondence and zero
depth delta in those cycles. Selected confidence changes approximately
0.9223 -> 0.9121 and 0.9222 -> 0.9163. The report does not preserve the rejected
instance's bbox/confidence, so its exact rejection cause is unknown. These data
support this diagnostic trial, not general multi-target or dynamic-scene quality.

## Artifact binding and unchanged gates

The adapter separately verifies the new report status, new model SHA, static
FP32 export settings, 192x320 input fingerprints, 22-cycle/two-repeat coverage,
reviewed output statistics, selected correspondence and exact observed extra
instance pattern. It follows the hashed reviewed 256x384 report back to the
original exact PyTorch/ONNX review/checkpoint and checks package versions.
Observed mask-IoU/depth changes are pinned evidence, not newly invented live
quality thresholds. Other unreviewed reports do not qualify through this path.

The explicit launcher selects only the known 192x320 artifact/report. The
underlying 256x384 launcher keeps its default and refuses unknown input-mode
names. Active delegated ORT settings remain separately logged; compatibility
thread argument 2 is not claimed to describe the actual active session budget.

The same original-JPEG/new-RGB/fresh-stereo-during-segmentation/crop pipeline is
used. Target selection, confidence/edge rules, temporal matching, calibration,
diagnostic depth range, LR consistency, support, layer and MAD checks remain
unchanged. Local surfaces stay disabled. Neither timing nor depth is promoted
to an admitted control input. Quality assessment revision 2 remains in force.

## Pi run, person in view

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_192x320.sh
```

Confirm startup verifies shape [1,3,192,320]. Review MAD-coherent count, current
target refusals, frame-age median/max/within-250ms count, and journal errors.
The 17.658ms isolated saving is not a live freshness guarantee. Actual live
results are pending and scene-to-scene changes must not be attributed to size
without paired evidence.

Rollback: fresh shell with `run_control_input_preview_fresh_stereo.sh`, or set
`SIE_PREVIEW_REVIEWED_INPUT_MODE=256x384` before that launcher.

## Local checks

14 tests passed across reviewed ONNX binding, input-size comparison and single
segmentation. The new test rejects changed model/parent hashes, wrong shape,
repeat coverage, changed mask/depth results, changed unmatched-instance pattern
or additional eligible target. Two modified Python files compiled; both changed
shell launchers passed syntax checks. No real camera, ROS or ONNX live execution
was performed in the implementation environment.
