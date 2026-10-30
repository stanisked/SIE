# Shadow AR0234 detector screen: frozen moving-person images

This is an **offline diagnostic**, with motor power off. It neither modifies
the deployed RGB confidence threshold nor approves camera-based motion.

The source archive `shadow_rgb_dropouts_20261030.tar.gz` is SHA-256
`84c64b685949fe62d031a13bf28285011f036c1b8f59d5387930a0c05b9a85af`.
Each of its 13 selected AR0234 JPEGs visibly contains a person. The image and
audit hashes are recorded in `shadow_rgb_dropouts_20261030_labels.json`. The
operator confirmed that the 650 nm dot projector was ON in this series.
These are presence labels, **not** hand-drawn person boxes, person identity,
range truth, or a random representative sample. No empty-room frames were
provided, so specificity and false-positive rate are unknown.

## Partial result from operator-reported JPEG replay

Nine saved-JPEG maximum confidence scores were supplied by the operator and
frozen in `shadow_rgb_reported_scores_20261030.json`. Four other labeled
frames have **no supplied score**, so the denominator below is nine, not 13.

| Threshold | Positive JPEGs with any proposal above threshold | Denominator |
| --- | ---: | ---: |
| 0.70 (ROS node's source default) | 4 | 9 |
| 0.40 | 5 | 9 |
| 0.10 | 6 | 9 |

The original ROS audits, interpreted independently of the saved-JPEG replay,
contained an RGB candidate for frames 03, 09, 17, 19, 20, 21 and 25. Frames
20 and 21 subsequently failed a separate left-image-edge geometry check.
Frames 04, 05, 06, 22, 23 and 24 were `NO_TARGET`. The saved JPEG of frame 04
replays at score 0.7311 while the original audit was `NO_TARGET`: the cause is
still unknown. JPEG decoding/re-encoding and an unrecorded effective parameter
are possibilities, not established causes. Raw model output from the original
in-memory frame was not saved.

One high-scoring proposal is **not** a verified correct person box. Multiple
raw YOLO proposals may overlap one person. These numbers cannot demonstrate
true positive localization, operational recall, or a safe threshold change.
Even perfect RGB detection would not fix the separate stereo depth refusals.
The active OV9281 Pi v7 calibration and all Measurement gates stay unchanged.

## Reproduce full 13-frame saved-JPEG replay on Pi

Run this from the Pi workspace **after** bringing in the new script and label
file, while the original evidence is still present. The model is checked
against the hard-coded SHA-256 in the existing YOLO adapter. The command
opens only files and writes one new report; no camera or controller access.

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
PYTHONPATH="$PWD" /home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/analyze_shadow_rgb_detector.py \
  --evidence-root /home/elwis/.local/state/sie/evidence/shadow_moving_person_20261030_142721 \
  --labels docs/diagnostics/shadow_rgb_dropouts_20261030_labels.json \
  --model /home/elwis/dev_ws/model_artifacts/ar0234_person_upper_body_yolo11n_v1/best.onnx \
  --output /home/elwis/.local/state/sie/evidence/shadow_rgb_detector_full_20261030.json
```

If the full evidence root no longer contains all 13 identical files, first
extract the unchanged source archive to a separate directory and pass that
directory as `--evidence-root`; hashes must match before inference. The full
report adds raw proposal counts and **post-NMS counts** for each threshold.
Keep score screening separate from actual correct-box verification. To decide
whether to tune or replace the model, add hand-drawn person boxes and genuinely
person-absent scenes under the same capture settings, then compare detection
and false-alarm behavior across perspectives. Save the live detector's
original score and effective threshold per cycle on the next capture to
resolve the frame-04 mismatch.
