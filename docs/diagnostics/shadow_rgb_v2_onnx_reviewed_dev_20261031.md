# Shadow RGB v2: ONNX on reviewed development frames (2026-10-31)

This is an offline, camera-free check. It does not authorize Shadow motion or
change the active v1 model in the ROS camera node.

## Fixed inputs and policy

- v2 ONNX SHA-256: `5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b`.
- Labels archive SHA-256: `795511dd6d3e863902630550af347152bfb9a0c99b863c130c699b6d18c6dc44`.
- Image source archive SHA-256: `4fde6f8691ebc251b91bf2f44c501464a87e20fc839865beab2ce44fc3ba07c1`.
- Production letterbox and YOLO11 one-class decoder SHA-256: `2556c62b291ec28c6db5fb1787615e05f0c878ec0a6171d3115d80bc74329ff9`.
- Thresholds: confidence 0.40, NMS IoU 0.45, match IoU 0.50. 89 positive and 31 negative reviewed images, 5-second subsample.
- The evaluator verifies every image and label digest before inference.

## Results

| Pipeline | Matched positive images | False-positive negative images |
| --- | ---: | ---: |
| Previous v1 ONNX, same pre/post module | 71/89 | 0/31 |
| v2 PyTorch in Colab | 88/89 | 1/31 |
| v2 ONNX, production pre/post module, CPU | 89/89 | 1/31 |

The previous v1 ONNX and current v2 ONNX reports have the same validation archive
and pre/post module SHA-256. All 18 earlier v1 misses are matched by v2 ONNX.
The sole negative-image detection is `ar0234_000515.jpg`, confidence 0.42624,
box `[1827.99, 315.88, 1919.55, 1051.90]`, near the right image edge. Review
this annotation and the edge content before using it to tune thresholds. The
Colab/ONNX one-frame difference may arise from different preprocessing or
decoding and does not demonstrate a meaningful generalization difference.

These images were already reviewed during development and do not constitute a
blind test. We have not measured latency on Pi, continuous target identity,
stereo depth, or safe approach.

## Reproduce on Pi without motors

Download the verified v2 `best.onnx` from the Colab output folder onto the
laptop, and obtain the exact `shadow_rgb_validation_yolo_labels_20261030.tar.gz`
archive (SHA-256 shown above). Copy both from the laptop to Pi:

```bash
scp /path/to/downloaded/best.onnx \
    /path/to/shadow_rgb_validation_yolo_labels_20261030.tar.gz \
    elwis@100.70.100.29:/home/elwis/
```

On Pi, first verify both digests. Use an isolated worktree so this check does
not overwrite the main workspace or its untracked files. The evaluator does
not open a camera, UART or motor port.

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git fetch origin codex/shadow-rgb-detector-20261030
git worktree add --detach /home/elwis/dev_ws/sie_shadow_v2_eval_478d444 FETCH_HEAD
cd /home/elwis/dev_ws/sie_shadow_v2_eval_478d444
sha256sum /home/elwis/best.onnx \
  /home/elwis/shadow_rgb_validation_yolo_labels_20261030.tar.gz
/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/evaluate_shadow_rgb_dev_onnx.py \
  --repo-root . --model-version v2 \
  --model /home/elwis/best.onnx \
  --images-dir /home/elwis/.local/state/sie/evidence/shadow_rgb_presence_20261030_204750_ba9d67/annotation_subset_5s_v1/images \
  --labels-archive /home/elwis/shadow_rgb_validation_yolo_labels_20261030.tar.gz \
  --output /home/elwis/shadow_v2_onnx_pi_120.json
```

The result includes per-frame inference and complete JPEG-to-detections timing
on that Pi. If the source directory was moved, locate the corresponding
`annotation_subset_5s_v1/images` before running the command.

Before the next model decision, collect a **new** 10-minute, projector-off
session with two people entering and leaving alternately, plus empty frames.
Keep this session unseen by training and threshold selection. Label people and
absence independently, then report presence, IoU and negative-frame false
positives for v1 and v2 under the same Pi pipeline. Motor power remains off.

The existing camera-only capture command on Pi is:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/capture_shadow_rgb_presence.py \
  --scenario 'two_people_alternating_with_empty_frames_blind' \
  --projector-state off --duration-s 600 --sample-hz 2
```
