# Shadow moving-person camera evidence

Camera-only experiment. The cart remains stationary with motor power off. No
UART requests, ROS motion bridge, controller commands, or metric Measurement
are made by this wrapper. The person may move; adjacent detections do not
establish persistent identity.

## Setup and one run on Pi 5

Use the currently active OV9281 Pi v7 calibration (stereo SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`).
Keep the camera position and settings unchanged. With the cart fixed and motor
power off, let one person move slowly sideways and toward/away from the cameras
over an operator-observed approximate 1.5–3 m range. Include visible loss and
reacquisition if it happens naturally. Keep other people out of frame when
possible; do not treat an RGB box as proof of the same person's identity.

From a checkout containing this script:

```bash
SHADOW_OPERATOR_SCENARIO='one moving person, operator-estimated 1.5-3 m, transverse and depth motion' \
  bash vision_core/tools/run_shadow_moving_person_capture.sh
```

The wrapper sets a 60 s camera capture (override with integer
`SHADOW_CAPTURE_DURATION_S` in 10..120), retains the existing full-resolution
AR0234/OV9281 evidence, replays the frozen mask and disparity at an explicit
**diagnostic** 0.5–4.5 m interval, then writes
`shadow_temporal_depth_report.json` in the printed evidence root. The v1
activation profile used by this wrapper records 0.5–2.0 m; a separate
conditional v2 profile exists, but this experiment does not use or change it.
Replay beyond the v1 range does not change authorization. The wrapper uses the deployed ROS build in
`/home/elwis/dev_ws/sie_v6_temperature_disabled_mvp/install` and the runtime
environment `/home/elwis/dev_ws/runtime_envs/sie_mvp`.

## Read the result

Report the printed evidence root and the `SUMMARY` line. Examine
`coherent_fraction_of_rgb_candidate_cycles`, the per-cycle sequence, and
`gaps_between_coherent_observations`. The latter measures time **between
sampled coherent observations** containing one or more refused samples. It
does not prove that depth remained absent throughout the interval. The
`rgb_center_x_span_px` describes image motion, not physical velocity or
identity. Audit host capture timestamps and pair skew are not exposure times.

The session does not supply an independently surveyed person range, front
clearance, exposure synchronization, or a physical stopping bound. It cannot
authorize a Shadow motor run or a calibrated range expansion.
