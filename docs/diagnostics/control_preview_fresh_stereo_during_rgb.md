# Fresh stereo arrival during current RGB segmentation

2026-10-16. Opt-in diagnostic mode; live validation pending. No Measurement,
navigation or motor commands. Default capture scheduling remains unchanged.

## Measured trigger

Original-JPEG run `control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_123558`:
70 steady cycles, 32 within the 250ms driver-age target, 38 above it. In the
38 exceeded cycles, OV9281 was older in 32, AR0234 in six. Median OV9281
received-to-selection wait was 28.19ms in exceeded cycles versus 12.24ms in
within-target cycles. Driver-to-dequeue remained about 16ms and JPEG transfer
less than 1ms. AR0234 received-to-selection remained about 0.07ms. This identifies
already received stereo-frame waiting as a candidate to address; it does not
prove the cameras or driver have a defect.

## New scheduling

`preview_fresh_stereo_during_rgb=true` requires original native RGB JPEG, new-RGB
arrival triggering and the existing single segmentation/crop/decode-overlap mode.

1. Freeze a newly arrived original RGB JPEG and its driver metadata.
2. Submit segmentation for that same cycle immediately.
3. While segmentation runs, snapshot OV9281 with both sequence advancement and
   packet-received-time at or after the stereo wait request. No cached-frame
   fallback is allowed on timeout or reader error.
4. Complete the pair metadata, then decode the selected stereo JPEG while the
   segmentation request remains active where scheduling permits.
5. Await the current target, apply existing temporal-skew/geometry gates, then
   calculate stereo crop and masked depth for this cycle only.

RGB payload, driver timestamp and sequence never change when completing the
pair. `rgb_packet_frozen_monotonic_s` records its earlier selection. The common
`pair_selected_monotonic_s` records when both packets are fixed, preserving the
existing age decomposition. The OV metadata records the minimum received-time
used to request its snapshot. Driver clocks/skew remain unmodified. This is
software association, not hardware synchronization.

`capture_stereo_select_s` is the foreground wait duration;
`rgb_stereo_arrival_wait_overlap_s` measures its actual overlap with RGB RPC.
These nested timing groups must not be summed. Original JPEG decode, matching,
mask ownership, calibration, range and quality thresholds remain unchanged.

A timeout refuses the cycle and drains the started segmentation request. An
excessive driver timestamp skew refuses before stereo matching. No prior depth,
frame or target is substituted. Interrupts during the new wait also drain the
started request before propagating. Local diagnostics remain disabled in this
latency launcher. Driver timing remains diagnostic, not physical command-age
certification or motor permission.

## Pi launch

Person in view; same camera placement. One short diagnostic run:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_fresh_stereo.sh
```

Inspect frame ages, MAD statuses, pair-skew/timeouts and actual wait overlap.
Success cannot be inferred from host duration alone or fast refusals. Any
improvement in final frame age must be measured in this run.
Rollback: new shell with the previous `run_control_input_preview_original_jpeg.sh`,
or explicitly set `SIE_PREVIEW_FRESH_STEREO_DURING_RGB=false` before using it.

## Local validation

24 tests passed across control scheduling, latest native capture, quality-shadow
assessment and original JPEG transport. Tests cover active same-cycle inference
during fresh stereo wait, source RGB preservation, matched pair completion,
no fallback on timeout, unchanged excessive-skew refusal and draining without
stale maps. Four Python files compiled and both shell scripts passed syntax
checks. Real cameras/ROS/ONNX are not available in the implementation environment;
Pi live results are pending.
