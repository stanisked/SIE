# Original RGB JPEG control-input preview

Status: opt-in diagnostic implementation, live validation pending (2026-10-16).
No Measurement publication, navigation or motor commands are enabled.

## Offline evidence

Pi report: `/home/elwis/.local/state/sie/evidence/jpeg_rgb_transport_20261016_122332.json`.
On the same saved source JPEGs, reference parent decode plus mmap median/max was
0.152358/0.159425 s; original JPEG transport was 0.135142/0.140553 s.
All 44 comparisons had exactly equal decoded pixels, instances/masks, target
selection and complete depth results. All 44 evaluated depth. Inputs unchanged.
This is an isolated offline comparison including diagnostic fingerprints, not a
live end-to-end latency result or proof of physical depth accuracy.

## Implementation and ownership

`preview_native_rgb_jpeg=true` requires native continuous capture, single
segmentation, current-target stereo crop and RGB/stereo-decode overlap. Defaults
remain unchanged. The dedicated launcher enables this combination.

Both compressed sensor packets are selected and frozen before processing, with
the existing new-RGB arrival trigger and driver timestamp/skew checks. The parent
passes immutable original AR0234 JPEG bytes to `infer_jpeg`, without decoding or
recompressing the RGB image. It decodes the selected stereo JPEG concurrently.
The segmentation worker verifies the packet's cycle/token/hash and decodes RGB
once; decoded shape must be 1200x1920x3 before inference can succeed. The known
1920-pixel capture width is used for geometry only after a successful current
segmentation response. A worker decode failure cannot yield a target or depth.

The returned masks remain in the worker's current-cycle cache. Selected-instance
fusion retains cycle, model/input binding and bbox validation. Started requests
are drained before the next cycle even on refusal. Existing range, support,
layer, MAD and quality-shadow criteria remain unchanged. Rare local diagnostics
remain disabled in this latency experiment and cannot authorize another frame.

`rgb_decode` records the decode owner, original-JPEG configuration and validation
requirement. RGB worker timers include `worker_jpeg_decode_s`; that duration is
nested in worker input-open/verification time and must not be added again.
Driver timestamp freshness remains explicitly diagnostic and independently
unvalidated as physical exposure timing. Fast refusals are excluded from the
coherent-depth freshness group.

## Run on Pi with a person in view

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_original_jpeg.sh
```

Review coherent-depth driver-age median/max and within-250ms count, mask statuses,
cycle refusals, JPEG decode/RGB RPC timings and journal errors. Compare with the
previous decode-overlap run. Do not count cancelled or no-target records as
successful fresh depth. A live 250ms result is pending, not assumed from the
17.216ms offline median saving.

Rollback uses `run_control_input_preview_decode_overlap.sh` with
`SIE_PREVIEW_NATIVE_RGB_JPEG=false` (a fresh shell also restores the default).

## Local verification

21 tests passed across control preview scheduling, original JPEG transport,
RGB shared buffers and native capture/freshness. Tests cover unchanged packet
ownership without parent decode, current-cycle JPEG selection/fusion, worker
refusal without stereo/depth reuse, asynchronous draining, packet corruption,
shape/binding checks and refusal-aware freshness grouping. Python compilation
and shell syntax checks passed. Real cameras, ONNX inference and ROS execution
require the Pi and have not been run in the implementation environment.
