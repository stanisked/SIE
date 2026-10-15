# Offline PyTorch / ONNX segmentation comparison

Date: 2026-10-15. Prepared implementation; real export/model timings pending Pi.
No camera, ROS graph, Measurement, navigation or actuator interface is used.

## Reason and scope

The single-segmentation live run on 2026-10-14 completed 25 coherent masked-depth
diagnostic cycles. Post-first host median 867.4 ms; segmentation round trip
631.8 ms including inference 592.5 ms. Stereo overlaps inference. The complete
250 ms exposure-to-command target is not met. Optimize the same segmentation
checkpoint before another camera trial, preserving current inputs and policies.

The tool exports a COPY of the same `yolo11n-seg.pt` into a fresh report directory,
then compares the original PyTorch and exported ONNX predictions on all 22 saved
AR frames from `person_live_local_surfaces_20261014_201809`. The newer compact
preview journals have no source images and must not be substituted for that set.
No checkpoint, saved source, calibration or activation file is overwritten.
No live worker/backend is switched. `PROJECT_CONTEXT.md` is not changed by this
implementation task; its 2026-10-14 checkpoint remains the last approved snapshot.

## Input parity before speed claims

The current live PyTorch call uses `imgsz=640` with rectangular auto-letterboxing.
A 1920x1200 frame may become 416x640, not 640x640. The tool probes the actual
preprocessed tensor of the installed live model, then exports that exact fixed
height/width at batch 1, FP32, opset 17, no dynamic dimensions, embedded NMS or
simplification. The exporter runs inside its own copied-checkpoint directory.

For the controlled comparison both wrappers use the discovered size, rect=False,
CPU, current conf=0.25, classes=[0], retina_masks=True, the same installed
Ultralytics NMS and full-resolution mask reconstruction. Before export comparison,
fixed PyTorch preprocessing must exactly match the live PyTorch tensor. Every
record checks PyTorch/ONNX tensor shape, dtype and SHA. A mismatch prevents a
successful report exit. This is more stringent than comparing input image names.

The model remains YOLO11n-seg COCO person. It is not relabelled as the frozen
custom upper-body Observation. Target selection uses the current diagnostic
helper with confidence 0.70 and no invented past stereo-confirmed track. It
reports unique/current-frame ambiguity outcomes; temporal tracking itself is
not revalidated. Geometry comparison executes the unchanged runtime method and
reads its actual declared edge/aspect defaults via AST without importing ROS.

## Environment and setup

Run on Pi, without another heavy camera node competing for CPU. No person is
needed. Existing segmentation environment supplies its already installed
PyTorch/Ultralytics/SciPy/OpenCV through a .pth reference from a NEW venv:
`/home/elwis/dev_ws/runtime_envs/sie_seg_onnx_benchmark`.
The existing segmentation/ROS environments are read-only inputs. This overlay
is not a hermetic copy: changing the base later changes inherited dependencies.
Base package lists before/after setup are compared, and resolved versions are
saved. First installation resolves available CPU-compatible wheels; its exact
resolved versions, not hypothetical pins, are retained for review/reproduction.
It does not download another Torch/CUDA stack.

First action:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/setup_segmentation_onnx_benchmark.sh
```

This installs onnx, onnxruntime and onnxscript into the new overlay using binary
wheels, verifies imports and prints versions/providers. Inspect a failure before
proceeding; do not install missing packages into `sie_mvp` or upgrade the working
segmentation environment to work around it.

Then, after setup succeeds:

```bash
bash vision_core/tools/run_segmentation_onnx_benchmark.sh
```

No colcon build or camera launch is required. Exporter autoinstall is disabled;
missing dependencies fail explicitly. Export/report filenames are new per run.
The launcher copies the resolved package list beside `comparison.json` after a
successful run; the environment already retains that list even if export fails.

## Timing and interpretation

ONNX runs through the same Ultralytics result pipeline. After wrapper setup, its
CPU session is explicitly replaced with a same-model ONNX Runtime session:
intra-op 1, 2 or 4 threads; inter-op 1; sequential graph execution; intra-op
spinning disabled. Session provider and input shape are checked rather than
silently falling back to another backend. Unsupported installed wrapper layouts
raise an error; they must be adapted explicitly after inspecting the report.

Two warmups exclude model/session construction. Two repeats per frame/thread
setting alternate PT/ONNX order to reduce order bias. Timing includes model
prediction, NMS, full-resolution mask processing and array extraction. File reads,
input-fingerprint checks, export and session setup are outside that interval.
No companion IPC, stereo competition, live capture or exposure buffering is
measured here. A smaller local inference time is not a full latency guarantee.

Boxes are paired one-to-one by maximum total bbox IoU, without an admission
cutoff. Reports retain unmatched counts, per-pair bbox IoU/coordinate difference,
confidence difference, full-resolution binary mask IoU and different-pixel count,
and equal-score pairing ambiguity. A permutation of model result indices is
not by itself a different target; selected correspondence is compared through
that pairing. Zero-overlap/tied pairing is never presented as a confirmed match.
Raw instances and selection/geometry results remain inspectable in each row.

The report includes input/model/export/code hashes, actual package versions,
thread settings, per-frame timing and refusal/disagreement details. All frozen
source and original model hashes are rechecked at completion. On failure,
`comparison.json` retains partial results and the error. No successful exit or
automatic winner follows from a fast but mismatched result. Physical depth
accuracy and generic/dynamic-scene safety are outside this offline comparison.

## Local validation and remaining work

Five tests check reordered one-to-one matching, mask differences, missing targets,
score ties, selection disagreement, actual runtime geometry, source bindings and
fixed-input arguments. Changed Python files compile and both shell scripts pass
bash -n. Real Torch/Ultralytics/ONNX/model execution has not been run in this
workspace; the Pi setup/export/comparison is still required. Review that report
before choosing a backend or requesting another short live check.

## Primary implementation references

- https://docs.ultralytics.com/modes/export/ (export arguments)
- https://github.com/ultralytics/ultralytics/blob/main/ultralytics/engine/predictor.py
  (rectangular preprocessing and static export shape handling; installed version
  is fingerprinted and checked, not assumed equal to current upstream)
- https://onnxruntime.ai/docs/api/python/api_summary.html (CPU session API)
- https://onnxruntime.ai/docs/performance/tune-performance/threading.html
  (thread settings; no assumed speedup from a thread count)

## 2026-10-15 Pi results and separate live ONNX backend

Completed report:
`/home/elwis/.local/state/sie/evidence/segmentation_onnx_comparison_20261015_105936/comparison.json`.
ONNX: `source_checkpoint.onnx` beside that report. All three CPU thread settings
reported 44/44 identical input tensors, mask IoU median/min 1.0, zero selection
status changes and zero geometry-status changes on the saved frames.

| ONNX intra-op threads | PT median | ONNX median |
| --- | ---: | ---: |
| 1 | 0.4369 s | 0.2699 s |
| 2 | 0.4314 s | 0.2636 s |
| 4 | 0.4317 s | 0.2629 s |

Two threads are selected for this explicit live diagnostic rollout: the measured
four-thread benefit is only 0.7 ms while stereo also consumes CPU. No claim that
two threads are universally optimal. Report/model hashes and exact compared
Ultralytics/ORT/Torch/NumPy versions are checked before this adapter starts;
thread rows must reproduce the reviewed exact-mask/selection/geometry result.
This binds the tested artifact, rather than defining a general mask-IoU policy.

`onnx_segmentation_backend.py` uses the same fixed-input Ultralytics wrapper and
CPU-session replacement as the completed challenge. One current inference still
produces all boxes/masks. Exact-index, cycle and RGB-hash ownership, confidence,
geometry, ambiguity checks and one-time mask consumption remain unchanged.
ONNX engine/provider/thread/input/export/report/source-checkpoint provenance is
included in the segmentation and shape journal records. Warm-up uses blank
frames during startup and never establishes a live target or cached mask.

The existing `run_control_input_preview_640x400.sh` remains the PyTorch reference.
A new explicit launcher uses the existing ONNX benchmark environment and these
exact report/export paths, at two CPU threads:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sie_ros2 --symlink-install
bash vision_core/tools/run_control_input_preview_640x400_onnx.sh
```

Stop another camera consumer; person in view for one 30-second implementation
check. No calibration/distance sweep. No dependency installation or checkpoint
copy/activation is performed by this launch. Model/runtime/manifest mismatch
refuses startup with a reason; there is no silent PyTorch or provider fallback.

Compare host median/max, segmentation round-trip and inference, stereo worker,
local surfaces/fusion and mask-depth statuses with the preceding 0.8674 s host
median. The local ONNX 0.2636 s excludes IPC and concurrent stereo. Even that
alone exceeds 250 ms; a successful export is not a claim that the complete age
target is met. Local shape residuals still are not physical accuracy, and no
Measurement, navigation or actuator commands are enabled.

Local checks: 19 targeted tests passed before the startup-error reporting
refinement; tests cover artifact/environment mismatches, reviewed-result
mismatches, CPU thread/shape/rect arguments, current mask protocol, selection,
callback isolation and shutdown. CPU adapter execution is mocked locally;
real ONNX inference was demonstrated by the operator's saved comparison, and
new concurrent live timing remains pending the command above.

## Smaller ONNX input challenge (2026-10-15)

Sampling local fits every five cycles produced 38 completed primary cycles:
32 coherent masked-depth diagnostics, six global-MAD refusals, eight local
results and 30 scheduled skips. No queue replacements or processing/write
errors. Host median 0.5327 s, maximum 0.6924 s; zero of 37 timed steady-state
cycles met 250 ms. Segmentation round trip median 0.3896 s; stereo 0.2629 s.
These different live frames do not establish a quality improvement from sampling.

Next offline experiment uses `run_segmentation_input_size_benchmark.sh`.
It verifies the reviewed 416x640 ONNX export/report/checkpoint and exports two
new FP32 static candidates, 320x512 and 256x384 (height x width), into a new
output directory. Existing model files, activation and live defaults stay intact.
All 22 saved cycles from `person_live_local_surfaces_20261014_201809` are used,
with two alternating-order repetitions per candidate and ORT CPU threads=2.

Stereo is recomputed once per saved JPEG pair at the existing diagnostic half
resolution and shared by every model size. Runtime selection, geometry, inner
ROI defaults and masked-depth gates are reused. Candidate-specific masks/ROIs
are evaluated without local fitting. Report rows preserve unmatched detections,
matching ties, selected-target correspondence, geometry and depth refusals,
mask IoU, depth/MAD/sample changes and model/input/source hashes. No failed
frame is removed from the comparison. Selection has no fabricated confirmed
track. Inference/extraction timers exclude image decoding, IPC, stereo and depth.

Run on Pi with other perception processes stopped, without a person or camera
capture. Review `SIZE` summaries and `comparison.json`; there is no automatic
winner or runtime switch. Agreement with the baseline is not physical accuracy
or validation for moving targets or arbitrary scenes.
