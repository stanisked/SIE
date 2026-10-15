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
