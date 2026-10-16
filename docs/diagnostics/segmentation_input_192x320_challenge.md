# Offline 192x320 segmentation input challenge

2026-10-16. Candidate experiment only; no live launch changes or automatic
promotion. No Measurement, navigation or motor commands.

## Purpose

Fresh-stereo live run `control_input_preview_half_640x400_single_segmentation_onnxruntime_20261016_130157`
reported 76/76 MAD-coherent cycles; of 75 steady cycles, 32 had both driver ages
within 250ms. RGB age median/max 251.29/273.02ms, OV9281 222.49/252.79ms.
Segmentation prediction/extraction median was 111.14ms. Waiting for newer stereo
did not improve the overall freshness fraction. Further capture scheduling is
paused; reducing compute cost is the next candidate.

The candidate reduces NN input from the currently reviewed 256x384 (98,304
pixels) to 192x320 (61,440), 37.5% fewer pixels. Runtime speed is not predicted
from that ratio. Weights and thresholds are unchanged, but masks/boxes can change.
There is no claim of exact output equality or physical accuracy.

## Comparison

The existing offline input-size tool has an explicit `--reference-current-256`
mode restricted to this one candidate. The original 416x640 reference challenge
remains its default. The new mode verifies the reviewed 256x384 baseline through
the existing adapter, follows its hashed parent report to the original checkpoint
and exports a new static FP32 model in a new evidence directory. Source images,
models and prior reports are never overwritten.

Both sizes process the same 22 saved RGB frames, twice in alternating order.
One half-resolution stereo map is recomputed per cycle from saved rectified JPEGs
and shared by both sizes, retaining current depth geometry and diagnostic range.
The report includes mask IoU and unmatched instances, target correspondence,
geometry acceptance, depth status transitions, median depth/MAD/support deltas
and timing. Comparison masks are reference predictions, not annotated truth.

The actual delegated ORT session metadata is recorded. Provider, execution mode
and thread budgets must agree; the tool refuses attribution to input size if
those differ. No thread tuning is introduced. This isolated timing omits live
capture/transport/contention and cannot establish live exposure-to-command age.
No temporal track is fabricated for saved independent frames.

The new report status is `OFFLINE_192X320_VS_REVIEWED_256X384_ONLY`, deliberately
unsupported by the live reviewed-export verifier. A successful benchmark alone
does not switch the live model or introduce a mask-IoU acceptance threshold.

## Pi command, cameras/person unnecessary

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/run_segmentation_input_192x320_benchmark.sh
```

Review `SIZE (192, 320)`, `INPUTS_UNCHANGED`, and the saved report. Input size is
retained as a candidate only if the measured speed benefit and output changes
justify a separate reviewed diagnostic rollout. Runtime remains 256x384.

## Local validation

14 tests passed across input-size comparison, reviewed ONNX adapter and output
matching. The new test covers explicit baseline/size restrictions, source
checkpoint and parent-report integrity, and distinct report status. Two Python
files compiled and the launcher passed shell syntax checks. Model export and
44 real inference comparisons require the Pi environment and are pending.
