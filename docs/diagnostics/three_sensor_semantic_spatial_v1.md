# SIEE Step 71 | Three-Sensor Semantic Spatial Association v1

**DECISION** `THREE_SENSOR_SEMANTIC_SPATIAL_ASSOCIATION_V1` is a motor-off
diagnostic evidence chain for the person-following MVP. It follows the
[Step 70 freeze](stereo_lidar_local_correspondence_v1.md#step-70-freeze-2026-11-26).
The first implementation uses the existing sensor-only Shadow preview as an
evidence producer and evaluates association offline. It adds no detector,
tracker, camera thread, actuator consumer or online association wait.

## Inputs and eligibility

**FACT** The bounded launcher runs the reviewed mask-only ONNX preview with
the existing latest-pair/new-RGB trigger. Its opt-in evidence mode binds
the original camera JPEGs, full original-grid selected-instance mask and
current track evidence in the existing latest-wins source snapshot writer.
It samples periodic snapshots every third cycle where a selected mask
is available, also retaining the existing special snapshot triggers.
The queue retains one pending source bundle and the writer has a 24-bundle
budget. Its normal mode and Step 57's metadata-only mode retain their defaults.
The Step 71 launch selects original RGB JPEG transport for exact input
fingerprinting. The model still runs through the existing worker, which
returns the instance mask in the original distorted RGB image coordinates.

**DECISION** Verify the snapshot archive and original JPEG hashes, the
decoded RGB fingerprint, mask hash, selected instance, cycle ID, model
digest, RGB-to-mask worker binding, and calibration/profile hashes. Require
the current preview to identify one eligible person plus matching current
track confirmation or current RGB continuity. A held track alone cannot
supply missing current semantics. Side-truncated targets refuse. Top- or
bottom-truncated targets require the current stable torso-core record and
its exact source bindings. There the existing torso-core fractions restrict
the valid mask pixels. A bounding box supplies no mask pixels.

**FACT** The independent LD19 subscriber retains bounded callback payloads only
in Step 71 mode. The temporal diagnostic writer converts and hashes the exact
LD19 callback chosen by the Step 57 RGB-anchor comparison. It binds the
payload by header, callback receipt sequence and monotonic receipt time.
Callback-side work retains a bounded reference. Disk work remains on the
diagnostic writer. The replay requires the actual JPEG stereo packet to be
the same one selected by the causal Step 57 RGB–Stereo link. It preserves
both RGB temporal dispositions and explicitly checks actual Stereo–LD19
header separation against the existing Step 66 **50 ms** trial. That third
edge gates this joint diagnostic only. The Step 57 25/50 ms link checks and
250 ms source ages retain their original meaning and code.

## Spatial policy

**DECISION** Use the same camera calibration artifacts and numeric paths for
independent reconstruction. Offline replay reuses Step 66 full-resolution
`sgbm7`, its **1.5 px** LR consistency check, v7 `Q` in millimetres and
`R1ᵀ`. The existing live half-resolution preview and torso-core computation
continue to operate on their own grid. Full-resolution replay runs on the
same archived stereo JPEG, so the Step 69 full-pixel local policy can be
applied unchanged. The original AR0234→physical-left calibration is
inverted as `Rᵀ(p_left − t)` before AR0234 projection with the original
intrinsic and distortion coefficients. Live masks are already on that
grid. Explicit resize/letterbox declarations in the pure API use
`(pixel + 0.5) × scale − 0.5 + pad`; a grid mismatch without a declaration
refuses. No fixed 2D mask transfer is used.

**DECISION** Nearest positive AR-optical depth per rounded original RGB
pixel handles projection collisions conservatively. Record the discarded
occluded samples, then consult the two-pixel mask interior and any torso
restriction. The existing mask depth-layer function supplies the **100
samples**, **25% competing support** and **150 mm or 4% depth gap** trial
rules. Within the one plausible depth layer, the existing Vision Core
eight-neighbor connected-component convention separates disconnected
regions. Require one plausible stereo component using the same count and
support scales. Preserve ambiguity from competing depths or components.
Global spread and the independent 40 mm plane screen are additional
diagnostics. They do not gate a local candidate on a non-planar person.

**DECISION** Freeze semantic ownership before examining LD19. The unchanged
Step 69 local algorithm examines all stereo support near each valid projected
LD19 hit, at original rectified stereo resolution. Require its anchor and all
selected supporting pixels to belong to the independently selected RGB-owned
stereo component. Refuse support that crosses that component or is ambiguous.
The LD19 hit's own projected RGB-mask membership is recorded separately
as a diagnostic. All reported residuals retain the Step 69 sign and magnitude,
including large residuals. Only source-bound local candidates contribute
to the summary distributions.

**FACT** The policy file is
[`three_sensor_semantic_spatial_v1.json`](../../vision_core/config/diagnostics/three_sensor_semantic_spatial_v1.json)
with SHA-256 `0a88bc3e5a9d79eaa658a5ddfd3c23fe18de21af5b46df0cab62eb5c1c894f35`.
The replay verifies the supplied v5 intrinsic, v7 stereo, calibrated inter-camera
extrinsic, mechanical geometry, estimated optical datum and Step 69 policy
digests. The static activation artifact is used only for provenance and to
resolve the corresponding extrinsic file.

## Diagnostic evidence

- **FACT** `policy.json` records the mask, temporal and spatial policy,
  artifact hashes, calibration IDs, coordinate frames, units, optical datum,
  sign convention, and software versions. It also hashes the original
  temporal journal.
- **FACT** Each `cycles.jsonl` row carries the diagnostic schema
  `sie.three_sensor_semantic_spatial.v1.diagnostic`. The
  [cycle schema](three_sensor_semantic_spatial_v1.schema.json) documents the
  versioned interchange. A refusal retains its reason and evidence
  available at that stage. Compared rows bind camera frame stamps and
  receipt times, track and mask identities, the raw LD19 callback hash,
  stereo component ID and semantic support. Per-beam fields separate
  original range/angle, calibrated projection, stereo support, RGB mask
  membership, component membership, local coherence and signed residuals.
- **FACT** `summary.json` reports refusal dispositions, bounded source
  journal counters, compared beam outcomes and candidate residual
  distributions. Missing snapshots, missing payloads and zero candidates
  are valid diagnostic outcomes. Keep images, masks, raw arrays and
  journals private.

## First motor-disabled Pi capture and replay

**DECISION** Run only as a separately initiated Pi experiment with traction
motor power physically disconnected. Use an existing sole LD19 publisher
on `/sie/lidar/scan_raw` and release other AR0234/OV9281 camera owners first.
Start with one visible person in the shared field of view, with the laser
plane intersecting the visible person. Avoid close foreground occluders in
the first sample. This launcher starts neither a second LD19 driver nor a
consumer of the isolated preview output. Use a current same-host,
same-session LD19 clock-evidence file from the observed publisher. The
clock reference alone does not establish publisher continuity.

From the updated Spatial Association checkout on the Pi:

```bash
git rev-parse HEAD
source /opt/ros/jazzy/setup.bash
ros2 topic info /sie/lidar/scan_raw -v
read -r -p 'Current LD19 clock evidence JSON path: ' clock_file
test -f "$clock_file" || exit 2
export SIE_TEMPORAL_LD19_CLOCK_EVIDENCE="sha256:$(sha256sum -- "$clock_file" | cut -d ' ' -f 1)"
bash vision_core/tools/run_three_sensor_semantic_shadow_v1.sh
```

Use the exact `Evidence:` directory printed by the launcher:

```bash
read -r -p 'Evidence directory printed by the launcher: ' run
test -d "$run" || exit 2
export PYTHONPATH="$PWD:$PWD/ros2/sie_ros2:$PWD/vision_core/tools${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
artifacts=/home/elwis/dev_ws/runtime_artifacts
model=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx
result="${run}_step71_$(date -u +%Y%m%dT%H%M%SZ)_$$"
"$runtime" vision_core/tools/replay_three_sensor_semantic_v1.py \
  --run "$run" --output "$result" \
  --ar-intrinsic "$artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json" \
  --stereo-calibration "$artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz" \
  --activation "$artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json" \
  --model "$model"
"$runtime" -m json.tool "$result/summary.json"
```

**OPEN** No Step 71 physical run or measured live overhead is available.
The LD19 header is a conversion stamp and its acquisition time remains
`UNKNOWN`. Camera exposure, physical target membership and the
camera–LiDAR extrinsic remain independently unverified. The optical datum
is an estimated +13.3022 mm with unknown uncertainty and confidence. No
diagnostic output admits a canonical Measurement, World State mutation,
Decision Engine input, TF publication or motor action. The ESP32 v13
brake fault is a separate open workstream.

**FACT** Local verification passed 215 targeted tests and 2 subtests,
including Step 57 and Step 69 regressions and a synthetic snapshot-to-replay
cycle using the reused SGBM matcher. Six Python files compiled. Both launcher
scripts and the documented Pi commands passed Bash syntax validation. The
cycle schema and a synthetic candidate record passed schema validation.
