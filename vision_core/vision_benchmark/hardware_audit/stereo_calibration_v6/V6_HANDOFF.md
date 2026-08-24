# SIE stereo calibration V6 handoff

Checkpoint date: `2026-08-22`

This document is the technical handoff for continuing the DECXIN dual OV9281 V6 stereo work in Codex. Read it before changing V6 files. The validated V5 implementation remains an immutable reference.

## 0. Role of V6 inside SIE

SIE means both **see** and **Spatial Intelligence Engine**. The project turns heterogeneous observations of the physical world into a continuously updated, evidence-backed world model and uses that model for reliable engineering decisions and safe actions.

The durable SIE flow is:

```text
Raw Observation
  -> Observation
  -> Measurement
  -> Interpretation
  -> Fact
  -> World State
  -> Decision
  -> Action
```

The DECXIN dual OV9281 V6 work is one replaceable sensor implementation at the beginning of this flow. It is not the SIE architecture itself. Its responsibility is to acquire traceable stereo observations and, only within a validated applicability envelope, produce metric depth measurements suitable for downstream reasoning.

V6 therefore demonstrates a central SIE principle: measured physical reality is preferred over prediction, evidence is preferred over opinion, and uncertainty must remain visible.

### 0.1 Architectural boundary

- The capture and stereo runtime belong to **Vision Core**. They may emit `Observation` and gated `Measurement` records.
- Calibration and runtime policy establish provenance and applicability. They do not create a `Fact`, update `World State`, or authorize an `Action` by themselves.
- Transforming V6 output into another coordinate system is the responsibility of the **Geometry Engine** and requires explicit source and target frames.
- Engineering interpretation belongs with the **Knowledge Engine** and downstream interpretation logic.
- Only the **World State Manager** may update entity-centric `World State` from verifiable `Fact` records.
- The **Task Evaluator** evaluates task state. The **Decision Engine** makes decisions using `World State`, engineering knowledge, and task evaluation.

Do not add shortcuts from a disparity or depth sample directly to `Fact`, `World State`, `Decision`, or `Action`.

### 0.2 Long-term invariants carried by V6

Any value promoted from V6 diagnostic output to an SIE `Measurement` must include:

- physical unit;
- explicit `reference_frame`;
- timestamp and source observation identity;
- confidence and quality state;
- calibration identity and hash;
- runtime policy and guard identity;
- sufficient provenance to reproduce the result from the frozen source data and declared software/configuration versions.

Every later `Fact`, `World State` update, `Decision`, and `Action` derived from V6 must remain traceable to those observations and measurements through the Evidence Graph.

Failure of temperature, calibration, ROI, synchronization, quality, freshness, or applicability checks must fail closed. Raw or diagnostic depth may still be logged for investigation, but it must not be presented as an approved SIE `Measurement`.

### 0.3 Meaning of `ACTIVE_CONDITIONAL`

`ACTIVE_CONDITIONAL` means that this exact calibration/runtime combination is approved only when all recorded guard conditions pass and the measurement remains inside the current validated operating envelope. It does not mean that the camera is universally calibrated, that all distances are valid, or that downstream engineering conclusions are automatically approved.

Expansion of the envelope requires new evidence, explicit review, a versioned policy change, and preservation of the previous artifacts. A promising diagnostic result outside policy is evidence for further validation, not permission to widen the policy silently.

### 0.4 Source of truth and evolution

This handoff is version-specific operational memory. It is subordinate to approved and frozen SIE Engineering Specifications, canonical architecture and data-contract documents, and `AI_CONTEXT.md`. `World State` is the operational source of truth about the modeled physical world, not the normative source for repository architecture. If this file conflicts with a higher-authority artifact, do not guess which behavior was intended: preserve both statements, report the conflict, and request an explicit resolution.

Keep future updates evidence-based:

- preserve immutable raw observations and validated V5/V6 artifacts;
- record commands, hashes, inputs, outputs, environment, acceptance criteria, and failures;
- distinguish established facts from hypotheses, planned work, and unverified observations;
- update this handoff whenever activation state, validated range, hardware mapping, calibration, runtime guard, or safe continuation order changes;
- create a new versioned artifact instead of mutating a frozen contract or concealing a behavior change behind the old name.

## 1. Outcome

`stereo_calibration_v6` passed fresh checkerboard rectification and physical depth checks within the currently approved range. The guarded V6 runtime was created and launched successfully.

Observed runtime behavior:

- approximately `390..2000 mm`: very good metric depth;
- below approximately `390 mm`: correspondence starts failing;
- approximately `3900 mm` physical distance: raw diagnostic depth around `3876..3890 mm`;
- distances above `2.55 m` remain outside the current policy and must not produce an approved SIE `Measurement`.

Current status:

```text
ACTIVE_CONDITIONAL
```

## 2. Hardware and capture contract

```text
stereo module: DECXIN dual OV9281
board marking: DECXIN-SM-1M-2583V1
physical baseline: 65.1 mm
device commonly used: /dev/video2
backend: V4L2
pixel format: MJPG
combined frame: 2560x800
individual frame: 1280x800
requested/actual FPS: 60
buffer size: 1
auto_exposure: 3
standard runtime warm-up: 180 successful frames
stable validation warm-up: 1800 successful frames
```

## 3. Canonical camera semantics

The final decision is to keep the proven V5 semantics:

| Combined frame | Physical camera | Canonical semantic |
| --- | --- | --- |
| LEFT half | physical RIGHT | `right` |
| RIGHT half | physical LEFT | `left` |

Equivalent runtime split:

```python
right_raw = combined_bgr[:, :1280]
left_raw = combined_bgr[:, 1280:]
```

This mapping must not be changed in V6.

## 4. Temperature instrumentation

| Channel | Sensor | ROM | Physical placement | Runtime role |
| --- | --- | --- | --- | --- |
| `camera_left` | S03 | `28F952990F510A0C` | behind physical LEFT | gating |
| `camera_right` | S02 | `28FE6BA3299C9AC4` | behind physical RIGHT | gating |
| `ambient` | S10 | `289452A555A32FDA` | free air away from camera | observational |

The labels refer to physical camera position when looking along the cameras' viewing direction from the rear of the module.

```text
state file: /tmp/sie_h05b_temperature_state.json
maximum state age: 5.0 s
```

Bridge command:

```bash
cd ~/dev_ws/src/sie

python3 vision_core/tools/sie_ds18b20_serial_bridge.py bridge \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --state-file /tmp/sie_h05b_temperature_state.json \
  --map camera_left=28F952990F510A0C \
  --map camera_right=28FE6BA3299C9AC4 \
  --map ambient=289452A555A32FDA \
  --startup-timeout-s 10 \
  --stream-timeout-s 5 \
  --console-interval-s 2
```

Current provisional conditional envelope:

| Channel | Minimum C | Maximum C | Gating |
| --- | ---: | ---: | --- |
| `camera_left` | 30.0000 | 32.3750 | yes |
| `camera_right` | 31.1250 | 33.9375 | yes |
| `ambient` | not applicable | not applicable | no |

Ambient must exist and remain fresh, finite, and correctly mapped. Its numeric temperature does not gate V6 depth.

## 5. Dataset V6 and freeze V2

Capture produced `74` synchronized stereo pairs:

```text
pair_000.png ... pair_073.png
left/: physical LEFT
right/: physical RIGHT
checkerboard: 9x6 inner corners
square size: 24.5 mm
image size: 1280x800
```

Source and frozen locations:

```text
/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/dataset_v6_raw

/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
stereo_calibration_v6_raw_frozen_2026-08-20_v2

/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
stereo_calibration_v6_raw_frozen_2026-08-20_v2.tar.gz
```

Frozen V2 hashes:

| Artifact | SHA-256 |
| --- | --- |
| `SHA256SUMS.txt` | `9b609b2ead73901c2b2adc6f7e523b959f74e8d1f5d4af8db1dcdc223795d3d0` |
| `FREEZE_MANIFEST.json` | `dc74101aa7fd87b293df3c49c2931cf8ce77864711d446412472ca2b4d47be9d` |
| `capture_metadata_v6.json` | `be53a9341593a3dcbc590ec9cccc92bd00d94702cea81e7e8757cc91aa8d3b54` |
| frozen V2 archive | `8f5a78071dc5eb6909ab52982d7a0227265913b32b7b7656b01070008710db12` |

Verification completed with:

```text
verification_exit_code=0
gzip_exit_code=0
```

Raw files are immutable. Never alter them to make a solver pass.

## 6. Corner-order issue and correction

The first simple solve failed badly despite good monocular RMS because three right-camera checkerboard detections used the opposite valid SB corner order.

Affected pairs:

```text
pair_015.png
pair_033.png
pair_061.png
```

Correction was applied only to detected right-corner arrays through an explicit manifest. Source PNG files were not modified.

```text
/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
corner_order_corrections_v1.json

SHA-256:
a61c2872860b044081c6bec46f336792d0ab0e99ae0fe16318b5071112c28440
```

The original bad solve showed why stereo RMS must not be interpreted without checking correspondence order:

```text
stereo RMS approximately 26 px
baseline approximately 54 mm
```

After the explicit corner-order correction, geometry became physically plausible.

## 7. Solver and authoritative solution

Solver:

```text
vision_core/tools/stereo_calibration_v6_solver_v1_2.py
SHA-256: 661aadc2cdef0df7bc9d1b3a065f1464d6e24fa4b1b520323f08c768b30ce756
```

Authoritative run:

```text
/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
solution_joint_refine_corner_order_filtered_freeze_v2_run07
```

```text
stereo_params_v6.npz
SHA-256: bb8fb665c6e06e2cbb633cf4c3c61aa74933dd253c9c7950a8420591975dd5e7

calibration_report.json
SHA-256: d289e1725da3c1bb971f19114c988a88722e84db25fec40a2f0eb5f9fe769ab7
```

Final solve metrics:

| Metric | Result |
| --- | ---: |
| input pairs | 74 |
| used pairs | 55 |
| outliers | 19 |
| left final RMS | 0.3277512646 px |
| right final RMS | 0.3481169092 px |
| stereo RMS | 0.3588656363 px |
| estimated baseline | 63.7540921190 mm |
| physical baseline difference | 1.3459078810 mm |
| rectified median `abs(dy)` | 0.1204452515 px |
| rectified p95 `abs(dy)` | 0.4134780884 px |
| within 1 px | 100% |
| positive disparity ratio | 1.0 |

Outliers excluded by the solver:

```text
pair_004.png, pair_007.png, pair_011.png, pair_012.png,
pair_016.png, pair_023.png, pair_028.png, pair_029.png,
pair_030.png, pair_031.png, pair_035.png, pair_040.png,
pair_046.png, pair_048.png, pair_050.png, pair_051.png,
pair_053.png, pair_059.png, pair_060.png
```

Run07 reproduced run06 geometry bit-for-bit. `K1`, `D1`, `K2`, `D2`, `R`, `T`, `E`, `F`, `R1`, `R2`, `P1`, `P2`, `Q`, ROIs, baseline, RMS, and rectification metrics were equal. Only provenance fields and solver version differed as intended.

## 8. Fresh checkerboard validation

Diagnostic tool:

```text
vision_core/tools/diagnose_v5_stereo_runtime.py
SHA-256: 58a554687b708448d3d8eefa8f04172fe2aca417aca00903bb9243c43e7f3f84
```

Accepted fresh run04:

```text
/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
fresh_checkerboard_validation_v1/run04/checkerboard_gate.json
SHA-256: 349ca5e827bbfd1f84e285d063b9dacc4d125d2b470cdb46257f2fc9560133f9
```

```text
ambient:      27.1875 C
camera_left:  30.5000 C
camera_right: 31.9375 C
```

| Metric | Result |
| --- | ---: |
| detection LEFT | 54/54 |
| detection RIGHT | 54/54 |
| right order reversed | false |
| median signed dy | -0.012299 px |
| median abs dy | 0.104126 px |
| p95 abs dy | 0.229604 px |
| maximum abs dy | 0.247162 px |
| within 0.25 px | 100% |
| within 0.50 px | 100% |
| within 1.00 px | 100% |
| positive disparity ratio | 1.0 |
| status | PASS |

## 9. Physical depth validation

All selected runs used V6 run07, StereoSGBM with `num_disparities=192`, a `100x100` ROI, and no hidden depth scale or offset correction.

### Near and mid

| Run | Ground truth datum | Measured | Raw difference | ROI valid | Disp. MAD | Depth MAD | Status | JSON SHA-256 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| `near_406mm_run01` | 406 mm | 402.269 mm | -3.7 mm, -0.92% | 1.0000 | 0.0 px | 0.0 m | PASS | `26bb2752285af8d4726422a9e5b723c87562b9e2f59bbfc58b842d6c96c87143` |
| `mid_1007mm_run01` | 1007 mm | 1004.722 mm | -2.3 mm, -0.23% | 1.0000 | 0.0625 px | 0.0009497 m | PASS | `e457f44d0333b6959e56010466ada491c1221c4c0116c66bdc4ba0a97e2838e1` |

### Mid-far, thermally stable triple

Ground-truth datum distance: `1785 mm`.

| Run | Left C | Right C | Measured mm | Raw difference mm | ROI valid | Disp. MAD px | Depth MAD m | SHA-256 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| run04 | 32.3750 | 33.8750 | 1749.573 | -35.427 | 1.0000 | 0.0625 | 0.0028776 | `8e3481fc7c7d5ef96a3e42e46744b998e5a38c71508aa46720f3db3edeff8c5c` |
| run05 | 32.3125 | 33.8750 | 1749.573 | -35.427 | 1.0000 | 0.0625 | 0.0028776 | `bec31501f3d74a90a5e8337c18a7a40dcd7cb6966ea56a7239c22bd6b20d3efc` |
| run06 | 32.1875 | 33.9375 | 1749.573 | -35.427 | 1.0000 | 0.0625 | 0.0028776 | `175bd25f6beaefda6d4876a47d09c7261285e10e5eb777149cf7ab1d96e88f82` |

Median span across the stable triple was `0 mm`.

### Far, thermally stable triple

Ground-truth datum distance: `2511 mm`.

| Run | Left C | Right C | Measured mm | Raw difference mm | ROI valid | Runtime 0.90 gate | SHA-256 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| run02 | 31.8125 | 33.3125 | 2458.312 | -52.688 | 0.9934 | PASS | `0fbca0eed67d202b62a46b9d5ad48c6a0103c25403c341ea9dc21a006970ad01` |
| run03 | 32.0625 | 33.6250 | 2458.312 | -52.688 | 0.9345 | PASS | `ccfa76b22a77160b8dc442e474a4116118af66d2ba6205d9c68b7d245cf7f055` |
| run04 | 32.1875 | 33.7500 | 2458.312 | -52.688 | 0.9921 | PASS | `c2fcb0912c1081f860b5906513e2c98dbbe04f8786158c048b5ddccb552da14b` |

The diagnostic labeled far run03 as FAIL only because it used a stricter `0.95` valid-ratio threshold. Under the frozen runtime policy threshold of `0.90`, all three far runs pass.

## 10. Mechanical ground-truth reference

Physical distance was measured from the front edge of the physical LEFT lens barrel:

```text
physical_left_lens_front_rim_frame
```

Stereo depth is expressed in:

```text
rectified_left_optical_frame
```

Required axial transform:

```text
Z_rectified_left_optical = Z_lens_front_rim + t_z
```

`t_z` is positive and has not yet been independently surveyed. Therefore the numeric differences in Section 9 are retained as raw cross-frame differences. They are useful evidence of behavior, but they are not yet approved absolute optical-frame errors.

Do not derive or apply a hidden offset. The transform must be measured mechanically and stored explicitly with uncertainty and reference-frame semantics.

## 11. Conditional activation chain

| Artifact | Status | SHA-256 |
| --- | --- | --- |
| `stereo_calibration_v6_extended_range_review_v1.json` | PASS_CONDITIONAL | `8ac1edfe9b70af249a4785cabe9cff13700472077fcfc1faf4fd891965a22ce3` |
| `stereo_calibration_v6_activation_conditional_v1.json` | ACTIVE_CONDITIONAL | `5d4c8020ed183af2fadd63bf63fff364bbf30c8849ba4b4591f0e18cb49bbb17` |
| `vision_core/config/runtime/stereo_calibration_v6_runtime_policy_v1.json` | ENABLED | `a26b4a9335296fe4f5584e4d6ba5a74a1eb03c795833c976db090c661e99464c` |

Integrity was checked:

- policy points to the exact activation SHA;
- activation and policy point to the exact review SHA;
- calibration ID and SHA are consistent across the chain.

Frozen policy:

| Parameter | Value |
| --- | ---: |
| capture mode | MJPG 2560x800 @ 60 FPS |
| num disparities | 192 |
| ROI center | 650, 350 |
| ROI size | 100x100 px |
| minimum valid pixel ratio | 0.90 |
| maximum disparity MAD | 0.50 px |
| maximum depth spatial MAD | 0.020 m |
| validated ground-truth range | 0.4..2.5 m |
| accepted measured depth | 0.38..2.55 m |
| measurement reference frame | rectified_left_optical_frame |
| outside-envelope action | BLOCK_DEPTH_MEASUREMENT |
| hidden scale/offset correction | forbidden |

## 12. V6 runtime files

| File | SHA-256 | Role |
| --- | --- | --- |
| `vision_core/stereo/stereo_calibration_guard_v6.py` | `42e8eff34700030e32cc0c1419122cd8e23e1d88055fc943f604fefb1f2e0d05` | activation, integrity, ROM, freshness, temperature gate |
| `vision_core/stereo/guarded_runtime_v6.py` | `e9602d318021a934f61116c3b1f6f75e6277b0686eab0810cd387f1a109edee1` | V5 mapping, rectification, SGBM, depth |
| `vision_core/tools/run_guarded_stereo_v6.py` | `2b4a30db75c7265ec800f943f3eb68502180ce054ccd849546b26a24b997bb79` | camera, preview, gates, ROI and Measurement flow |

V6 guard behavior was tested:

- ambient at an artificial out-of-range numeric value did not block while both camera channels were inside their envelope;
- camera_left outside its envelope blocked depth;
- mapping, freshness, reset, finite-value, calibration and activation integrity checks remained fail-closed.

### 12.1 Hardened V2 runtime activation package candidate

V1 remains the immutable validated baseline and the operational SOURCE tree
continues to use V1. The following V2 package is prepared for review and is not
deployed:

| Artifact | SHA-256 | Status |
| --- | --- | --- |
| `vision_core/stereo/guarded_runtime_v6_v2.py` | `54ac3c7a9d64cbdf6c1eb7fe5c1470a77ff88a1527be81624699b0f9230df3d9` | hardened candidate |
| `vision_core/tools/run_guarded_stereo_v6_v2.py` | `afc9d7a06674cd70a58082c7c1c4efe8a5ef0884a6fd39bb222c96b0cd29749f` | package runner |
| `stereo_calibration_v6_runtime_hardening_review_v2.json` | `1d0f5ec650ab2b9a20f142380088cefa346af734b46cd31e0cf24b33eb653390` | PASS |
| `stereo_calibration_v6_activation_conditional_v2.json` | `e38000102dd85bba56a0d256986d1ddf67d801fdb8b73d01e0d649a51333dc7d` | ACTIVE_CONDITIONAL package record |
| `vision_core/config/runtime/stereo_calibration_v6_runtime_policy_v2.json` | `0027a7914d474f9a50dc325260e730a03348e363f3e61f80609a4069cc7a2d71` | ENABLED package policy, not deployed |

Candidate commit:

```text
c03b64c749dce0e55ea2c73629a5ab75dec23d11
```

V2 removes the implicit `num_disparities=160` processor default, requires the
argument explicitly, and accepts only `192` before matcher creation. Candidate
tests passed `8` tests and the full repository suite passed `35` tests.

The calibration, extended-range review, geometry, SGBM192 computation,
camera-half split, rectification, depth formula, ROI, quality behavior,
temperature contract, reference frame, and operating range are unchanged. The
existing physical validation therefore remains applicable. V2 introduces no new
distance range, accuracy, or Measurement claim and applies no scale or offset
correction.

### 12.2 Stage 9 V2 live deployment challenge

Stage 9 completed with status `PASS_RUNTIME_V2_LIVE_CHALLENGE`. The challenge
processed `120` depth frames and published `13` rate-limited Measurements at
frame sequences `9..117` with a step of `9`. All ROI quality gates passed and
the temperature gate recorded `0` violations.

The physical ground-truth datum was `997 mm` from
`physical_left_lens_front_rim_frame`. Median runtime depth was
`990.663111 mm` in `rectified_left_optical_frame`, giving a raw cross-frame
difference of `-6.336889 mm`. The axial transform `t_z` remains pending, so
this difference is evidence only and is not an approved scale or offset
correction.

| Evidence | SHA-256 |
| --- | --- |
| `runtime_v2_deployment_challenge_v1/run01/deployment_challenge_report.json` | `a1251dda4a3ae9eca10210585647b9f6e2d0067c02e1265002be495b4c22bbaf` |
| external `console.log` | `47587501992ffef831c52f5b34aee64ab394730a75919c59079fa6a191d2b967` |
| external `measurements.jsonl` | `8eeb718a68e5d48cfee708cb88fe62fc9edd71ca5b8cfcc7b521f322d3db64c1` |

The runner's `console_interval_s=2.0` limited Measurement publication cadence;
the other processed frames were not rejected Measurements. For an operational
V2 run, the recommended console interval is `0.2 s`. This is an operational
cadence recommendation only and does not alter the validated runtime profile,
policy, quality gates, or Measurement range.

V2 remains `deployed=false`. The operational SOURCE tree continues to use V1.

## 13. Protected V5 reference

| File | SHA-256 |
| --- | --- |
| `vision_core/vision_benchmark/hardware_audit/capture_raw_stereo_pairs.py` | `0df83f99a6f47a498ca68d41a629fdd999c9278811e35867834f5b09f68378a8` |
| `vision_core/tools/sie_ds18b20_serial_bridge.py` | `0de8d8454dd6a130fefe70ab45c9cb39dcab3c1f628a1e79fdf4e416b5ed8dc5` |
| `vision_core/tools/run_guarded_stereo_v5.py` | `47519a5e36b28891e1c8898422edab8dad2ea1f7e5e5fba6449fc11f71c78218` |
| `vision_core/stereo/guarded_runtime.py` | `1a4763572768f402db3c782639c413a15e604a4013be5d1bd1aa5198859d648b` |

V5 is the behavioral template. V6 has separate versioned guard, runtime, runner, review, activation, and policy files. Do not modify V5 while continuing V6.

## 14. Latest live observations and interpretation

```text
390..2000 mm: very good depth
below 390 mm: begins to fail
physical distance around 3900 mm:
  displayed/raw depth around 3876 mm
  depth camera typically around 3880..3890 mm
```

The near behavior is consistent with the SGBM search range. With rectified focal length approximately `1041.101 px` and baseline `63.754 mm`, `f*B` is approximately `66.37 m*px`. Expected disparity approaches `192 px` near `346 mm`, leaving little robust search margin below about `390 mm`.

At `3900 mm`, expected disparity is only about `17.0 px`. A single `1/16 px` disparity step corresponds to roughly `14 mm` of depth, so far-range validation must use repeated measurements and a fully textured ROI.

The 3.9 m observation is promising diagnostic evidence only. Under the current policy, a depth above `2.55 m` must have:

```text
within_validated_depth_range: false
sie_measurement_emitted: false
status: ROI_DEPTH_REJECTED
```

If the runner emits an approved Measurement above `2.55 m`, treat that as a policy enforcement defect and stop the runtime.

## 15. Recommended continuation

Immediate checks:

1. Run `sha256sum` on the installed V6 guard, runtime, runner, policy, activation, review, and calibration.
2. Confirm the active runner is the latest hash `2b4a30db...`, not the earlier intermediate runner hash `1a6d432...`.
3. Capture the complete console JSON block at approximately `3900 mm`, including status, ROI valid ratio, disparity median/MAD, depth median/MAD, validated-range flag, emission flag, and temperatures.
4. Confirm that depth is computed diagnostically but no SIE `Measurement` is emitted beyond `2.55 m`.

If extended runtime range is required, create a separate challenge:

```text
approximately 3000 mm: at least 3 thermally stable runs
approximately 3900 mm: at least 3 thermally stable runs
approximately 4500 mm: at least 3 thermally stable runs
```

Requirements:

- camera temperatures inside the current V6 envelope;
- ambient recorded but not used as a numeric gate;
- same V6 run07 calibration and SGBM192 profile;
- large textured planar target fully covering the `100x100` ROI;
- explicit ground-truth datum and uncertainty;
- valid ratio, disparity median/MAD, depth median/MAD, repeatability and signed raw cross-frame difference recorded;
- no hidden scale or offset correction;
- new review, activation, and policy versions if accepted.

Do not expand the current policy in place.

## 16. First prompt for a new Codex session

```text
Continue SIE stereo_calibration_v6.
Read AI_CONTEXT.md first. Then read the applicable approved/frozen SIE
Engineering Specifications and canonical Data Contracts, followed by AGENTS.md and
vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md
before doing anything.

Verify the current working tree and all installed V6 SHA-256 values.
Report any conflict with a higher-authority SIE architecture or Data Contract.
Do not modify V5 files.
Do not modify frozen datasets or calibration evidence.
Do not expand the 0.38..2.55 m Measurement range.
First report any mismatch between the files, hashes, linked JSON records,
camera semantics, ROM mapping, and the handoff checkpoint.
```
