# SIE project instructions for Codex

## Scope

This repository is the implementation workspace for SIE. The current active task is the DECXIN dual OV9281 stereo pipeline using `stereo_calibration_v6`.

Before changing V6 calibration, guard, runtime, capture, or validation code, read:

```text
vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md
```

If that file is not present in the repository, stop and ask for it. Do not reconstruct the current state from filenames alone.

## Required reading order

For every engineering task:

1. Read `AI_CONTEXT.md` as the repository entry point.
2. Read the applicable approved and frozen Engineering Specifications, including the Phase 1 `ES-000` through `ES-006` set when relevant.
3. Read the applicable canonical architecture, rules, data-model, spatial-model, Evidence Graph, and Data Contract documents.
4. Read this `AGENTS.md` for operational working rules.
5. Read the version-specific handoff, such as `V6_HANDOFF.md`.
6. Inspect the implementation, tests, reports, and current working tree.

Reading order does not change normative precedence. If documents disagree, use the precedence defined below.

## SIE project identity and purpose

SIE means both **see** and **Spatial Intelligence Engine**. Its purpose is to turn heterogeneous observations of the physical world into a continuously updated, evidence-backed world model and to use that model for reliable engineering decisions and safe actions.

The project follows this direction:

```text
seeing -> understanding -> deciding -> acting
```

SIE is not an LLM, VLM, VLA, computer-vision library, ROS 2 replacement, or generic workflow engine. Those systems may be replaceable components or integration surfaces. The durable product is the architecture, data contracts, evidence chain, world state, and decision discipline that remain valid when a camera, model, algorithm, or middleware implementation changes.

Observations may come from physical sensors or trusted external engineering sources such as BIM/CAD data, PLC state, maps, and approved operator input. Their origin and authority must remain explicit.

## Canonical information flow

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

Do not collapse adjacent stages or promote data to a later stage merely because it looks plausible.

- **Vision Core** performs perception and measurement. It may emit `Observation` and, after all applicability gates pass, `Measurement`.
- **Geometry Engine** owns transformations between explicit reference frames.
- **Knowledge Engine** owns engineering knowledge, constraints, and approved domain rules.
- **World State Manager** is the only component allowed to update entity-centric `World State` from verifiable `Fact` records.
- **Task Evaluator** evaluates task state and outcomes; it does not make decisions.
- **Decision Engine** decides using `World State`, `Knowledge Engine`, and `Task Evaluator` outputs.
- Execution components perform an approved `Action` and must preserve its link to the originating `Decision`.

## Non-negotiable architectural invariants

- Exchange data only through approved SIE Data Contracts. Library-specific arrays, tensors, OpenCV objects, ROS-specific messages, and similar structures are implementation details, not canonical contracts.
- Every spatial value must declare `reference_frame`, physical `unit`, `confidence`, and provenance. A spatial value without a valid reference frame is not usable for an engineering decision.
- A `Measurement` is quantitative and must state units, reference frame, confidence, provenance, timestamp, and the calibration or method used to produce it.
- `World State` may contain only verifiable `Fact` records with source, timestamp, confidence, and evidence linkage.
- Every entity must have stable identity, state, properties, and lifecycle semantics. Do not use display labels as identity.
- Every `Fact`, `World State` update, `Decision`, and `Action` must be traceable back to source `Observation` records through the Evidence Graph.
- Calibration is a registry, provenance record, and applicability gate. Unknown, invalid, expired, or out-of-scope calibration must block `Measurement` emission.
- Processing cycles must be deterministic and reproducible from declared inputs, versions, configuration, calibration, and policy.
- Insufficient confidence, missing evidence, stale inputs, or an applicability failure requires additional observation or a safe refusal. Never convert uncertainty into an unmarked best guess.
- Diagnostic estimates may be recorded for analysis, but they must remain clearly separated from approved `Measurement`, `Fact`, and `World State` data.

## Durable repository memory and source of truth

Use the following normative precedence when repository artifacts disagree:

1. approved and frozen Engineering Specifications, including the Phase 1 `ES-000` through `ES-006` set;
2. canonical architecture, rules, data-model, spatial-model, Evidence Graph, and Data Contract documents;
3. `AI_CONTEXT.md` as the concise repository entry point;
4. version-specific handoffs such as `V6_HANDOFF.md`;
5. implementation code, tests, examples, generated reports, and filenames.

`AGENTS.md` defines operational behavior for Codex. It must apply the normative documents above and must not silently redefine them. Higher-precedence normative artifacts win. If a conflict cannot be resolved from the repository, stop and report the exact conflicting statements.

`World State` is the operational source of truth about the current modeled physical world. It is not the normative source of truth for repository architecture or Data Contracts.

Keep durable knowledge separate from session state:

- record accepted design decisions and invariants in canonical documents;
- record version-specific evidence, hashes, commands, results, limits, and open items in the relevant handoff;
- label hypotheses, pending measurements, and proposed policy changes explicitly;
- never rewrite an unverified claim as an established fact;
- update the handoff in the same change that materially alters a validated implementation, activation state, operating envelope, or next-step sequence.

Changes to frozen SIE contracts follow this lifecycle:

```text
Research -> Draft -> Engineering Review -> at most 1-2 revisions -> Approved -> Freeze -> Implementation
```

Implementation convenience is not sufficient reason to bypass this lifecycle or weaken an invariant.

## Project artifact storage

- For each new SIE topic, use a dedicated folder inside the main SIE project folder in Google Drive.
- Save completed logical blocks as they become ready. Do not wait for the entire topic to finish.
- Preserve existing Drive organization and file identity. Replace or version the corresponding artifact instead of creating uncontrolled duplicates.
- Calibration datasets, manifests, checksums, policies, reports, scripts, validation evidence, architecture documents, and reproducibility records must have a durable project copy.
- Local files are working copies and must not be the only persistent record.
- If Google Drive is unavailable, prepare the local artifact, report the pending upload explicitly, and never claim that it was saved remotely.

## Working rules

- Inspect the working tree and relevant files before editing.
- Treat existing user changes as authoritative. Do not overwrite unrelated modifications.
- Check SHA-256 before replacing any frozen artifact, calibration, activation record, review, policy, guard, or runtime file.
- Preserve raw datasets. Never edit, rename, reorder, rescale, rectify, recompress, or overwrite frozen source images.
- Do not silently correct depth with a scale factor or offset.
- Every spatial measurement must declare its `reference_frame`.
- A depth value may become an SIE `Measurement` only after calibration, temperature, ROI, and quality gates pass.
- Challenge a calibration with fresh checkerboard and physical near/mid/far validation before activation or policy expansion.
- Keep V6 work isolated from V5. Add new versioned files instead of modifying validated V5 files.
- After Python changes, run `python3 -m py_compile` on every modified Python file and the smallest relevant diagnostic or guard test.
- Report exact paths, commands, outputs, and SHA-256 values. Do not report a test as passed unless it was actually executed.

## Protected V5 reference implementation

Do not modify these files unless the user explicitly requests a V5 change:

```text
vision_core/vision_benchmark/hardware_audit/capture_raw_stereo_pairs.py
vision_core/tools/sie_ds18b20_serial_bridge.py
vision_core/tools/run_guarded_stereo_v5.py
vision_core/stereo/guarded_runtime.py
vision_core/stereo/stereo_calibration_guard.py
vision_core/config/runtime/stereo_calibration_v5_runtime_policy_v3.json
```

Known reference hashes:

| File | SHA-256 |
| --- | --- |
| `capture_raw_stereo_pairs.py` | `0df83f99a6f47a498ca68d41a629fdd999c9278811e35867834f5b09f68378a8` |
| `sie_ds18b20_serial_bridge.py` | `0de8d8454dd6a130fefe70ab45c9cb39dcab3c1f628a1e79fdf4e416b5ed8dc5` |
| `run_guarded_stereo_v5.py` | `47519a5e36b28891e1c8898422edab8dad2ea1f7e5e5fba6449fc11f71c78218` |
| `guarded_runtime.py` | `1a4763572768f402db3c782639c413a15e604a4013be5d1bd1aa5198859d648b` |

If a local hash differs, do not overwrite the file. Inspect the diff and report the mismatch.

## Canonical stereo image semantics

V6 deliberately uses the proven V5 mapping:

| Combined frame region | Physical camera | Saved/runtime semantic |
| --- | --- | --- |
| left half, `x=0:1280` | physical RIGHT | `right_raw` / `right` |
| right half, `x=1280:2560` | physical LEFT | `left_raw` / `left` |

Do not reverse this mapping based on display position, board orientation, filename order, or temperature sensor labels.

Capture contract:

```text
V4L2, MJPG, 2560x800, 60 FPS, buffer size 1, auto_exposure=3
```

The physical baseline measured on the module is `65.1 mm`.

## Temperature channel contract

Final physical mapping:

| Channel | Sensor | ROM | Placement |
| --- | --- | --- | --- |
| `camera_left` | S03 | `28F952990F510A0C` | rear of physical LEFT camera |
| `camera_right` | S02 | `28FE6BA3299C9AC4` | rear of physical RIGHT camera |
| `ambient` | S10 | `289452A555A32FDA` | free air away from the module |

Rules:

- `camera_left` and `camera_right` are gating channels.
- `ambient` is observational only. It must still be present, mapped correctly, finite, and fresh, but its numeric temperature must not block depth.
- Temperature state file: `/tmp/sie_h05b_temperature_state.json`.
- Maximum state age: `5.0 s`.
- Controller reset, stale state, invalid values, missing channels, or ROM mismatch must fail closed.
- Current provisional V6 envelope:
  - `camera_left`: `30.0000..32.3750 C`
  - `camera_right`: `31.1250..33.9375 C`
- Outside the envelope, preview may continue but SIE depth `Measurement` must be blocked.

Reference bridge command:

```bash
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

## Authoritative V6 calibration and runtime

Authoritative source calibration:

```text
/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/
solution_joint_refine_corner_order_filtered_freeze_v2_run07/
stereo_params_v6.npz
```

Expected calibration SHA-256:

```text
bb8fb665c6e06e2cbb633cf4c3c61aa74933dd253c9c7950a8420591975dd5e7
```

V6 implementation files:

```text
vision_core/stereo/stereo_calibration_guard_v6.py
vision_core/stereo/guarded_runtime_v6.py
vision_core/tools/run_guarded_stereo_v6.py
vision_core/config/runtime/stereo_calibration_v6_runtime_policy_v1.json
```

Expected hashes:

| File | SHA-256 |
| --- | --- |
| `stereo_calibration_guard_v6.py` | `42e8eff34700030e32cc0c1419122cd8e23e1d88055fc943f604fefb1f2e0d05` |
| `guarded_runtime_v6.py` | `e9602d318021a934f61116c3b1f6f75e6277b0686eab0810cd387f1a109edee1` |
| `run_guarded_stereo_v6.py` | `2b4a30db75c7265ec800f943f3eb68502180ce054ccd849546b26a24b997bb79` |
| `stereo_calibration_v6_runtime_policy_v1.json` | `a26b4a9335296fe4f5584e4d6ba5a74a1eb03c795833c976db090c661e99464c` |

The V6 runtime must keep the V5 SGBM profile, including `num_disparities=192`, unless a separate versioned challenge proves a replacement.

## Frozen runtime limits

| Parameter | Value |
| --- | ---: |
| ground-truth validation range | `0.4..2.5 m` |
| accepted measured depth | `0.38..2.55 m` |
| minimum ROI valid ratio | `0.90` |
| maximum disparity MAD | `0.50 px` |
| maximum depth spatial MAD | `0.020 m` |
| ROI center | `(650, 350)` |
| ROI size | `100x100 px` |
| reference frame | `rectified_left_optical_frame` |

Depth can be computed outside `0.38..2.55 m` for diagnostics, but it must not be emitted as an approved SIE `Measurement` under the current policy.

Do not lower the near limit or increase the far limit from a single observation. Use a new versioned validation review and policy.

## Ground-truth reference

The current physical ground-truth datum is the front edge of the physical LEFT lens barrel:

```text
physical_left_lens_front_rim_frame
```

Runtime depth is expressed in:

```text
rectified_left_optical_frame
```

The axial conversion is not yet independently surveyed:

```text
Z_rectified_left_optical = Z_lens_front_rim + t_z
```

`t_z` is positive and pending. Until measured, direct differences between tape or laser readings from the lens rim and stereo optical-frame depth are cross-frame differences, not an approved scale or offset calibration. Hidden correction remains forbidden.

## Current status and safe next steps

- V6 runtime is operational.
- Depth is reported as very good from approximately `390 mm` through `2000 mm`.
- Below approximately `390 mm`, SGBM support starts to fail as disparity approaches the `192 px` search limit.
- At approximately `3900 mm`, diagnostic depth around `3876..3890 mm` has been observed, but this is outside the approved runtime range and must remain non-emitting until an extended-range challenge passes.

Safe continuation:

1. Verify installed V6 hashes and linked JSON integrity.
2. Preserve the current conditional policy unchanged.
3. Independently survey `t_z` from the physical LEFT lens front rim to the optical reference.
4. If extended range is required, run a separate challenge at approximately `3000`, `3900`, and `4500 mm`, with at least three thermally stable repetitions per distance and a textured target fully covering the ROI.
5. Create a new review, activation record, and runtime policy version only after the challenge is accepted.
