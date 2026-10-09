# SIEE Project Context

**Snapshot date:** 2026-10-09
**Purpose:** durable entry point for a human or AI agent continuing SIE work.  
**Status:** active engineering validation, not autonomous operation.

## Update rule

This file is the change-controlled snapshot of the active project state.

- Update it only after the user gives the exact command **`обновить документ`**.
- Do not silently update it after code edits, tests, experiments, or conversation summaries.
- When updating, preserve accepted evidence, distinguish fact from hypothesis, and state the revision date and what changed.
- This file describes current implementation and safe continuation. It never overrides approved Engineering Specifications, canonical Data Contracts, or calibration artifacts.

## Project name and compatibility

The project name is **Spatial Intelligence Evidence Engine (SIEE)**.

Existing technical identifiers remain unchanged for compatibility: repository
history, Python packages, ROS 2 packages, topics such as `/sie/...`, and schema
names such as `sie.*` continue to use `sie`. This naming decision does not rename
or invalidate frozen contracts, calibration artifacts, evidence IDs, or prior
reports. Historic wording `Spatial Intelligence Engine (SIE)` refers to the same
project unless it explicitly names a legacy technical artifact.

## Read this first

Read in this order before changing SIE:

1. `AI_CONTEXT.md` for the normative architecture and precedence rules.
2. This `PROJECT_CONTEXT.md` for the current cross-project state.
3. `AGENTS.md` for operational rules.
4. The relevant version-specific handoff, especially `vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/V6_HANDOFF.md` before V6 work.
5. The implementation, tests, reports, activation profile, and `git status` for the task at hand.

If these sources conflict, use the precedence in `AI_CONTEXT.md`; do not guess.

## What SIEE is

SIEE, Spatial Intelligence Evidence Engine, is an evidence-based spatial-intelligence architecture for a construction object. It integrates project and BIM/CAD data, surveying, inspections, sensor observations, material and zone state, work records, and reports from admitted robotic complexes, machines, and mechanisms into a traceable model of the physical world. SIEE uses that model for safe engineering decisions and authorized actions. It is not a particular neural model, camera, ROS 2 package, or robot base.

Canonical flow:

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

Important invariants:

- Every spatial value has a unit, explicit reference frame, timestamp, confidence, and provenance.
- Calibration is an applicability gate, not a shortcut to trust.
- Diagnostic evidence is never an approved Measurement, Fact, World State update, Decision, or Action.
- Models are replaceable. Data contracts and frame semantics are not.
- Missing, stale, ambiguous, or low-quality evidence fails closed.

## Current product direction

The long-term product is a construction-object spatial intelligence system: it maintains an evidence-backed spatial state of the project and coordinates data and authorized actions for admitted robotic complexes, machines, and mechanisms. A mobile base that follows a semantic target is one future executor and validation scenario, not the definition or boundary of SIEE. **The active stage deliberately excludes mobile-base navigation and motor control.** No output of the current dynamic fusion diagnostic may authorize motion.

The intended, future operating model is:

```text
BIM / project / schedule
+ surveying / inspections / sensors
+ material, zone, and quality state
+ reports from admitted robotic complexes, machines, and mechanisms
  -> evidence-backed SIEE World State
  -> Task Evaluator and Knowledge Engine
  -> Decision Engine
  -> authorized action for a specific executor
  -> evidence-backed re-observation and state update
```

The current RGB-to-stereo person experiment is one Vision Core validation path within this broader system. The target may be a person today and another object later. The geometry and decision layers must not be designed around `person_upper_body`.

### Current RGB + stereo architecture

| Responsibility | Current implementation | Architectural rule |
| --- | --- | --- |
| Semantic target | AR0234 RGB camera plus custom YOLO11 ONNX model, currently classed `person_upper_body` | AR0234 is the sole semantic authority in the new fusion path. The model label is a temporary adapter, not a system-wide object definition. |
| 2D target region | YOLO `bbox_xyxy_px`; a future approved backend may produce a real mask | A bbox is an area of uncertainty, not a depth point. Do not synthesize a mask from a bbox. |
| Geometry and depth | DECXIN dual OV9281 stereo, rectification, disparity, calibrated 3D reconstruction | Stereo provides geometry, not a second mandatory semantic person detector. |
| Cross-camera association | Validated AR0234-to-OV9281 physical-left extrinsic transform | Project the RGB target region through calibrated geometry into an admissible stereo frustum; do not measure the bbox centre. |
| Quality | Left-right disparity consistency, spatial connected component, depth clusters, MAD, ambiguity and temporal checks | Refuse a result when more than one plausible 3D surface, poor disparity, or inadequate evidence remains. |

The current dynamic diagnostic node intentionally does **not** require MediaPipe on OV9281. Earlier live metric code still contains an OV9281 MediaPipe detector; it is legacy static-MVP behavior and must not become a requirement for the intended RGB-semantic + stereo-geometry architecture.

### Required evidence for every fusion diagnostic cycle

A diagnostic result is useful only when it can be audited. Each cycle must either retain the complete evidence set named in the approved next sequence or explicitly identify which source was unavailable. Its outcome must be one of: `ASSOCIATED_DIAGNOSTIC_ONLY`, `TARGET_UNAVAILABLE`, `ASSOCIATION_UNAVAILABLE`, or a more specific evidence-backed rejection. Diagnostic evidence is not a Metric Measurement and must never be silently promoted to World State, navigation, or an actuator command.

## Hardware and runtime

| Item | Current fact |
| --- | --- |
| Compute | Raspberry Pi 5 `piecake`, Ubuntu Server 24.04, headless over SSH |
| Middleware | ROS 2 Jazzy, normally `ROS_DOMAIN_ID=42` |
| RGB camera | AR0234 UVC, 1920x1200, runtime auto exposure must be `3` |
| Stereo camera | DECXIN dual OV9281 UVC, combined 2560x800 at 60 FPS, MJPG, auto exposure `3` |
| Stereo half mapping | combined first half is physical **RIGHT**; combined second half is physical **LEFT** |
| Stereo physical baseline | 65.1 mm |
| Current camera-block pose | The rigid AR0234 + OV9281 assembly is pitched approximately 7° upward relative to the floor for the controlled semantic tests recorded on 2026-10-05. Its inter-camera mount was not changed. |
| Motor capability | ESP32 bounded-forward API exists; its use is separately supervised and is not enabled by diagnostic fusion |

Do not independently detach, loosen, rotate, bend, or otherwise change AR0234-to-OV9281 mechanical geometry without a new calibration plan. A rigid-body adjustment of the complete two-camera block preserves the AR0234-to-OV9281 extrinsic, but changes the block pose relative to the robot base/floor and must be recorded. It is not, by itself, a validation of a base/world-frame measurement configuration.

## Calibration and proven measurement envelope

The following artifacts are bound into the current ROS2 work. Their SHA-256 values must be verified from the actual files before use:

| Artifact | Identifier / SHA-256 |
| --- | --- |
| AR0234 intrinsic | `ar0234_intrinsic_v5_final_20260910`, `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` |
| OV9281 stereo calibration | active v7 Pi artifact SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`; its far-range metric accuracy is under re-validation |
| OV9281 v8 candidate | `ov9281_pi_stereo_candidate_v8`, SHA-256 `6b037ec3a79bd6d340875586353ea0554cc78f5332db8b4929462e80e944d956`; **rejected**, not active, after comparative physical-depth checks |
| OV9281 v8b candidate | `ov9281_pi_stereo_candidate_v8b`, SHA-256 `6691a86dd8daaa84d87cf5c124ba4309471ff92416f200bf4799843299dcf76d`; **rejected**, not active, after same-setup 1.5 m comparative check |
| AR0234 -> OV9281 physical-left extrinsic | `ar0234_ov9281_physical_left_extrinsic_v1`, `10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc` |
| RGB + stereo decision profile | `runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json` |

The 0.5 to 4.5 m v2 profile remains a conditional supervised diagnostic/recommendation constraint. It is not current evidence that v7 has physically validated metric accuracy across that whole interval, and it is not an authorization for dynamic fusion, autonomous navigation, or motors.

The current Pi copy of the 0.5 to 4.5 m v2 profile has SHA-256 `c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9` and status `ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY`. A diagnostic run outside 2.0 m must not silently fall back to the frozen original profile `ar0234_ov9281_decision_chain_v1`, whose authorized range is only 0.5 to 2.0 m.

### Static planar-depth evidence and datum limitation

The following checks compare the validator's `z` in `rectified_left_optical_frame` with laser distance from the **front edge of the physical-left OV9281 lens to the chessboard centre**. They are valuable screening evidence, but they are not yet a strict metric-depth certification: the laser datum is not the left optical centre, the rigid camera block is pitched upward, and the board was not mechanically constrained perpendicular to the left optical axis. A `z` coordinate, a slant range, and the median depth of a tilted board's corners are different quantities.

| Reference distance, m | Median depth, m | Absolute error, m | Per-run depth MAD, m | Evidence SHA-256 |
| ---: | ---: | ---: | ---: | --- |
| 1.500 | 1.497994 | 0.002006 | 0.007302 | `8a8344ab188457ba57d1ef6ac337eb125f5d9e7f7ee3f1ea1cae9968870821d4` |
| 2.000 | 2.052748 | 0.052748 | 0.012017 | `549b6bd5890d711524cb03e33a4f654de4c58b7c07715f62d64798bd3e0c2c1b` |
| 2.500 | 2.606853 | 0.106853 | 0.011260 | `19814eafe433178eee63acc3143b28e82ec361f6364db1565ac070a137de5c96` |
| 3.065, small-board repeat after 180-frame warm-up | 3.250298 | 0.185298 | 0.028556 | `8eb6de8ebd4973f9a2a1153b622428063cc443f51e3ea0770ed73dc8caa44480` |
| 3.065, new matte 48 mm board after 180-frame warm-up | 3.230612 | 0.165612 | 0.020497 | `e43525e697702d463887aa5825230d6243e4ca31ed0d6438da0ee55646aad64f` |

The 3.065 m matte-board run is important: replacing the small/glossy target reduced the spread and reduced the apparent positive difference only slightly. This excludes neither datum/pose error nor a calibration problem. It is **not proof of a physical cause** and must not be used alone to certify or reject v7 at a given range.

With `P2[0,3] = -75954.18816496264`, the implied disparity departure from the reference increased with distance: +0.068 px at 1.5 m, -0.976 px at 2.0 m, -1.245 px at 2.5 m, and -1.375 px at 3.065 m. This calculation inherits the same datum/pose limitation. Do not apply a scale or offset correction: it is a diagnostic signal, not a runtime patch.

### 2026-10-07 matte-board screening series

With the same 48 mm matte 9x6-inner-corner board, active v7, and 30 frames per run, the operator collected the following screening series. The 2.5 m first repeat that returned 2.046 m was taken before the board was correctly repositioned and is excluded; the output path was then overwritten by the valid run.

| Laser reference, m | Median `z`, m | Difference `z - reference`, m | Depth MAD, m | Run |
| ---: | ---: | ---: | ---: | --- |
| 1.500 | 1.500170 | +0.000170 | 0.014973 | current same-setup v7 control |
| 2.000 | 2.032131 | +0.032131 | 0.012147 | run01 |
| 2.000 | 2.048996 | +0.048996 | 0.015191 | run02 |
| 2.500 | 2.550983 | +0.050983 | 0.019792 | run01 |
| 2.500 | 2.558118 | +0.058118 | 0.020823 | run02 |
| 3.000 | 3.064197 | +0.064197 | 0.022729 | run01 |
| 3.000 | 3.079035 | +0.079035 | 0.026277 | run02 |
| 3.500 | 3.631426 | +0.131426 | 0.035391 | run01 |
| 4.000 | 4.268161 | +0.268161 | 0.046942 | run01 |

The series shows a repeatable, distance-growing difference between current v7 `z` and the stated laser procedure. Its frame-to-frame spread remains much smaller than the growing mean difference through 3.0 m. This is enough to motivate a pose-aware validation; it is **not enough** to declare a 2%, 3%, or 5% physical-depth pass/fail at those ranges. In particular, a camera pitch or an off-axis board normally makes optical-axis `z` less than slant range, whereas the observed difference is mostly positive; any eventual explanation must model all reference points and board pose rather than assume that one effect explains all values.

The earlier low-board and chest-board observations remain separate semantic/ROI evidence. In particular, the chest-board run where AR0234 selected a curtain is semantic-target-selection failure, not a v7 calibration diagnosis. All results still require separation of semantic selection, RGB-to-stereo association, stereo observability, and metric calibration.

### Rejected OV9281 v8 candidate (2026-10-06)

The first v8 capture combined 144 raw 48 mm matte-board pairs. Its initial audit passed the then-current centre-span criteria: horizontal centre span about 789 px, vertical span about 205 px, board-area range 5.70×, and strong sharpness. The independent holdout geometry was also superficially strong: stereo RMS 0.3396 px, physical-baseline difference 0.0683 mm, median holdout reprojection RMS 0.3580 px, and median rectified vertical residual 0.0987 px.

That solver result is **not** sufficient for acceptance. Direct physical checks, both with 180-frame warm-up, rejected the candidate:

| Candidate and reference | Distance datum, m | Median depth, m | Absolute cross-frame difference, m | Depth MAD, m |
| --- | ---: | ---: | ---: | ---: |
| v8, matte 48 mm board | 3.065 | 2.926988 | 0.138012 | 0.012358 |
| v8, matte 48 mm board | 1.500 | 1.428038 | 0.071962 | 0.010431 |
| active v7, same matte board and same 1.500 m setup | 1.500 | 1.524911 | 0.024911 | 0.015382 |

The same-setup 1.5 m A/B comparison rules out the new matte board or simple datum placement as a sufficient explanation for v8's 72 mm underestimate. v8 must remain a rejected evidence artifact; do not activate, copy, or correct it with a scale/offset.

One data-backed explanation to test, not an established root cause: the first v8 calibration set had no board centres low enough in the image. The physical-left and physical-right centre ranges were only Y=175.9..381.1 px and Y=163.5..368.5 px in 800 px-high images. v8's rectified `P1/P2` Y centre became about 421.8 px, versus about 372.2 px in v7, while its depth constant fell from 75,954.2 to 62,926.8 mm·px. These values motivate the controlled lower-field recapture; they do not by themselves identify the physical cause.

For v8b the audit now requires a board centre near the upper field and a board centre at or below the lower-field condition (`y_min <= 250 px`, `y_max >= 550 px`) in both physical images. A third 72-pair lower-field session improved combined vertical span to about 355 px and board-area diversity to 11.49×, but the combined 216-pair audit correctly rejected it: maximum centres were only 531.6 px (physical-left) and 518.1 px (physical-right). The next capture tool accepts an explicit Y band and prints the accepted centre; it is prepared for a fourth lower-field session.

### Rejected OV9281 v8b candidate (2026-10-07)

The gated fourth lower-field session completed, so the combined v8b capture contains 288 pairs. Its strengthened audit passed: physical-left board-centre coverage was X=257.6..1125.9 px and Y=175.9..678.5 px; physical-right was X=169.3..1048.1 px and Y=163.5..664.0 px. The board-area ratio was 11.49× and the audit report SHA-256 is `a0489583d16f2ac81eab16aa1debfc2acb1b409a7863148e34773b7f7e1ba2a0`.

The non-active v8b solver used 216 train and 72 holdout pairs. It reported stereo RMS 0.378096 px, baseline 64.842164 mm versus physical 65.1 mm, holdout reprojection median/p95 0.416312/0.667915 px, and holdout rectified vertical residual median/p95 0.131134/0.298921 px. The candidate report SHA-256 is `9bba447084015a090dd19cead8fca70c86ae4126cdaa38abc29a528af7c30d77`.

Those geometry statistics again did not predict field metric performance. With 180 warm-up frames and the same matte board at stated 1.5 m, v8b produced median `z=1.441585` m, a 58.415 mm difference from the stated laser datum. An immediate active-v7 control in the same setup produced `z=1.500170` m, a 0.170 mm difference. This is sufficient comparative evidence to reject v8b as a replacement for active v7. It must not be activated, copied into runtime artifacts, or corrected by an empirical scale/offset.

## What is implemented and tested

### ROS 2 contracts and static supervised chain

- The ROS 2 contract chain has been exercised on Pi: perception contract, navigation contract, and supervisor contract.
- AR0234 observation publishes structured observations with model SHA, bbox, confidence, truncation, and target status.
- AR0234 plus OV9281 metric Measurement has produced real live range/bearing records in the 0.5 to 4.5 m conditional profile.
- Navigation and supervisor can produce recommendations and blocked states.
- The normal pipeline keeps the actuator bridge disabled.

### Existing bounded motion capability

The ESP32 interface exposes `bounded_forward_0.10_m`. Its supervised profile requires an explicit `SUPERVISED_PERSON_APPROACH_SESSION`, short evidence freshness, ESP32 READY state, no fault, active brake, no retry, and mandatory re-observation. Earlier physical checks showed about 9.38 cm travel for a requested 0.10 m step.

This capability is a tested actuator primitive. It is **not** permission to run it from the current dynamic fusion diagnostic.

### Current dynamic fusion diagnostic

Implementation:

```text
ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py
topic: /sie/diagnostics/ar0234_ov9281/dynamic_association
```

The node:

- acquires AR0234 and OV9281 frames;
- obtains the semantic target from AR0234 YOLO only;
- uses the calibrated AR0234-to-physical-left transform to build a stereo frustum;
- applies left-right disparity consistency;
- keeps only a sufficiently supported connected spatial component;
- evaluates depth clusters inside that component without trimming the full cluster to hide spread;
- reports `ASSOCIATED_DIAGNOSTIC_ONLY`, `ASSOCIATION_UNAVAILABLE`, or `TARGET_UNAVAILABLE`;
- includes fail-closed temporal association logic, but a trustworthy live `TEMPORAL_CURRENT_FRAME_MATCH` has not yet been established as a validated dynamic capability.

It never publishes a Metric Measurement, navigation decision, or motor command. `metric_measurement_authorized` remains `false`.

Earlier 42-cycle diagnostic evidence:

| Outcome | Count |
| --- | ---: |
| `ASSOCIATED_DIAGNOSTIC_ONLY` | 5 |
| `ASSOCIATION_UNAVAILABLE` | 17 |
| `TARGET_UNAVAILABLE` | 20 |

The five associations had large coherent left-right-consistent spatial components, but many other frames were refused because full selected depth-cluster MAD exceeded the current 0.05 m limit. The correct response is to investigate target surface selection and evidence quality, not silently relax MAD, calibration, or range limits.

### Controlled AR0234 framing and stationary semantic range gate (2026-10-05)

The earlier horizontal-camera 3.0 m test exposed a framing problem rather than a model or stereo-calibration conclusion: the AR0234 at 0.43 m above the floor was parallel to the floor, so the standing person's head and upper body were clipped by the top of the source image. The full dynamic recorder produced only one semantic target in 58 cycles even though the person remained in view.

The complete AR0234 + OV9281 rigid assembly was then pitched approximately **7° upward**, with no relative adjustment between the cameras. This is the configuration for the following narrow controlled test set:

- one stationary standing person;
- artificial lighting and a mostly uniform wall background;
- AR0234 semantic node at a 0.4 confidence threshold and 2 Hz requested rate;
- no metric Measurement, navigation, or motion authorization.

| Distance | Semantic availability | Median confidence | Interpretation |
| --- | --- | ---: | --- |
| 2.0 m | 67 / 84 `SINGLE_TARGET` (79.8%) | 0.6246 | Not a confirmed stable semantic distance. |
| 2.5 m | 86 / 86 (100%) | 0.9031 | Stable in this controlled scenario. |
| 3.0 m | 14 / 14 (100%) in full dynamic evidence cycles | 0.9223 | Semantic gate stable; stereo association still refused. |
| 3.5 m | 97 / 97 (100%) | 0.8455 | Stable semantic availability; lower-image bbox contact occurred in some frames. |
| 4.0 m | 87 / 87 (100%) | 0.9380 | Stable in this controlled scenario. |
| 4.5 m | 88 / 88 (100%) | 0.8799 | Semantic reserve only; outside the stated 4.2 m room-use extent. |

For the present room, the evidence-backed **controlled semantic reference interval is 2.5 to 4.2 m**. The 4.5 m result demonstrates reserve in this scene; it does not expand the operating room requirement. The 2.0 m result must remain outside the confirmed stable interval.

This is not a general person-detection or operational-range claim. It has not tested natural motion, varied illumination, non-uniform backgrounds, clothing variation, multiple people, occlusion, or the stability of a true 2 Hz end-to-end diagnostic recorder.

At 3.0 m after the pitch adjustment, the first full dynamic evidence recorder retained semantic targets in all 14 cycles, with approximately 7.1k to 7.4k left-right-consistent points. Every cycle refused association as `ASSOCIATION_UNAVAILABLE` with `FRUSTUM_DEPTH_SAMPLES_OUT_OF_RANGE` and only 0 to 17 in-range depth samples. This was later traced to an activation-profile selection error: the run explicitly loaded the original 0.5 to 2.0 m profile, not a lack of stereo depth at 3.0 m. It must not be cited as evidence of inadequate human-surface stereo support.

The same 3.0 m full-evidence run took about 51.8 seconds for 14 cycles, roughly one stored cycle every 4 seconds. This is diagnostic evidence-recorder throughput, including capture, rectification, disparity, and evidence writes; it is not the AR0234 frame rate or a permitted runtime rate. Profile it separately before making a rate decision.

Evidence archives supplied by the operator:

| Archive | SHA-256 |
| --- | --- |
| `ar0234_3p0m_pitchup7_b_diagnostic_20261005.tar.gz` | `0d218ad8a149af67b05dc00d6c790876f3903de0babf4c068cf86c5092c0727c` |
| `ar0234_range_pitchup7_2p0_20261005.tar.gz` | `856fc424aa86e4e6b466273c2c1cf3780d07d63be194b3cd8a9144298d86b191` |
| `ar0234_range_pitchup7_2p5_20261005.tar.gz` | `5d7446f9ea03f641b75a8d3d74732c5864ff3f3914bc241a991eaf43a1a2e8b7` |
| `ar0234_range_pitchup7_3p5_4p0_20261005.tar.gz` | `3539279bcd371eaeaf2ab123ec68d1effe68a4d881e4084dd714ed3fe4f810c1` |
| `ar0234_range_pitchup7_4p5_20261005.tar.gz` | `c4eb2a340ddfdacc59e89186925b7b1a3e25f1143ca9e3501b3a4b3ec64c71f9` |
| `ar0234_depth_support_3p0_20261006.tar.gz` | `eb3c2b3d01dbbfec41b093fc02755054c095f1778aa6e9a738d7ac66f3a703be` |
| `ar0234_depth_support_3p0_profilev2_20261006.tar.gz` | `927a9846af77c3f4569d8d661dcfaa77ac9af22f936cd41a06cd0204cd2715d9` |

### 3.0 m human-surface audit with the 0.5 to 4.5 m v2 profile (2026-10-06)

The repeated stationary 3.0 m audit loaded the SHA-verified v2 extended-range profile. It retained complete evidence for 14 cycles over about 49.6 seconds. This test used the same restricted scene as the preceding semantic tests: one standing stationary person, artificial light, a mostly uniform wall, and the rigid camera block pitched about 7° upward.

| Check | Observed result |
| --- | --- |
| AR0234 semantic target | 14 / 14 cycles; confidence 0.874357 to 0.906889 |
| LR-consistent disparity support | 7,653 to 7,975 sampled points per cycle |
| Points inside the 0.5 to 4.5 m range | 7,437 to 7,845 per cycle |
| Points reprojected inside the AR0234 inner ROI | 3,413 to 3,569 per cycle |
| Spatial components | exactly one plausible connected component in every cycle |
| Depth clusters | exactly one plausible cluster in every cycle |
| `ASSOCIATED_DIAGNOSTIC_ONLY` | 8 / 14 cycles |
| `ASSOCIATION_UNAVAILABLE` | 6 / 14 cycles, all only because full-cluster depth MAD was 0.0538 to 0.0579 m, above the unchanged 0.050 m limit |

The eight associated cycles had diagnostic `z` values from 3.3024 to 3.3946 m, median 3.3245 m; their depth MAD was 0.0271 to 0.0481 m. The median projected point lay near the centre of the AR0234 torso region, and the associated bearing median was about -0.21°. This is evidence that the calibrated RGB-to-stereo path can locate a dense, connected 3D surface in the intended 2D person region under this controlled condition.

It is not a new Metric Measurement, range-accuracy validation, dynamic-fusion approval, navigation input, or actuator authorization. The label "3.0 m" is not yet an accuracy ground truth for this diagnostic `z`: the physical datum must be declared relative to the relevant camera optical frame before any difference is interpreted as bias. The six MAD refusals remain correct fail-closed outcomes. A possible transient at camera/exposure warm-up is a hypothesis only and must be tested without relaxing the MAD limit.

### Recent fusion-validation results

A later route-style audit produced **57 diagnostic cycles**. AR0234 produced a target in 12 cycles and stereo association succeeded in 10 of those 12. Representative associated depths included approximately:

- nominal 4 m -> about 4.220 m;
- nominal 3 m -> about 3.285 to 3.293 m;
- nominal 2 m -> about 2.259 m.

This run showed that the fusion chain can associate some targets over range, but it also made clear that **semantic target availability/selection was the dominant practical limiter in that run**. These measurements must not be treated as a new calibrated range profile because the route geometry, target surface, and ground-truth procedure were not equivalent to the controlled planar validation.

A later 26-cycle person test produced:

- 9 AR0234 detections with confidence about 0.91 to 0.94;
- roughly 7.4k to 8.6k left-right-consistent stereo points in detected frames;
- only 0 to 36 valid in-range depth samples in the projected target ROI in those frames;
- a later example with 8,683 LR-consistent points and 9,271 positive-disparity samples but only 56 in-range ROI samples, below the current minimum of 100.

The system correctly failed closed. This evidence is important because it shows that a globally healthy disparity field does not guarantee sufficient metric support on the projected human target surface.

### Static target-selection diagnosis

Controlled static-board runs isolated a semantic failure mode:

- `static_board_chest_1p5m` was not valid evidence for human-surface stereo quality because the detector frequently selected the curtain instead of the intended person/board target.
- The projected OV9281 ROI followed the incorrect AR0234 bbox, as it should under the current architecture.
- Therefore, the resulting stereo refusals must not be attributed to calibration or disparity until semantic selection is first proven correct on the source RGB frame.

The next controlled human test must first verify that the green AR0234 bbox continuously covers the actual person for the full observation interval before using ROI depth statistics to judge stereo support.

## Current engineering interpretation

The latest evidence requires explicit separation of two independent stereo-quality axes.

### 1. Geometric accuracy

Question:

> When a valid stereo correspondence exists, how accurately does calibrated geometry reconstruct metric depth?

The current v7 geometry agrees closely with the present 1.5 m same-setup control. The board series also shows a repeatable positive difference between v7 `z` and the current laser procedure from 2.0 m onward. Because that procedure does not yet place the board perpendicular to the optical axis or measure from the optical centre, it does not prove the physical-depth error of v7 at those ranges. It is nevertheless not sufficient to treat the v7 artifact as metrically proven across 0.5 to 4.5 m.

Relevant factors include intrinsic calibration, baseline, extrinsics, rectification, disparity precision, and physical calibration stability.

### 2. Stereo observability / target-surface support

Question:

> Does the intended target surface actually provide enough trustworthy stereo correspondence inside the projected ROI?

Relevant factors include surface texture, local contrast, illumination, exposure, motion blur, occlusion, repetitive patterns, matcher behavior, and semantic ROI placement.

The human-target evidence remains important, but it no longer justifies assuming that calibration accuracy is settled at 2.0 to 4.5 m. Good calibration cannot create disparity where the projected human surface has insufficient observable correspondence; conversely, good surface support cannot correct an unresolved range-dependent depth discrepancy.

This distinction is an engineering interpretation supported by the latest controlled tests. It does **not** yet approve a new measurement policy, a different matcher, active texture, another sensor modality, or online recalibration.

### External research note: calibration as runtime health

Recent external stereo-depth research reviewed during this work reinforced a useful hypothesis: high-precision stereo systems may need runtime calibration-health monitoring because heat, vibration, mechanical shifts, or impacts can degrade extrinsic alignment after nominal calibration.

For SIEE, a future `Calibration Health Observation` is therefore a candidate concept, for example carrying epipolar residual, drift suspicion, confidence, timestamp, and state such as `VALID`, `SUSPECT`, or `BLOCKED`.

This is **not an accepted architecture rule yet**. Do not implement online recalibration or add a new canonical object solely from vendor literature. First demonstrate an actual runtime calibration-drift failure mode in SIEE evidence.

## Current blockers and safety boundary

### Dynamic timing is not validated

AR0234 and OV9281 are currently captured sequentially by the host. Observed host pairing skew is about 11 to 12 ms. These timestamps are not exposure timestamps. Dynamic association can therefore be investigated only as diagnostic evidence.

Reliable dynamic engineering fusion requires proof of exposure synchronization. Repository history contains conflicting assumptions about external triggering support for this camera combination. Resolve that conflict from camera documentation and safe electrical evidence before wiring anything.

Until then:

- no dynamic Metric Measurement;
- no dynamic navigation authorization;
- no actuator command driven by dynamic fusion.

### Semantic target selection is not yet reliable enough

The chest-board diagnostic showed a high-confidence wrong-region failure mode: the RGB detector can select background structure such as a curtain, after which the calibrated projection correctly carries the wrong semantic target into stereo.

Therefore:

- no stereo-quality conclusion is valid for a cycle until the source AR0234 bbox is verified to cover the intended person;
- do not repair wrong RGB target selection by widening stereo ROI or relaxing stereo acceptance gates;
- confidence threshold changes must be justified by evidence. A higher threshold may suppress some false targets but is not a substitute for validating target identity.

The 2026-10-05 upward-pitch tests do establish continuous correct availability for one stationary person at 2.5 to 4.2 m in the stated artificial-light/plain-wall scenario. They do not resolve the general false-target risk above.

### Human-surface stereo support is not yet validated

The 2026-10-06 3.0 m v2-profile audit supersedes the earlier interpretation that the person ROI had too few in-range samples: that earlier result used the 0.5 to 2.0 m profile. With the correct 0.5 to 4.5 m profile, every cycle had thousands of in-range candidates, one connected reprojected component, and one depth cluster; eight cycles passed all current diagnostic gates.

Human-surface stereo support is still not generally validated. Six of the 14 controlled cycles failed the unchanged full-cluster MAD gate by a small margin, and the evidence does not yet distinguish camera/exposure settling, clothing/surface observability, or another source of spread. Motion, varied illumination, backgrounds, clothing, multiple people, occlusion, and exposure synchronization remain untested or unproven.

Do not attribute a failed human ROI automatically to calibration. However, the controlled planar range series independently challenges v7 at 2.0 to 3.065 m, so metric interpretation of the existing 2.5 to 4.2 m human-association diagnostics is provisional until a new candidate is validated.

### Electrical and calibration safety

- Do not connect ESP32 GPIO to camera TRG or STRB pins until pinout, signal levels, polarity, and common ground are verified with safe measurements.
- Do not tie STRB outputs together.
- No oscilloscope or logic analyser is available; a multimeter and then protected ESP32 event capture may be used only after electrical validation.
- Preserve the calibrated physical camera mount.

## OV9281 v7 relative and temporal validation (2026-10-09)

Active stereo calibration v7 remained SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`; no calibration, camera-control, Measurement, or motor policy was changed. The matte 9 x 6 inner-corner, 48 mm checkerboard was evaluated in `relative_axis_translation` mode. The front rim of the physical-left lens is a repeatable physical datum, but its offset to the optical centre is unknown. Thus the reported front-rim distance cannot establish absolute `rectified_left_optical_frame z`.

- Earlier manual 1.5 to 2.0 m axis translation: stereo delta 0.478089 m versus physical 0.500 m, relative error 4.382%, passing the *declared relative-translation* 5% screening criterion. A 1.5 to 4.0 m endpoint comparison likewise screened at about 3.41% relative error. Manual repositioning changes lateral alignment and board normal; these results neither certify all intermediate ranges nor prove 5% absolute depth accuracy. Further multi-point manual translations are deferred until a repeatable axial fixture or suitable independent metrology exists.
- In one continuously open-camera run, 10/10 frames passed each checkpoint at 0, 5, 15, and 30 minutes. Median stereo depths were respectively 1.490845, 1.466907, 1.468182, and 1.469608 m. Median disparities were 50.9471, 51.7785, 51.7335, and 51.6833 px. The 0-to-5-minute depth change was -23.94 mm while PnP board-centre z changed about -0.21 mm. Physical front-rim-to-board-plane distance was 1499 mm before and 1497 mm after the run (manual reading, declared 5 mm uncertainty). One normal-pose rejection occurred at the 5-minute checkpoint.
- After about 59 minutes disconnected from USB, a second continuously open-camera 0-to-5-minute run accepted 10/10 frames at each checkpoint without rejections. Median stereo depth went from 1.492973 to 1.461779 m (-31.19 mm), median disparity from 50.8745 to 51.9601 px (+1.0857 px), and PnP board-centre z from 1.515210 to 1.514748 m (-0.46 mm). The manually measured front-rim distance was 1497 mm before and after. Lateral board pose differed across sessions, so cross-session absolute depth differences are not attributed to camera drift.
- The within-run disparity and depth shifts greatly exceed the within-run PnP z changes and point to a stereo correspondence or relative-image-geometry effect under the existing intrinsics. PnP shares those intrinsics and is not an independent physical range measurement. It does help reject a board translation of roughly 24 to 31 mm as the sole explanation. Neither capture identifies a unique mechanism or supplies a correction coefficient.
- Historical v5 thermal evidence recorded the right module warmer than the left (ending near 37.125 versus 35.313 °C in a different experiment). The old DS18B20 sensors are currently removed. Temperature during these v7 runs was **not measured**; asymmetric heating is a hypothesis, not a demonstrated cause or a transferable v5 correction.

Source reports on the Pi: `~/.local/state/sie/evidence/ov9281_v7_relative_axis_20261008/point_1p5m_continuous_0_5_15_30min.json` and `point_1p5m_cool45_repeat_0_5min.json` in the same directory. The latter filename says `cool45`, while the operator's recorded USB-off interval was about 59 minutes. Retain the raw checkpoint and pose data with these reports. These diagnostic results do not admit stereo depth to operational Measurement.

## Active development stage

SIEE is in **pose-aware OV9281 depth validation plus evidence-driven AR0234-to-OV9281 target-association validation**, before validated dynamic measurement. Both v8 and v8b are rejected as replacements despite strong solver metrics. Active v7 remains unchanged. A narrow stationary-person semantic gate is confirmed at 2.5 to 4.2 m for the current 7° upward rigid camera-block pose, but its metric depth values remain provisional because no optical-frame-consistent physical validation has yet established its bounds above 1.5 m.

The architecture is accepted: AR0234 provides semantics; OV9281 provides geometry. The active uncertainty has three practical parts, with reproducible startup disparity drift now an explicit part of the first:

1. what the active v7 physical-depth error envelope is when the board pose and ground-truth datum are expressed in the same optical frame, and how its startup drift is detected or bounded;
2. whether AR0234 continuously selects the intended person rather than a false background target; and
3. when it does, whether the projected OV9281 ROI contains a connected, temporally stable, metrically plausible stereo surface belonging to that person.

The central question is now:

> Can active v7 meet the stated depth-uncertainty envelope under optical-frame-consistent validation, and then does an AR0234-selected person ROI retain enough trustworthy connected stereo support to form a stable 3D target surface?

The immediate goal is to establish the validity domain and uncertainty of a 3D target measurement in real scenes, including naturally moving people. A plausible range or bearing is not an approved navigation input until this question is answered experimentally.

### Proposed staged MVP direction, not motion authorization

The intended MVP combines AR0234 semantic detection, OV9281 relative range, base odometry/velocity, and LiDAR map/localisation. The working operating hypothesis is:

- a terminal zone at or inside 1.5 m, where v7 has a strong present planar control and where final alignment remains slow, supervised, and repeatedly re-observed;
- an approach zone outside the terminal zone, where OV9281 may contribute only after a pose-aware test establishes an explicit error envelope, with an initial engineering goal of no more than 5% range error;
- LiDAR and the map constrain the robot's own pose and safe route; they do not by themselves correct an unmodelled stereo bias to a moving person or object;
- a state estimator may fuse valid measurements and their covariances, but a Kalman filter cannot manufacture a correct value from an unknown systematic camera bias. Stereo bias and uncertainty must therefore be part of the measurement model and must be gated.

This is an MVP direction, not an approved change to the current motor-disabled safety boundary. Any future bounded motion requires a separate versioned Measurement policy, fresh physical ground truth, estimator validation, obstacle and stop behavior, and explicit supervised authorization.

## Approved next sequence

1. **Preserve active v7 unchanged.** Keep its artifact, activation profiles, camera geometry, and thresholds as evidence references. Do not overwrite it or insert a scale/offset correction.
2. **Characterize the reproduced startup shift without moving the board.** Keep the camera open continuously across timed checkpoints; preserve per-frame raw left/right images, disparity, PnP pose, camera controls, elapsed time, and rejection reasons. Repeat with independently recorded module temperatures if sensors can be mounted without perturbing camera geometry. Compare left/right image geometry and correspondence before assigning thermal causality or designing a correction.
3. **Defer a certifying multi-range envelope until the physical setup supports it.** Use a repeatable axial fixture or independent optical-frame survey for 1.5, 2.0, 3.0, and 4.0 m points, including uncertainty and repeatability. The earlier relative-translation 5% passes are screening evidence only. Establish an absolute optical-frame error criterion and a separate tighter terminal-zone criterion before admitting depth to Measurement.
4. **Keep v8 and v8b rejected.** Their solver metrics are useful evidence but no activation follows from them. Do not start another calibration candidate unless the pose-aware v7 test identifies a geometry problem that a recapture can address.
5. **Retain the complete per-cycle fusion evidence.** Preserve AR0234 source and semantic result; physical-left and physical-right rectified OV9281 frames; disparity before and after left-right validation; AR inner ROI, stereo frustum, valid-depth mask; pre-range depth diagnostic; connected-component and depth-cluster statistics; profile path/SHA, calibration hashes, timestamps, reference frame, confidence, and explicit refusal reason.
6. **Validate the staged MVP estimator in simulation/replay before motion.** Its state, measurements, covariances, bias model, innovation gating, LiDAR/map constraints, loss-of-target behavior, and stop policy must be explicit. Do not use a map to silently replace or recalibrate a person-range observation.
7. **Resume bounded human-surface diagnostics in a documented warm state.** Keep the 0.050 m MAD gate unchanged; record startup elapsed time and stereo stability, then test motion, illumination, background, clothing, multiple people, occlusion, recorder throughput, and exposure synchronization. Do not use the diagnostic result for navigation or motors. Operational use still requires an optical-frame error envelope and Measurement policy.
8. **Return to supervised bounded motion only after the vision and estimator gates pass.** Then make the semantic input contract generic as `SemanticTarget2D`, add tracking, validate a versioned dynamic Measurement policy against fresh physical ground truth, and enable the supervised `bounded_forward_0.10_m` primitive with mandatory re-observation. A segmentation backend is not a repair for a geometric calibration error, noisy disparity, weak texture, or unsynchronized exposure.

## Rules for the next agent

- Start from the current task and this file; do not revive obsolete plans from filenames or old branches.
- Explain why a command, test, or code change is needed before asking the operator to run it.
- Use headless-safe evidence collection on Pi. Do not assume a GUI.
- Use `auto_exposure=3` for both AR0234 and OV9281 unless a new evidence-backed experiment explicitly changes it.
- Do not add a mandatory OV9281 semantic detector to the RGB + stereo architecture.
- Do not replace calibrated geometry, relax safety thresholds, widen the range, or enable motors merely because a diagnostic frame looks promising.
- Preserve raw evidence, calibration hashes, activation profiles, and reference-frame semantics.
- When diagnosing sparse person depth, first prove the RGB bbox is on the intended person, then separate stereo geometric accuracy from target-surface observability.

## Revision history

| Date | Change |
| --- | --- |
| 2026-10-01 | Initial consolidated project snapshot: SIE architecture, Pi ROS2 state, RGB-to-stereo direction, dynamic diagnostic evidence, and explicit safety gates. |
| 2026-10-01 | Project name changed to Spatial Intelligence Evidence Engine (SIEE); existing `sie` technical identifiers explicitly retained for compatibility. |
| 2026-10-01 | Active scope narrowed to evidence-driven precise vision: AR0234 semantic target to OV9281 3D association, reprojection validation, per-cycle evidence, and temporal validation must be proven before motion returns to scope. |
| 2026-10-02 | Added latest fusion evidence: 57-cycle route audit, fail-closed person test with globally strong disparity but sparse in-ROI depth, and semantic false-target diagnosis from static chest-board runs. |
| 2026-10-02 | Added controlled 1.50 m front-board evidence: 10-cycle mean about 1.49998 m with approximately ±5.7 mm spread, strengthening the distinction between stereo geometric accuracy and human-surface observability. |
| 2026-10-02 | Updated active stage and next sequence: first prove continuous correct person bbox, then benchmark target-surface support; added plane/surface quality metrics and a conditional future calibration-health hypothesis. |
| 2026-10-05 | Clarified accepted system scope: SIEE maintains an evidence-backed spatial state of the construction object and coordinates data and authorized actions for admitted robotic complexes, machines, and mechanisms. A mobile base is one future executor, not the boundary of SIEE. Current Vision Core safety gates are unchanged. |
| 2026-10-05 | Recorded the rigid approximately 7° upward camera-block pitch and controlled stationary-person AR0234 range evidence. Confirmed semantic availability at 2.5 to 4.2 m only for artificial light, a stationary person, and a mostly uniform wall; recorded 4.5 m as reserve, 2.0 m as not stable, persistent 3.0 m frustum-depth refusals, and the full-evidence recorder throughput constraint. |
| 2026-10-06 | Diagnosed the earlier 3.0 m stereo refusal as use of the frozen 0.5 to 2.0 m activation profile. With SHA-verified 0.5 to 4.5 m v2, 8 of 14 controlled 3.0 m cycles reached diagnostic RGB-to-stereo association; the six remaining cycles correctly failed only the unchanged 0.050 m depth-MAD gate. Added activation-profile integrity and 3.0 m MAD-stability checks to the next sequence. |
| 2026-10-06 | Added direct OV9281 planar checkerboard range series: v7 is near exact at 1.5 m but has repeatable positive depth bias at 2.0, 2.5, and 3.065 m. A 180-frame warm-up and a new matte 48 mm board reduced neither the far-range bias nor its implied disparity deficit enough to explain it away. Kept v7 and all activation profiles unchanged; prepared non-active v8 capture, audit, and candidate-solver tooling. |
| 2026-10-06 | Built and rejected the non-active v8 candidate. Although 144 raw pairs, solver RMS, physical baseline, and holdout residuals were strong, 180-frame-warm physical tests underestimated 1.5 m by 72 mm and 3.065 m by 138 mm. A same-board/same-distance v7 control ruled out a simple test-board or datum explanation. Strengthened audit requirements after identifying insufficient lower-field capture coverage; three v8b sessions now contain 216 pairs but still fail the new lower-Y gate. Prepared a gated fourth lower-field capture for the next session. |
| 2026-10-09 | Recorded the manual relative-axis screening limits and two continuous-camera v7 runs with reproducible 0-to-5-minute depth/disparity shifts of -23.94 and -31.19 mm while board PnP z shifted less than 0.5 mm. No current module temperatures were measured; retained thermal asymmetry as a hypothesis. Deferred further manual multi-range certification, prioritized fixed-board startup diagnostics, and kept v7/runtime/motor policy unchanged. |
| 2026-10-07 | Completed the gated fourth v8b capture, passed the strengthened coverage audit with 288 pairs, solved v8b, and rejected it after a same-setup 1.5 m comparative check: v8b `z=1.441585` m versus active v7 `z=1.500170` m. Recorded the new 1.5 to 4.0 m v7 matte-board screening series, explicitly reclassified it as non-certifying because the laser datum and board pose do not yet match `rectified_left_optical_frame z`, and replaced blind recapture with pose-aware v7 validation. Added the proposed staged LiDAR/odometry/stereo estimator MVP direction, while retaining the existing motor-disabled safety boundary. |
