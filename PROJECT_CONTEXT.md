# SIEE Project Context

**Snapshot date:** 2026-10-02  
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

SIEE, Spatial Intelligence Evidence Engine, is an evidence-based robotics architecture. It turns sensor observations into a traceable model of the physical world and uses that model for safe decisions and actions. It is not a particular neural model, camera, ROS 2 package, or robot base.

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

The long-term product is a mobile base that can eventually follow a semantic target while continuously re-observing it. **The active stage deliberately excludes mobile-base navigation and motor control.** No output of the current dynamic fusion diagnostic may authorize motion.

The intended, future behavior is:

```text
RGB semantic target
  -> calibrated RGB-to-stereo association
  -> stereo metric range/bearing/uncertainty
  -> SIE decision
  -> bounded motion
  -> mandatory re-observation and correction
  -> stop at the allowed distance or on uncertainty
```

The target may be a person today and another object later. The geometry and decision layers must not be designed around `person_upper_body`.

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
| Motor capability | ESP32 bounded-forward API exists; its use is separately supervised and is not enabled by diagnostic fusion |

Do not detach, loosen, rotate, bend, or otherwise change AR0234-to-OV9281 mechanical geometry without a new calibration plan.

## Calibration and proven measurement envelope

The following artifacts are bound into the current ROS2 work. Their SHA-256 values must be verified from the actual files before use:

| Artifact | Identifier / SHA-256 |
| --- | --- |
| AR0234 intrinsic | `ar0234_intrinsic_v5_final_20260910`, `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` |
| OV9281 stereo calibration | active Pi artifact SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52` |
| AR0234 -> OV9281 physical-left extrinsic | `ar0234_ov9281_physical_left_extrinsic_v1`, `10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc` |
| RGB + stereo decision profile | `runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json` |

Independent physical depth checks supported the conditional **0.5 to 4.5 m** profile. This profile binds calibration and range for the static supervised chain. It is not an authorization for dynamic fusion, autonomous navigation, or motors.

### Latest static physical-depth evidence

Recent controlled static-board work materially strengthened the conclusion that the calibrated OV9281 stereo geometry itself can be metrically accurate when the intended target surface is actually selected.

- A final front-facing board test at nominal **1.50 m** produced a 10-cycle series with mean depth about **1.49998 m** and approximately **±5.7 mm** spread.
- This result does **not** validate arbitrary human surfaces, dynamic fusion, semantic selection, or exposure synchronization. It validates that, under a controlled textured planar target, the active stereo calibration can produce near-ground-truth range at 1.5 m.
- The experiment did not show a need for the auxiliary laser to achieve this controlled planar range accuracy.
- A prior low-board run was not a clean planar validation because the projected ROI mixed board and clothing; two cycles had depth MAD about 52.7 and 55.8 mm.
- A later chest-board run was invalid as a semantic-target test because the AR0234 detector selected a curtain in 31 of 36 cycles, with bbox x approximately 1812..1917. The stereo ROI followed that false semantic target. This is evidence of semantic target-selection failure, **not** evidence of stereo calibration failure.

These results require strict separation of three questions: semantic target selection, RGB-to-stereo association, and stereo surface quality.

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

Controlled textured planar tests now provide strong evidence that this can be very good near 1.5 m with the current calibration.

Relevant factors include intrinsic calibration, baseline, extrinsics, rectification, disparity precision, and physical calibration stability.

### 2. Stereo observability / target-surface support

Question:

> Does the intended target surface actually provide enough trustworthy stereo correspondence inside the projected ROI?

Relevant factors include surface texture, local contrast, illumination, exposure, motion blur, occlusion, repetitive patterns, matcher behavior, and semantic ROI placement.

The current human-target evidence suggests this question is now at least as important as calibration accuracy. Good calibration cannot create disparity where the projected human surface has insufficient observable correspondence.

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

### Human-surface stereo support is not yet validated

Even with confident AR0234 detections and thousands of globally LR-consistent stereo points, recent person tests often produced fewer than 100 in-range ROI depth samples. The cause is not yet proven. Candidate factors include target-surface observability, ROI placement, stereo matching quality on clothing/skin, occlusion, and timing.

Do not conclude that calibration is bad merely from sparse human ROI depth, because controlled board range accuracy is currently much stronger than human-surface support.

### Electrical and calibration safety

- Do not connect ESP32 GPIO to camera TRG or STRB pins until pinout, signal levels, polarity, and common ground are verified with safe measurements.
- Do not tie STRB outputs together.
- No oscilloscope or logic analyser is available; a multimeter and then protected ESP32 event capture may be used only after electrical validation.
- Preserve the calibrated physical camera mount.

## Active development stage

SIEE is in **evidence-driven AR0234-to-OV9281 target association and human-surface stereo-support validation**, before validated dynamic measurement.

The architecture is accepted: AR0234 provides semantics; OV9281 provides geometry. Controlled planar tests now show that stereo range accuracy can be excellent when the intended textured surface is correctly observed. The active uncertainty has moved toward two practical questions:

1. does AR0234 continuously select the intended person rather than a false background target; and
2. when it does, does the projected OV9281 ROI contain a connected, temporally stable, metrically plausible stereo surface belonging to that person?

The central question is now:

> When AR0234 continuously and correctly detects the person, does the projected OV9281 ROI remain on that person and contain enough trustworthy, connected stereo support to form a stable 3D target surface?

The goal is not to make the robot move. The goal is to establish the validity domain and uncertainty of a 3D target measurement in real scenes, including naturally moving people. A plausible range or bearing is not an approved navigation input until this question is answered experimentally.

## Approved next sequence

1. **Run the controlled 30-second stationary-person semantic gate.** Keep the person still and observe continuously. First verify that the green AR0234 bbox stays on the actual person for the full interval. Preserve every false-target, target-loss, and truncation event. Do not use stereo support statistics to judge the person surface in cycles where the semantic bbox is wrong.
2. **Complete the per-cycle fusion recorder.** For every diagnostic attempt, retain: AR0234 source frame and semantic result; physical-left and physical-right rectified OV9281 frames; disparity before and after left-right validation; projected RGB ROI or mask, stereo frustum, valid-depth mask; connected-component and depth-cluster statistics; calibration IDs/hashes, timestamps, reference frame, confidence, and explicit refusal reason. Do not assume every current diagnostic path already preserves all of this evidence.
3. **Quantify stereo support only on semantically valid person cycles.** Record in-ROI valid-depth count and ratio, LR-consistent support, connected-component size, depth median, MAD, temporal jitter, and rejection reason. Separate globally good disparity from target-ROI support.
4. **Add a controlled surface-observability benchmark.** At a fixed known distance, compare at least: calibration board / strongly textured planar target, textured fabric/object, plain clothing, and a stationary human torso. Use the same calibration and geometry. The purpose is to distinguish geometric accuracy from poor target-surface observability.
5. **Add surface-quality metrics beyond a single median depth.** For planar targets, fit a plane and record RMS and robust tail error in addition to range bias. For person targets, measure connected support and temporal surface consistency rather than relying only on an absolute minimum point count.
6. **Measure extrinsic geometry rather than trusting it.** Quantify AR0234-to-physical-left OV9281 reprojection error throughout the usable field and depth range. Validate visually and quantitatively that projected target ROIs land on the physical person, not neighbouring background.
7. **Define single-frame acceptance and refusal gates.** A valid result must distinguish verified 3D target, wrong semantic target, ambiguous target, insufficient target-surface depth, geometric mismatch, and temporal mismatch. Do not relax MAD, calibration, range, or point-count limits merely to increase acceptance.
8. **Validate temporal consistency.** Establish repeatable target identity, range, bearing, connected support, and uncertainty over sequences. An isolated successful frame is not a validated target. Static or slow association remains explicitly uncertain until exposure timing is proven.
9. **Resolve exposure synchronization safely.** Establish camera trigger capability, pinout, levels, polarity, and common-ground facts without disturbing mechanical calibration. Do not promote host-sequential pairing beyond diagnostics; reliable dynamic fusion remains blocked until temporal applicability is proven.
10. **Investigate calibration health only if evidence demands it.** Measure vertical epipolar residual and repeat controlled planar tests over time, temperature, or mechanical events. Add online calibration-health logic only if SIEE evidence demonstrates a meaningful drift mode.
11. **Return to motion only after the vision gates pass.** Then, and only then, make the semantic input contract generic as `SemanticTarget2D`, add tracking, validate a versioned dynamic Measurement policy against fresh physical ground truth, and enable the supervised `bounded_forward_0.10_m` primitive with mandatory re-observation. Consider a genuine segmentation backend only if evidence shows that bbox background inclusion causes false 3D association; it is not a repair for noisy disparity, weak target texture, or unsynchronized exposure.

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
