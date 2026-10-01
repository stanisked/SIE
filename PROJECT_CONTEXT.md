# SIEE Project Context

**Snapshot date:** 2026-10-01  
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
- reports `ASSOCIATED_DIAGNOSTIC_ONLY`, `ASSOCIATION_UNAVAILABLE`, or `TARGET_UNAVAILABLE`.

It never publishes a Metric Measurement, navigation decision, or motor command. `metric_measurement_authorized` remains `false`.

Latest observed spatial-gate evidence from a 42-cycle diagnostic run:

| Outcome | Count |
| --- | ---: |
| `ASSOCIATED_DIAGNOSTIC_ONLY` | 5 |
| `ASSOCIATION_UNAVAILABLE` | 17 |
| `TARGET_UNAVAILABLE` | 20 |

The five associations had large coherent left-right-consistent spatial components, but many other frames were refused because full selected depth-cluster MAD exceeded the current 0.05 m limit. The correct response is to investigate target surface selection and evidence quality, not silently relax MAD, calibration, or range limits.

## Current blockers and safety boundary

### Dynamic timing is not validated

AR0234 and OV9281 are currently captured sequentially by the host. Observed host pairing skew is about 11 to 12 ms. These timestamps are not exposure timestamps. Dynamic association can therefore be investigated only as diagnostic evidence.

Reliable dynamic engineering fusion requires proof of exposure synchronization. Repository history contains conflicting assumptions about external triggering support for this camera combination. Resolve that conflict from camera documentation and safe electrical evidence before wiring anything.

Until then:

- no dynamic Metric Measurement;
- no dynamic navigation authorization;
- no actuator command driven by dynamic fusion.

### Electrical and calibration safety

- Do not connect ESP32 GPIO to camera TRG or STRB pins until pinout, signal levels, polarity, and common ground are verified with safe measurements.
- Do not tie STRB outputs together.
- No oscilloscope or logic analyser is available; a multimeter and then protected ESP32 event capture may be used only after electrical validation.
- Preserve the calibrated physical camera mount.

## Active development stage

SIEE is in **evidence-driven AR0234-to-OV9281 fusion validation**, before validated dynamic measurement.

The architecture is accepted: AR0234 provides semantics; OV9281 provides geometry. The active work is to demonstrate, rather than assume, that a target detected by AR0234 projects onto the same physical target in the physical-left OV9281 image and has sufficient, connected, metrically correct stereo support across the conditional 0.5 to 4.5 m envelope.

The central question is:

> When AR0234 confidently detects a person, does its projected OV9281 ROI cover that person rather than background or another surface, and does it contain valid depth that remains temporally consistent?

The goal is not to make the robot move. The goal is to establish the validity domain and uncertainty of a 3D target measurement in real scenes, including naturally moving people. A plausible range or bearing is not an approved navigation input until this question is answered experimentally.

## Approved next sequence

1. **Complete the per-cycle fusion recorder.** For every diagnostic attempt, retain: AR0234 source frame and semantic result; physical-left and physical-right rectified OV9281 frames; disparity before and after left-right validation; projected RGB ROI or mask, stereo frustum, valid-depth mask; connected-component and depth-cluster statistics; calibration IDs/hashes, timestamps, reference frame, confidence, and explicit refusal reason. Do not assume every current diagnostic path already preserves all of this evidence.
2. **Measure extrinsic geometry rather than trusting it.** Quantify AR0234-to-physical-left OV9281 reprojection error throughout the usable field and depth range. Validate visually and quantitatively that projected target ROIs land on the physical person, not neighbouring background.
3. **Characterize stereo support inside the projected target ROI.** Test coverage, connected surface support, disparity/depth distribution, outliers, range error, false association, clutter, partial occlusion, lateral movement, and approach/recede across the conditional 0.5 to 4.5 m envelope.
4. **Define single-frame acceptance and refusal gates.** A valid result must distinguish verified 3D target, ambiguous target, insufficient depth, geometric mismatch, and temporal mismatch. Do not relax MAD, calibration, or range limits merely to increase acceptance.
5. **Validate temporal consistency.** Establish repeatable target identity, range, bearing, and uncertainty over sequences. An isolated successful frame is not a validated target. Static or slow association remains explicitly uncertain until exposure timing is proven.
6. **Resolve exposure synchronization safely.** Establish camera trigger capability, pinout, levels, polarity, and common-ground facts without disturbing mechanical calibration. Do not promote host-sequential pairing beyond diagnostics; reliable dynamic fusion remains blocked until temporal applicability is proven.
7. **Return to motion only after the vision gates pass.** Then, and only then, make the semantic input contract generic as `SemanticTarget2D`, add tracking, validate a versioned dynamic Measurement policy against fresh physical ground truth, and enable the supervised `bounded_forward_0.10_m` primitive with mandatory re-observation. Consider a genuine segmentation backend only if evidence shows that bbox background inclusion causes false 3D association; it is not a repair for noisy disparity or unsynchronized exposure.

## Rules for the next agent

- Start from the current task and this file; do not revive obsolete plans from filenames or old branches.
- Explain why a command, test, or code change is needed before asking the operator to run it.
- Use headless-safe evidence collection on Pi. Do not assume a GUI.
- Use `auto_exposure=3` for both AR0234 and OV9281 unless a new evidence-backed experiment explicitly changes it.
- Do not add a mandatory OV9281 semantic detector to the RGB + stereo architecture.
- Do not replace calibrated geometry, relax safety thresholds, widen the range, or enable motors merely because a diagnostic frame looks promising.
- Preserve raw evidence, calibration hashes, activation profiles, and reference-frame semantics.

## Revision history

| Date | Change |
| --- | --- |
| 2026-10-01 | Initial consolidated project snapshot: SIE architecture, Pi ROS2 state, RGB-to-stereo direction, dynamic diagnostic evidence, and explicit safety gates. |
| 2026-10-01 | Project name changed to Spatial Intelligence Evidence Engine (SIEE); existing `sie` technical identifiers explicitly retained for compatibility. |
| 2026-10-01 | Active scope narrowed to evidence-driven precise vision: AR0234 semantic target to OV9281 3D association, reprojection validation, per-cycle evidence, and temporal validation must be proven before motion returns to scope. |
