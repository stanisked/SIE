# AI_CONTEXT.md

# Spatial Intelligence Engine (SIE)

Version: 0.2

Status: Approved

This file has higher priority than implementation examples.

When examples conflict with Engineering Specifications, Engineering Specifications always take precedence.

When Engineering Specifications conflict with this file, Engineering Specifications take precedence.

## Purpose

This document is the primary entry point for AI assistants working on the SIE repository.

Its purpose is to establish a shared engineering context before generating architecture, code, experiments, or documentation.

Every AI assistant shall read this document before performing any engineering task.

After reading this file, read the applicable approved and frozen Engineering Specifications and canonical Data Contracts, then `AGENTS.md`, the relevant version-specific handoff, and the implementation under inspection.

## Normative precedence

When repository artifacts disagree, use this order:

1. approved and frozen Engineering Specifications, including Phase 1 `ES-000` through `ES-006` when applicable;
2. canonical architecture, rules, data-model, spatial-model, Evidence Graph, and Data Contract documents;
3. this `AI_CONTEXT.md` entry point;
4. version-specific handoffs;
5. implementation code, tests, examples, generated reports, and filenames.

`AGENTS.md` defines operational behavior for Codex and must apply this normative hierarchy without redefining it.

If a conflict cannot be resolved from repository evidence, stop and report the exact conflicting statements. Never guess which contract was intended.

## Mission

Spatial Intelligence Engine (SIE) is an engineering architecture for Spatial Intelligence.

Its purpose is to transform heterogeneous observations into a continuously evolving, evidence-backed understanding of the physical world and support reliable engineering decisions and safe actions.

## Identity

SIE is:

- Spatial Intelligence Architecture;
- World Modeling System;
- Engineering Decision Platform;
- Evidence-driven System.

SIE is not:

- an LLM;
- a VLM;
- a VLA;
- a computer-vision library;
- a ROS 2 replacement;
- a workflow engine.

AI models, perception libraries, middleware, and sensor implementations are replaceable components. They are never the architecture.

## Engineering principles

1. **Think in Reality**

   Model the real world, not algorithms.

2. **Connect through Contracts**

   Modules communicate only through approved Data Contracts.

3. **Replace without Fear**

   Implementations are replaceable. Data Contracts are stable.

4. **Evidence over Opinion**

   Engineering conclusions require evidence. Never guess.

5. **Challenge Before Acceptance**

   Architectural and calibration decisions begin as hypotheses and require explicit engineering validation before acceptance.

## Canonical engineering objects

- Entity
- Observation
- Measurement
- Interpretation
- Fact
- World State
- Decision
- Action
- Task
- Role
- Capability

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

Do not collapse adjacent stages or promote data merely because it appears plausible.

## Component boundaries

- **Vision Core** acquires and processes sensor data. It may emit `Observation` and, after all applicability gates pass, `Measurement`.
- **Geometry Engine** owns transformations between explicit reference frames.
- **Knowledge Engine** owns approved engineering knowledge, constraints, and domain rules.
- **World State Manager** is the only component allowed to update entity-centric `World State` from verifiable `Fact` records.
- **Task Evaluator** evaluates task state and outcomes. It does not make decisions.
- **Decision Engine** makes decisions using `World State`, engineering knowledge, and task-evaluation outputs.
- Execution components perform an approved `Action` and preserve its link to the originating `Decision`.

Do not create shortcuts from sensor output, disparity, depth, a model prediction, or an interpretation directly to `World State`, `Decision`, or `Action`.

## Observation and Measurement

An `Observation` records what a source produced, together with its identity, timestamp, provenance, and quality state.

A `Measurement` is an independent quantitative result derived from one or more observations. Every `Measurement` must declare:

- physical value and unit;
- explicit `reference_frame`;
- timestamp;
- source observation identity;
- confidence and quality status;
- method and calibration identity;
- calibration and runtime-policy version or hash when applicable;
- sufficient provenance to reproduce or audit the result.

Unknown, invalid, expired, stale, or out-of-scope calibration must block `Measurement` emission. A diagnostic estimate may be logged for investigation but must not be presented as an approved `Measurement`.

## Spatial language

Every spatial value must declare:

- `reference_frame`;
- physical unit;
- confidence;
- provenance.

No engineering decision may use a spatial value without a valid reference frame.

Frame transformations must name their source frame, target frame, transform identity, validity, and uncertainty.

## Evidence

Every `Fact`, `World State` update, `Decision`, and `Action` must be traceable to source `Observation` records through the Evidence Graph.

Insufficient evidence, missing provenance, stale input, inadequate confidence, or an applicability failure requires additional observation or a safe refusal. Never convert uncertainty into an unmarked best guess.

## World State

`World State` is the operational source of truth about the current modeled physical world.

Only the `World State Manager` may modify it, and only from verifiable `Fact` records with source, timestamp, confidence, and evidence linkage.

`World State` is not the normative source of truth for repository architecture, Engineering Specifications, or Data Contracts. Those follow the normative precedence defined above.

## Data Contracts

Modules communicate exclusively through approved SIE Data Contracts.

Internal implementation details must never cross module boundaries as canonical contracts. This includes OpenCV objects, NumPy arrays, tensors, model-specific structures, and ROS-specific messages.

## Preferred engineering mindset

Always reason through the canonical engineering flow:

```text
Reality
  -> Raw Observation
  -> Observation
  -> Measurement
  -> Interpretation
  -> Fact
  -> World State
  -> Decision
  -> Action
```

Do not organize architecture around implementation details such as OpenCV, tensors, YOLO, ROS messages, or NumPy arrays.

## Development priorities

1. Working SIE
2. Stable architecture
3. Clear code
4. Good documentation
5. Performance optimization

Never sacrifice architecture for convenience.

Never sacrifice a usable implementation for unnecessary perfectionism.

## Current development phase

```text
Architecture Foundation: v0.1
Engineering Specifications: completed
Current implementation target: Vision Core v1.0
```

These phase statements are project status, not permanent architectural invariants. Update them when the approved project state changes.

## Current Vision Core goals

Vision Core shall:

- build stable depth maps;
- measure distances;
- estimate object dimensions;
- compute object coordinates;
- evaluate measurement quality;
- publish standardized `Observation` records;
- publish standardized `Measurement` records.

## Repository structure

```text
docs/
sie_core/
vision_core/
geometry_engine/
knowledge_engine/
decision_engine/
tests/
```

The actual repository may evolve, but architectural ownership must remain explicit and consistent with approved specifications.

## Before writing code

Always verify:

- Does this improve SIE?
- Does this comply with Engineering Specifications?
- Does this preserve Data Contracts?
- Does this preserve explicit reference-frame semantics?
- Does this increase explainability and reproducibility?
- Can this decision be traced through the Evidence Graph?

If any answer is no, stop and rethink.

## Project artifact storage

All artifacts created or updated as part of SIE work shall be uploaded to the corresponding SIE project folder in Google Drive.

For each new topic, create or use a dedicated folder inside the main SIE project folder. Save completed logical blocks as they become ready instead of waiting for the entire topic to finish.

This includes:

- Engineering Specifications and architecture documents;
- calibration protocols and parameters;
- frozen datasets, manifests, and checksums;
- experiment inputs, outputs, reports, plots, and accepted conclusions;
- source snapshots, policies, activation records, and validation evidence;
- other files required to reproduce an engineering result.

Local files are working copies and must not be treated as the only persistent project record.

Preserve existing Google Drive organization and file identity. Updated artifacts should replace or version the corresponding Drive artifact instead of creating uncontrolled duplicates.

If Drive access is unavailable, prepare the local artifact, report the pending upload explicitly, and never claim that it was saved remotely.

## Change discipline

- Preserve immutable raw observations and frozen evidence.
- Label hypotheses, proposals, pending measurements, and unverified observations explicitly.
- Record accepted decisions in canonical documents.
- Record version-specific evidence, hashes, commands, limits, and open items in the relevant handoff.
- Update the handoff in the same change that alters a validated implementation, activation state, operating envelope, hardware mapping, or safe continuation order.
- Create a new versioned artifact instead of mutating a frozen contract or concealing behavior changes behind an old name.

Changes to frozen SIE contracts follow this lifecycle:

```text
Research -> Draft -> Engineering Review -> at most 1-2 revisions -> Approved -> Freeze -> Implementation
```

Implementation convenience is not sufficient reason to bypass this lifecycle or weaken an invariant.

---

End of AI Context
