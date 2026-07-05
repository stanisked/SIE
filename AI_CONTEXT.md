This file has higher priority than implementation examples.

When examples conflict with Engineering Specifications,
Engineering Specifications always take precedence.

When Engineering Specifications conflict with this file,
Engineering Specifications take precedence.

# AI_CONTEXT.md

# Spatial Intelligence Engine (SIE)

Version: 0.1

Status: Approved

---

# Purpose

This document is the primary entry point for AI assistants working on the SIE repository.

Its purpose is to establish a shared engineering context before generating architecture, code or documentation.

Every AI assistant SHALL read this document before performing any engineering task.

---

# Mission

Spatial Intelligence Engine (SIE) is an engineering architecture for Spatial Intelligence.

Its purpose is to transform heterogeneous observations into a continuously evolving understanding of the physical world and support reliable engineering decisions.

---

# Identity

SIE IS:

- Spatial Intelligence Architecture
- World Modeling System
- Engineering Decision Platform
- Evidence-driven System

SIE IS NOT:

- LLM
- VLM
- VLA
- Computer Vision Library
- ROS2 replacement
- Workflow Engine

AI models are replaceable implementation components.

They are never the architecture.

---

# Engineering Principles

1. Think in Reality

Model the real world, not algorithms.

2. Connect through Contracts

Modules communicate only through approved Data Contracts.

3. Replace without Fear

Implementations are replaceable.

Data Contracts are stable.

4. Evidence over Opinion

Engineering conclusions require evidence.

Never guess.

5. Challenge Before Acceptance

Architectural decisions begin as hypotheses and require engineering validation.

---

# Canonical Engineering Objects

Entity

Observation

Measurement

Interpretation

Fact

World State

Decision

Task

Role

Capability

---

# Canonical Information Flow

Raw Observation

↓

Observation

↓

Measurement

↓

Interpretation

↓

Fact

↓

World State

↓

Decision

↓

Action

---

# Spatial Language

Every spatial value MUST declare:

- Reference Frame
- Unit
- Confidence

No engineering decision may use spatial values without Reference Frame.

---

# Evidence

Every Fact,

World State update,

Decision,

and Action

must be traceable through Evidence Graph.

---

# World State

World State is the Single Source of Truth.

Only World State Manager may modify World State.

---

# Data Contracts

Modules communicate exclusively through approved Data Contracts.

Internal implementation details shall never cross module boundaries.

---

# Preferred Engineering Mindset

Always think in terms of:

Reality

↓

Observation

↓

Measurement

↓

Knowledge

↓

Decision

Never think in terms of:

OpenCV

Tensor

YOLO

ROS message

NumPy array

These are implementation details.

---

# Development Priorities

1. Working SIE
2. Stable architecture
3. Clear code
4. Good documentation
5. Performance optimization

Never sacrifice architecture for convenience.

Never sacrifice implementation for unnecessary perfectionism.

---

# Current Development Phase

Architecture Foundation v0.1

Engineering Specifications: Completed

Current implementation target:

Vision Core v1.0

---

# Current Vision Core Goals

Vision Core shall:

- build stable depth maps
- measure distances
- estimate object dimensions
- compute object coordinates
- evaluate measurement quality
- publish standardized Observations
- publish standardized Measurements

---

# Repository Structure

docs/

sie_core/

vision_core/

geometry_engine/

knowledge_engine/

decision_engine/

tests/

---

# Before Writing Code

Always verify:

Does this improve SIE?

Does this comply with Engineering Specifications?

Does this preserve Data Contracts?

Does this increase explainability?

Can this decision be traced through Evidence Graph?

If any answer is NO,

stop and rethink.

---

End of AI Context