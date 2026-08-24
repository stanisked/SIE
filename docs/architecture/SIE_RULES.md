# Engineering Specification ES-002

**Document:** SIE_RULES.md

**Title:** SIE Rules

**Project:** Spatial Intelligence Engine (SIE)

**Version:** 0.1

**Status:** Approved

**Authority:** ES-001 SIE Architecture Constitution

**Related Specifications:**
- ES-000 Documentation Standard
- ES-001 SIE Architecture Constitution
- ES-003 SIE Engineering Principles
- ES-004 SIE Data Model
- ES-005 SIE Spatial Model
- ES-006 SIE Evidence Graph

---

# 1. Purpose

This specification defines the normative architectural rules governing Spatial Intelligence Engine (SIE).

Every implementation of SIE MUST comply with these rules.

---

# 2. Scope

This specification applies to:

- SIE Core
- Vision Core
- Geometry Engine
- Knowledge Engine
- World State Manager
- Decision Engine
- Lifecycle Engine
- Sensor Manager
- Every future SIE module

---

# 3. Normative Language

The keywords

**MUST**

**MUST NOT**

**SHOULD**

**SHOULD NOT**

**MAY**

are interpreted as described in RFC 2119 and RFC 8174.

---

# 4. Architecture Rules

---

## ARCH-001

### Title

Data Contracts

### Status

Approved

### Level

MUST

### Requirement

All SIE modules MUST communicate exclusively through approved Data Contracts.

Implementation-specific internal structures MUST NOT cross module boundaries.

---

## ARCH-002

### Title

Vision Responsibility

### Status

Approved

### Level

MUST

### Requirement

Vision Core MUST perform perception and measurement only.

Vision Core MUST NOT contain engineering knowledge.

Vision Core MUST NOT make engineering decisions.

---

## ARCH-003

### Title

Sensor Definition

### Status

Approved

### Level

MUST

### Requirement

A sensor is any information source capable of improving SIE's understanding of the physical world.

Physical sensors and digital information sources SHALL be treated uniformly.

---

## ARCH-004

### Title

World State Facts

### Status

Approved

### Level

MUST

### Requirement

World State MUST contain only verifiable facts.

Every fact MUST include:

- source
- timestamp
- confidence

---

## ARCH-005

### Title

Task Evaluator Responsibility

### Status

Approved

### Level

MUST

### Requirement

Task Evaluator evaluates tasks.

Task Evaluator MUST NOT make decisions.

---

## ARCH-006

### Title

Decision Responsibility

### Status

Approved

### Level

MUST

### Requirement

Decision Engine MUST make decisions using:

- World State
- Knowledge Engine
- Task Evaluator

Decision Engine MUST NOT bypass these components.

---

## ARCH-007

### Title

Entity-Centric World

### Status

Approved

### Level

MUST

### Requirement

World State SHALL be composed of Entities.

Every Entity MUST have:

- identity
- state
- properties
- lifecycle

---

## ARCH-008

### Title

Lifecycle Management

### Status

Approved

### Level

MUST

### Requirement

Lifecycle Engine MUST manage the complete lifecycle of every Entity.

Entities MUST NOT be immediately removed after observation loss.

---

## ARCH-009

### Title

Processing Cycle

### Status

Approved

### Level

MUST

### Requirement

SIE MUST operate using deterministic processing cycles.

Every cycle SHOULD be reproducible.

---

## ARCH-010

### Title

Knowledge Separation

### Status

Approved

### Level

MUST

### Requirement

Knowledge Engine is the only component responsible for engineering knowledge.

Engineering knowledge MUST NOT exist inside perception modules.

---

## ARCH-011

### Title

Confidence Before Decision

### Status

Approved

### Level

MUST

### Requirement

If confidence is insufficient,

SIE MUST acquire additional observations.

SIE MUST NOT guess.

---

## ARCH-012

### Title

Engineering Language

### Status

Approved

### Level

MUST

### Requirement

The canonical internal language of SIE SHALL use engineering concepts.

English SHALL be the canonical language.

---

## ARCH-013

### Title

Evidence Traceability

### Status

Approved

### Level

MUST

### Requirement

Every Fact,

World State update,

Decision,

and Action

MUST be traceable to originating observations through Evidence Graph.

---

# 5. Data Model Rules

Reference:

ES-004 SIE Data Model

Applicable Rules:

- DM-001
- DM-002
- DM-003
- DM-004
- DM-005
- DM-006
- DM-007
- DM-008
---

# 6. Spatial Rules

Reference:

ES-005 SIE Spatial Model

Applicable Rules:

- SP-001

---

# 7. Compliance

An implementation is considered SIE-compliant only if:

- all Architecture Rules are satisfied;
- all applicable Data Model Rules are satisfied;
- all applicable Spatial Rules are satisfied.

Violation of any **MUST** requirement makes the implementation non-compliant.

---

# 8. Rule Lifecycle

Architecture Rules are permanent identifiers.

A rule MAY have one of the following statuses:

- Draft
- Approved
- Deprecated
- Superseded

Rules SHALL never be deleted.

---

# 9. References

- ES-000 Documentation Standard
- ES-001 SIE Architecture Constitution
- ES-003 SIE Engineering Principles
- ES-004 SIE Data Model
- ES-005 SIE Spatial Model
- ES-006 SIE Evidence Graph

---

End of Specification
