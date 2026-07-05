# SIE Documentation Standard

**Document:** DOCUMENTATION_STANDARD.md

**Project:** Spatial Intelligence Engine (SIE)

**Version:** 1.0

**Status:** Approved

**Authority:** SIE Architecture Foundation

---

# 1. Purpose

This document defines the documentation standard for the Spatial Intelligence Engine (SIE) project.

Its purpose is to ensure that all official documents are:

- consistent;
- traceable;
- machine-readable;
- maintainable;
- suitable for long-term engineering development.

This standard applies to every official document in the SIE repository.

---

# 2. Documentation Philosophy

Documentation is part of the architecture.

Documentation is not a description of the code.

Code implements the documentation.

Whenever code and documentation disagree, the approved documentation takes precedence.

---

# 3. Documentation Hierarchy

The official documentation hierarchy is:

Level 0

Repository metadata

- README.md
- LICENSE
- CONTRIBUTING.md

---

Level 1

Constitution

- SIE_ARCHITECTURE.md

---

Level 2

Normative Specifications

- SIE_RULES.md
- SIE_ENGINEERING_PRINCIPLES.md
- SIE_DATA_MODEL.md
- SIE_SPATIAL_MODEL.md
- SIE_EVIDENCE_GRAPH.md

---

Level 3

Architecture Decision Records

ADR-0001
ADR-0002
...

---

Level 4

Implementation Specifications

Vision Core

Geometry Engine

Knowledge Engine

Decision Engine

...

---

Level 5

Developer Documentation

Examples

Tutorials

Notes

Experiments

---

# 4. Document Header

Every official document MUST begin with the following header.

```text
Document:
Project:
Version:
Status:
Owner:
Authority:
Last Updated:
Related Documents:
```

---

# 5. Document Status

Official document status values are:

Draft

Review

Approved

Frozen

Deprecated

Superseded

Only Approved and Frozen documents are normative.

---

# 6. Rule Status

Architecture rules use the following states.

Draft

Approved

Deprecated

Superseded

Rules are never deleted.

Deprecated rules remain part of the project history.

---

# 7. Rule Identifier Format

Every rule SHALL have a permanent identifier.

Examples:

ARCH-001

ARCH-002

DM-001

SP-001

EG-001

Identifiers are immutable.

Numbers are never reused.

---

# 8. Normative Language

The keywords

MUST

MUST NOT

SHOULD

SHOULD NOT

MAY

are interpreted according to RFC 2119 and RFC 8174.

Only uppercase keywords are normative.

---

# 9. Rule Structure

Every rule SHALL use the following structure.

ID

Title

Status

Level

Applies To

Requirement

Verification

Rationale

Related ADR

---

Example

ARCH-001

Title

Data Contracts

Status

Approved

Level

MUST

Applies To

All Modules

Requirement

All modules MUST communicate exclusively through approved Data Contracts.

Verification

Inspection of public interfaces.

Rationale

See ADR-0001.

---

# 10. Architecture Decision Records

Every major architectural decision SHALL have an ADR.

ADR files use the format:

ADR-0001-Title.md

Example:

ADR-0004-Evidence-Graph.md

An ADR contains:

Status

Context

Decision

Consequences

Alternatives Considered

Related Rules

---

# 11. Cross References

Documents SHALL reference rules using permanent identifiers.

Correct:

ARCH-011

Incorrect:

Rule 11

---

# 12. Versioning

Architecture documents use semantic versioning.

Major

Breaking architectural changes.

Minor

New approved concepts.

Patch

Editorial improvements only.

---

# 13. Language

The canonical language of SIE documentation is English.

Translations are informative only.

All identifiers, APIs, contracts and engineering concepts SHALL use English.

---

# 14. Machine Readability

Documents SHALL be written so that they can be interpreted by:

- developers;
- AI assistants;
- documentation generators;
- static analyzers;
- CI systems.

The document structure SHALL remain consistent across the project.

---

# 15. Traceability

Every normative requirement SHOULD reference:

Architecture Rule

ADR

or another approved document.

Architectural decisions must always be traceable.

---

# 16. Stability Policy

Approved documents evolve conservatively.

Frozen documents evolve only through formal architectural review.

No implementation may redefine an approved architectural requirement.

---

# 17. Documentation Lifecycle

Every new document follows the lifecycle:

Idea

↓

Research

↓

Challenge

↓

Draft

↓

Review

↓

Approved

↓

Frozen

↓

Implementation

This lifecycle follows SIE Engineering Principle:

Challenge Before Acceptance.

---

End of Document