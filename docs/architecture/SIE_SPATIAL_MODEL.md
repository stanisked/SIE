# Engineering Specification ES-005

**Document:** SIE_SPATIAL_MODEL.md

**Title:** SIE Spatial Model

**Project:** Spatial Intelligence Engine (SIE)

**Version:** 0.1

**Status:** Approved

**Authority:** ES-001 SIE Architecture Constitution

**Related Specifications:**
- ES-000 Documentation Standard
- ES-001 SIE Architecture Constitution
- ES-002 SIE Rules
- ES-004 SIE Data Model
- ES-006 SIE Evidence Graph

---

# 1. Purpose

This specification defines the canonical spatial model of Spatial Intelligence Engine (SIE).

Every spatial measurement, object, relation and decision SHALL use this model.

---

# 2. Design Goals

The Spatial Model SHALL provide:

- consistent coordinate systems;
- traceable spatial measurements;
- deterministic transformations;
- independence from robotics middleware;
- engineering-level interoperability.

---

# 3. Fundamental Principle

Every spatial value SHALL explicitly declare its Reference Frame.

Spatial values without Reference Frame are invalid for engineering decisions.

---

# 4. Reference Frame

A Reference Frame defines the coordinate system in which a spatial value is expressed.

Every Reference Frame SHALL have:

- Frame ID
- Parent Frame (optional)
- Name
- Description

Reference Frames form a hierarchy.

---

# 5. Canonical Frame Hierarchy

Typical hierarchy:

Site Frame

↓

Building Frame

↓

Level Frame

↓

Zone Frame

↓

Robot Base Frame

↓

Sensor Frame

↓

Optical Frame

Projects MAY extend this hierarchy.

---

# 6. Spatial Value

Every spatial value SHALL contain:

- Value
- Unit
- Reference Frame
- Timestamp
- Confidence

Optional:

- Quality
- Evidence

---

# 7. Transform

A Transform defines the relationship between two Reference Frames.

Every Transform SHALL include:

- Parent Frame
- Child Frame
- Translation
- Rotation
- Timestamp
- Confidence

Transforms SHALL be deterministic.

---

# 8. Spatial Registry

Spatial Registry manages:

- Reference Frames
- Frame hierarchy
- Transforms
- Calibration metadata

Spatial Registry SHALL be the authoritative source of spatial relationships.

---

# 9. Geometry

Geometry represents measurable properties of physical objects.

Typical geometry includes:

- Position
- Orientation
- Dimensions
- Surface
- Plane
- Volume
- Bounding Geometry

---

# 10. Semantic Layer

Geometry describes where an object is.

The Semantic Layer describes what the object means.

Examples:

Building

Level

Zone

Wall

Task Area

Safety Zone

The Semantic Layer SHALL reference geometric entities rather than duplicate geometry.

---

# 11. Coordinate Independence

The Spatial Model SHALL remain independent of:

- ROS2 tf2
- OpenCV
- Open3D
- CUDA
- Any robotics framework

Adapters MAY synchronize external coordinate systems with the SIE Spatial Model.

---

# 12. Spatial Ownership

Ownership:

Vision Core
→ image-space measurements

Geometry Engine
→ geometric measurements

Spatial Registry
→ coordinate systems

World State Manager
→ validated spatial facts

Decision Engine
→ spatial reasoning

---

# 13. Compliance

Every spatial object exchanged inside SIE SHALL comply with this specification.

Spatial values lacking Reference Frame are non-compliant.

---

# 14. References

- ES-001 SIE Architecture Constitution
- ES-002 SIE Rules
- ES-004 SIE Data Model
- ES-006 SIE Evidence Graph

---

End of Specification