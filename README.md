# SIE — Spatial Intelligence Engine
Engineering architecture for robotic perception, spatial reasoning and decision support.
SIE is a systems-engineering project for transforming sensor observations into calibrated, traceable and physically meaningful information that robotic systems can use for reasoning and action.

## Vision

Robots should reason about the physical world using measurable, verifiable and traceable information.

SIE is an engineering architecture that transforms raw sensor observations into reliable spatial understanding for autonomous robotic systems.

Instead of relying on model-specific assumptions, SIE builds decisions upon calibrated measurements, geometry and explicit evidence.


## Why SIE

Modern robotic software often treats AI models as the primary source of truth.
SIE follows a different philosophy:

- Measurements are trusted only after physical validation
- Every decision is traceable
- Every spatial value has an explicit reference frame
- Every conclusion can be verified


## Engineering Principles

- Geometry First — spatial reasoning begins with geometry and physical constraints.
- Measured Reality over Predictions — predictions do not replace validated measurements.
- Evidence over Opinion — engineering conclusions must be supported by evidence.
- Reference Frames Everywhere — spatial values without a reference_frame are invalid.
- Replaceable AI Components — models may be upgraded without redefining the system architecture.
- Data Contracts Between Modules — modules exchange structured data objects rather than untyped internal state.
- Engineering Validation First — improvements must be demonstrated through reproducible physical validation.


## System Architecture

Calibration does not produce an observation. Sensors and information sources produce Observations; the Calibration Registry determines whether those observations are suitable for physical measurement and how their geometry must be interpreted.

```mermaid
flowchart TD
    SENSORS["Sensors and Information Sources"]
    OBS["Observations"]
    CAL["Calibration Layer"]
    PERCEPTION["Perception Components"]
    MEAS["Measurements"]
    EVIDENCE["Evidence Graph"]
    WORLD["World State"]
    TEMPORAL["Temporal Perception"]
    TASK["Task Evaluator"]
    DECISION["Decision Engine"]
    ACTION["Actions"]
    LOG["Deterministic Cycle Log"]

    SENSORS --> OBS
    OBS --> PERCEPTION
    CAL -. "geometry and applicability" .-> PERCEPTION
    PERCEPTION --> MEAS
    CAL -. "calibration provenance" .-> MEAS

    MEAS --> EVIDENCE
    MEAS --> WORLD
    EVIDENCE --> WORLD

    WORLD --> TEMPORAL
    TEMPORAL --> WORLD

    WORLD --> TASK
    TASK --> DECISION
    WORLD --> DECISION

    DECISION --> ACTION

    OBS --> LOG
    MEAS --> LOG
    WORLD --> LOG
    TASK --> LOG
    DECISION --> LOG
    ACTION --> LOG
```
### Data Flow

1. Sensors and information sources produce structured Observations.
2. The Calibration Registry supplies sensor geometry and verifies calibration applicability.
3. Perception components use observations and applicable calibration to produce quantitative Measurements.
4. Measurements are expressed in physical units and linked to a reference_frame and calibration provenance.
5. The Evidence Graph preserves provenance from observation to measurement, decision and action.
6. The World State maintains the current structured representation of the physical environment.
7. Temporal Perception evaluates change, stability and entity lifecycle over time.
8. The Task Evaluator assesses the World State against the active task.
9. The Decision Engine uses only the World State and Task Evaluator output to select an action.
10. The complete processing cycle is recorded in a deterministic log.

### Architectural Constraints

- Modules exchange structured Data Contracts rather than untyped internal data.
- Spatial values without an explicit `reference_frame` are invalid.
- Measurements produced under unknown, expired or unvalidated calibration are rejected.
- AI models are replaceable components and are not treated as authoritative sources of truth.
- Every engineering fact, decision and action must remain traceable to supporting evidence.
- Insufficient confidence must trigger additional observation, measurement or safe refusal.


## Current Status

SIE is currently in the engineering validation stage.

The active development focus is the Vision Core and calibration infrastructure required to produce physically meaningful spatial measurements.

### Baseline Completed

- Stereo camera calibration pipeline
- Correct physical left/right camera mapping
- Rectified stereo input pipeline
- ROI-based depth measurement
- Median-based distance estimation
- Temporal depth stabilization
- Confidence and measurement quality evaluation
- Cross-range stereo validation
- Pixel-to-3D conversion geometry utilities
- Plane fitting experiments
- baseline depth-measurement validation rules
- Versioned calibration concept and calibration applicability checks

### Active Development

- Stereo calibration v3 validation
- Calibration Registry
- Structured `Observation` and `Measurement` contracts
- Measurement uncertainty propagation
- Target-specific ROI validation
- Point cloud quality evaluation
- Object dimension measurement
- World State representation

### Planned

- Temporal Perception
- Entity lifecycle tracking
- Multi-sensor fusion
- IMU integration
- Visual-Inertial Odometry
- SLAM
- Task Evaluator
- Decision Engine
- Action validation and execution contracts


## Validation Results

The published Vision Core baseline has been validated against physical reference measurements using a calibrated stereo camera system.

Calibration status: calibration v3 is currently under independent validation and is not yet used as the published engineering baseline.

### Stereo Calibration Baseline v2

| Metric | Value |
|---------|------:|
| Stereo RMS | **0.479 px** |
| Physical baseline | **65.10 mm** |
| Solved baseline | **64.305 mm** |
| Rectification median vertical error | **0.322 px** |

### Depth Measurement Validation

| Ground Truth | Measured | Absolute Error | Relative Error |
|--------------|---------:|---------------:|---------------:|
| 500 mm | 508.7 mm | +8.7 mm | 1.74 % |
| 1000 mm | 1009.2 mm | +9.2 mm | 0.92 % |

### Validation Methodology

Validation was performed using:

- physically measured reference distances;
- frozen calibration parameters;
- ROI-based median disparity estimation;
- repeated measurements;
- engineering acceptance criteria;
- cross-range validation.

### Engineering Notes

Current validation focuses on:

- depth measurement accuracy;
- measurement repeatability;
- calibration consistency;
- geometry correctness.

Validation of object dimensions, point cloud accuracy and uncertainty propagation is currently in progress.


## Repository Structure

```text
SIE/
├── configs/              # Runtime and component configuration
├── decision_engine/      # Early decision-engine scaffolding and experiments
├── docs/                 # Architecture and engineering documentation
├── examples/             # Example applications
├── geometry_engine/      # Geometry processing and 3D algorithms
├── knowledge_engine/     # Domain knowledge and structured reasoning support
├── sie_core/             # Shared contracts, identifiers, reference frames and core data models
├── tests/                # Automated and validation tests
├── tools/                # Capture, calibration and diagnostic utilities
├── vision_core/          # Stereo vision and perception pipeline
└── README.md
```
The architecture will be split further as concrete implementations of the Evidence Graph, World State, Temporal Perception and related components are introduced.


## Hardware Platform

Current development hardware

### Computing

- Raspberry Pi 5
- Ubuntu 24.04
- ROS2 Jazzy

### Sensors

- OV9281 global shutter stereo camera
- Planned IMU integration
- Planned ToF sensor integration

### Robotics

- LeRobot SO-101
- ESP32 embedded controllers

### Development Tools

- OpenCV
- Python
- C++
- Docker
- Git


## Reproducibility

Current validation environment

| Component | Configuration |
|-----------|---------------|
| Operating System | Ubuntu 24.04 |
| Stereo Resolution | 2560×800 |
| Frame Rate | 60 FPS |
| Pixel Format | MJPG |
| Camera Exposure | Auto Exposure = 3 |
| Stereo Baseline | 64.305 mm |
| Calibration | v2, frozen externally |
| Calibration Artifact | Not stored in this repository; artifact-hash publication is pending |
| Validation | Physical ruler measurements |


## Roadmap

| Component | Status |
|-----------|--------|
| Calibration Framework Baseline| ✅ Completed |
| Stereo Depth Baseline| ✅ Completed |
| Vision Core | 🚧 In Progress |
| Depth Measurement Validation | ✅ Completed |
| Geometry Engine | 🚧 In Progress |
| Calibration Registry | 🚧 In Progress |
| World State | 🚧 In Progress |
| Temporal Perception | 📋 Planned |
| Sensor Fusion | 📋 Planned |
| Decision Engine | 📋 Planned |


## Project Maturity
SIE is an active research and engineering project. Public interfaces, repository layout and internal contracts may change while the architecture is being validated.

Published metrics describe specific frozen baselines and should not be interpreted as general performance guarantees for all scenes, distances, targets or operating conditions.


## License

Copyright © 2026 SIE contributors. All rights reserved.
No open-source or commercial license is currently granted. See LICENSE for the authoritative terms.
