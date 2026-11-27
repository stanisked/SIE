# SIE Three-Sensor Spatial Association v1 | Handoff 2026-11-26

**DECISION** SIE's objective is a unified evidence-backed system spanning
AR0234 RGB semantics, OV9281 stereo depth, LD19 LiDAR, World State and robot
control. The spatial association stage began with diagnostic work. Its entry
baseline is `TEMPORAL_ASSOCIATION_DIAGNOSTIC_COMPLETE` on
`codex/siee-live-temporal-association-shadow-v1-20261125`. Start with the
[Step 57 freeze](SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md#step-57-diagnostic-baseline-freeze-2026-11-26)
and Git baseline `23dbfdb4a60fbdca8eec5480d9d19751a7e2c317`.
World State, Measurement and motor admission remain disabled.

The handoff path and linked evidence retain their historical identifiers.
[Canonical naming decision](../../PROJECT_CONTEXT.md#project-name-and-compatibility)

**DECISION — architecture scope.** The [Fundamental Architecture Principles](../../PROJECT_CONTEXT.md#fundamental-architecture-principles) govern the use of complementary evidence from distinct surfaces. Step 70 and Step 71 retain their original diagnostic scope and evidence limits.

For distinguishing candidate associations from established facts and determining when further information is needed, see the [Knowledge Architecture Principles](../../PROJECT_CONTEXT.md#knowledge-architecture-principles).

Use the [Responsible Autonomy and Decision Governance Principles](../../PROJECT_CONTEXT.md#responsible-autonomy-and-decision-governance-principles) when reviewing any future proposal to use this diagnostic evidence in a decision or execution path.

The [Mission and Practical Intelligence Principles](../../PROJECT_CONTEXT.md#mission-and-practical-intelligence-principles) require verified executor capabilities and evidence of physical outcomes for future task execution.

The [Fundamental Architecture Baseline](../../PROJECT_CONTEXT.md#fundamental-architecture-baseline) indexes the owner-approved principles. For continuation, start with the [Step 72 replay report](#step-72--operator-reported-three-sensor-replay) and the [Step 73 proposal](../../PROJECT_CONTEXT.md#step-73--entity-centric-knowledge-integration-for-mvp).

## Source and frame inventory

- **FACT** The camera audit node projects AR0234 rays into physical-left and
  rectified-left stereo geometry using the stored RGB-to-left extrinsic,
  stereo rectification and projection. The audit includes stereo and
  RGB-to-physical-left extrinsic hashes. Start from the [audit node](../../ros2/sie_ros2/sie_ros2/ar0234_ov9281_dynamic_association_audit_node.py),
  [activation documentation](../calibration/ar0234_ov9281_decision_chain_activation_v1.md)
  and [frozen-run launcher](../../vision_core/tools/run_live_temporal_shadow_v1.sh)
  to trace the exact artifacts, direction, units and validity scope.
- **FACT** The [ROS launch file](../../ros2/sie_ros2/launch/sie_pipeline.launch.py)
  declares LD19 frame `ld19_sensor` and configures a provisional static
  `base_link` → `ld19_sensor` TF with translation `(-0.023, 0, 0.212) m`
  and zero roll, pitch and yaw. **OPEN** Its physical fit, axis alignment,
  measurement uncertainty and relation to the camera reference frame require
  verification. Camera-to-LiDAR extrinsic calibration remains unverified.
- **DECISION** Use the [Spatial Model](../architecture/SIE_SPATIAL_MODEL.md)
  and [Evidence Graph](../architecture/SIE_EVIDENCE_GRAPH.md) to define the
  candidate common frame, transform authority, units, timestamp, confidence
  and evidence linkage. Keep the distinction between physical-left,
  rectified-left, LD19 and base frames explicit.

## First deliverable and evaluation questions

**DECISION** Begin with a read-only frame and calibration inventory. Trace the
active AR0234/OV9281 calibration hashes and transforms, document LD19 frame
and mounting geometry with their evidence, and identify every unverified
edge in the common-frame chain. A provisional static TF supplies a candidate
transform to investigate.

**OPEN** After that inventory, design a diagnostic comparison using RGB masks,
stereo depth surfaces and LD19 scan points with known geometry. Reuse the
independent temporal links and source identities from the frozen journal.
Preserve the RGB receipt cutoff, current `latest_pair` selection and separate
source/refusal fields. The metadata journal identifies timing inputs and
contains no image, disparity or laser range payloads. A spatial evaluation
requires separately traceable payloads and ground truth, with no collection
authorized by this handoff.

**HYPOTHESIS** When the transforms, timestamps and visibility are applicable,
some laser returns may correspond to the same surface as the stereo region
selected by an RGB semantic mask. Test valid overlap, occlusion, competing
surfaces, absent returns, stale timing and extrinsic sensitivity. Report
ambiguity, residuals and uncertainty explicitly. A time match establishes no
spatial correspondence, and the physical LD19 acquisition time is `UNKNOWN`.

**DECISION** Preserve current camera calibration, segmentation, tracking,
stereo processing and control-preview behavior. Keep diagnostic IDs local
until provenance and linkage are established through the approved Evidence
Graph. Any calibration promotion, canonical Data Contract change, World
State update or actuator authorization requires separate review. The ESP32
v13 brake FAULT remains a separate blocker. This handoff adds no Spatial
Association implementation.

## Step 60 | Mechanical extrinsic v1

**FACT** The 2026-11-26 operator geometry is recorded in the
[Step 60 profile and transform notes](../diagnostics/camera_lidar_mechanical_geometry_v1.md).
The pure transform terminates at the physical-left OV9281 lens rim and
refuses an optical-center transform. No physical validation was run.

**OPEN** Independent common-surface validation, the optical-center offset
and alignment uncertainties remain open. The Step 60 notes contain the
proposed static validation design. Diagnostic admission boundaries still apply.

## Steps 65–66 | Horizontal closure and joint static validation

**DECISION** The project owner closed the horizontal mechanical stage as
`MECHANICAL_HORIZONTAL_GEOMETRY_DIAGNOSTIC_COMPLETE`. Keep the Step 60
mechanical geometry and retain the original T02 result and anomaly count.

**FACT** The [Step 66 notes](../diagnostics/stereo_lidar_joint_static_v1.md)
record operator-reported T01–T05 evidence and the standalone Pi procedure.
The separate optical datum is explicitly estimated. The analysis reports
infinite-plane agreement separately from finite-surface candidates and
preserves missing or ambiguous returns.

**OPEN** No Step 66 physical experiment has run. T02 background contamination
remains a hypothesis. Diagnostic timing and spatial evidence do not admit
an extrinsic, Measurement, World State update or motor control.

## Steps 67–69 | Global refusals and local replay

**FACT / operator report** Uniform target Step 67 recorded 30 stereo frames
and 80 scans, with 25 temporal matches, 5 temporal refusals, 24 planarity
refusals and one grazing-plane refusal. Textured target Step 68 recorded 30
frames and 79 scans, with 17 temporal matches, 13 temporal refusals and 17
planarity refusals. Neither produced a Step 66 spatial comparison. Frame 15
in Step 67 and frame 12 in Step 68 had 69.3016% and 86.2188% ROI support and
global plane P95 of 90.917 and 84.311 mm, respectively. The polygons differed.
These two frames do not describe all frames in either run.

**DECISION** The project owner approved
`LOCAL_STEREO_LIDAR_SPATIAL_CORRESPONDENCE` for separately judged local
stereo surfaces, including non-planar targets. Keep the existing global
method and its 40 mm P95 gate, the Step 66 timing and reconstruction, and
the estimated +13.3022 mm optical datum. Use the frozen local trial policy
and distinct replay outputs. [Policy, output and exact Pi replay commands](../diagnostics/stereo_lidar_local_correspondence_v1.md).

**OPEN** `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE` for the Step 67 and Step 68
archives. No real-data replay or new acquisition occurred in Step 69.
Projected coincidence and measured residuals do not establish target
membership or authorize Measurement, World State, Decisions or motion.

## Step 70 | Local stereo–LiDAR freeze

**DECISION** On 2026-11-26 the owner approved
`STEREO_LIDAR_LOCAL_SPATIAL_CORRESPONDENCE_DIAGNOSTIC_COMPLETE`.
[Step 70 metrics and evidence index](../diagnostics/stereo_lidar_local_correspondence_v1.md#step-70-freeze-2026-11-26)
preserves both sessions and original hashes. Step 71 adds current AR0234
semantic target ownership and track binding while retaining independent
stereo and LD19 observations. The actuator fault remains a separate workstream.

**FACT / operator report** All 42 matched frames across the two runs had
local candidates, with 377 candidates and 39 ambiguous beams overall.
The six Pi artifacts are `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`.
No physical membership or combined accuracy is established.

**FACT** GitHub PR #13 was checked as open draft, head
`bdb29621d698d8e3192ef5fc6c0eeff59f6e2c97` and base
`codex/siee-person-follow-mvp-20261120`. Its head and the Step 57 freeze
are ancestors of the Step 70 starting HEAD. The verified local Graphify
snapshot is an ancestor at `db8097ea4239dcf45ef7598c831a45648d2498fd`.
It covers the earlier preview, instance-mask and torso-core code. It
contains no Step 57 live temporal or Step 69 local Stereo–LiDAR modules.
Current sources govern those dependencies. No graph extraction was run.

## Step 71 | Three-sensor semantic spatial diagnostic

**FACT** The diagnostic replays bounded source snapshots from the existing
Shadow producer and reads the same Step 57 LD19 callback chosen by the
causal comparison. It reconstructs full-resolution v7 stereo offline,
projects through the calibrated AR0234 inverse transform and requires
current track, mask, and unique visible stereo-component evidence before
using the unchanged local Stereo–LiDAR policy.

**DECISION** Preserve both RGB temporal links and independently screen the
actual Stereo–LD19 header delta with the existing Step 66 50 ms trial.
Keep ambiguous stereo support and untrimmed residuals in the diagnostic
record. The [Step 71 guide](../diagnostics/three_sensor_semantic_spatial_v1.md)
contains provenance, the scoped contract and exact Pi commands.

**FACT / Step 71 implementation checkpoint.** The capture and replay path
was software-verified with synthetic inputs. Physical-run evidence, target
membership verification, and measured Pi overhead were unavailable at that
checkpoint. The Step 70 freeze, diagnostic admission boundaries, and separate
ESP32 v13 fault still apply.

## Step 72 — Operator-reported three-sensor replay

**FACT / operator report, 2026-11-26.** The owner reported these experimental
diagnostic results from the Step 71 capture and replay path.

| Metric | Reported value |
| --- | ---: |
| Journal cycles | 54 |
| Working cycles | 53 |
| Coherent RGB–Stereo masked-depth diagnostic cycles | 53/53 |
| Completed source snapshots | 18/18 |
| Snapshots with three-source timing diagnostic acceptance | 15/18 |
| Coherent three-sensor Surface-Level candidates | 0 |
| Beam instances projected outside the selected RGB–Stereo component | 944 |
| Journal write or evaluation errors | 0 reported |

The operator reported the replay evidence at
`/home/elwis/.local/state/sie/evidence/live_temporal_shadow_iWIjW8zB_step71_20261126T171308Z_1333`.

| Artifact | Operator-reported SHA-256 |
| --- | --- |
| `summary.json` | `73a8b0cdd801020c50778df93d38b56bf9f693e6e342e29d8df7ba08bd539b61` |
| `cycles.jsonl` | `08bb65b49454c57131682faf77b8f6ada16e510f7fbe9c84ef7428c60d917655` |
| `policy.json` | `164cfb4c452d33f14f30aca55b42cf68e6a5c23d580d7d159074322f4f76115b` |

**OPEN.** The primary artifacts are `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`.
The counts and hashes retain their operator-reported status. The zero
Surface-Level candidates describe the result under the selected component
and diagnostic correspondence policy. A claim of sensor failure would require
separate evidence. Physical membership of LD19 reflections in the observed
person remains unverified. Entity-Level Association can use complementary
observations of different physical surfaces when evidence supports common
identity. The diagnostic admission boundaries remain in force.

**PROPOSAL.** Begin Step 73 with these results and independent LD19 and camera
observations. Establish the minimum evidence integration needed for practical
entity representation and subsequent safe navigation design.
[Step 73 — Entity-Centric Knowledge Integration for MVP](../../PROJECT_CONTEXT.md#step-73--entity-centric-knowledge-integration-for-mvp)

**OPEN / separate actuator blocker.** ESP32 v13 still has the unresolved brake
fault documented in the [v13 handoff](SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md).

## Step 73.3 — Entity-Centric Diagnostic Preview v1

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Preview complementary sensor
evidence offline under `entity-centric-preview-v1-TRIAL`. Keep LD19 obstacle
evidence independent of the AR0234 and OV9281 candidate. Surface-Level
correspondence is optional context.

**FACT.** The preview uses the Step 57 timing journal and Step 71 camera
snapshots, plus existing camera audit or LD19 diagnostic records where
available. One current RGB observation and its unique stereo component can
support a provisional entity. LD19 membership remains unlinked. Existing
runtime, calibration, and admission paths remain unchanged.
[Design and exact Pi replay commands](../diagnostics/entity_centric_diagnostic_preview_v1.md)

**FACT / software verification.** 211 targeted tests and five subtests
passed, including 17 preview tests. The synthetic replay produced a camera
entity with zero Surface-Level candidates and retained independent LD19
observations. Diagnostic rows and summaries were byte-identical on replay.

**FACT / operator report.** The operator verified the three recorded Step 72
replay hashes on the Pi in Step 73.1 and identified
`step73_entity_integration_evidence_20261127.tar.gz`.

**OPEN.** The archive was unavailable locally, and the documented Pi SSH
endpoint rejected authentication. No new physical run or primary-data
replay was performed. The Graphify report and graph hash were checked
against the 2026-11-16 `db8097ea` snapshot. That snapshot predates Step 71
and Step 73.3, so current sources governed implementation. The ESP32 v13
braking fault remains a separate actuator blocker.

## Step 73.4 — Offline entity integration complete

**DECISION — `OWNER_APPROVED`, 2026-11-27.**
`ENTITY_CENTRIC_OFFLINE_INTEGRATION_COMPLETE`. Continue to the opt-in live
diagnostic in Step 74.

**FACT / operator report.** The Raspberry Pi 5 deployment HEAD was
`1862677368308b2d4d2a2de72a0d817bdb8dfdd5`. Git preflight and all three
Step 72 replay evidence hash checks passed.

| Metric | Reported value |
| --- | ---: |
| Source cycles | 54 |
| Diagnostic rows | 54 |
| Camera snapshots | 18 |
| Current semantic candidates | 53 |
| Spatial evidence | 17 |
| Provisional entity candidates | 17 |
| Independent LD19 obstacle observations | 52 |
| Unique LD19 scans | 52 |
| Valid LD19 return instances | 22290 |
| LD19 entity associations | 0 |
| `ENTITY_MEMBERSHIP_UNRESOLVED` | 17 |
| Association unavailable | 37 |
| Surface-Level `NO_CANDIDATES` | 15 |
| Surface-Level `REJECTED` | 39 |
| Spatial snapshot unavailable | 36 |
| Actual camera pair not bound at zero wait | 1 |
| Standalone LD19 records | 0 |
| Offline wall time | 36902156068 ns |

The 36 unavailable spatial rows reflect bounded recording coverage. The
17 provisional candidates are per-cycle results with physical identity
unresolved. The offline wall time includes input reads, reconstruction,
and output writes. Live latency remains unmeasured.

The operator reported the evidence directory
`/home/elwis/.local/state/sie/evidence/live_temporal_shadow_iWIjW8zB_step73_20261127T082006Z_1318`
and these SHA-256 digests.

| Artifact | Operator-reported SHA-256 |
| --- | --- |
| `summary.json` | `0524a558609c5d5192dbbdfe6df801165979dc06e37a3d57c7cd3babdd22b405` |
| `cycles.jsonl` | `e809651628e155e9b4c54ed1f49fef089c278f8cc01d28ad683b97695803b166` |
| `processing_cost.json` | `70d0b213e41267dfd80d0efc4f4dc68c682aa3524c78ad9f367c26cc97805a76` |
| Source temporal journal | `deee8832f29c5c8761c8811a3b9042430c397a338355b752881a99cd1b3b9eff` |

The recorded policy was `entity-centric-preview-v1-TRIAL`. The operator
reported all admission and authorization flags false. Physical laser
acquisition time remains `UNKNOWN`, and physical camera exposure timing
is independently unverified. These results are operator-reported. LD19
membership, live performance, independent safety, and autonomous
navigation remain unvalidated. The ESP32 v13 braking fault is separate.
