# SIE Three-Sensor Spatial Association v1 | Handoff 2026-11-26

**DECISION** SIE's objective is a unified evidence-backed system spanning
AR0234 RGB semantics, OV9281 stereo depth, LD19 LiDAR, World State and robot
control. The next stage is diagnostic spatial association. Its entry baseline
is `TEMPORAL_ASSOCIATION_DIAGNOSTIC_COMPLETE` on
`codex/siee-live-temporal-association-shadow-v1-20261125`. Start with the
[Step 57 freeze](SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md#step-57-diagnostic-baseline-freeze-2026-11-26)
and Git baseline `23dbfdb4a60fbdca8eec5480d9d19751a7e2c317`.
World State, Measurement and motor admission remain disabled.

The handoff path and linked evidence retain their historical identifiers.
[Canonical naming decision](../../PROJECT_CONTEXT.md#project-name-and-compatibility)

**DECISION — architecture scope.** The [Fundamental Architecture Principles](../../PROJECT_CONTEXT.md#fundamental-architecture-principles) govern the use of complementary evidence from distinct surfaces. Step 70 and Step 71 retain their original diagnostic scope and evidence limits.

For distinguishing candidate associations from established facts and determining when further information is needed, see the [Knowledge Architecture Principles](../../PROJECT_CONTEXT.md#knowledge-architecture-principles).

Use the [Responsible Autonomy and Decision Governance Principles](../../PROJECT_CONTEXT.md#responsible-autonomy-and-decision-governance-principles) when reviewing any future proposal to use this diagnostic evidence in a decision or execution path.

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

**OPEN** The capture and replay path is software-verified with synthetic
inputs. No physical run, target-membership verification or measured Pi
overhead is available. The Step 70 freeze, diagnostic admission boundaries
and separate ESP32 v13 fault still apply.
