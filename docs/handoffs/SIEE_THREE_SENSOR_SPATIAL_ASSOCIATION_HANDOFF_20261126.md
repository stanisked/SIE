# SIEE Three-Sensor Spatial Association v1 | Handoff 2026-11-26

**DECISION** SIEE's objective is a unified evidence-backed system spanning
AR0234 RGB semantics, OV9281 stereo depth, LD19 LiDAR, World State and robot
control. The next stage is diagnostic spatial association. Its entry baseline
is `TEMPORAL_ASSOCIATION_DIAGNOSTIC_COMPLETE` on
`codex/siee-live-temporal-association-shadow-v1-20261125`. Start with the
[Step 57 freeze](SIEE_TEMPORAL_ASSOCIATION_HANDOFF_20261125.md#step-57-diagnostic-baseline-freeze-2026-11-26)
and Git baseline `23dbfdb4a60fbdca8eec5480d9d19751a7e2c317`.
World State, Measurement and motor admission remain disabled.

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
