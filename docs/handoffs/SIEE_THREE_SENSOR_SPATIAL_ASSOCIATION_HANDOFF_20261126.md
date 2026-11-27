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

## Step 74 — Live entity-centric integration v1

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Use a separate opt-in diagnostic
companion for the existing perception pipeline. Retain two independent
latest-wins paths for camera entities and LD19 obstacles.

**FACT.** The companion consumes the live camera audit and LD19 LaserScan.
It preserves the live spatial estimate's method and datum, with no
additional camera capture, inference, tracker, stereo reconstruction,
or cross-sensor wait. Camera target loss leaves LD19 observations
independent. The isolated String topics have disabled admission and
execution flags. [Interface, evidence, and exact Pi commands](../diagnostics/live_entity_centric_integration_v1.md)

**FACT / software verification.** 247 targeted tests and seven subtests
passed, including 16 Step 74 cases. The localhost ROS 2 Lyrical smoke test
used only generated data. Both modes received 20 camera audits and 60
scans. Baseline advertised neither diagnostic output. Enabled published
20 camera entities and 60 independent obstacle updates with zero pending
replacements. The November 16 Graphify snapshot at `db8097ea` predates
these paths, so current source files governed implementation.

## Steps 74.2 and 74.3 — Static Pi diagnostics

**FACT / operator report, 2026-11-27.** The original 250 ms companion
screen classified all 115 working camera observations `STALE`. Both
trials retained coherent masked depth throughout their working camera
cycles. The operator reported maximum driver-declared ages below 500 ms.

| Metric | Step 74.2 Static Baseline | Step 74.3 Static Enabled |
| --- | ---: | ---: |
| Working camera cycles | 49 | 66 |
| Coherent masked depth | 49/49 | 66/66 |
| RGB producer Hz | 2.993832 | 3.271951 |
| RGB–Stereo spatial producer Hz | 2.993817 | 3.271847 |
| LD19 callbacks | 309 | 307 |
| LD19 callback Hz | 9.903604 | — |
| LD19 obstacle updates | — | 307 |
| LD19 obstacle update Hz | — | 9.871155 |
| Camera observations classified `STALE` | 49/49 | 66/66 |
| Camera diagnostic publications | — | 66 |
| Provisional entity candidates | — | 0 |
| Mean camera callback-to-preview, approx. ms | 1.003 | 1.108 |
| Mean LD19 callback-to-preview, approx. ms | 0.714 | 4.443 |
| Maximum LD19 callback-to-preview, ms | — | 51.128 |
| Pending replacements | 0 | 0 |
| Processing and publication errors | 0 | 0 |

The operator reported the baseline directory
`/home/elwis/.local/state/sie/evidence/live_entity_baseline_vmzor4IV`
and enabled directory
`/home/elwis/.local/state/sie/evidence/live_entity_enabled_OG20l4Ps`.

| Artifact | Baseline reported SHA-256 | Enabled reported SHA-256 |
| --- | --- | --- |
| `preview/summary.json` | `cb1b0d99dd77374159d374e43630658b446acb5e9d9dc904249defbab1f13373` | `d1e3727824aa13f569135d12a6d91fcd0ba624c485de22a499ed14170ce05338` |
| `preview/camera/events.jsonl` | `da24f4ba8e7b561a3eb2fd1f2f31d36971b2128f3e0f7c3938f018b508412d7d` | `939cec6d8f38bfdf6d19c4be03689a0f8ef8b6fde1f2100c8d71874bc97f4362` |
| `preview/ld19/events.jsonl` | `9063b0067af5388c8947f5cb44b2acac52a7954b5ebe544f63b8eb569dbe82b5` | `68624f9e60d9f0ecaeda740faaec1ec3ce29339ede8fe7f7229e7647525eb121` |

**FACT / interpretation of operator report.** Step 74.3 partially passed.
Independent live LD19 diagnostic processing passed on the Pi. Camera
freshness blocked provisional entities. No camera throughput degradation
was observed in this single comparison. A throughput improvement and
physical camera exposure and laser acquisition timing remain unverified.
The underlying artifacts were unavailable locally for independent review.

## Step 74.3a — Diagnostic camera freshness policy v2

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Apply the inclusive 500 ms
`live-entity-camera-freshness-v2-500ms-TRIAL` to AR0234 and OV9281 in the
opt-in companion's baseline and enabled modes. Retain the original 250 ms
camera age reference explicitly in both modes. Preserve the Step 57
policy, 25 ms RGB–Stereo skew, 250 ms LD19 local receipt screen, and all
existing admission, authorization, and safety boundaries. This is a
diagnostic trial policy only.

**FACT / software verification.** 295 targeted tests and seven subtests
passed. The companion reports the applied policy and reference dispositions
in both modes. The 500 ms boundary, stale and future data, held and reused
observations, and the unchanged Step 57 and LD19 limits were exercised.

**OPEN.** Step 74 remains open pending the 30-second Static Enabled retest
with v2. The ESP32 v13 braking fault remains a separate actuator blocker.
[Pinned-worktree deployment and validation commands](../diagnostics/live_entity_centric_integration_v1.md#raspberry-pi-deployment-and-validation-commands)

## Step 74.5 — Live entity-centric diagnostic integration complete

**DECISION — `OWNER_APPROVED`, 2026-11-27.**
`LIVE_ENTITY_CENTRIC_DIAGNOSTIC_INTEGRATION_COMPLETE`. The owner closed Step 74
after successful static and moving diagnostic trials and opened Step 75, ROS 2
Spatial Navigation Integration. Preserve the demonstrated perception baseline.
Repeat perception or calibration work only for newly identified defect evidence.

**FACT / operator report.** Raspberry Pi 5 implementation
`35e103d700a8d764241f08c9eb770dd51b8983dc`, with camera diagnostic policy
`live-entity-camera-freshness-v2-500ms-TRIAL`.

| Metric | Static Enabled V2 | Moving Enabled V2 |
| --- | ---: | ---: |
| Working camera cycles | 61 | 68 |
| Current semantic observations | 61/61 | 68/68 |
| Available spatial observations | 61/61 | 68/68 |
| Provisional entity updates | 61 | 68 |
| Actual compatible RGB–Stereo pairs | 61/61 | 68/68 |
| Unique producer track IDs | 1 | 1 |
| Missing track IDs | 0 | 0 |
| Track changes | 0 | 0 |
| Stable Torso Core method | 61/61 | 68/68 |
| Coherent masked depth | 56 | 52 |
| `DEPTH_MAD_EXCEEDED` | 5 | 16 |
| Independent LD19 obstacle updates | 302 | 315 |
| RGB producer Hz | 3.030629 | 3.389866 |
| Entity preview Hz | 3.040273 | 3.398196 |
| LD19 obstacle update Hz | 9.901484 | 9.843111 |
| Mean camera callback-to-preview, approx. ms | 1.103 | 1.180 |
| Mean LD19 callback-to-preview, approx. ms | 5.149 | 4.509 |
| Pending replacements | 0 | 0 |
| Processing and publishing errors | 0 | 0 |

The static track was `ar0234-audit-track:cdf3ad8b49bd10a6:1` and the moving
track was `ar0234-audit-track:f3f5e4498816cc76:1`. Those IDs are session-local.
The moving run reported a masked-depth candidate range of approximately
1.534–3.395 m, including values from `DEPTH_MAD_EXCEEDED` cycles. Physical
trajectory and Metric Measurement remain unverified.

The operator reported the static directory
`/home/elwis/.local/state/sie/evidence/live_entity_enabled_e3MiJTtG`
and moving directory
`/home/elwis/.local/state/sie/evidence/live_entity_enabled_yilFcNEC`.

| Artifact | Static reported SHA-256 | Moving reported SHA-256 |
| --- | --- | --- |
| `preview/summary.json` | `f22f245e0db6a8407eaabc2d539730e61938c38d02f9873e52344bdafd282825` | `2615681b9ed48d89ace763d66f7baf47f1928d2adb7d89682bf91c170c457bb4` |
| `preview/camera/events.jsonl` | `1fd4c4a037260f30a2fa5e0702830ee89cf82f71c53c340a8b0add69432c5215` | `7d791b5a000bdc31fd95704661b949796149c4dd1042c92304ebb084a9ac9d4a` |
| `preview/ld19/events.jsonl` | `c2a879050dc4e01468e477ef00f2bc31cbf5ffe5ac435afe6dedace62f3e1142` | `39fe9bf161ed71f2851eaec41e0c93869242b237d923ae02cc53af47bd542f4b` |

**FACT / operator checksum verification.** All six reported checksum
entries per trial passed, including the original temporal association
journal. The primary files were unavailable for independent inspection
during this audit.

All 129 available spatial observations used
`STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY`. Whole-mask depth was coherent
in 108 and failed its MAD criterion in 21. The independently evaluated
torso-core result remains a partial longitudinal known term. Its available
fields do not establish a complete front range or 3D pose.

Within the reported conditions, the trials demonstrated concurrent camera
and independent LD19 processing, continuous session-local target tracking,
source-bound spatial evidence, independent obstacle geometry, and bounded
asynchronous diagnostic publication on Pi 5. The original 250 ms camera
reference remained unsatisfied. Physical acquisition synchronization, LD19
entity membership, complete target pose, navigation safety, and motion
authorization remain unverified. All admission and execution authorization
flags remain false. The ESP32 v13 braking fault remains a separate blocker.
