# SIEE Step 69 | Local Stereo–LiDAR Spatial Correspondence v1

**DECISION** The project owner approved
`LOCAL_STEREO_LIDAR_SPATIAL_CORRESPONDENCE` as an additive offline diagnostic
for the MVP. The existing Step 66 global point-to-plane method remains a
separate diagnostic with its 40 mm stereo plane P95 screen. Local processing
uses the original OV9281 v7 calibration, physical stream identities, SGBM/LR
policy, LD19 mechanical profile, optical-datum estimate and Step 66 temporal
selector. There is no fitting to validation data.

## Step 67 and 68 evidence

**FACT / operator report** Step 67 used a uniform 400 mm target. The archive
contained 30 stereo frames and 80 LD19 scans. There were 25 temporal matches
and 5 temporal refusals. Of the matched frames, 24 refused with
`STEREO_TARGET_NOT_PLANAR_UNDER_TRIAL_LIMIT` and one with
`STEREO_PLANE_TOO_GRAZING`. There were no Step 66 spatial comparisons.

**FACT / operator report** Step 68 used a textured 400 mm target. The archive
contained 30 stereo frames and 79 LD19 scans. There were 17 temporal matches
and 13 temporal refusals. All 17 matched frames refused with
`STEREO_TARGET_NOT_PLANAR_UNDER_TRIAL_LIMIT`. There were no Step 66 spatial
comparisons.

| Representative metrics | Step 67 frame 15 | Step 68 frame 12 |
| --- | ---: | ---: |
| Usable ROI points | 95,535 | 111,862 |
| ROI coverage | 69.3016% | 86.2188% |
| Median disparity (px) | 47.125 | 47.000 |
| Median longitudinal distance from rim (m) | 1.582020 | 1.591865 |
| Global plane P95 (mm) | 90.917 | 84.311 |

**FACT / operator report** The textured representative frame had higher
stereo support and still failed the global plane gate. The two target
polygons differed slightly. These per-frame values do not characterize
their full datasets.

**OPEN** `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`. The Pi was unreachable
from the development host, and the four primary archives were unavailable
locally. No Step 69 replay result is asserted for either dataset.

## Frozen local trial policy

**DECISION** The packaged
[local policy](../../vision_core/config/diagnostics/local_stereo_lidar_correspondence_v1.json)
is frozen before real-evidence replay. Its SHA-256 is
`f2c8947121035c5ea8b87a7581099654b41e639163fb889fd026439965f77624`.
There are no threshold overrides in the replay CLI. Subsequent parameter
exploration requires explicit provenance, a separate output and an
exploratory designation.

| Parameter | Step 69 trial setting and rationale |
| --- | --- |
| Neighborhood | Radius 7 px, at most 15 × 15 full-resolution pixels. Its approximate angular span is 15 divided by rectified focal length in pixels. It is smaller than the 32 px tile in the existing Vision Core local disparity diagnostic to reduce mixing across nearby edges. |
| Anchoring | A plausible component must have a pixel within 3 px of the continuous projection and at least 8 points. The tolerance is local to the 15 px window and allows a small LR hole. It supplies no extrinsic error bound. |
| Support | A unique candidate needs 24 usable points and 35% of the polygon pixels in the clipped window. The count follows the existing local disparity diagnostic baseline. The fraction is a Step 69 trial requirement to prevent a small fragment from representing the window. |
| Connectivity | Eight image neighbors, following the Vision Core component diagnostic. An edge also needs optical-Z difference at most the larger of 35 mm and 2% of the nearer depth. A disparity pixel changes depth by roughly depth divided by disparity, so a local step allowance admits ordinary short-range disparity variation while preserving a large depth boundary. The LR tolerance remains 1.5 px. |
| Coherence | Optical-Z MAD at most 50 mm and p95–p05 span at most 150 mm, using numerical scales from the existing mask depth diagnostic. The span also detects broad depth ramps that can connect through small adjacent steps. These are local stereo trial limits. |

**FACT** The local module uses unchanged LD19-to-estimated-optical geometry,
then projects with `R1` and `P1`. Finite positive optical depth, image
membership and the predeclared polygon are recorded separately. Stereo XYZ
comes from LR-consistent disparity and `Q` in millimetres, converted to
metres and rotated into the physical-left optical frame with `R1ᵀ`. The
component graph uses only stereo depth and image adjacency inside the
window. It retains local depth-step counts, candidate indices, sample
counts, coverage, selected pixel coordinates and depth quantiles.
If multiple components are plausible near the beam's projected location,
the diagnostic records ambiguity without choosing a component. The use of
image connectivity and a stereo depth edge augments the existing
occupancy-only Vision Core grouping. The optional local plane uses the
existing least-squares plane fitter and imposes no planarity gate.

**DECISION** Compute signed `LD19 − stereo median` residuals for a coherent
candidate on two axes. Optical Z compares the estimated LD19 optical origin
with the physical-left stereo optical origin under the Step 66 assumption
that these coincide. Longitudinal distance uses the mechanical model's
robot-forward direction from the physical-left lens rim. On optical
coordinates this is `sin(20°)·Y + cos(20°)·Z − 0.0125 m`. The optical
datum remains an estimated **+13.3022 mm** axial offset with unknown
uncertainty and confidence. The measured LD19 range determines where a
beam projects. After projection, stereo component selection receives only
the pixel and stereo data. No agreement threshold or residual-based trimming
is applied. Polygon membership, local stereo availability, coherence,
depth agreement and independently verified target membership remain
distinct. An apparent candidate can still come from a different physical
surface.

## Exact Pi commands for the two offline replays

**DECISION** Run from the updated Spatial Association checkout on the Pi.
These commands read the old capture and analysis directories and create two
new directories. The replay verifies the journal hashes in the archived
analysis policy and uses its original polygon and sector. A present array
archive must match its SHA-256, exact frame and scan references, temporal
selection, and matcher/LR provenance. If no referenced array exists in the
row, replay reconstructs from verified original MJPEG using unchanged Step
66 rectification and matcher code. A referenced artifact that is missing,
altered or bound to the wrong inputs is an integrity failure.

```bash
set -e
git rev-parse HEAD
export PYTHONPATH="$PWD:$PWD/ros2/sie_ros2:$PWD/vision_core/tools${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
tool=vision_core/tools/replay_stereo_lidar_local_v1.py
evidence="$HOME/.local/state/sie/evidence"
suffix="$(date -u +%Y%m%dT%H%M%SZ)_$$"
out67="$evidence/step69_from_step67_${suffix}_local"
out68="$evidence/step69_from_step68_${suffix}_local"

"$runtime" "$tool" \
  --capture "$evidence/step67_joint_20261126T153420Z_1333_capture" \
  --global-analysis "$evidence/step67_joint_20261126T153420Z_1333_analysis" \
  --output "$out67"

"$runtime" "$tool" \
  --capture "$evidence/step68_textured_20261126T154602Z_1333_capture" \
  --global-analysis "$evidence/step68_textured_20261126T154602Z_1333_analysis" \
  --output "$out68"

"$runtime" -m json.tool "$out67/summary.json"
"$runtime" -m json.tool "$out68/summary.json"
```

## Output and boundaries

- **FACT** `replay-policy.json` records the frozen policy, original archive
  hashes, calibration/profile hashes, coordinate frames, sign convention and
  numerical library versions.
- **FACT** `comparisons.jsonl` contains capture and temporal refusals in
  separate fields. For a paired frame, the camera sequence and receipt times,
  scan identity and hashes live in a shared frame envelope with its per-beam
  range, angle, projection, neighborhood, local selection and residual
  evidence. Every beam in a paired scan has a disposition, including beams
  outside the sector and missing or invalid returns. A temporally refused
  frame has no attributable scan and no spatial attempt.
- **FACT** `summary.json` counts temporal matches/refusals, projection,
  polygon membership, stereo support, coherent candidates, ambiguous beams
  and distinct paired and compared scans. It pools median, MAD and P95
  absolute residual per beam per stereo frame, so scan reuse is possible.
  Global results retain the original 40 mm policy and are reported alongside
  local results, including frames with a local candidate and a global refusal.
  Zero candidates produce a valid summary with null residual statistics.

**OPEN** The exact physical target membership, alignment accuracy and
uncertainty remain unresolved. The LD19 stamp is a ROS conversion header.
Physical acquisition time remains `UNKNOWN`. Local results are diagnostic
only and carry no independent extrinsic validation, canonical Measurement,
World State update, Decision Engine authorization, ROS TF or actuator route.
This work starts no acquisition and changes no operational Vision Core
runtime.

**FACT** Local verification passed 157 targeted tests, including 29 Step 69
cases. The three Python files compiled, the Pi replay commands passed Bash
syntax validation, and the original Step 66 files are unchanged.

## Step 70 freeze, 2026-11-26

**DECISION** The owner approved
`STEREO_LIDAR_LOCAL_SPATIAL_CORRESPONDENCE_DIAGNOSTIC_COMPLETE`.
Continue with `THREE_SENSOR_SEMANTIC_SPATIAL_ASSOCIATION_V1` using
source-bound AR0234 semantic evidence instead of the hand-marked polygon.
Keep this local policy, the mechanical geometry, optical datum, matcher,
LR consistency, temporal thresholds and the independent 40 mm global
plane screen.

**FACT / operator-reported Pi offline replay** Residual sign is LD19 minus
local stereo median. The results are per session.

| Metric | Step 67 uniform | Step 68 textured |
| --- | ---: | ---: |
| Captured stereo frames | 30 | 30 |
| Temporal matches | 25 | 17 |
| Temporal refusals | 5 | 13 |
| Distinct compared LD19 scans | 25 | 17 |
| Global plane refusals | 25 | 17 |
| Frames with local candidates | 25 | 17 |
| Beams inside target polygon | 328 | 221 |
| Beams with usable local stereo | 324 | 221 |
| Beams with sufficient support | 218 | 189 |
| Coherent local candidates | 203 | 174 |
| Ambiguous local surfaces | 20 | 19 |
| Global disposition mismatches | 0 | 0 |
| Verified archived stereo arrays | 25 | 17 |
| Longitudinal median (mm) | +14.349338 | +24.429402 |
| Longitudinal MAD (mm) | 23.987157 | 21.211243 |
| Longitudinal P95 absolute (mm) | 87.017823 | 83.663491 |
| Optical Z median (mm) | +12.371768 | +23.978198 |
| Optical Z MAD (mm) | 23.898281 | 21.044955 |
| Optical Z P95 absolute (mm) | 85.672200 | 82.280133 |

**FACT / operator report** The runs total 377 coherent candidates, local
candidates in 42/42 temporally matched frames, 42 distinct compared LD19
scans, 39 ambiguous beams and zero global disposition mismatches. No
calibration or parameter changes were reported. These are diagnostic
candidates with unverified physical target membership. No combined
residual quantile or physical accuracy estimate was calculated.

**FACT / operator-supplied SHA-256** The private Pi root is
`/home/elwis/.local/state/sie/evidence/`.

| Replay directory | Artifact | Supplied SHA-256 |
| --- | --- | --- |
| `step69_from_step67_20261126T161455Z_1333_local` | `summary.json` | `9ab47a398a17657b65828f2e1c36ac60cb44b77330c3f12857503401a4a497a4` |
| Same | `comparisons.jsonl` | `5b72cae88a54a9978ca5ae211f58c71e0caa5d632fcc157a3dbaed3ebdcd0355` |
| Same | `replay-policy.json` | `438ddf0aa83f158b1c1a50f93a0fa95a29723376d659f4e2c844f736b4588c5f` |
| `step69_from_step68_20261126T161455Z_1333_local` | `summary.json` | `c68a8e4edfcc12e6ffe25c7514e3e9472cf67d91fc5f03f39ae076faeb63a2df` |
| Same | `comparisons.jsonl` | `4242db656fc88f37b1183f174978589cf30c0cc5a313040c61e4f5ed7db5ab8d` |
| Same | `replay-policy.json` | `5b974cc1813bf889c46693490937e70ee6a93c41d3f9f10207a437369b8cbbdc` |

**OPEN** `PRIMARY_EVIDENCE_NOT_LOCALLY_AVAILABLE`. The Pi was unreachable
and the six private result files could not be independently hashed in this
task. The packaged policy hash above was independently verified from Git.
Physical LD19 acquisition time is `UNKNOWN`. Camera exposure time,
physical target membership and the camera–LiDAR extrinsic remain
independently unverified. The optical datum is an estimate with unknown
uncertainty and confidence. Measurement, World State and motor admission
remain disabled.
