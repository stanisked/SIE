# Step 73.3 | Entity-Centric Diagnostic Preview v1

**DECISION — `OWNER_APPROVED`, 2026-11-27.** Add an offline preview of
complementary AR0234, OV9281, and LD19 evidence, with diagnostic-only output
under `entity-centric-preview-v1-TRIAL`. Use the
[fundamental architecture baseline](../../PROJECT_CONTEXT.md#fundamental-architecture-baseline)
and the [Step 73 continuation proposal](../../PROJECT_CONTEXT.md#step-73--entity-centric-knowledge-integration-for-mvp).

## Inputs and behavior

- **FACT.** The replay preserves the Step 57 journal's independent RGB–Stereo
  and RGB–LD19 dispositions. Camera samples use the same 250 ms diagnostic
  freshness policy evaluated at the recorded RGB receipt. Spatial binding
  additionally requires the snapshotted stereo packet to be the accepted
  causal RGB–Stereo match within the existing 25 ms window. Recorded clock,
  source and receipt information remains available for audit.
- **FACT.** The camera path reuses Step 71 snapshot identity and digest
  checks, AR0234 instance and track binding, OV9281 v7 rectification,
  full-resolution SGBM7 and left-right consistency, and the visible
  RGB-owned stereo component selector. Reconstruction runs offline. Component
  depth is optical Z and position is the componentwise median XYZ, both in
  metres in `physical_left_optical_frame`. Ambiguous layers or components
  keep the RGB evidence and prevent creation of a spatially supported entity.
- **FACT.** When a bounded snapshot is missing or refused, a same-cycle
  producer audit, if available, can preserve a current semantic observation
  and producer-reported mask reference. It must match the RGB packet in
  the temporal journal. Stereo reconstruction still requires the bound
  snapshot. Held selection and tracking metadata remain traceable and cannot
  establish a current observation. The Step 71 capture only archives a
  subset of cycles and does not create a stereo snapshot for every RGB frame.
- **FACT.** LD19 retains source-local geometry from the existing diagnostic
  LaserScan records. Each valid beam gives range and XY in metres and
  bearing in radians in `ld19_sensor`. Invalid, missing, and out-of-bounds
  ranges have explicit dispositions and provide no free-space evidence.
  Physical acquisition time remains `UNKNOWN`. The Step 71 archive includes
  only the retained RGB-linked callback payload. A separate existing LD19
  diagnostic journal can be replayed on its own or alongside the camera archive.
  Its rows stay separate and their cross-source timing is unassessed.
- **DECISION.** A current RGB observation and its uniquely bound stereo
  component can support a provisional entity even with zero Surface-Level
  candidates or no LD19 payload. An optional existing Step 71/72 replay
  supplies Surface-Level context only after its journal, policy, and
  per-cycle source references are checked. No LD19 entity-membership policy
  is approved in this preview. Unresolved and ambiguous relationships leave
  independent obstacle evidence available.
- **FACT.** The preview runs only on explicit offline invocation. It has
  no capture, inference, ROS publishing, synchronization waits, navigation,
  or actuator path. Calibration and matcher parameters are unchanged.
  No output grants admission as a Fact or Metric Measurement or authorizes
  a World State update, a Decision Engine action, navigation, or motion.

## Output

The output directory must be new. Keep the directory and source artifacts private.

| File | Contents |
| --- | --- |
| `cycles.jsonl` | Source-bound diagnostic rows, disposition reasons, current semantic evidence and mask references, spatial component provenance, independent LD19 beam geometry, and provisional entity references. |
| `summary.json` | Source cycle and archive coverage counts, source dispositions and reasons, semantic and spatial evidence counts, obstacle observations and unique scan counts, entity candidates, and association dispositions. Includes SHA-256 and policy provenance. |
| `processing_cost.json` | Measured offline wall time including archive reads, reconstruction, and writes. Excluded from the deterministic diagnostic rows and summary. |

Source availability counts refer to recorded source packet identities.
Evidence dispositions distinguish available observations from unavailable,
stale, ambiguous, and rejected evidence. A scan reused across multiple
cycles contributes multiple obstacle observation instances and one unique
scan. Independently available evidence is retained in its original frame
with its own timing. Only the RGB–Stereo pair supports an entity in this
version.

## Raspberry Pi deployment and offline replay

Use the Pi's existing SIE checkout as the Git source and the existing Python
environment and calibration artifacts. The commands use a separate worktree
for the preview and existing evidence. They do not start ROS nodes or devices.

```bash
set -euo pipefail
repo=/home/elwis/dev_ws/sie_live_identity_20261119
branch=codex/siee-three-sensor-spatial-association-v1-20261126
checkout=/home/elwis/dev_ws/sie_step73_preview
git -C "$repo" fetch origin "$branch"
git -C "$repo" worktree add --detach "$checkout" FETCH_HEAD
cd "$checkout"
git rev-parse HEAD
export PYTHONPATH="$PWD:$PWD/ros2/sie_ros2:$PWD/vision_core/tools${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1

runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
artifacts=/home/elwis/dev_ws/runtime_artifacts
model=/home/elwis/.local/state/sie/evidence/segmentation_input_size_20261015_123033/input_256x384/checkpoint.onnx
run=/home/elwis/.local/state/sie/evidence/live_temporal_shadow_iWIjW8zB
prior="${run}_step71_20261126T171308Z_1333"
out="${run}_step73_$(date -u +%Y%m%dT%H%M%SZ)_$$"
test -d "$run/shadow-source-snapshots"
surface_args=()
if [[ -d "$prior" ]]; then
  (
    cd "$prior"
    sha256sum -c <<'SUMS'
73a8b0cdd801020c50778df93d38b56bf9f693e6e342e29d8df7ba08bd539b61  summary.json
08bb65b49454c57131682faf77b8f6ada16e510f7fbe9c84ef7428c60d917655  cycles.jsonl
164cfb4c452d33f14f30aca55b42cf68e6a5c23d580d7d159074322f4f76115b  policy.json
SUMS
  )
  surface_args=(--surface-replay "$prior")
fi
"$runtime" vision_core/tools/replay_entity_centric_diagnostic_v1.py \
  --run "$run" "${surface_args[@]}" --output "$out" \
  --ar-intrinsic "$artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json" \
  --stereo-calibration "$artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz" \
  --activation "$artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json" \
  --model "$model"
"$runtime" -m json.tool "$out/summary.json"
"$runtime" -m json.tool "$out/processing_cost.json"
```

For a trusted copy of the evidence at different paths, set `run` and `prior`
to the existing source and replay directories. For an LD19-only replay with
an existing JSONL journal, use these arguments from the same checkout
and Python environment.

```bash
read -r -p 'Existing LD19 diagnostic JSONL path: ' ld19
test -f "$ld19"
"$runtime" vision_core/tools/replay_entity_centric_diagnostic_v1.py \
  --ld19-journal "$ld19" --output "${ld19%.jsonl}_step73_$(date -u +%Y%m%dT%H%M%SZ)_$$"
```

## Verification and limits

**FACT / software verification.** 211 targeted tests and five subtests
passed, including 17 Step 73.3 test cases. The synthetic archive replay
produced a provisional camera entity with zero Surface-Level candidates,
retained independent LD19 observations across both cycles, and produced
byte-identical diagnostic rows and summary on repeated runs. The LD19-only
test used the existing scan format without camera inputs. Python compilation
and Bash syntax checks passed.

**FACT / operator report.** Step 73.1 verified the three reported Step 72
replay hashes on the Pi. The operator named the source archive
`step73_entity_integration_evidence_20261127.tar.gz`.
[Step 72 metrics and hashes](../handoffs/SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md#step-72--operator-reported-three-sensor-replay)

**OPEN.** The named archive was unavailable locally and the read-only Pi
SSH attempt failed authentication. No primary-data replay or independent
check of those hashes was performed here. LD19 physical target membership
remains unverified. Physical validation is a separate step. The ESP32 v13
braking fault remains a separate actuator blocker.

## Step 73 closure

**DECISION — `OWNER_APPROVED`, 2026-11-27.**
`ENTITY_CENTRIC_OFFLINE_INTEGRATION_COMPLETE`. The operator's Step 73.4
Pi replay produced 17 provisional camera entity candidates and 52 independent
LD19 observations. LD19 entity membership remained unresolved. The reported
counts are per-cycle evidence dispositions.
[Step 73.4 results, limits, and hashes](../handoffs/SIEE_THREE_SENSOR_SPATIAL_ASSOCIATION_HANDOFF_20261126.md#step-734--offline-entity-integration-complete)

## Step 74 live companion

The [Step 74 live diagnostic](live_entity_centric_integration_v1.md) reuses
the source-local Step 73 integration and adds separate ROS observers for
the existing camera audit and LD19 scan. Step 73 replay behavior and its
full-resolution spatial semantics remain unchanged.
