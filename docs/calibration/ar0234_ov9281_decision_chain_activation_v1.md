# AR0234 + OV9281: frozen extrinsic evidence and decision-chain activation

**Recorded:** 2026-09-29  
**Git branch:** `codex/sie-ros2-integration-v1`  
**Scope:** Raspberry Pi 5 `piecake`; static / stop-and-measure ROS2 decision recommendations only.

## Decision

The AR0234-to-physical-left-OV9281 extrinsic candidate is independently validated on a capture session that did **not** contribute to its 62-pair solve.  It is activated as:

`ACTIVE_CONDITIONAL_SUPERVISED_DECISION_RECOMMENDATION_ONLY`

This permits the policy chain:

`Observation -> Interpretation -> Metric Measurement -> navigation recommendation -> supervisor state`

It does **not** permit physical execution:

- `execution_authorized=false`
- `actuator_bridge=DISABLED_PHASE_1`
- `motor_command_performed=false`
- no ESP32/motor command and no dynamic fusion while moving.

## Bound artifacts

| Artifact | SHA-256 | Pi path |
| --- | --- | --- |
| AR0234 intrinsic v5 | `e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096` | `/home/elwis/dev_ws/runtime_artifacts/ar0234_intrinsic_v5_final_20260910/calibration_fullres.json` |
| OV9281 Pi stereo v7 | `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52` | `/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz` |
| AR0234-to-physical-left extrinsic candidate | `10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc` | selected by exact source-stereo SHA; copied into the activation directory |
| Decision-chain activation record | generated on Pi | `/home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_v1/ar0234_ov9281_decision_chain_activation.json` |

The candidate is bound to exactly the AR v5 and OV9281 v7 SHA-256 values above.  A different SHA is a different calibration and must not reuse this activation.

## Stream identity and units

- Captured first half of the combined 2560x800 OV9281 frame: **physical right**.
- Captured second half: **physical left**.
- The extrinsic transform target is physical left.
- OV9281 stereo `T` is stored in millimetres; PnP validation operates in metres and converts it explicitly.

## Independent holdout gate

Fresh holdout dataset:

`/home/elwis/dev_ws/ar0234_ov9281_extrinsic_holdout_v1`

- 12/12 fresh checkerboard poses accepted.
- The flat 9x6-inner-corner chessboard with 24.5 mm cells was used.
- These poses were not among the 62 pairs used by the extrinsic solve.

| Metric | Observed | Accepted limit |
| --- | ---: | ---: |
| AR0234 cross-reprojection median | 1.0906 px | <= 1.5 px |
| AR0234 cross-reprojection P95 | 2.7569 px | <= 3.0 px |
| Physical-right reprojection median | 0.4697 px | <= 0.5 px |
| Physical-right reprojection P95 | 0.5096 px | <= 1.0 px |
| Rectified median absolute dy | 0.3530 px | <= 0.40 px |
| Rectified median absolute dy P95 | 0.4324 px | <= 0.75 px |

The initial 0.35 px median dy limit rejected the dataset by 0.0030 px despite a P95 of 0.4324 px.  The final validation report records the explicit 0.40 px median policy limit; the P95 limit remains 0.75 px.

## Reuse and invalidation

Do **not** repeat this calibration merely because time has passed.  Reuse this activation only when all bound SHA-256 values match and the following remain unchanged:

- AR0234 and OV9281 cameras, lens/focus, rigid mount, and physical baseline;
- 1920x1200 AR0234 and 2560x800 combined OV9281 modes;
- the first/right, second/left OV9281 stream identity;
- runtime range 0.5 to 2.0 m and stop-and-measure operation.

Create a new calibration and independent holdout gate if any bound SHA changes, camera geometry or resolution changes, a camera is remounted/refocused, or an operating mode outside this conditional scope is required.
