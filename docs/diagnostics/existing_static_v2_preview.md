# Existing static v2 profile: explicit diagnostic mode and offline replay

2026-10-21. No activation file is edited or generated. No Measurement, canonical
Decision, navigation or motor command is authorized.

## Reviewed installed artifact

Operator uploaded ar0234_ov9281_decision_chain_activation.json:
SHA-256 c835a304cc2758843747400c30f09a567b5ac74ebfd7908ea8ee14a013f519d9.
Path: /home/elwis/dev_ws/runtime_artifacts/ar0234_ov9281_decision_chain_0p5_to_4p5m_v2/ar0234_ov9281_decision_chain_activation.json.

It names active v7 39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52,
AR intrinsic e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096,
and extrinsic 10b5fdd9b3467ba5e04a8bd6d2daad8440382798374c39bfd4b9037c2bf5b9cc.
Operator separately checked the installed extrinsic hashes for v1 and v2.

This is the existing conditional 0.5–4.5 m static recommendation profile.
It explicitly disables execution and dynamic fusion. It is not a new calibration,
a 5% physical-error guarantee or authority for the half-resolution segmentation
diagnostic backend to emit operational Measurement.

Static policies actually present in this artifact:
- single target; no top/left/right truncation; bottom truncation is permitted;
- two consecutive observations, maximum centre displacement 12 px;
- maximum relative bbox-area change 0.10; candidate gap at most 5 s;
- apply before stereo fusion.

These are historical profile parameters, not new thresholds selected from the
current successful frames. Existing stricter frontend target-selection rules
remain in place; the bottom exception does not relax the segmentation selector.

## Gap found in implementation

The parent static Measurement node interprets these policies, but its diagnostic
subclasses deliberately skipped static policy interpretation. Merely changing
activation_profile in the dynamic preview would therefore bind the range without
applying the corresponding static conditions.

A separate opt-in preview_existing_static_profile_v2 parameter now requires the
reviewed exact profile SHA, current-target crop and single-segmentation frontend.
StaticProfilePreviewGate applies the above visual checks after existing target
selection/geometry and before current-target stereo submission. The first
observation is PENDING, visible movement is FAIL and restarts confirmation.
Missing target, invalid edge/clock/cycle or an unevaluated/cancelled processing
cycle clears confirmation. Visual stability is not proof of physical stillness
or exposure synchronisation.

The gate result is bound to cycle ID and reviewed profile hash in the existing
control-input shadow. A missing/wrong/pending/failed result cannot be inherited
as a pass. All previews remain STOP and zero speed; no actuator transport exists.

## Separate launcher, previous default preserved

run_control_preview_existing_static_v2.sh opts into the existing profile, then
uses the existing front-geometry/fast-local diagnostic chain. The shared launcher
still selects original v1 unless this explicit option is set. Profile hash and
policy are verified before camera startup in the supplied launcher; the existing
parent verifies actual calibration/extrinsic hashes on node startup.

The final summary now includes static gate status, control-preview checks and
assessment time. No live run is claimed by this change.

## Offline first, no new capture

run_existing_static_v2_replay.sh applies the same gate to the already recorded
99-cycle 20261021_113225 journal, checks recorded calibration bindings, and
compares existing range checks with an explicitly labelled v2 scenario.
It preserves original journals, recomputes draft STOP and hashes all sources.

The scenario does not rewrite history: its visual gate was NOT run before
historical stereo. Host UTC cycle-start timestamps are the available replay
clock; the live gate uses monotonic time at evaluation. Frame gap/order problems
remain visible. A scenario pass does not establish current profile applicability,
physical error, timing or permission.

Sixteen host tests passed across static profile, control-input binding and draft
admission. They cover exact profile pinning/no-execution, temporal recovery,
truncation, multiple targets, gap/clock/duplicate cycles, current-cycle binding,
counterfactual replay and unchanged sources. An AST-isolated execution of the
existing static producer's actual temporal method agrees on the tested sequence.
The actual uploaded profile passed SHA/policy verification. Five changed Python
files compile; all three shell launchers pass bash syntax checking.

Actual 99-cycle Pi replay and live v2 integration are pending. Existing v7,
legacy MAD thresholds, physical error bounds, profile files and motor permissions
remain unchanged.

## Latest confirmed live integration before this change

The v1 control-input shadow integration produced 99 camera cycles, all STOP
bound to the correct source cycle. After first-frame exclusion, 98/98 coherent
cycles had driver-declared ages and host durations within 250 ms; host median
195.3 ms, max 242.2 ms. Median candidate optical Z was 3.1484 m, span 32.3 mm.
The added admission assessment cost median/max 0.208/0.306 ms, outside that host
timer. All 99 failed the selected v1 0.5–2.0 m range; one failed driver age.
These measurements are not absolute depth accuracy or exposure-to-command bounds.
