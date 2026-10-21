# Current camera cycle to control-input shadow binding

2026-10-21. Diagnostic only. No Measurement, canonical Decision, World State or motor command is created.

## Concrete gap closed

The live control preview previously called approach_control_preview() without any
inputs. Its generic STOP was independent of the detailed current-frame depth
admission checks, which existed only in offline replay.

control_input_shadow.evaluate_control_input_shadow now recomputes that existing
draft from each current audit. The live node attaches depth_admission_draft,
control_input_shadow and a cycle-bound decision_preview before diagnostic
publication and journaling. Existing task-budget preview fields are preserved;
concrete failed and unknown checks accompany STOP. Caller-supplied old assessments
or claimed permissions are ignored. An assessment exception also produces STOP.

The existing offline analyzer uses the same binding and reports its per-cycle
preview, checks and summary reason counts. Source journal/policy hashes are
checked, old recordings are preserved, and cancellations remain excluded from
offline completed-cycle summaries. Live cancellation still has a STOP preview.

## Interpretation

- candidate_optical_z_m is diagnostic Z in rectified_left_optical_frame, metres,
  with unvalidated confidence. It is never robot-front clearance.
- failed_checks distinguish observed failures such as missing target, stale
  driver metadata, bad local binding or out-of-activation-range depth.
- missing_checks distinguish unknown physical bounds, admission policy,
  geometry, command-age, stopping and path-observation evidence.
- Every preview remains STOP with zero recommended speed, no admitted source
  Measurement and no actuator access. This is a diagnostic explanation, not a
  controller stop request or navigation approval.
- The previous global-MAD refusal remains visible and unchanged. No local
  residual threshold was invented or substituted for a physical error bound.
- Sparse asynchronous local results do not authorize later frames. The existing
  draft requires exact current-frame fast-local provenance.
- Legacy admission_blockers are retained with an explicit legacy-screen label;
  their host-duration screen is not the new binding's freshness check.
- assessment_processing_s records the added live assessment time. Driver age is
  sampled before this assessment; it is not an end-to-end exposure-to-command
  bound. No latency improvement or 250 ms compliance is claimed here.

The v7 calibration, existing range activation, target-selection rules,
50 mm legacy MAD limit and drive permissions are unchanged.

## Verification and next operator step

Nine host unit/replay tests passed, including exact current-cycle mapping,
wrong-mask provenance, stale age, target loss, missing cycle identity,
cancellation, fabricated prior permission, evaluator failure, malformed shape
and subprocess replay with unchanged input bytes. Four changed Python files
compiled. No ROS runtime, Pi camera or ESP32 command was executed here.

On Pi, replay the already saved 91-cycle journal without cameras or ROS build:

```bash
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
git pull --ff-only
bash vision_core/tools/run_depth_admission_draft_replay.sh
```

Actual Pi replay and subsequent live integration/latency are pending. Replaying
old data cannot demonstrate current camera or network performance.

## Current operator context

The operator chose to continue with v7 while retaining its known geometry warning
and periodic calibration checks. In the conversation's offline hypothesis test,
a roughly 0.10-degree pitch or a 2.11 px vertical shift each reduced held-out
corner medians to about 0.13–0.15 px; the three nearly identical board poses
cannot identify a unique physical cause. No inferred correction is applied.

Operator read-only /status on 2026-10-21 confirmed Pi-to-ESP32 access through
SIE-Mobile at http://10.138.186.229, build label v4_2_remote_stop_observed_v1,
boot DB48F24762C93392, READY, no active command/fault, both PWMs zero.
This DHCP address is observational configuration, not a hard-coded endpoint or
motion authorization. The firmware build label does not establish a binary hash.
