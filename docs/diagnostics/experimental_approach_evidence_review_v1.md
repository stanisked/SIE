# Experimental approach evidence review v1

Status: DRAFT_NOT_ACTIVATED. Reviewed repository commit: `4f1bf52255b4370a88cfe478a71b2e421b4355b1`.

This review prepares the next camera-driven approach integration. It grants no motor command, Measurement emission, capability qualification, or range activation. The companion JSON is a planning record, not a runtime configuration. Existing active profiles remain authoritative within their own scope.

## Reusable drive evidence

| Evidence | Result | Applicability |
| --- | --- | --- |
| Supervised bounded forward | Requested 0.100 m, reported mean 0.0938 m, heading 0.002 rad | Historical bounded-step behavior; not continuous visual approach qualification |
| Separate completion-window floor trial | Requested 0.100 m, manually 0.098 m, encoder mean 0.0972 m | Separate trial, not a repeat of the preceding record |
| Loaded speed comments | 67 right and 69 left counts per 0.5 s; approximately 0.133 and 0.137 m/s from recorded effective diameters/counts | Rounded source comments, not raw speed traces or an upper bound |
| Firmware profile | 0.140 m/s maximum profile speed | Configuration; the working target remains up to 0.150 m/s |
| Capability registry | NOT_QUALIFIED | Historical supervised sessions do not change this registry |

The three-step historical session ended on `YOLO_MULTIPLE_TARGETS`. The legacy profile requires a new observation after each terminal step, prohibits POST retries, and does not authorize dynamic fusion while moving. Its older 1 s decision and 5 s measurement ages are not inherited by the new 250 ms target.

## Concrete stop-path gap

In the reviewed v4.2 source, `handleStop()` calls `stopMotors()` (all motor PWM outputs zero), resets reported speeds, marks READY and completes an active command as STOPPED immediately. `completeActiveCommand()` clears the bounded-motion bookkeeping. This route does not call `brakeMotors()` or wait for settled encoder counts. Bounded target completion uses a different active-brake/settle path.

Therefore the HTTP acknowledgement and terminal state do not establish physical rest. The configured 400 ms brake hold, 100 ms settle observation, and stopping-distance model are not measurements proving the requested additional 25 mm / 0.5 s stop bounds. The finding concerns repository source; the flashed controller identity is still unknown. Relevant raw braking traces were not found in the repository evidence and performed Drive searches.

Next: identify the deployed controller through read-only status/build information; if this source applies, make a separately reviewed stop-path change before using loss of vision to request a physical stop. Do not flash firmware or execute movement from this review.

## Vision and geometry handoff

The 91-cycle admission replay has 90 cycles within the driver-declared age target and one refusal. All 91 remain outside the current activated 0.5–2.0 m metric range, and none is admitted. The 0.5–4.5 m range is diagnostic only. Driver timestamp semantics are not independent exposure-to-command timing validation. Low local surface residuals are not a physical depth error bound.

The JSON preserves the agreed working targets and leaves unknown evidence null. The proposed extra 100 mm reserve is explicitly unvalidated. No quality threshold is inferred from a successful static scene.

Camera-to-front clearance requires an explicit survey:

`p_base = R_base_from_rectified_left * p_rectified_left + t_base_from_rectified_left`

Record the rotation, translation in metres, robot-front footprint, survey method and uncertainty. Evaluate owned target surface points relative to the footprint along the intended path. A lens-front-rim distance is not an optical-centre transform; median optical depth is not automatically front clearance. These quantities are pending, not zero.

## Finite continuation

1. Verify controller identity and close the concrete remote-stop gap.
2. Recover or establish the stopping bound on that controller/platform; reuse existing drive evidence where its scope matches.
3. Survey the missing mounting transform and front footprint, then review metric range applicability and the physical error/association budgets.
4. Connect current-frame depth admission to an explicit approach experiment only after its prerequisites are evidenced. Preserve refusals and post-terminal reobservation.

No further static camera run is requested here. The immediate engineering work concerns controller stop semantics and robot-frame clearance. Source paths and SHA-256 hashes are recorded in `vision_core/config/diagnostics/experimental_approach_profile_draft_v1.json`.
