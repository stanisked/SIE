# Continuous approach to a moving semantic target — operator requirement

Recorded 2026-10-27 from the operator's correction. This is the intended behavior for the mobile-base experiment, not an activated controller or a change to canonical Measurement admission.

## Required behavior

1. Acquire the selected semantic object from a fresh camera observation. The object may be anywhere in the image. The acquisition must establish which object is being followed; today's person detector is only one adapter.
2. Start forward motion as soon as that object is acquired and the separately required motion conditions are satisfied. Do not demand that the object be centered and do not request an initial turn merely to center it.
3. While the **same forward command remains active**, consume successive fresh observations of the same tracked object. Recompute its current position/bearing and adjust the relative right/left wheel drive in response to its motion relative to the cart. Steering begins after a later observation reveals an error; it is not a prerequisite for the initial motor start.
4. The target is allowed to move. Its first image coordinate seeds acquisition and identity; it must not be treated as a permanently static target pose. Tracking must preserve object identity through the cart's own motion and target motion. A changed or ambiguous identity cannot be silently replaced by another detection.
5. If the target disappears, identity becomes ambiguous, camera observations become stale, a drive fault occurs, or the bounded motion envelope is exhausted, stop the active drive and record the reason. Resume only from a new acquisition and a newly authorized motion command.

The desired control sequence is **acquire → begin moving → observe and steer during motion → stop at the applicable boundary**. A series of forward steps separated by stop-and-turn commands is a diagnostic predecessor, not this behavior.

## Baseline implementation gap

`vision_core/tools/run_v7_supervised_approach.py` starts its first 0.10 m step after an eligible detection, then stops and reobserves. Later steering uses a separate `/turn-left` or `/turn-right` request before another straight step. Its original-coordinate comparison is useful diagnostic evidence, but it is not live tracking of a moving target during an active forward command.

The current ESP32 bounded API accepts separate `/move-forward` and `/turn-*` commands; it has no command-bound update for left/right wheel setpoints while forward motion is active. The v5 stronger-start sketch was retired after a lifted-wheel `BOUNDED_DISTANCE_LIMIT` fault (14/16 counts against 13/13 targets). Motor quality and unequal wheel response must be treated as measured plant behavior; they do not change the target-following requirement.

## Implementation direction

- Keep target detection/tracking separate from motor control. Each update must carry the current target identity, capture time, camera calibration, and target bearing or position with provenance. The controller should use a new observation, not assume the target stayed at its acquisition coordinate.
- Build a diagnostic-only controller that turns a stream of current target observations and wheel-encoder feedback into bounded differential wheel commands. Show the intended wheel outputs in a journal before allowing motor posts. The first output is straight drive; later outputs may contain steering.
- Add a versioned ESP32 command interface for **one active bounded forward motion** plus command-ID-bound steering updates. It needs encoder feedback for both imperfect motors, a watchdog for missing updates, bounded wheel outputs and travel, and a confirmed stop path. It must reject updates for another boot session or command.
- Validate the interface with motors lifted, then in clear space without a target person, then with a moving target under an explicitly supervised trial. Record physical path and stopping behavior. Promote none of these diagnostic outputs to canonical Measurement or autonomous motion admission without the separate evidence and contract review.

## Versioned candidate prepared

`traction_control_v4_2_live_steering_v1` adds `/move-forward-steered` and command-bound `/steer-update`. The Pi runner `run_v7_live_steered_forward.py` acquires one current RGB person instance anywhere in view, sends a straight bounded forward command, and uses later fresh frames to update differential wheel PWM while that command remains active. Its live-target check binds the current segmentation, calibration and driver timestamp, then uses adjacent-frame box correspondence. It does not require the static-v2 stillness gate or stereo mask, which are for different evidence. A top-clipped head may remain trackable; a side-clipped or ambiguous instance is refused. A missing frame, identity mismatch, rejected update, or controller fault causes `/stop` and zero-PWM confirmation. The first command is bounded to 0.02–0.30 m; the initial physical trial should use 0.10 m. The camera-only `--preview-only` mode journals proposed bias before motor use.

Operator sequence after updating Pi and flashing the candidate ESP32 build:

```bash
bash vision_core/tools/run_v7_live_steered_forward.sh --preview-only --preview-duration-s 20
# After reviewing the preview journal and verifying /status, first supervised test:
bash vision_core/tools/run_v7_live_steered_forward.sh --execute --base-url http://10.13.32.229 --front-clearance-m 2.5 --distance-m 0.10
```

Use the measured current front clearance and controller address; the example values are not assertions about the next test scene. Inspect the new evidence summary and physical wheel motion before considering a longer command.

The current AR0234 semantic source supplies person instances. In the moving-target preview on 2026-10-27, the static-v2 gate rejected `MOTION_DETECTED`, and the original selector rejected top-edge boxes even while the instance confidence remained about 0.84–0.93. The separate RGB diagnostic path addresses those two specific causes without altering the static profile or canonical Measurement. The present AR0234 semantic source selects a person instance. The control contract can accept another semantic target adapter later, but arbitrary object classes are not yet detected or tracked. Adjacent-frame box correspondence is conservative and stops on ambiguity; it is not a persistent multi-object tracker. The firmware source is unflashed and uncompiled in this environment, and no physical steering or stopping bound has been established for this candidate. Canonical Measurement and autonomous motion admission are unchanged.
