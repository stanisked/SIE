# Continuous approach to a moving semantic target — operator requirement

Recorded 2026-10-27 from the operator's correction. This is the intended behavior for the mobile-base experiment, not an activated controller or a change to canonical Measurement admission.

## Required behavior

1. Acquire the selected semantic object from a fresh camera observation. The object may be anywhere in the image. The acquisition must establish which object is being followed; today's person detector is only one adapter.
2. Start forward motion as soon as that object is acquired and the separately required motion conditions are satisfied. Do not demand that the object be centered and do not request an initial turn merely to center it.
3. While the **same forward command remains active**, consume successive fresh observations of the same tracked object. Recompute its current position/bearing and adjust the relative right/left wheel drive in response to its motion relative to the cart. Steering begins after a later observation reveals an error; it is not a prerequisite for the initial motor start.
4. The target is allowed to move. Its first image coordinate seeds acquisition and identity; it must not be treated as a permanently static target pose. Tracking must preserve object identity through the cart's own motion and target motion. A changed or ambiguous identity cannot be silently replaced by another detection.
5. If the target disappears, identity becomes ambiguous, camera observations become stale, a drive fault occurs, or the bounded motion envelope is exhausted, stop the active drive and record the reason. Resume only from a new acquisition and a newly authorized motion command.

The desired control sequence is **acquire → begin moving → observe and steer during motion → stop at the applicable boundary**. A series of forward steps separated by stop-and-turn commands is a diagnostic predecessor, not this behavior.

## Current implementation gap

`vision_core/tools/run_v7_supervised_approach.py` starts its first 0.10 m step after an eligible detection, then stops and reobserves. Later steering uses a separate `/turn-left` or `/turn-right` request before another straight step. Its original-coordinate comparison is useful diagnostic evidence, but it is not live tracking of a moving target during an active forward command.

The current ESP32 bounded API accepts separate `/move-forward` and `/turn-*` commands; it has no command-bound update for left/right wheel setpoints while forward motion is active. The v5 stronger-start sketch was retired after a lifted-wheel `BOUNDED_DISTANCE_LIMIT` fault (14/16 counts against 13/13 targets). Motor quality and unequal wheel response must be treated as measured plant behavior; they do not change the target-following requirement.

## Implementation direction

- Keep target detection/tracking separate from motor control. Each update must carry the current target identity, capture time, camera calibration, and target bearing or position with provenance. The controller should use a new observation, not assume the target stayed at its acquisition coordinate.
- Build a diagnostic-only controller that turns a stream of current target observations and wheel-encoder feedback into bounded differential wheel commands. Show the intended wheel outputs in a journal before allowing motor posts. The first output is straight drive; later outputs may contain steering.
- Add a versioned ESP32 command interface for **one active bounded forward motion** plus command-ID-bound steering updates. It needs encoder feedback for both imperfect motors, a watchdog for missing updates, bounded wheel outputs and travel, and a confirmed stop path. It must reject updates for another boot session or command.
- Validate the interface with motors lifted, then in clear space without a target person, then with a moving target under an explicitly supervised trial. Record physical path and stopping behavior. Promote none of these diagnostic outputs to canonical Measurement or autonomous motion admission without the separate evidence and contract review.

No new motor command, firmware deployment, or approach admission is implied by this document.
