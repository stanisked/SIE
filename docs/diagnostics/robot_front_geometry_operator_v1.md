# Partial robot-front geometry diagnostic v1

Status: diagnostic only, opt-in, no Measurement or motor command. This is an additional current-frame description; existing admission and MAD/range policies remain unchanged.

## Operator geometry

The operator confirmed on 2026-10-16 that the physical LEFT OV9281 lens front rim is 0.210 m behind the robot front along travel, and the block is pitched 15 degrees UP from horizontal. These values are stored with provenance in `vision_core/config/diagnostics/robot_front_mount_operator_v1.json`. Their survey uncertainties were not supplied. Zero block yaw and roll are explicit unsurveyed hypotheses, not extra measured facts. No full base_link transform or footprint is asserted.

## Per-point calculation

For each selected LR-consistent point owned by the current eroded RGB instance mask and ROI:

1. Reconstruct rectified-left XYZ from native disparity-scale pixel centres, metre depth and the original P1.
2. Undo stereo rectification with R1 transpose to get physical-left optical XYZ. Using the block pitch directly on the rectified coordinates would omit this calibrated rotation.
3. Project onto the hypothesized forward direction: `forward = Y_raw * sin(15 deg) + Z_raw * cos(15 deg)` (optical Y down, Z forward).
4. Compute the known term `K = forward - 0.210 m`.

The remaining relation is `front_distance = K - rim_forward_of_optical_center_m`. The last quantity is the forward component from optical centre to the lens rim and is unknown. It is never replaced with zero. `distance_from_front_m` stays null. The report supplies the distribution of K, including median and p05/p95, as `known_term_m`. It is not labelled an admitted robot clearance. At Y_raw=0 and Z_raw=1.5 m, K is about 1.2389 m; that is a formula example, not a physical measurement.

Full robot-frame coordinates, robot width/height, swept-path obstacles, target extent behind missing stereo support, physical depth accuracy and timing applicability are outside this partial longitudinal calculation. A person's median surface coordinate is not automatically their nearest relevant surface. Neither K nor its minimum/quantile is a motion permission.

## Integration

The optional ROS parameter `preview_front_mount_profile` is empty by default. A non-empty reviewed diagnostic profile enables the calculation in the existing isolated mask worker, on the exact same selected samples. It does not alter disparity, masks, selected depth, local quality thresholds or the previous refusals. The additional result is stored as `shape_diagnostic.front_geometry` with profile hash and current cycle/mask/instance/model/RGB/fusion-array bindings. The client rejects a mismatched binding/profile or a claimed complete distance. P1/R1 are carried in the hashed fusion payload, including the prepared-geometry path. Invalid geometry fails explicitly through the existing unavailable diagnostic path.

The result includes `processing_s`; runtime latency impact must be measured on Pi, not inferred from synthetic tests. Disabled mode preserves the old path and adds no per-point projection.

## One camera-only check

On Pi, keep motors disabled, update the branch and build sie_ros2 as usual. Then:

```bash
bash vision_core/tools/run_control_preview_front_geometry.sh
```

This delegates to the current fast-local diagnostic setup, retaining its reviewed model, native capture and matching settings. It records a short current scene and prints FRONT_GEOMETRY, FRONT_KNOWN_TERM and processing time. Existing timing/freshness output remains available. No board sweep, range activation, firmware flash or movement is requested. Inspect this result before using it in any later approach policy.

## Verification

Nine unittest cases pass: upward-pitch sign, distinct Y at equal Z, a nonidentity rectification rotation, half-resolution pixel centres, invalid rotations/profiles/nonfinite inputs, empty points, immutable inputs/profile hash, selected-instance provenance rejection and preservation of original depth statistics with optional front math. Integration tests isolate image ownership and transport; they do not validate OpenCV image processing, ROS startup or Pi performance. Changed Python files pass py_compile; launch shell scripts pass bash -n. Real Pi output is pending.

## Drive handoff

The corrected timer trial remote_stop_trial_20261016_181217_a36eace3 interrupted initial wheel movement: terminal STOPPED, READY/no latch, total encoder mean 14.4 mm, about 3.97 mm additional wheel displacement after firmware brake onset. The 602 ms value is brake/quietness observation duration, not measured physical stopping time. The source-hashed review is `remote_stop_trial_20261016_interrupted_review.json`. This establishes the remote-stop path in that launch trial; it does not establish maximum-speed stopping bounds.
