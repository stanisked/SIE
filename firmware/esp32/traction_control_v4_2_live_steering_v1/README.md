# Live steering firmware candidate v1

Versioned candidate based on `traction_control_v4_2_loaded_start_v4`. It does not replace the installed build until it has been compiled, flashed and observed on the cart. Keep the v5 stronger-start experiment retired.

## New API

- `POST /move-forward-steered?boot_session_id=...&command_id=...&distance_m=...` starts a bounded forward command, initially with equal steering bias. Allowed distance: 0.02–0.30 m. The old `/move-forward` endpoint retains its 0.10 m maximum.
- `POST /steer-update?boot_session_id=...&command_id=...&seq=...&bias_pwm=...` updates differential forward PWM for the same active command. `seq` must rise monotonically. Bias range is −20…20 PWM units. Positive bias boosts the left forward wheel and steers toward a target on the right; negative bias boosts the right wheel. No wheel reverses for steering.
- A nonzero bias is accepted only in `DRIVING`, after startup. During `STARTING`, a zero-bias update refreshes the feedback watchdog. If updates stop for over 700 ms while directed drive is active, the bounded braking path stops the command with `STEER_FEEDBACK_TIMEOUT`.
- Existing per-wheel encoder cutoff, 5 s command timeout, fault latch and `/stop` observation remain active. `/status` exposes `steering_api_version`, active bias and the last update sequence/count.

This is a diagnostic motor interface. RGB observations and their age are checked on Pi, not independently verified by ESP32. The physical stopping bound and motion admission remain unestablished.

## Build preparation

Use the same ESP32 board/core and wiring as the loaded-start v4 sketch. Copy the local, private `wifi_credentials.h` into this sketch directory; `wifi_credentials.example.h` shows the format. Do not commit the private file. Compile and flash this directory as a new candidate. Before any movement, read `/status` and verify `firmware_build_id=v4_2_live_steering_v1`, `steering_api_version=v1_active_forward_pwm_bias`, `state=READY`, no fault latch, and both PWM values zero.

The companion Pi runner is `vision_core/tools/run_v7_live_steered_forward.sh`. Start with `--preview-only` to inspect the selected target and intended wheel bias without HTTP or motor posts. A subsequent supervised motor trial uses `--execute` and a freshly measured front clearance. Begin at the default 0.10 m bounded length; the optional `--distance-m` may be raised only after the shorter command's physical behavior is reviewed.
