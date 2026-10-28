# Live steering firmware candidate v4

Derived from `traction_control_v4_2_live_steering_v3`. The Pi runner for this
candidate requires `firmware_build_id=v4_2_live_steering_v4`.

## Why v4

In the 2026-10-28 v3 floor trial, the camera saw the person 8.5–11.7 degrees
right of the optical axis. Five accepted DRIVING updates requested the maximum
+20 PWM rightward correction, but the cart turned about 1.5 degrees left.
The operator measured about 0.20 m on the floor; encoders reported right
0.2010 m, left 0.1939 m, center 0.1974 m and heading +0.0244 rad. The bounded
command ended `SUCCESS`, with READY, zero PWM and no latched fault. Evidence:
`v7_live_steered_forward_1793187350_8e0ba8d8/summary.json` on the Pi.

The v3 controller boosted the selected wheel but its cross-wheel speed and
distance synchronization still targeted equal travel. With the observed drive
asymmetry, the right wheel remained ahead even under maximum rightward bias.

## Change

For `/move-forward-steered` only, v4 accumulates a desired left-minus-right
encoder travel difference from each control-period center increment and the
latest command-bound steering bias. Full bias requests 0.18 m wheel difference
per metre of subsequent center travel, corresponding to about 7 degrees of
yaw over 0.20 m if applied throughout. The initial target acquisition still
starts with zero bias; the accumulated request starts only after later camera
feedback. When the target changes side, later increments use the new sign.

The existing cross-wheel target-speed and lagging-wheel PWM corrections now
track the accumulated desired difference instead of zero. The +20 PWM boost
on the requested wheel remains, as do the loaded sustain levels. No wheel is
commanded in reverse for forward steering. `/status` adds
`steered_desired_wheel_delta_m` for diagnosis.

The v3 center, wheel and heading limits, independent 300 ms motor guard,
700 ms camera-feedback watchdog, explicit `/stop` and command binding remain.
Other movement endpoints retain their prior behavior.

## Build and validation

Use ESP32 DevKit / ESP-WROOM-32 with the established wiring. Copy private
`wifi_credentials.h` into this sketch directory; do not commit it. Compile
with Espressif Arduino core 3.3.12 and `esp32:esp32:esp32`.

This is a diagnostic candidate. Compilation does not establish physical
steering accuracy. Verify v4, READY, zero PWM and no fault with `/status`
before an operator-observed bounded floor trial. Record the target bearing,
accepted steering updates, encoder wheel distances, actual floor travel and
turn direction. Canonical Measurement and autonomous motion admission remain
unchanged.
