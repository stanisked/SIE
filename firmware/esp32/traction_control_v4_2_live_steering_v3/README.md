# Live steering firmware candidate v3

Versioned diagnostic candidate derived from `traction_control_v4_2_live_steering_v2`. The 2026-10-28 floor trial of v2 completed approximately 0.19 m of a 0.20 m request and visibly steered left, but latched `BOUNDED_DISTANCE_LIMIT` because the outer/right wheel crossed its straight-line 0.20 m cap. Two nonzero steer updates were acknowledged. A 0.463 s `GET /status` timeout preceded the hard-limit fault.

## Curved forward envelope

`/move-forward-steered` treats the requested distance as chassis-center arc travel. Normal predictive braking uses center progress. The diagnostic completion window for a 0.20 m request is 0.18–0.22 m of encoder-estimated center travel. The hard center overrun is 0.02 m, the outer-wheel allowance is 0.075 m beyond requested center distance, and the heading envelope is ±0.35 rad (about 20 degrees). For a 0.20 m command, each wheel's absolute cap is about 0.275 m. These bounds allow the observed v2 path (right 0.2049 m, left 0.1770 m, center 0.1910 m, heading 0.0963 rad) while still stopping on excessive center travel, wheel travel, or yaw. Other bounded endpoints retain their v2 limits.

If the steered command stops below the completion window without a fault, it reports `PARTIAL_PROGRESS` and requires new observation. It does not automatically continue.

## Independent motor guard

A higher-priority ESP32 task, separate from `WebServer.handleClient()`, monitors encoder hard limits and control-loop service. It electrically brakes and latches a fault when a steered command reaches its wheel, center, or heading cap or the control loop goes unserviced for more than 300 ms. Motor writes use a mutex; the main loop records the fault after it resumes. `/status` exposes `steered_safety_latched` and `steered_safety_reason`. The ordinary 700 ms steering-feedback watchdog, explicit `/stop`, encoder guard, and fault latch remain in place.

The matching Pi runner checks the v3 build and independent-guard status. Once it has confirmed `DRIVING`, it uses acknowledged steer updates without a redundant `/status` before every camera frame; it still polls for terminal state and confirms zero PWM after stop.

## Build and first validation

Use ESP32 DevKit / ESP-WROOM-32 with the existing wiring. Copy private `wifi_credentials.h` into this sketch directory; do not commit it. Compile with Espressif Arduino core 3.3.12 and `esp32:esp32:esp32`. Expected `firmware_build_id=v4_2_live_steering_v3`; `steering_api_version=v1_active_forward_pwm_bias`.

This candidate has compiled and passed dry-run Pi tests. The new concurrent motor guard has not been physically validated. First confirm zero PWM, no fault, and `steered_safety_latched=false` through `/status`. Perform one wheels-lifted bounded trial before one floor trial under operator observation. Do not promote this diagnostic to metric Measurement or autonomous motion admission.
