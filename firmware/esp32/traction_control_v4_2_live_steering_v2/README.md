# Live steering firmware candidate v2

Versioned diagnostic candidate derived from `traction_control_v4_2_live_steering_v1`. The v1 physical trial on 2026-10-28 failed with `BREAKAWAY_NOT_FOUND` during a 0.20 m request. This v2 build is not physically validated.

## Change from v1

For `/move-forward-steered` only, cap the bounded breakaway detection threshold at the existing 15 encoder counts that represent confirmed translation. On the 0.20 m trial, the old formula produced about 95 counts, so the 450 ms startup deadline expired before both wheels crossed it. At the deadline, the recorded wheel counts were 100 right and 90 left. The other bounded endpoints retain their prior threshold and startup behavior. At the deadline, the steered endpoint checks the latest encoder sample before declaring `BREAKAWAY_NOT_FOUND`.

The API and steering feedback watchdog remain the same as v1: `/move-forward-steered` accepts 0.02–0.30 m, `/steer-update` accepts command-bound sequence and ±20 PWM bias, and absent updates stop directed drive after 700 ms. The first accepted target starts with zero bias; only a later fresh frame may steer.

## Build and deployment

Use ESP32 DevKit / ESP-WROOM-32 with the same wiring and board configuration as the loaded-start v4 controller. Copy the private `wifi_credentials.h` into this sketch directory; never commit or publish it. Compile with Espressif Arduino core 3.3.12, board `esp32:esp32:esp32`. The expected `firmware_build_id` is `v4_2_live_steering_v2`; `steering_api_version` remains `v1_active_forward_pwm_bias`.

The Pi runner `vision_core/tools/run_v7_live_steered_forward.sh` must be updated to the matching source commit before any new motor trial. Confirm `/status` reports the exact build, `READY`, no fault latch, no active command and both PWM values zero. A later supervised trial needs a newly measured clear approach corridor and operator-accessible motor power cutoff.

This is still a diagnostic controller. It does not admit metric Measurement or autonomous motion. Physical steering and stopping behavior of v2 have not yet been established.
