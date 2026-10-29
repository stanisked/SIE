# Shadow UART candidate, 2026-10-29

The separate `v4_2_uart_shadow_v1_candidate` image is an operator-supervised engineering candidate. The Raspberry Pi runner deliberately has no public motor `--execute` route. It requires a fresh associated RGB target, calibrated range from the robot front, and target continuity information that the current camera preview does not supply. This checkpoint does not authorize autonomous floor following or canonical Measurement.

The protocol keeps one `command_id` across bounded segments. A target loss or 700 ms steering feedback gap brakes to `PAUSED` with zero PWM; a sequenced `SHADOW_RESUME` on that same ID starts a further bounded segment. An explicit Shadow STOP ends the session. Terminal FAULT remains terminal. Encoder envelope and watchdog protections remain active. The provisional front range gate holds at 0.8 m and resumes above 0.95 m, without a measured physical stopping bound. A future associated LiDAR range can serve as fallback when OV9281 depth is unavailable; raw optical Z or unassociated range cannot.

Hardware evidence: with motors unpowered, the probe confirmed same-ID PAUSE/RESUME and READY. An early lifted-wheels attempt while motor power was off produced `BREAKAWAY_NOT_FOUND`. With motors powered and the wheels lifted, two approximately 0.02 m segments completed under one ID; both wheels stopped during PAUSE, rotated after RESUME, encoder drift while paused was zero, and STOP was confirmed. No Shadow floor trial has occurred.

The sketch is at `firmware/esp32/traction_control_v4_2_uart_shadow_v1/`. The UART facade, supervisor, range gate, runner and probes are in `vision_core/tools/`. Their host tests use an injected controller. The flashed hardware result is stronger evidence for PAUSE/RESUME than the host tests, but it does not validate person tracking, front clearance or an emergency floor stop.

Next: capture synchronized mask and stereo diagnostics on moving/lost/reacquired target frames, establish range from the robot front with age and association checks, then review the runner input and run a supervised floor trial with a physical cutoff and both wheels observed.
