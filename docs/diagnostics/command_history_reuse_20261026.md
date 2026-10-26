# ESP32 bounded command history after the five-step camera run

The 2026-10-26 supervised approach stopped when `/move-forward` returned HTTP
409 `COMMAND_HISTORY_FULL` on the fifth step. The preceding four movements were
observed, and the controller later reported `READY`, zero PWM and no active
motion. The refusal came from a 16-record history that never reused completed
entries. It is distinct from a camera-depth refusal or failed remote brake.

`v4_2_reusable_history_v1` keeps the latest completed command available in
`/status`, never replaces an active command, and reuses only an older terminal
record. A separate 4096-entry, 64-bit fingerprint list remembers every accepted
ID in the current boot session, including IDs whose detailed records were
reused. A repeated retained ID gets the existing duplicate/conflict response;
an older ID is refused with `STALE_COMMAND_ID`. A hash collision can refuse a
new ID but cannot allow a previously accepted ID to execute again. When the
4096-entry list fills, further movement is refused with
`COMMAND_ID_HISTORY_EXHAUSTED`. The fixed list consumes 32 KiB of global RAM.
Reboot creates a new session ID, so old requests are refused by the existing
boot-session check. Neither refusal triggers movement.

`test_command_replay_guard.cpp` is a host-only test: it fills all 4096 entries,
checks old-ID refusal, checks capacity refusal, and checks slot selection with
the active and last-completed slots protected. Compile and run without motors:

```bash
g++ -std=c++11 -Wall -Wextra -Werror -O2 \
  -I firmware/esp32/traction_control_v4_2_bounded_motion_api \
  firmware/esp32/tests/test_command_replay_guard.cpp -o /tmp/sie_replay_test
/tmp/sie_replay_test
python3 -m unittest firmware.esp32.tests.test_remote_stop_trial
```

The host test is not an ESP32 build. Compile the current sketch in Arduino IDE
with the existing private `wifi_credentials.h` and inspect Flash/RAM use before
uploading. The old `build_remote_stop_candidate.py` verifies a frozen historical
manifest and is not the build path for this revision. After upload, a read-only
`GET /status` should report `firmware_build_id=v4_2_reusable_history_v1`, a new
`boot_session_id`, `state=READY`, and zero PWM. The Pi and ESP32 must be on a
reachable network; the controller's current LAN address is `10.13.32.229`.
Physical motion requires a separate operator-controlled trial and observation.
