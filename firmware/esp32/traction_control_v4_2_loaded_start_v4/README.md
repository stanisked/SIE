# Loaded micro-turn start candidate v4

This is an isolated, uncompiled firmware candidate for the observed three-degree turn failure under floor load. The deployed v3 sketch and supervised approach runner remain unchanged.

The marked-wheel stand test showed right forward and left backward movement, with terminal partial progress. The earlier floor turn latched BREAKAWAY_NOT_FOUND: the left wheel had not broken away. In v3 a two-count brake start caps startup PWM at 190; the source records loaded breakaway around 215–225.

This candidate brakes each wheel at its own brake-start encoder count while still in STARTING. Only after one wheel has been braked may the other wheel ramp to the existing 235 PWM cap. A transition to DRIVING preserves the individual brake. The bounded per-wheel encoder limit, 450 ms startup timeout, stop/brake settle, and fault latch remain in force. These changes are hypotheses pending compilation and physical validation. Partial progress is possible and requires a fresh observation.

## Compile on the computer with the ESP32 toolchain

Use the installed board FQBN for the ESP32 DevKit. Keep the private Wi-Fi header on the computer and pass its path; do not commit it.

```bash
python3 firmware/esp32/tools/build_loaded_turn_candidate.py \
  --fqbn esp32:esp32:esp32 \
  --credentials /path/to/private/wifi_credentials.h \
  --output /path/to/new/build-output
```

The script invokes arduino-cli compile only, never upload. It writes build_report.json and a local compiler log. If using Arduino IDE, open traction_control_v4_2_loaded_start_v4.ino in its own sketch folder, copy the private header into that folder locally, select the installed ESP32 board, and use Verify. The compiler result is still needed before any flashing decision.

After a successful compile, compare the build ID and source SHA-256 in the report with this candidate. Only then consider a wheels-lifted test using the separate one-shot diagnostic. The person-approach runner does not accept build v4 yet. Do not resume person approach based on compilation alone.
