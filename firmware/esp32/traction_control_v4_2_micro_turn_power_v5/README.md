# Bounded micro-turn startup-power candidate v5

The v4 floor tests established a physical left turn for requested 5° and 6° commands, but a 3° command only jerked before the two-count brake point. The 5° result used breakaway PWM 200/205 and settled at 6/7 counts from 13/13; the 6° result used 210/215 and settled at 10/9 from 16/16. These readings support raising the initial micro-turn PWM.

This isolated candidate starts bounded micro-turns at PWM 210, ramps at the existing 5 PWM per 20 ms, and caps the initial ramp at 230. Once one wheel is individually braked, the lagging wheel retains the existing 235 cap. A wheel that has already broken away keeps at least its observed breakaway PWM until the other wheel starts. Forward-motion launch remains at PWM 180. The 450 ms startup timeout, per-wheel encoder cutoff, early individual brake, active-brake settle, fault latch, and remote-stop code are unchanged.

**This candidate does not change the short-turn brake point.** A requested 3° turn still begins individual braking at roughly two encoder counts; higher power alone may not produce useful chassis yaw. The correct next evidence is compilation, then one wheels-lifted 3° test, followed by a separately reviewed single floor 3° test if the stand outcome is sound. Do not use this candidate for person approach.

On a computer with arduino-cli, compile without upload using:

```bash
python3 firmware/esp32/tools/build_loaded_turn_power_candidate.py \
  --fqbn esp32:esp32:esp32 \
  --credentials /path/to/private/wifi_credentials.h \
  --output /path/to/new/build-output
```

For Arduino IDE, open `traction_control_v4_2_micro_turn_power_v5.ino` in its own sketch folder, place the private `wifi_credentials.h` there locally, select the installed ESP32 board and use Verify. The helper writes a local build report and never uploads. After an explicit flash, verify `firmware_build_id` from live `/status` before any motor test. The approach runner and floor diagnostic still refuse v5.
