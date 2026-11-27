# Step 75.3a — Continuous wheel encoder telemetry v1

**DECISION — OWNER_APPROVED, 2026-11-27.** Implement boot-scoped encoder
telemetry as a software-only diagnostic. This is the first part of Step 75.3.
The controller emits ticks and time. The future Pi adapter owns odometry
calculations and their validation. No Metric Measurement, World State, ROS
odometry, TF, or actuator admission follows from these samples.
[Step 75.2 closure and Step 75.3 decision](../handoffs/SIE_SPATIAL_NAVIGATION_HANDOFF_20261127.md#step-752-closure-and-step-753-owner-decision-2026-11-27)

## Source and dependency audit

**FACT.** Source baseline `909e1e3d676da146c89cc64081dc1327ec74f56a`.
The [v13 sketch](../../firmware/esp32/traction_control_v4_2_uart_shadow_v13_brake_continuity/traction_control_v4_2_uart_shadow_v13_brake_continuity.ino)
attaches one rising-A interrupt per wheel in `setup`. Each ISR reads its B
pin once and updates its one volatile signed 32-bit raw count. Both raw count
writers are those ISRs and `resetEncoderCounts`, called by `startMotion`.
The reset and several raw-count readers disable interrupts locally.
`readMotionCounts` first corrects forward polarity (right = −raw, left =
raw), then applies requested-motion signs. PID, encoder guarding, braking
trace, status and distance calculations consume the legacy counters or
their derived values. The independent task pinned to core 1 reads the
raw counters for wheel, center, heading, short-forward and turn limits.
It uses the existing turn diagnostic mux and motor-output mutex on their
respective paths. The legacy reads are retained as written. A volatile
pair and local interrupt masking are insufficient evidence for a coherent
cross-core diagnostic snapshot.

**FACT.** `loop → pollUart → handleUartFrame` handles requests before its next
motion service. Existing framing validates version, field count, upper-case
hex request ID and CRC before dispatch. `sendJson → sendUartReply` echoes the
request ID. The [host UART facade](../../vision_core/tools/uart_http_transport.py)
maps bounded routes, checks CRC and request ID, and does not retry. Existing
runner and probe firmware allowlists remain untouched. The [v15 derivative](../../firmware/esp32/traction_control_v4_2_uart_shadow_v15_cumulative_encoder_telemetry/traction_control_v4_2_uart_shadow_v15_cumulative_encoder_telemetry.ino)
forks v13 under build ID `v4_2_uart_shadow_v15_cumulative_encoder_telemetry`.
The four inherited headers are byte-identical. The sketch diff changes the
build ID, adds a diagnostic store to each existing ISR, and adds one read
dispatcher and its helpers. Full and compact replies keep their fields and
meaning, with the expected new build ID. The proposed brake diagnostic v14
retains its historical meaning.

**FACT.** A fresh local code-only Graphify extraction from the clean
`909e1e3` tree indexed 667 files (6,023 nodes and 18,368 edges).
Its `graph.json` SHA-256 was
`df32105f5d131f5dac9f73c0e432368f6cb8b23b4abe396664b25f862aeb4d9e`.
It indexed host `UartHttp` use by motion and diagnostic tools. It did not
index the sketch ISR symbol. The firmware findings are from tracked source
and the linked binary. The `db8097ea` Graphify snapshot remains historical.

## Counters and coherent sample

**FACT.** Each delivered GPIO callback updates its existing raw count
first using the unchanged B edge interpretation. The same B read updates
its independent signed 64-bit total once. Right adds +1 on B low and −1
on B high. Left adds +1 on B high and −1 on B low. A left pivot therefore
has right-positive and left-negative ticks. A right pivot has the opposite
signs. Updates are independent of request direction, controller state and
PWM. The totals start at zero on each boot and start counting when the
interrupts attach, before the first request and possibly before the existing
random 16-hex-character boot ID is generated. The first read can be nonzero.
No command or status request can zero these totals.

At either signed bound, an attempted step past the bound freezes that
wheel at the bound and latches its overflow flag for the boot. The other
wheel continues. The record is unsuitable for paired continuity after
either overflow flag. The sample sequence is unsigned 64-bit, starts at 1
on the first successful acquisition, and reaches its maximum once with
`sequence_exhausted=false`. Later reads retain the maximum and set
`sequence_exhausted=true`. Neither counter nor wire sequence wraps.
Reboot creates a distinct baseline identified by the existing boot ID.
The ID is not a persistent serial number or cryptographic proof of reboot.

**FACT.** The ESP32 Arduino core installs one GPIO interrupt service and
uses the same ISR dispatch path for both pins. That service serializes
the two callbacks. The cumulative totals are single-writer data. Only
32-bit atomic words are shared with the UART task. Each ISR brackets
the affected wheel's low/high count words and overflow flag with a shared
odd/even revision using sequentially consistent atomic stores. A successful
read captures the pair and calls `esp_timer_get_time()` between equal, even
revision reads. This places the sampled timestamp at the same logical
instant as the committed pair. An edge pending dispatch can appear in a
later sample. The timestamp is a signed 64-bit firmware monotonic time in
microseconds since timer initialization during boot, with 1 µs resolution.
It is a sampling timestamp. GPIO transition timing and host clock alignment
remain unknown.

The UART task makes at most four attempts. Each successful attempt also
requires ordered, nonnegative clock readings across an inclusive 1 ms
window spanning both revision reads. The internal unsigned 32-bit revision
wraps by design. Accepting a wrapped alias would require at least
2^31 dispatched callbacks within the 1 ms window. No successful window
returns a snapshot if its revision changed. An unavailable sample reports
status 503 and consumes no sequence. A lost or rejected response after
successful acquisition can instead leave a wire sequence gap.

**FACT.** No new ISR locks, waiting for readers, serial calls, logging,
memory allocation or retry loops are introduced. Format and transmission
run in the UART task after acquisition. The independent safety task has no
telemetry dependency and acquires no telemetry lock. The compiled ISR
arithmetic and atomic accesses add instructions. UART response work also
shares the loop with the motion state machine. Elapsed ISR cost, UART
traffic interference, GPIO edge loss, and loaded braking behavior require
hardware evidence.

## UART diagnostic interface

**FACT.** Read only, argument-free, and opt-in per request. No periodic
stream is enabled. UART2 is the existing 115200 baud, 8N1 connection.
CRC16 uses initial `0xFFFF`, polynomial `0x1021`, and covers the ASCII bytes
before the final pipe. Requests terminate with LF and firmware replies with
CRLF. The existing 255-byte request collector and 2,048-byte host reply
limit apply.

```text
@1|<8 uppercase hex request ID>|WHEEL_ENCODERS_V1|<4 uppercase hex CRC16>\n
@1|<same request ID>|200|<JSON>|<4 uppercase hex CRC16>\r\n
```

| JSON field | Encoding and meaning |
| --- | --- |
| `version` | Integer 1, the telemetry schema version. |
| `boot_session_id` | Existing 16-character uppercase hex boot identity. Verify firmware build with the existing `STATUS` when a future hardware session starts. |
| `sample_sequence` | Canonical decimal string, unsigned 64-bit, zero excluded. |
| `sample_monotonic_us` | Canonical decimal string, nonnegative signed 64-bit microseconds on the ESP32 clock. |
| `right_total_signed_ticks`, `left_total_signed_ticks` | Canonical signed 64-bit decimal strings, physical forward positive, one tick per delivered rising-A callback. |
| `controller_state`, `fault_latched` | Existing loop motion state name and bounded fault latch. The `READY` state includes stationary idle. A `FAULT` state and the bounded latch are separate pieces of evidence. |
| `steered_safety_latched` | Independent safety task's atomic latch as read by the loop when assembling the reply. It can change after the encoder timestamp. |
| `right_counter_overflow`, `left_counter_overflow`, `sequence_exhausted` | Boolean terminal invalidation details. Retain these rows as diagnostics. |

A 200 reply can carry terminal flags. Other replies include reason
`WHEEL_ENCODER_SNAPSHOT_UNAVAILABLE` (503), or
`WHEEL_ENCODER_FORMAT_ERROR` (500) if the length guard fails. Existing
silent rejection for malformed frames or CRC failures and existing
`INVALID_UART_OPERATION` for a valid frame with the wrong arguments are
preserved. `STATUS_COMPACT` retains version 1. Existing STOP, fault,
duplicate-command and motion routes retain their v13 behavior.

A request is 35 bytes. The longest schema body is 407 bytes, bounded by a
416-byte buffer including the terminator. Framing makes a worst-case
reply 430 bytes. Request plus reply uses up to about 40.4 ms on the wire
at 115200 baud with 10 bits per byte. Scheduling, parsing, turnaround,
buffering and timeout add separate delays. Use a single outstanding
diagnostic request during qualification and do not share UART ownership
with any motion runner.

**FACT.** The [offline Python codec](../../firmware/esp32/tools/wheel_encoder_telemetry_v1.py)
validates complete reply framing, CRC, status, matching ID, field presence,
exact integer and Boolean encodings, state and terminal flag consistency.
It opens no ports. The continuity tracker accepts a new boot as a new
baseline, refuses late records from retired boots, identifies identical
and conflicting duplicates, out-of-order sequences, reversed or equal
timestamps, and terminal conditions. A same-boot sequence decrease at the
numeric boundary is refused as rollover or reordering. A missing sequence
is reported as a gap, with the number missing. Cumulative net ticks can
remain available across a gap, while intermediate motion history stays
unknown. No pose integration occurs. Keep ESP32 time separate from
Raspberry Pi monotonic and ROS time.

## Software verification

**FACT — PASS.** Arduino CLI 1.6.0 from the installed Arduino IDE 2.3.11
AppImage, ESP32 Arduino core 3.3.12, Xtensa GCC 14.2.0
(`esp-14.2.0_20260121`). ESP32 Dev Module
(`esp32:esp32:esp32`), default board menu with 240 MHz `F_CPU`, 4 MB
flash, DIO at 80 MHz and default partition layout. Compile only.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --build-path /tmp/sie-step753a-arduino/v15 \
  firmware/esp32/traction_control_v4_2_uart_shadow_v15_cumulative_encoder_telemetry
```

The build used 517,795/1,310,720 bytes of program storage and
72,560/327,680 bytes of globals. The same toolchain built unmodified v13 at
515,967 program bytes and 72,480 global bytes. The v15 application binary
SHA-256 was
`7e666c121770651738b49f9a7f6fa97003f0dd1da31db16afd8b6662378b9ac0`.
In the linked ESP32 ELF, both ISRs were in IRAM (199 bytes each,
versus 39 in v13). Their one call remained the original GPIO read.
The checked 64-bit arithmetic was inlined. Atomic loads and stores compiled
to aligned word access and `memw` barriers, with no atomic-library or lock
calls in the added ISR path. The existing GPIO read placement and GPIO
interrupt configuration are inherited. Binary size is a static observation.
Elapsed times and worst-case edge service were not measured.

**FACT — PASS.** 33 focused tests passed. The host C++ harness compiled
selected production sketch functions and the production header with GCC
15.2.0 and UndefinedBehaviorSanitizer. GPIO, timer and UART were simulated.
Cases exercised physical signs in both directions and turns, pulses in
idle and transient states, local resets, multi-command continuity, a
concurrent writer and reader, forced contention, snapshot-window refusal,
wire validation and counter/sequence bounds. Python checks exercised
reboot, gaps, duplicates, ordering, timestamps, malformed fields, CRC and
framing. The static whole-sketch comparison accounts for every v15 edit
against v13. It covers legacy status/control code paths without relying
on the mocked legacy response bodies as proof of hardware equivalence.

**FACT — FAIL on both candidate and untouched baseline.** The broader
suite reported 242 passed, 114 subtests passed and three failed. The
clean `909e1e3` baseline reported 209 passed, 114 subtests passed and the
same three failed. They concern the older remote-stop build-wrapper
source hash, a legacy encoder guard order assertion, and a v4 shadow-turn
terminal result. The six standalone C++ replay, brake and fault-snapshot
regressions, Python compilation and whitespace checks passed.

```bash
python3 -m pytest -q firmware/esp32/tests/test_wheel_encoder_telemetry_v1.py
python3 -m pytest -q --tb=short firmware/esp32/tests \
  vision_core/tests/test_{run_shadow_uart,esp32_bounded_motion_contract,shadow_supervisor,live_steered_forward,probe_shadow_lifted_wheels,shadow_stop_distance_0p5,shadow_edge_turn_v4,shadow_edge_turn_v5,shadow_edge_turn_v6,shadow_dynamic_steer,shadow_short_forward_brake_v7,shadow_stale_pause_v8,shadow_early_pause_v8,shadow_delayed_brake_v8,shadow_feedback_pause_race_v8,shadow_partial_resume_v8,shadow_fault_report_v8,shadow_steer_heartbeat_v8,supervised_bounded_executor,supervised_static_target_mvp_regressions,metric_first_target_supervisor}.py
for test_file in firmware/esp32/tests/test_*.cpp; do
  g++ -std=c++17 -Wall -Wextra -Werror \
    -I firmware/esp32/traction_control_v4_2_uart_shadow_v13_brake_continuity \
    "$test_file" -o /tmp/sie-wheel-encoder-regression &&
    /tmp/sie-wheel-encoder-regression || exit 1
done
python3 -m py_compile firmware/esp32/{tools/wheel_encoder_telemetry_v1.py,tests/test_wheel_encoder_telemetry_v1.py}
git diff --check
```

**FACT — Step 75.3a software-only boundary.** That step involved no flash,
reboot, UART device access, fault acknowledgement, wiring change, or motor
test. The v13
`BOUNDED_DISTANCE_LIMIT / ENCODER_LIMIT_REACHED` fault remains open. Static
equivalence of the bounded motion code and a compatible build do not
qualify its runtime latency or braking.
[v13 fault evidence and limits](../handoffs/SIEE_PERSON_FOLLOW_V13_BRAKE_CONTINUITY_HANDOFF_20261123.md)

## Step 75.3b motor-off qualification plan

**PROCEDURE.** Step 75.3b results are recorded in the
[closure below](#step-753b-motor-off-qualification-closure). Before any
flash or reboot, obtain
physical confirmation from the operator that traction motor power is
disconnected independently of software PWM, STOP or fault status. Record
who checked it, when and what was disconnected. Keep motion launchers
stopped and verify exclusive diagnostic ownership of the existing UART.
If the serial adapter's open behavior could toggle reset, resolve that
before opening it.

1. Preserve the existing v13 evidence privately. From a read-only view of
   `/home/elwis/.local/state/sie/evidence/v13_right8_20261123_205406`,
   copy the full directory into a private archive and calculate a sorted
   per-file SHA-256 manifest. Verify those hashes against the copy, then
   verify the archive SHA-256 on a second storage location. Include
   `events.jsonl`, `fault_diagnostics.json`, `initial_status.json`,
   `turn_terminal_status.json`, and `final_status.json`. Capture the
   manifest, archive hash and verification result. Refuse any flash or
   reboot if preservation, checksum verification, or physical isolation
   is incomplete.
2. Obtain separate authorization before flashing the exact v15 candidate.
   Record build options, firmware SHA-256 and deployment evidence.
   Independently verify the reported build ID with the existing read-only
   `STATUS` request, then start a `WHEEL_ENCODERS_V1` session with distinct
   request IDs and one request in flight. The codec supplies the request
   bytes and offline validator. The existing motion transport is not the
   collector. Record the first decoded sample and its boot identity.
3. With the wheels stationary, acquire 20 consecutive read-only samples
   spaced at least 100 ms apart. Require a stable boot ID and both totals
   unchanged, with sequence increments of one and strictly increasing
   device timestamps. The spacing is a qualification procedure, not a
   rate or latency guarantee. Record actual host monotonic request and
   receipt times separately from ESP32 time.
4. Keep traction power physically disconnected. Slowly rotate one wheel
   manually in its physical forward direction, then reverse. Record
   the wheel, direction, start/end sample IDs and stops. Repeat for the
   other wheel. Require nonzero positive/negative total changes on the
   moved wheel respectively, and zero change on the stationary wheel.
   Acquire samples throughout and repeat the stationary series after the
   rotations. Slow manual rotation is sufficient for sign and accumulation
   evidence. Do not issue motion or fault commands to exercise states.
5. Reboot qualification requires separate authorization to reboot, with
   the same evidence and power checks in force. Hold both wheels still
   during reboot. Require a changed boot ID, initial sample sequence 1
   when the collector is the first reader, and a new baseline with no
   count or time difference across boots. Record any pulses before the
   first read as such. Without authorization, mark reboot qualification
   NOT RUN.
6. Keep `qualification.json` (operator confirmation, test order, build
   identity and hash), `uart_attempts.jsonl` (each raw request and response
   as hex, boot context, host timestamps, timeout or framing reason),
   `encoder_samples.jsonl` (decoded samples, continuity result and trial
   labels), and `SHA256SUMS.txt` in a private evidence directory. Include
   stationary and rotation deltas, missing sequences, refused reads and
   the operator's outcome. Refuse the affected trial for build or boot
   mismatch, failed power/evidence checks, unexpected wheel changes,
   missing or ambiguous direction, missing samples, invalid timestamps,
   CRC/framing failure, duplicate/out-of-order records, visible fault or
   terminal telemetry flags. Preserve the evidence for a refusal.

No oscilloscope is available. The procedure can qualify observed software
telemetry continuity in those motor-off conditions. It cannot establish
electrical transitions, ISR service latency, maximum loss-free pulse rate,
loaded scale or navigation accuracy, or braking performance.

## Step 75.3b motor-off qualification closure

**DECISION — OWNER_CONFIRMED.** Close Step 75.3b as
`WHEEL_ENCODER_TELEMETRY_QUALIFIED_MOTOR_OFF`. The closure covers the
motor-off stationary, manual polarity and independence, and boot boundary
qualification under the tested conditions.

### Deployment and evidence preservation

**FACT.** Deployed firmware
`v4_2_uart_shadow_v15_cumulative_encoder_telemetry` was built from
`abfdde4ada5de1fe92b88196e113de32615805b5`. Application SHA-256
`7e666c121770651738b49f9a7f6fa97003f0dd1da31db16afd8b6662378b9ac0`.
The upload log records verification of all four flash regions.
The operator confirmed physical traction power disconnection and verified
local and independent external v13 backups. The external v13 archive
SHA-256 is
`fb3f661f38cc5e0001635507d9e18e5cb3d6db8478c658df4b0f897a6f4c37cd`.

**FACT — operator-reported UART2 verification.** Read-only `STATUS`
verified v15 on deployment boot `3BBA9D5615754514`, with `READY`, PWM
0/0, null active command and fault, both fault and safety latches false,
and remote stop pending false.

### Qualification results

**FACT — operator-reported telemetry.** The five completed telemetry trials
are listed below. The initial stationary invocation had a boot-ID mismatch
and decoded zero samples.

| Trial | Decoded | Accepted | Result |
| --- | ---: | ---: | --- |
| Stationary | 20 | 20 | PASS |
| Manual rotation attempt 1 | 288 | 287 | FAIL |
| Manual rotation attempt 2 | 1,308 | 1,307 | FAIL |
| Manual rotation attempt 3 | 720 | 720 | PASS |
| Boot continuity boundary | 40 | 40 | PASS |
| **Total** | **2,376** | **2,374** | |

The stationary trial held totals at R = +2 and L = −23 ticks. The median
ESP32 sample interval was 100,998 µs, approximately 10 Hz under the
tested conditions.

| Failed manual trial | Stationary phase | Sequence | Measured delta, ticks |
| --- | --- | ---: | --- |
| Attempt 1 | `stationary_before_right_forward` | 308 | R +2 |
| Attempt 2 | `stationary_before_left_backward` | 1,616 | L −1 |

Both samples had valid consecutive telemetry. The nonzero delta during
the stationary phase tripped the phase contract and the collector refused
the sample. Both trials remain FAIL and their evidence is retained.

**FACT — operator explanation.** The owner confirmed accidental wheel
rotation or contact before the `GO` prompt in both failures. The telemetry
records the count changes and their phases. The physical cause is
operator-reported.

**FACT — successful manual rotation.** Right forward +214 ticks, right
backward −211 ticks, left forward +208 ticks, and left backward −208
ticks. The opposite wheel remained unchanged in each rotation. Both
20-sample stationary baselines passed. The trials qualify the observed
polarity and independent accumulation. Manual angular displacement was
not independently measured and the results justify no calibration change.

### Sequence coverage and boot boundary

| Boot session ID | Trial | Sequence range |
| --- | --- | --- |
| `BCC97F6E304A8459` | Stationary | 1–20 |
| `BCC97F6E304A8459` | Manual attempt 1 | 21–308 |
| `BCC97F6E304A8459` | Manual attempt 2 | 309–1616 |
| `BCC97F6E304A8459` | Manual attempt 3 | 1617–2336 |
| `BCC97F6E304A8459` | Pre-reboot | 2337–2356 |
| `E41B90553131A237` | Post-reboot | 1–20 |

**FACT — operator-reported boot boundary.** The authorized reset separated
the two qualification epochs. Pre-reboot stationary totals were R = 145,
L = 199 ticks. Post-reboot totals were R = 0, L = 0 ticks. The first new-boot
sample was classified as `REBOOT_BASELINE`. The production continuity
validator rejected offline replay of a preserved old-boot sample as
`RETIRED_BOOT`. No count or clock delta was computed across the reset.

The ranges are contiguous over the full recorded sequence space in each
epoch, including the two decoded but refused manual samples. Physical
motion between collector sessions remains unobserved.

**OPEN.** The earlier change from deployment boot `3BBA9D5615754514` to
`BCC97F6E304A8459` remains unexplained. The stationary collector refused
the mismatch before acquiring encoder samples. Two subsequent `STATUS`
checks confirmed the new boot before the stationary retry. The later
authorized reset established `E41B90553131A237` as a separate event.

### Private evidence index and limits

**FACT — operator-reported evidence.** The root is
`/home/elwis/.local/state/sie/evidence/`. The operator reported SHA-256
verification PASS for all five completed telemetry trial evidence sets,
including the failed manual trials. The private Pi artifacts were not
independently inspected for this documentation update.

| Subdirectory | Record |
| --- | --- |
| `step75_3b_2_status.iTKYlv` | Deployment UART2 STATUS |
| `step75_3b_3_stationary.h5fmrlc5` | Initial stationary invocation refused on boot ID mismatch |
| `step75_3b_2_status.AISJ6J` | STATUS recheck of `BCC97F6E304A8459` |
| `step75_3b_2_status.1C8TPq` | Independent STATUS recheck of `BCC97F6E304A8459` |
| `step75_3b_3_stationary._mbx2u8y` | Stationary PASS |
| `step75_3b_4_manual.n_ay9tu1` | Manual attempt 1 FAIL |
| `step75_3b_4_manual.r9kvpag4` | Manual attempt 2 FAIL |
| `step75_3b_4_manual.sixarbde` | Manual attempt 3 PASS |
| `step75_3b_5_boot.57bhi8pv` | Boot continuity boundary PASS |

**OPEN.** Loaded wheel scale, odometry accuracy, pulse-loss limits,
braking performance, and navigation safety remain unverified. The v13
`BOUNDED_DISTANCE_LIMIT / ENCODER_LIMIT_REACHED` fault remains open.
The closure does not authorize powered motion or change calibration,
Measurement, World State, or execution admission.
