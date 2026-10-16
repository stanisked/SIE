# Remote stop observed candidate v1

Status: CANDIDATE, not compiled for ESP32, not deployed or physically validated.
Build label: `v4_2_remote_stop_observed_v1`. The label identifies this source revision; it is not a binary checksum. Source and observer SHA-256 are recorded in `firmware/esp32/traction_control_v4_2_bounded_motion_api/remote_stop_candidate_manifest_v1.json`. Historical change_manifest.json records earlier evidence and does not qualify this candidate.

## Trigger and resulting behavior

Previously an HTTP stop during a bounded command set motor PWM to zero and returned terminal STOPPED immediately. This candidate asserts the existing TA6586 active brake in the handler, clears pending motion/square requests, leaves the command BRAKING and prevents normal drive updates until observation finishes. A repeated stop retains the original clock and observation. Fault acknowledgement during observation is refused without releasing the brake.

The observer retains the existing 400 ms brake hold, then takes encoder samples at least 100 ms apart. It requires two unchanged intervals after the hold; the earliest terminal result is therefore about 600 ms. Active braking remains asserted throughout observation. Terminal results record final encoder counts and require reobservation. Existing faults remain FAULT, never STOPPED. A 2 s observation deadline latches REMOTE_STOP_SETTLE_TIMEOUT; it does not release the brake or declare the vehicle stopped. Observation continues and the command remains busy until encoder quietness is seen, then ends FAULT. This deadline limits how long missing settlement goes unreported; it is not a physical stopping bound.

Encoder quietness cannot detect wheel slip, a blocked encoder or platform sliding. A physical stopping check on the loaded platform remains necessary. Neither the 25 mm nor the 0.5 s requested physical stopping bound is established. Protocol terminal observation can finish later than the physical stop.

## HTTP compatibility

During active/pending motion `/stop` now returns HTTP 202 with `accepted:true`, `stopped:false`, `stop_pending:true`. This is acknowledgement, not completion. Poll the existing command/status endpoint using its command ID and boot session. Never retry the movement POST. Idle stop returns HTTP 200 with basis NO_ACTIVE_MOTION; it makes no independent physical-rest claim.

Clients requiring HTTP 200 or immediate `stopped:true` must be updated/reviewed before deployment. Existing motion API version is retained; this stop-response revision is additive in the candidate manifest. No live client or motor integration is activated by this change.

`/status` adds firmware_build_id, remote_stop_pending, remote_stop_timed_out, remote_stop_observation_complete, elapsed time and start/observed encoder counts. These describe the most recent stop observation, not a new observation for every later motion.

## Verification and handoff

Run `python3 firmware/esp32/tests/test_remote_stop.py`. The host test compiles the actual handler/update functions with simulated I/O and the actual observer, checking delayed terminal, repeated stop, queued start/square cancellation, movement during settling, timeout, preserved faults and uint32 clock rollover. It is not an ESP32 SDK compile or electrical test.

Next: build this sketch with the matching ESP32 board/toolchain and both local headers (wifi_credentials.h and remote_stop_observer.h); review the stop client's asynchronous response handling; only then arrange an explicit deployment and loaded stopping observation. No flash or movement was performed by this change. Camera processing, calibration, admission thresholds and active range remain unchanged.

Read-only controller evidence supplied 2026-10-16: firmware traction_control_v4_2_bounded_motion_api, API v2_bounded_motion_api, boot session 94308DB022AB4747, READY, no active command/fault, zero PWM. This establishes the family, not an exact match to the previous source hash.

## Compile-only workflow and client review (2026-10-16)

The current `sie_core/supervised_bounded_executor.py` never sends `/stop`. It already recognizes STOPPED as terminal and polls the matching command ID within its boot session. A new injected-transport regression verifies that BRAKING (including an old unrelated STOPPED command) is ignored until the current command becomes STOPPED, followed by AWAIT_REOBSERVATION. No new automatic stop sender or motion permission has been added. An external/manual stop sender must treat HTTP 202 as acceptance and poll status; it must not interpret that response as completed braking.

Compile helper: `firmware/esp32/tools/build_remote_stop_candidate.py`. It checks candidate hashes, requires an explicit ESP32 FQBN, stages the two candidate source files and a private credentials header in a temporary directory, then calls only version/core-list/compile. It never uploads, opens a serial port or sends HTTP. It records the supplied FQBN, installed CLI/core metadata, source hashes and output binary hashes in build_report.json. Core installation/version selection remains explicit; no compiler/core is downloaded or silently upgraded.

For the generic ESP32 Dev Module definition, after confirming this matches the operator's installed board selection:

```bash
python3 firmware/esp32/tools/build_remote_stop_candidate.py \
  --fqbn esp32:esp32:esp32 \
  --output "$HOME/.local/state/sie/evidence/remote_stop_build_$(date +%Y%m%d_%H%M%S)"
```

The default credentials location is the sketch-local ignored wifi_credentials.h. `--credentials /absolute/private/header.h` selects another local header. Never publish that file or compile.local.log. The helper requires a fresh output directory and leaves the candidate sources unchanged.

Verification in this editing environment: 39 firmware source-contract assertion functions passed through a direct assertion runner; three Git-dependent checks were excluded because this working copy has no Git metadata. The current asynchronous STOPPED executor regression passed. The actual-handler C++ host simulation passed. Two build-wrapper unit tests passed with a fake compiler; this is not an ESP32 compile. Modified Python files passed py_compile. pytest and arduino-cli are unavailable here; an attempted real compile stopped at the missing-tool check before creating build output. The next operator step is toolchain inventory on the laptop used for ESP32 work, then the compile-only command above. No flash was performed.
