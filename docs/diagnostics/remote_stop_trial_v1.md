# One-command remote-stop diagnostic trial

This is a separately supervised drivetrain trial, not camera control or a new actuator qualification. It requests exactly one 0.10 m bounded forward action and one remote stop while the action is active. Requested distance is a controller bound, not an independently proven physical travel guarantee.

## Deployment evidence before this trial

The operator reported successful Arduino IDE compilation: 1,018,379 bytes Flash out of 1,310,720 (77%); 53,740 bytes global RAM out of 327,680 (16%). After upload, read-only status reported `firmware_build_id=v4_2_remote_stop_observed_v1`, boot session `2C0A9781CCBA9BD2`, READY, no latch, no active command and zero PWM. This confirms the reported build label is running, not an independently verified binary hash. Board FQBN, ESP32 core/compiler version and binary hash remain unrecorded. The source candidate manifest preserves its earlier predeployment status; this later operator evidence supersedes that status only, not its physical-validation limits.

## Execution

On the laptop or Pi on the controller network, use the current branch:

```bash
python3 firmware/esp32/tools/run_remote_stop_trial.py --execute
```

Default base URL is `http://192.168.0.17`; `--base-url` can select the explicit controller address. Omit `--execute` for read-only preflight. No dependency installation is needed. Before an executing run, enable motor power and place the platform on a clear straight floor section with its starting position marked. Keep direct access to motor power. The script does not replace an independent emergency stop.

The runner checks the exact expected build label, API, boot session, READY, no active command/latch/pending stop, zero PWM and no active square sequence. It creates a unique command ID and sends one bounded movement POST. It polls current status, requests stop when reported progress reaches 40 mm, or at 1 s from the move request if progress never reaches that trigger. The 40 mm trigger leaves room within the 100 mm request to exercise remote stop; it is not a newly approved stopping-distance threshold. The fallback avoids waiting indefinitely for progress. Both are recorded per trial.

Stop acknowledgement is not completion. The runner waits for the matching command's terminal state in the same boot session. An unrelated older STOPPED record is ignored. It does not retry movement or stop POSTs; transport ambiguity, read failure and Ctrl+C after the movement attempt cause one best-effort stop if none has been sent. If terminal observation is unavailable, it reports UNKNOWN rather than success. This cleanup cannot run after SIGKILL, power loss or a blocked/unreachable network, and does not establish a physical stopping bound. No automatic fault reset or subsequent movement occurs.

## Evidence and interpretation

Each run creates a new folder under `~/.local/state/sie/evidence/remote_stop_trial_*` with `http.jsonl` and `summary.json`. The trace stores request start/end host monotonic times, UTC, complete responses and errors. It preserves the pre-stop status, whether the controller was driving and whether encoder movement had occurred. Early natural target completion is explicitly not a remote-stop trial.

The encoder stop proxy uses the firmware's brake-onset and final observed counts, effective diameters right 0.0646 m / left 0.0652 m and counts/revolution right 205 / left 206. It reports each wheel and their mean in metres. Its datum is brake onset, not the earlier host stop request; network/handler latency precedes that datum. These are diagnostic wheel-odometry estimates, not a surveyed robot displacement or an SIE Measurement.

`remote_stop_elapsed_ms` covers the brake/quietness observation (earliest about 600 ms). Host stop-request-to-terminal time includes network and polling. Neither is the time until physical motion ceased. Independent physical post-request displacement and stop time remain null; do not conclude PASS/FAIL against the 25 mm / 0.5 s working requirements from these proxies. A single interrupted short step also does not establish a maximum-speed stopping bound.

Validation: eight offline unittest cases cover the normal asynchronous path, preflight refusal, read-only default, unknown movement/stop replies, status failure, interruption, reboot, observation timeout, natural completion, fault and progress fallback. No network/motor action was executed by these tests.
