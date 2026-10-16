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

The runner checks the exact expected build label, API, boot session, READY, no active command/latch/pending stop, zero PWM and no active square sequence. It creates a unique command ID and sends one bounded movement POST. It schedules stop for 300 ms from the host movement POST start. After movement acceptance it waits only until that monotonic deadline, sends /stop, then starts status polling. No /status request can block the stop dispatch window. If acceptance itself arrives late, stop is sent immediately and lateness is recorded. The 300 ms value is a diagnostic interruption schedule chosen from the preceding short-step trace, not a physical stop-time bound or a camera freshness threshold. Host scheduling and HTTP delivery still have uncertainty.

Stop acknowledgement is not completion. The runner waits for the matching command's terminal state in the same boot session. An unrelated older STOPPED record is ignored. It does not retry movement or stop POSTs; transport ambiguity, read failure and Ctrl+C after the movement attempt cause one best-effort stop if none has been sent. If terminal observation is unavailable, it reports UNKNOWN rather than success. This cleanup cannot run after SIGKILL, power loss or a blocked/unreachable network, and does not establish a physical stopping bound. No automatic fault reset or subsequent movement occurs.

## Evidence and interpretation

Each run creates a new folder under `~/.local/state/sie/evidence/remote_stop_trial_*` with `http.jsonl` and `summary.json`. The trace stores request start/end host monotonic times, UTC, complete responses and errors. It preserves the movement POST start, scheduled stop deadline, actual stop dispatch time and lateness. No pre-stop drive state is claimed without a status observation. Both wheels must have positive firmware brake-onset counts to classify the result as a moving remote-stop diagnostic; otherwise it records STOP_OBSERVED_WITHOUT_BOTH_WHEEL_PROGRESS. Early natural target completion is explicitly not a remote-stop trial.

The encoder stop proxy uses the firmware's brake-onset and final observed counts, effective diameters right 0.0646 m / left 0.0652 m and counts/revolution right 205 / left 206. It reports each wheel and their mean in metres. Its datum is brake onset, not the earlier host stop request; network/handler latency precedes that datum. These are diagnostic wheel-odometry estimates, not a surveyed robot displacement or an SIE Measurement.

`remote_stop_elapsed_ms` covers the brake/quietness observation (earliest about 600 ms). Host stop-request-to-terminal time includes network and polling. Neither is the time until physical motion ceased. Independent physical post-request displacement and stop time remain null; do not conclude PASS/FAIL against the 25 mm / 0.5 s working requirements from these proxies. A single interrupted short step also does not establish a maximum-speed stopping bound.

Validation: ten offline unittest cases cover the normal asynchronous path, preflight refusal, read-only default, unknown movement/stop replies, status failure, interruption, reboot, observation timeout, natural completion, fault, slow status responses, late movement acceptance and missing prior wheel progress. No network/motor action was executed by these tests.

## First floor trial and runner correction

Trial `remote_stop_trial_20261016_180046_220e99d6` used the initial progress-poll runner. The operator observed a normal start and approximately 0.10 m travel followed by a stop. Controller mean encoder progress was 0.0952 m. The left final count 94 was one below the configured lower success bound 95; right 98 was within its 96..102 window. Therefore the command ended FAULT/BOUNDED_TARGET_NOT_REACHED. No tolerance was changed.

The trace showed progress/status replies of 4 mm after a 205 ms request, 38.7 mm after 197 ms, then 95.2 mm/BRAKING after 479 ms. Only then was /stop sent; it returned NO_ACTIVE_MOTION in 35 ms. Remote stop observation was never started. This is an unexercised remote-stop path, not evidence that remote braking failed. The original summary/http files remain unchanged. Their hashes and selected observations are in `remote_stop_trial_20261016_first_review.json` beside this document.

The initial runner's decision depended on delayed status replies. The corrected deadline mode removes those intermediate GETs instead of expanding distance or relaxing tolerances. It is an experiment runner correction, not a fix for the measured HTTP status latency in a future camera-driven loop. No firmware reflash is required. The latched fault must be acknowledged explicitly while the robot is stationary before a new trial; the runner never acknowledges it automatically.
