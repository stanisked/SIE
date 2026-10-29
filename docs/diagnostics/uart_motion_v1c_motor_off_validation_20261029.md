# UART motion v1c: motor-off validation (2026-10-29)

Status: supervised diagnostic checkpoint. This record does not authorize floor motion, canonical Measurement, or autonomous operation.

## Identity and setup

- ESP32 USB boot log showed `FIRMWARE_BUILD_ID=v4_2_uart_motion_v1c`, `UART_ONLY_NETWORK_NOT_STARTED`, and `SIE_UART_READY`. The supplied capture showed one `POWERON_RESET`, without a repeated reset.
- Pi used `/dev/ttyAMA0` over the verified three-wire GPIO UART link at 115200 baud. User `elwis` was added to `dialout` after an initial permission refusal.
- Motor power was physically disconnected for the direct valid-command test; the operator confirmed the cart stayed stationary. The camera-runner test was instructed with motor power off. No independent physical observation for that second run was supplied.
- Active stereo calibration remains OV9281 Pi v7, SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`. No calibration or Measurement admission changed.

## UART evidence

| Check | Observed result |
| --- | --- |
| Initial STATUS | HTTP 200; v1c build; boot session `27F906E8AAFBC92F`; `READY`; no active command or fault; PWM 0/0. |
| Idle probe | 100/100 STATUS, median 61.94 ms, max 61.97 ms; wrong-boot MOVE rejected; final `READY`. |
| During 45 s camera-only preview | Operator confirmed overlap; 100/100 STATUS, median 61.93 ms, max 68.25 ms; same boot session, wrong-boot MOVE rejected, final `READY`. Preview had 186 frames, zero journal drops/errors, and 186 STOP decisions. |
| Direct valid MOVE then STOP, motors off | One 0.020 m MOVE accepted (202), STOP accepted pending (202); final STATUS `READY`, last command `STOPPED`, no active command, no fault, PWM 0/0, encoder distance 0. The operator confirmed no cart motion. |
| Camera-driven UART runner, motors instructed off | One 0.020 m MOVE accepted; three steering requests: `+4` refused while STARTING, zero bias with seq 1 accepted, next `+4` refused after breakaway fault. Runner sent STOP and polled until terminal `FAULT`, no active command, `remote_stop_pending=false`, PWM 0/0. Result `TERMINAL_FAULT_STOPPED`, reason `BREAKAWAY_NOT_FOUND`. No nonzero steering update was accepted. |
| Fault recovery | One EN reset with motor power instructed off changed boot session to `89B3128C692078E3`. STATUS returned 200, `READY`, no active command or fault, fault latch false, PWM 0/0. |

Runner evidence on Pi: `/home/elwis/.local/state/sie/evidence/v7_live_steered_forward_1793268841_a930bcb3/summary.json` and `http.jsonl`. The operator supplied both to the review. The journal includes the complete request/response order. The v1c source calls `abortMotion("BREAKAWAY_NOT_FOUND")` when breakaway is not confirmed during STARTING; the zero encoder result with motors off is consistent with this fault. The cause of earlier Wi-Fi timeouts remains unresolved.

## Code checkpoint

- `uart_http_transport.py` and `probe_uart_motion_off.py` on the Pi matched the packaged v1c source byte-for-byte, respectively SHA-256 `8ab7ca5dffd319bfc5914417ec35a0ba57bc68aed43c28bf73d2e81f14155d86` and `c27d8c9307324e9c203cde20464bd772e7de2a3157d500dc4706650a02100bd6`.
- The runner patch SHA-256 is `f527effa9520023350b7260e19ecbee0e6396de2297013571ef37c3a420a3177`. It selects `UartHttp` with `--serial-port`, checks exact v1c build ID, and retains the existing Wi-Fi v4 mode and stop logic.
- Local `git apply --check`, Python compilation, and 19 runner tests passed; the operator applied the patch on the Pi and reported 19 tests passing there. The real runner evidence above is the first motor-off hardware check of the patched path.
- Expected hashes of the patched runner and test from the reviewed source are `233e8f7aed318461e1cc6971b2ffdc62cc9916330ecbf1d8f42cf29ae0e465df` and `475b8ef4aa8b0d9483bb86e225f41662e295b90a7006107ef1fb9d039a975eb7`. Verify these on the Pi before the source-control checkpoint.

## Next gate

Commit only the runner, its focused test, the matching v1c UART transport and probe, and this handoff. Exclude `probe_uart_motion_off.py.bak`, build outputs, and unrelated untracked work. With the motor power cutoff accessible and the wheels securely raised, perform one supervised, bounded 0.020 m UART trial to observe actual encoder motion and a nonzero steering acknowledgement. Read the full journal and final STATUS before any floor test. A raised-wheel result does not establish ground steering direction, stopping distance, metric Measurement, or autonomous operation. Do not repeat on `FAULT` or an unconfirmed STOP; inspect the evidence and restore `READY` with motor power off first.
