# ROS 2 supervised bounded-forward MVP

**Recorded:** 2026-09-29  
**Branch:** `codex/sie-ros2-integration-v1`  
**Target:** SIE mobile trolley on Raspberry Pi 5. This is not the SO-101
manipulator path.

## Activated scope

The ROS 2 supervisor can run a separately launched,
operator-confirmed `SUPERVISED_PERSON_APPROACH_SESSION`. The ordinary
`sie_pipeline.launch.py` remains phase-1 disabled.

The supervised profile authorizes only:

- adapter `esp32_zk5ad_sgm37_520`;
- ESP32 `v2_bounded_motion_api` at `http://192.168.0.17`;
- one non-retried `POST /move-forward` with `distance_m=0.1`;
- at most three steps per launched session;
- a fresh `FORWARD_REOBSERVE` decision no older than 1.0 s;
- a new post-terminal Observation before every subsequent step.

It never authorizes turns, correction pulses, POST retries, automatic
capability qualification, or dynamic fusion while moving.

## Preserved evidence

The profile records the prior physical result: a 0.10 m request produced
approximately 0.0938 m mean progress, 0.0940 m right, 0.0935 m left, and
0.002 rad heading. A three-step supervised session completed and was stopped
by `YOLO_MULTIPLE_TARGETS`, not an ESP32 or network failure.

This remains supervised-session evidence. The ROS navigation Decision retains
`execution_authorized=false`; the separate operator confirmation authorizes
the bounded supervisor session without mutating capability qualification.

## Runtime state machine

1. Validate the navigation Decision contract.
2. Refuse duplicate, stale/future, non-forward, blocked, low-confidence,
   unavailable-depth, or turn decisions.
3. Require ESP32 `READY`, no bounded fault, no active command, and the
   expected motion API version.
4. Build one deterministic command ID and issue one POST through the existing
   bounded executor.
5. Never retry the POST.
6. Accept only terminal `SUCCESS` or `PARTIAL_PROGRESS` with no bounded
   fault.
7. Require a new Measurement captured after the terminal result.
8. Latch any unknown/faulted executor outcome and refuse the rest of the
   session.

Every supervisor state is appended to:

`~/.local/state/sie/streams/supervised_bounded_forward_mvp.jsonl`

## Launch boundary

Disabled pipeline:

`ros2 launch sie_ros2 sie_pipeline.launch.py`

Supervised actuator pipeline:

`ros2 launch sie_ros2 sie_pipeline_supervised_mvp.launch.py ...`

The supervised launch requires the absolute repository root, the exact
source-controlled execution profile, the ESP32 base URL, and the literal
operator session confirmation. Merely building or sourcing the package cannot
move the trolley.
