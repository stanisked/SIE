# SIE ROS 2 phase 1

This package layers ROS 2 transport on top of SIE Data Contracts.

## Scope

`perception_contract_node` accepts a canonical JSON Measurement on
`/sie/perception/raw_measurement`, validates it, and publishes it to
`/sie/perception/measurement`.

`navigation_contract_node` consumes only that validated Measurement and emits
a non-executing recommendation on `/sie/navigation/decision`.

`supervisor_contract_node` publishes the final phase-1 state on
`/sie/supervisor/state`. It has no actuator client, serial port access, HTTP
transport, or motor command path. Every output declares
`motor_command_performed: false`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/SIE
colcon build --base-paths ros2
source install/setup.bash
ros2 launch sie_ros2 sie_pipeline.launch.py \
  config:=$PWD/ros2/sie_ros2/config/sie_pipeline.yaml
```

The first runtime adapter will publish the existing JSONL evidence as
`/sie/perception/raw_measurement`. It is intentionally not included in phase 1:
the legacy fusion runtime still has laptop-specific paths and an archived AR0234
v3 binding which must be refactored to v5 before being connected.
