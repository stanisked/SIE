#!/usr/bin/env bash
set -eo pipefail
cd /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec /home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/run_v7_supervised_one_step.py "$@"
