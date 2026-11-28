#!/usr/bin/env bash
# Prepared for a separately authorized, stationary motor-off Step 75.5b run.
set -eo pipefail
umask 077
mode="${1:-}"
expected_boot="${2:-}"
if [[ "$mode" != status && "$mode" != live ]] ||
   [[ "$mode" == live && ! "$expected_boot" =~ ^[0-9A-F]{16}$ ]]; then
  echo 'Usage: live-on-pi.sh status | live BOOT_ID' >&2
  exit 2
fi
audit_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage="$(cd -- "$audit_dir/../../.." && pwd)"
source_checkout=/home/elwis/dev_ws/sie_step75_5a_baseline
baseline=2a00bd80348e8d10d4577e886f354d30e24057c1
test "$(git -C "$source_checkout" rev-parse HEAD)" = "$baseline"
test -z "$(git -C "$source_checkout" status --porcelain)"
source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$source_checkout/ros2/sie_ros2:$source_checkout${PYTHONPATH:+:$PYTHONPATH}"
read -r -p 'For this authorized step, confirm traction power physically disconnected, wheels stationary and motion runners stopped. Type MOTOR-OFF: ' confirmation
test "$confirmation" = MOTOR-OFF

if systemctl is-active --quiet serial-getty@ttyAMA0.service; then
  printf 'ESP32 UART serial getty is active\n' >&2
  exit 1
fi

if [[ "$mode" == status ]]; then
  for runner in run_shadow_uart supervised_bounded_executor live_steered_forward shadow_supervisor shadow_edge_turn; do
    rc=0
    pgrep -f "$runner" >/dev/null || rc=$?
    if (( rc != 1 )); then
      printf 'Motion runner check refused (%s, exit %s)\n' "$runner" "$rc" >&2
      exit 1
    fi
  done
  python3 - "$source_checkout" "$stage" <<'PY'
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import yaml
from sie_ros2.wheel_odometry_uart import ReadOnlyUart, open_uart

source, evidence = map(Path, sys.argv[1:])
configuration = yaml.safe_load((source / "ros2/sie_ros2/config/wheel_odometry_v1.yaml").read_text())
timeout = configuration["sie_wheel_odometry"]["ros__parameters"]["uart_timeout_s"]
attempt = status = None
try:
    with open_uart("/dev/ttyAMA0", Path.home() / ".local/state/sie") as fd:
        uart = ReadOnlyUart(fd, timeout, time.time_ns)
        attempt = uart.make_attempt(status=True)
        status = uart.exchange(attempt)
    expected = {
        "state": "READY", "right_pwm": 0, "left_pwm": 0, "active_command_id": None,
        "fault": None, "bounded_fault_reason": None, "steered_safety_latched": False,
        "bounded_fault_latched": False, "remote_stop_pending": False,
    }
    mismatches = [key for key, value in expected.items()
                  if key not in status or type(status[key]) is not type(value) or status[key] != value]
    if mismatches:
        raise SystemExit("STATUS_MISMATCH:" + ",".join(mismatches))
    print("Verified current ESP32 boot ID:", status["boot_session_id"])
    print("Review the boot ID before passing it to the live launcher.")
finally:
    path = evidence / "current_boot_status.json"
    path.write_text(json.dumps({
        "attempt": asdict(attempt) if attempt is not None else None,
        "status": status,
    }, indent=2) + "\n")
    print("Read-only STATUS evidence:", path)
PY
  exit
fi

export ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_NAMESPACE
for key in CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; do
  if [[ -n "${!key}" ]]; then
    printf 'Custom DDS profile requires review: %s\n' "$key" >&2
    exit 1
  fi
done
export ROS_LOG_DIR="$stage/ros_log"
dpkg-query -W ros-jazzy-rclpy ros-jazzy-rclcpp ros-jazzy-slam-toolbox | tee "$stage/packages.txt"
test "$(ros2 pkg prefix slam_toolbox)" = /opt/ros/jazzy
version="$(dpkg-query -W -f='${Version}' ros-jazzy-slam-toolbox)"
if [[ "$version" != 2.8.5-* ]]; then
  printf 'Jazzy SLAM Toolbox package requires review: %s\n' "$version" >&2
  exit 1
fi
graph="$(timeout 8s ros2 node list --no-daemon --spin-time 2 -a)"
if [[ -n "$graph" ]]; then
  printf 'ROS domain 178 is occupied:\n%s\n' "$graph" >&2
  exit 1
fi
port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
test -c "$port" && test -r "$port" && test -w "$port"
test "$(readlink -e "$port")" != "$(readlink -e /dev/ttyAMA0)"
python3 - "$port" <<'PY'
from pathlib import Path
import sys

from sie_ros2.wheel_odometry_uart import owners

for port in [Path("/dev/ttyAMA0"), Path(sys.argv[1])]:
    pids = owners(port.resolve(strict=True))
    if pids:
        raise SystemExit(f"DEVICE_OWNED:{port}:{sorted(pids)}")
PY
ld19_executable="$(
  source "$HOME/siee_lidar_ws/install/local_setup.bash"
  printf '%s/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node\n' "$(ros2 pkg prefix ldlidar_stl_ros2)"
)"
test -x "$ld19_executable"
cmake -S "$audit_dir" -B "$stage/build" 2>&1 | tee "$stage/tf-audit-configure.log"
cmake --build "$stage/build" --parallel 2 2>&1 | tee "$stage/tf-audit-build.log"

(
  source "$HOME/siee_lidar_ws/install/local_setup.bash"
  exec "$ld19_executable" --ros-args -r __node:=siee_ld19 -p product_name:=LDLiDAR_LD19 \
    -p "port_name:=$port" -p port_baudrate:=230400 \
    -p topic_name:=/sie/lidar/scan_raw -p frame_id:=ld19_sensor \
    -p laser_scan_dir:=true -p enable_angle_crop_func:=false -p use_sim_time:=false
) > "$stage/ld19.log" 2>&1 &
ld19_pid=$!
cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if kill -0 "$ld19_pid" 2>/dev/null; then
    kill -INT "$ld19_pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$ld19_pid" 2>/dev/null || break
      sleep .2
    done
    if kill -0 "$ld19_pid" 2>/dev/null; then
      printf 'LD19 cleanup escalated to SIGTERM\n'
      kill -TERM "$ld19_pid" 2>/dev/null || true
      for _ in {1..10}; do
        kill -0 "$ld19_pid" 2>/dev/null || break
        sleep .2
      done
      if kill -0 "$ld19_pid" 2>/dev/null; then
        printf 'LD19 cleanup escalated to SIGKILL\n'
        kill -KILL "$ld19_pid" 2>/dev/null || true
      fi
      code=1
    fi
  else
    printf 'LD19 exited before requested cleanup\n'
    code=1
  fi
  wait "$ld19_pid" 2>/dev/null || true
  if ! timeout 8s ros2 node list --no-daemon --spin-time 2 -a > "$stage/graph-after-ld19.txt" ||
     [[ -s "$stage/graph-after-ld19.txt" ]]; then
    printf 'Final ROS graph not empty or unverified\n' >&2
    code=1
  fi
  printf 'Step 75.5b staging and LD19 logs: %s\n' "$stage"
  exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
timeout 12s ros2 topic echo /sie/lidar/scan_raw sensor_msgs/msg/LaserScan \
  --no-daemon --once --field header --qos-reliability best_effort | tee "$stage/ld19-first-header.txt"
# The diagnostic repeats the safe STATUS check against the separately verified boot ID.
printf '%s\n' "$confirmation" | python3 "$stage/tools/diagnostics/step75_5a_slam.py" live \
  --domain "$ROS_DOMAIN_ID" --source "$source_checkout" --expected-boot "$expected_boot" \
  --tf-audit-executable "$stage/build/sie_tf_receipts" 2>&1 | tee "$stage/live.log"
