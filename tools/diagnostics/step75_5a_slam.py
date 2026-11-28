"""Opt-in, isolated SLAM Toolbox verification. Synthetic modes never access UART."""
import argparse
from collections import Counter, defaultdict
from contextlib import suppress
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_path
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSCompatibility, qos_check_compatible, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCAN = "/sie/lidar/scan_raw"
WHEEL_TF = "/sie/step75_5a/wheel_tf"
DIAG = "/sie/diagnostics/wheel_odometry"
UART = "/dev/ttyAMA0"
ODOM_EDGE = ("odom", "base_link")
MOUNT_EDGE = ("base_link", "ld19_sensor")
MAP_EDGE = ("map", "odom")
CHECK = "/sie_step755_check"
TF_AUDIT = "/sie_step755_tf_receipts"
STATIC = "/sie_step754_ld19_candidate_tf"
MAPPER = "/slam_toolbox"
WHEEL = "/sie_wheel_odometry"
LD19 = "/siee_ld19"
TF_AUTHORITIES = {"/tf": {ODOM_EDGE: CHECK, MAP_EDGE: MAPPER}, "/tf_static": {MOUNT_EDGE: STATIC}}
NS = 1_000_000_000
# Broad domain/clock guard for a bounded smoke test, independent of acquisition age.
MAX_HEADER_OFFSET_NS = 2 * NS


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def edge(transform):
    return (transform.header.frame_id, transform.child_frame_id)


def full_name(endpoint):
    return endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name


def json_file(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def run_text(command, *, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True, timeout=5).strip()


def config(source, evidence):
    package = get_package_share_path("slam_toolbox")
    default_path = package / "config/mapper_params_online_async.yaml"
    baseline = yaml.safe_load(default_path.read_text())
    parameters = baseline["slam_toolbox"]["ros__parameters"]
    path = ROOT / "ros2/sie_ros2/config/diagnostics/slam_toolbox_async_v1.yaml"
    parameters.update(yaml.safe_load(path.read_text())["slam_toolbox"]["ros__parameters"])
    effective_path = evidence / "effective_mapper.yaml"
    effective_path.write_text(yaml.safe_dump(baseline))
    timeout = parameters["transform_timeout"]
    if not math.isfinite(timeout) or not 0 < timeout <= 2:
        raise ValueError("UNSUPPORTED_MAPPER_TF_TIMEOUT")
    interval = parameters["map_update_interval"]
    if not math.isfinite(interval) or not 0 < interval <= 30:
        raise ValueError("UNSUPPORTED_MAPPER_UPDATE_INTERVAL")
    profile = source / "ros2/sie_ros2/config/diagnostics/robot_geometry_footprint_v1.json"
    geometry = json.loads(profile.read_text())
    if (geometry["status"] != "ROBOT_GEOMETRY_AND_FOOTPRINT_MOTOR_OFF_QUALIFIED"
            or geometry["navigation_admitted"] or geometry["powered_motion_authorized"]):
        raise ValueError("GEOMETRY_PROFILE_NOT_QUALIFIED_FOR_MOTOR_OFF")
    tf = geometry["ld19_mount"]["candidate_static_tf"]
    if (tf["parent_frame_id"], tf["child_frame_id"]) != MOUNT_EDGE:
        raise ValueError("UNEXPECTED_GEOMETRY_FRAMES")
    deb = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", f"ros-{os.environ['ROS_DISTRO']}-slam-toolbox"],
        capture_output=True, text=True, timeout=5,
    )
    version = {
        "upstream": ET.parse(package / "package.xml").findtext("version"),
        "debian_installed": deb.stdout if deb.returncode == 0 else None,
        "package_share": str(package),
    }
    return effective_path, float(timeout), tf, version


class Processes:
    def __init__(self, evidence):
        self.evidence = evidence
        self.children = []

    def start(self, name, command, *, env=None):
        with (self.evidence / f"{name}.log").open("w") as log:
            log.write(json.dumps(command) + "\n")
            log.flush()
            child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        self.children.append((name, child))

    def check(self):
        for name, child in self.children:
            if child.poll() is not None:
                raise RuntimeError(f"{name.upper()}_EXITED:{child.returncode}")

    def stop(self):
        results = {}
        for name, child in reversed(self.children):
            signals = []
            for sig, limit in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                if child.poll() is not None:
                    break
                with suppress(ProcessLookupError):
                    os.killpg(child.pid, sig)
                    signals.append(sig.name)
                try:
                    child.wait(timeout=limit)
                except subprocess.TimeoutExpired:
                    continue
            results[name] = {"signals": signals, "exit_code": child.poll()}
        return results


class Check(Node):
    def __init__(self, mode, mount, evidence, expected_boot, native_tf_audit=False):
        super().__init__(CHECK.lstrip("/"), parameter_overrides=[])
        self.mode = mode
        self.evidence = evidence
        self.expected_boot = expected_boot
        self.mount = mount
        self.failure = None
        self.collect = False
        self.wheel_ready = False
        self.scans = {}
        self.pose_stamps = set()
        self.map_summaries = []
        self.native_tf_audit = native_tf_audit
        self.tf_endpoints = defaultdict(dict)
        self.tf_observed_stamps = defaultdict(set)
        self.tf_attributed_stamps = defaultdict(set)
        self.edge_owners = defaultdict(set)
        self.edge_gids = defaultdict(set)
        self.forwarded = 0
        self.last_wheel_stamp = 0
        self.stationary_pose = None
        self.relay = None
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, SCAN, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 30)
        self.create_subscription(TFMessage, "/tf_static", self.on_static, latched)
        if self.mode == "live":
            self.create_subscription(DiagnosticArray, DIAG, self.on_wheel_diagnostics, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.mapper_state = self.create_client(GetState, MAPPER + "/get_state")
        self.mapper_transition = self.create_client(ChangeState, MAPPER + "/change_state")

    def endpoints(self, topic):
        return list({bytes(ep.endpoint_gid): ep
                     for ep in self.get_publishers_info_by_topic(topic)}.values())

    def names(self, topic):
        return sorted(full_name(endpoint) for endpoint in self.endpoints(topic))

    def inventory(self):
        return {topic: [{
            "node": full_name(ep), "type": ep.topic_type,
            "gid": bytes(ep.endpoint_gid).hex(),
            "reliability": ep.qos_profile.reliability.name,
            "durability": ep.qos_profile.durability.name,
        } for ep in self.endpoints(topic)] for topic in (
            SCAN, "/odom", WHEEL_TF, "/tf", "/tf_static", "/map", "/pose", "/clock", "/cmd_vel",
        )}

    def refuse(self, reason):
        if self.failure is None:
            self.failure = reason

    def on_scan(self, message):
        if not self.collect:
            return
        stamp = stamp_ns(message.header.stamp)
        receipt = self.get_clock().now().nanoseconds
        if message.header.frame_id != MOUNT_EDGE[1] or not 0 < stamp:
            self.refuse("INVALID_SCAN_HEADER")
        elif abs(receipt - stamp) > MAX_HEADER_OFFSET_NS:
            self.refuse("SCAN_CLOCK_DOMAIN_MISMATCH")
        elif stamp in self.scans or (self.scans and stamp <= next(reversed(self.scans))):
            self.refuse("NON_INCREASING_SCAN_HEADER")
        else:
            self.scans[stamp] = {
                "header_stamp_ns": stamp, "frame_id": message.header.frame_id,
                "receipt_ros_ns": receipt, "receipt_monotonic_ns": time.monotonic_ns(),
                "beams": len(message.ranges), "input_tf": None, "map_tf": False,
            }

    def on_pose(self, message):
        if self.collect:
            if message.header.frame_id != MAP_EDGE[0]:
                self.refuse("INVALID_SLAM_POSE_FRAME")
            self.pose_stamps.add(stamp_ns(message.header.stamp))

    def on_map(self, message):
        if not self.collect or not message.info.width or not message.info.height:
            return
        size = message.info.width * message.info.height
        if message.header.frame_id != MAP_EDGE[0] or size != len(message.data) or message.info.resolution <= 0:
            self.refuse("INVALID_OCCUPANCY_GRID")
            return
        cells = Counter(message.data)
        record = {
            "header_stamp_ns": stamp_ns(message.header.stamp), "frame": message.header.frame_id,
            "resolution_m": message.info.resolution, "width": message.info.width,
            "height": message.info.height, "cells_by_occupancy": dict(cells),
        }
        if not self.map_summaries:
            json_file(self.evidence / "first_occupancy_grid.json", {
                **record, "origin_xy_m": [message.info.origin.position.x, message.info.origin.position.y],
                "data": list(message.data),
            })
        self.map_summaries.append(record)

    def remember_tf_endpoints(self, topic):
        for endpoint in self.endpoints(topic):
            gid = bytes(endpoint.endpoint_gid).hex()
            owner = full_name(endpoint)
            previous = self.tf_endpoints[topic].setdefault(gid, owner)
            if previous != owner:
                self.refuse(f"TF_ENDPOINT_IDENTITY_CHANGED:{topic}:{gid}")

    def attribute_tf(self, topic, pair, stamp, gid):
        expected = TF_AUTHORITIES.get(topic, {}).get(pair)
        if expected is None:
            self.refuse(f"UNEXPECTED_TF_EDGE:{topic}:{pair}")
            return
        if not gid or not any(bytes.fromhex(gid)):
            self.refuse(f"TF_PUBLISHER_GID_UNAVAILABLE:{topic}:{pair}")
            return
        self.edge_gids[pair].add(gid)
        if len(self.edge_gids[pair]) != 1:
            self.refuse(f"MULTIPLE_TF_AUTHORITIES:{pair}")
            return
        owner = self.tf_endpoints[topic].get(gid)
        if owner != expected:
            self.refuse(f"UNEXPECTED_TF_AUTHORITY:{pair}:{owner or gid}")
            return
        self.edge_owners[pair].add(owner)
        self.tf_attributed_stamps[pair].add(stamp)

    def observe_tf(self, message, info, topic):
        self.remember_tf_endpoints(topic)
        # Lyrical supplies PublisherGID.data. Jazzy exposes no GID in
        # rclpy, so receipt attribution comes from the native subscriber.
        gid_info = info.get("publisher_gid")
        gid = bytes(gid_info["data"]).hex() if isinstance(gid_info, dict) else None
        for transform in message.transforms:
            pair = edge(transform)
            if pair not in TF_AUTHORITIES[topic]:
                self.refuse(f"UNEXPECTED_TF_EDGE:{topic}:{pair}")
                continue
            stamp = stamp_ns(transform.header.stamp)
            self.tf_observed_stamps[pair].add(stamp)
            if not self.native_tf_audit:
                self.attribute_tf(topic, pair, stamp, gid)

    def reconcile_native_tf(self):
        # The receipt writer is stopped before reconciliation. Read to EOF,
        # and validate every receipt including stamps the observer did not see.
        with (self.evidence / "tf_receipts.txt").open() as journal:
            for line in journal:
                kind, topic, gid, stamp, parent, child = shlex.split(line)
                if kind != "TF_RECEIPT" or not line.endswith("\n"):
                    raise ValueError("INVALID_NATIVE_TF_RECEIPT")
                self.attribute_tf(topic, (parent, child), int(stamp), gid)
        for pair, observed in self.tf_observed_stamps.items():
            if observed - self.tf_attributed_stamps[pair]:
                self.refuse(f"TF_RECEIPT_COVERAGE_INCOMPLETE:{pair}")

    def on_tf(self, message, info):
        self.observe_tf(message, info, "/tf")

    def on_static(self, message, info):
        self.observe_tf(message, info, "/tf_static")
        for transform in message.transforms:
            xyz = transform.transform.translation
            quat = transform.transform.rotation
            actual = (xyz.x, xyz.y, xyz.z, quat.x, quat.y, quat.z, quat.w)
            wanted = (*self.mount["translation_xyz_m"], 0, 0, 0, 1)
            if not all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(actual, wanted)):
                self.refuse("UNEXPECTED_LD19_MOUNT_TRANSFORM")

    def start_relay(self):
        self.relay = self.create_publisher(TFMessage, "/tf", 30)
        self.create_subscription(TFMessage, WHEEL_TF, self.on_wheel_tf, 30)

    def on_wheel_tf(self, message):
        if self.relay is None or self.failure or not self.wheel_ready:
            return
        expected = WHEEL if self.mode == "live" else CHECK
        if self.names(WHEEL_TF) != [expected]:
            self.refuse("WHEEL_TF_SOURCE_NOT_EXCLUSIVE")
            return
        if len(message.transforms) != 1 or edge(message.transforms[0]) != ODOM_EDGE:
            self.refuse("UNEXPECTED_PRIVATE_WHEEL_TF")
            return
        stamp = stamp_ns(message.transforms[0].header.stamp)
        if stamp <= self.last_wheel_stamp:
            self.refuse("NON_INCREASING_WHEEL_TF")
            return
        t = message.transforms[0].transform
        if not all(map(math.isfinite, (t.translation.x, t.translation.y, t.translation.z,
                                 t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))):
            self.refuse("NON_FINITE_WHEEL_TF")
            return
        self.last_wheel_stamp = stamp
        self.forwarded += 1
        self.relay.publish(message)

    def on_odom(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        if self.stationary_pose is None:
            self.stationary_pose = pose
        if (message.header.frame_id != ODOM_EDGE[0] or message.child_frame_id != ODOM_EDGE[1]
                or pose != self.stationary_pose or twist.linear.x != 0 or twist.angular.z != 0):
            self.refuse("WHEEL_ODOMETRY_NOT_STATIONARY")

    def on_wheel_diagnostics(self, message):
        for status in message.status:
            if status.name != "sie/wheel_odometry":
                continue
            values = {pair.key: pair.value for pair in status.values}
            if values.get("navigation_admitted") != "false" or values.get("motion_continuity_lost") != "false":
                self.refuse("WHEEL_ODOMETRY_ADMISSION_OR_CONTINUITY")
            boot = values.get("boot_session_id")
            if boot != self.expected_boot:
                self.refuse("WHEEL_ODOMETRY_BOOT_CHANGED_OR_UNVERIFIED")
            if values.get("telemetry_valid") == "true":
                self.wheel_ready = True
            elif self.wheel_ready or status.message not in {"BASELINE", "FIRMWARE_VERIFIED_AWAITING_SAMPLE"}:
                self.refuse(f"WHEEL_ODOMETRY_REFUSED:{status.message}")

    def check_graph(self, expected):
        for topic, owners in expected.items():
            actual = self.names(topic)
            # Some versions expose more than one endpoint for a mapper node.
            # Distinct endpoint IDs carrying the same TF edge are still refused.
            if topic == "/tf" and MAPPER in owners:
                actual = [name for name in actual if name != MAPPER] + ([MAPPER] if MAPPER in actual else [])
            if sorted(actual) != sorted(owners):
                raise RuntimeError(f"UNEXPECTED_GRAPH:{topic}:{self.names(topic)}")
        pubs = self.endpoints(SCAN)
        subs = [ep for ep in self.get_subscriptions_info_by_topic(SCAN) if full_name(ep) == MAPPER]
        if subs and (len(pubs) != 1 or len(subs) != 1 or any(
            qos_check_compatible(pub.qos_profile, sub.qos_profile)[0] != QoSCompatibility.OK
            for pub in pubs for sub in subs
        )):
            raise RuntimeError("SLAM_SCAN_QOS_UNVERIFIED")

    def update_scan_tf(self, timeout):
        now = time.monotonic_ns()
        for stamp, record in reversed(self.scans.items()):
            age = (now - record["receipt_monotonic_ns"]) / NS
            # Keep the eventual scan-time chain separate from timely mapper input.
            if age > 2:
                break
            at = Time(nanoseconds=stamp)
            if record["input_tf"] is None:
                if age > timeout:
                    record["input_tf"] = "MISSING_WITHIN_TIMEOUT"
                elif self.tf_buffer.can_transform(ODOM_EDGE[0], MOUNT_EDGE[1], at):
                    record["input_tf"] = "AVAILABLE"
                    record["input_tf_observed_after_ms"] = age * 1000
            if not record["map_tf"]:
                record["map_tf"] = self.tf_buffer.can_transform(MAP_EDGE[0], MOUNT_EDGE[1], at)


def spin(check, processes, seconds, condition=None, tick=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not check.failure:
        processes.check()
        if condition is not None and condition():
            return True
        if tick:
            tick()
        rclpy.spin_once(check, timeout_sec=0.01)
    if check.failure:
        raise RuntimeError(check.failure)
    return condition is None or condition()


def fixture_scan(stamp, x, mount):
    """A 501-beam scan of a fixed 6 × 4 m rectangle, from a translating fixture."""
    scan = LaserScan()
    scan.header.stamp = stamp.to_msg()
    scan.header.frame_id = MOUNT_EDGE[1]
    scan.angle_min, scan.angle_max = 0., 2 * math.pi
    scan.angle_increment = 2 * math.pi / 500
    scan.range_min, scan.range_max = 0.02, 12.
    scan.scan_time = 0.125
    ox, oy, _ = mount["translation_xyz_m"]
    origin_x = x + ox
    ranges = []
    for beam in range(501):
        angle = beam * scan.angle_increment
        dx, dy = math.cos(angle), math.sin(angle)
        tx = ((3 if dx > 0 else -3) - origin_x) / dx if abs(dx) > 1e-8 else math.inf
        ty = ((2 if dy > 0 else -2) - oy) / dy if abs(dy) > 1e-8 else math.inf
        ranges.append(min(tx, ty))
    scan.ranges = ranges
    return scan


def fixture_tf(stamp, x):
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = ODOM_EDGE
    message.header.stamp = stamp.to_msg()
    message.transform.translation.x = x
    message.transform.rotation.w = 1.
    return TFMessage(transforms=[message])


def mount_command(mount):
    xyz, rpy = mount["translation_xyz_m"], mount["rpy_rad"]
    command = [str(Path(get_package_prefix("tf2_ros")) / "lib/tf2_ros/static_transform_publisher")]
    for option, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), [*xyz, *rpy]):
        command.extend([f"--{option}", str(value)])
    return command + ["--frame-id", MOUNT_EDGE[0], "--child-frame-id", MOUNT_EDGE[1],
                      "--ros-args", "-r", f"__node:={STATIC.lstrip('/')}"]


def live_uart_status(source, evidence, expected_boot):
    from sie_ros2.wheel_odometry_uart import ReadOnlyUart, open_uart, owners

    existing = owners(Path(UART).resolve(strict=True))
    if existing:
        raise RuntimeError(f"UART_ALREADY_OWNED:{sorted(existing)}")
    for name in ("run_shadow_uart", "supervised_bounded_executor", "live_steered_forward",
                 "shadow_supervisor", "shadow_edge_turn"):
        running = subprocess.run(["pgrep", "-f", name], capture_output=True, text=True, timeout=2)
        if running.returncode != 1:
            raise RuntimeError(f"MOTION_RUNNER_CHECK_REFUSED:{name}")
    path = source / "ros2/sie_ros2/config/wheel_odometry_v1.yaml"
    timeout = yaml.safe_load(path.read_text())["sie_wheel_odometry"]["ros__parameters"]["uart_timeout_s"]
    attempt, status = None, None
    try:
        with open_uart(UART, Path.home() / ".local/state/sie") as fd:
            serial = ReadOnlyUart(fd, timeout, time.time_ns)
            attempt = serial.make_attempt(status=True)
            status = serial.exchange(attempt)
        expected = {
            "boot_session_id": expected_boot, "state": "READY",
            "active_command_id": None, "fault": None, "bounded_fault_reason": None,
            "bounded_fault_latched": False, "steered_safety_latched": False,
            "remote_stop_pending": False, "right_pwm": 0, "left_pwm": 0,
        }
        mismatches = [key for key, value in expected.items()
                      if key not in status or type(status[key]) is not type(value) or status[key] != value]
        if mismatches:
            raise RuntimeError(f"STATUS_MISMATCH:{','.join(mismatches)}")
    finally:
        json_file(evidence / "preflight_status.json", {
            "attempt": asdict(attempt) if attempt is not None else None, "status": status,
        })
    return path


def run(mode, source, check, processes, mapper_yaml, timeout):
    live = mode == "live"
    has_wheel_tf = mode != "missing-tf"
    empty = {topic: [] for topic in check.inventory()}
    spin(check, processes, 1.5)
    def mapper_scan_subscribers():
        return [ep for ep in check.get_subscriptions_info_by_topic(SCAN) if full_name(ep) == MAPPER]
    expected = {**empty, SCAN: [LD19] if live else []}
    check.check_graph(expected)
    json_file(check.evidence / "graph_preflight.json", check.inventory())
    if has_wheel_tf:
        check.start_relay()
    if live:
        wheel_yaml = live_uart_status(source, check.evidence, check.expected_boot)
        processes.start("wheel", [
            sys.executable, "-c", "from sie_ros2.wheel_odometry_node import main; main()",
            "--ros-args", "--params-file", str(wheel_yaml), "-p", "publish_tf:=true",
            "-p", "use_sim_time:=false", "-p", f"journal_path:={check.evidence / 'wheel_odometry.jsonl'}",
            "-r", f"/tf:={WHEEL_TF}",
        ])
        if not spin(check, processes, 8, lambda: check.wheel_ready and check.forwarded > 0):
            raise RuntimeError("WHEEL_ODOMETRY_STARTUP_TIMEOUT")
    else:
        synthetic_scan = check.create_publisher(LaserScan, SCAN, 10)
        if has_wheel_tf:
            synthetic_wheel = check.create_publisher(TFMessage, WHEEL_TF, 30)
            check.wheel_ready = True

    processes.start("static", mount_command(check.mount))
    if not spin(check, processes, 6, lambda: MOUNT_EDGE in check.tf_observed_stamps):
        raise RuntimeError("STATIC_TF_STARTUP_TIMEOUT")
    processes.start("mapper", [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"slam_params_file:={mapper_yaml}", "use_sim_time:=false",
        "autostart:=true", "use_lifecycle_manager:=false",
    ])
    if not spin(check, processes, 14, lambda: (
        check.names("/map") == [MAPPER] and check.names("/pose") == [MAPPER]
        and len(mapper_scan_subscribers()) == 1
    )):
        raise RuntimeError("MAPPER_STARTUP_TIMEOUT")

    mapping = {
        **empty, SCAN: [LD19 if live else CHECK], "/odom": [WHEEL] if live else [],
        WHEEL_TF: ([WHEEL if live else CHECK] if has_wheel_tf else []),
        "/tf": [CHECK, MAPPER] if has_wheel_tf else [MAPPER],
        "/tf_static": [STATIC], "/map": [MAPPER], "/pose": [MAPPER],
    }
    json_file(check.evidence / "graph_active.json", check.inventory())
    check.check_graph(mapping)
    if not check.mapper_state.wait_for_service(timeout_sec=2):
        raise RuntimeError("MAPPER_LIFECYCLE_SERVICE_UNAVAILABLE")
    state = check.mapper_state.call_async(GetState.Request())
    if not spin(check, processes, 3, state.done) or state.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
        raise RuntimeError("MAPPER_NOT_ACTIVE")
    json_file(check.evidence / "mapper_scan_subscription.json", [{
        "node": full_name(ep), "reliability": ep.qos_profile.reliability.name,
        "durability": ep.qos_profile.durability.name,
    } for ep in mapper_scan_subscribers()])

    check.collect = True

    def tick():
        check.update_scan_tf(timeout)

    if live:
        # The preflight physically limits this run to stationary smoke evidence.
        for _ in range(6):
            spin(check, processes, 5, tick=tick)
            check.check_graph(mapping)
    else:
        for i in range(64 if has_wheel_tf else 32):
            stamp = check.get_clock().now()
            x = min(i * 0.05, 1.6)
            if has_wheel_tf:
                synthetic_wheel.publish(fixture_tf(Time(nanoseconds=stamp.nanoseconds - 5_000_000), x))
                synthetic_wheel.publish(fixture_tf(Time(nanoseconds=stamp.nanoseconds + 5_000_000), x))
                if not spin(check, processes, 0.085, lambda: check.tf_buffer.can_transform(
                        ODOM_EDGE[0], MOUNT_EDGE[1], stamp)):
                    raise RuntimeError("SYNTHETIC_TF_DELIVERY_TIMEOUT")
                # DDS delivery to the mapper is independent of the observer.
                spin(check, processes, 0.04)
            synthetic_scan.publish(fixture_scan(stamp, x, check.mount))
            spin(check, processes, 0.11, tick=tick)
        # Allow the installed upstream map-update interval, plus discovery/QoS margin.
        parameters = yaml.safe_load(mapper_yaml.read_text())["slam_toolbox"]["ros__parameters"]
        spin(check, processes, float(parameters["map_update_interval"]) + 0.5, tick=tick)
    check.check_graph(mapping)
    check.collect = False
    # Final TF classifications are based on original stamps. A live scan
    # arriving at the window boundary gets its full observation timeout.
    spin(check, processes, timeout + 0.08, tick=tick)
    for stamp, record in check.scans.items():
        if not record["map_tf"]:
            record["map_tf"] = check.tf_buffer.can_transform(MAP_EDGE[0], MOUNT_EDGE[1], Time(nanoseconds=stamp))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("synthetic", "missing-tf", "live"))
    parser.add_argument("--domain", type=int, required=True, choices=range(1, 233), metavar="1..232")
    parser.add_argument("--source", type=Path, default=ROOT)
    parser.add_argument("--evidence-root", type=Path, default=Path.home() / ".local/state/sie/evidence")
    parser.add_argument("--expected-boot")
    parser.add_argument("--tf-audit-executable", type=Path, help="Native receipt subscriber, required on Jazzy")
    args = parser.parse_args()
    live = args.mode == "live"
    if os.environ.get("ROS_DISTRO") == "jazzy" and args.tf_audit_executable is None:
        parser.error("Jazzy requires --tf-audit-executable for message-level TF attribution")
    if live and (os.environ.get("ROS_DISTRO") != "jazzy"
                 or not re.fullmatch(r"[0-9A-F]{16}", args.expected_boot or "")):
        parser.error("Live mode requires Jazzy and --expected-boot with the currently verified ESP32 boot ID")
    if live and input("After authorization, confirm traction power physically disconnected and wheels stationary. Type MOTOR-OFF: ") != "MOTOR-OFF":
        parser.error("Operator did not confirm motor-off stationary operation")

    os.umask(0o077)
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    evidence = Path(tempfile.mkdtemp(prefix="step75_5a_", dir=args.evidence_root.resolve()))
    source = args.source.resolve()
    print(f"Evidence directory: {evidence}", flush=True)
    processes = Processes(evidence)
    check = None
    refusal = []
    uart_check_enabled = False
    summary = {
        "mode": args.mode, "ros_distro": os.environ.get("ROS_DISTRO"), "ros_domain_id": args.domain,
        "diagnostic_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "tf_attribution": "RCLCPP_MESSAGE_INFO" if args.tf_audit_executable else "RCLPY_MESSAGE_INFO",
        "ros_clock": "SYSTEM_TIME", "physical_ld19_acquisition_time": "UNKNOWN",
        "ld19_header_provenance": ("DRIVER_CONVERSION_HOST_ROS_TIME" if live else "SYNTHETIC_SYSTEM_TIME"),
        "navigation_admitted": False, "powered_motion_authorized": False, "expected_boot": args.expected_boot,
    }
    try:
        for key in ("ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_NAMESPACE"):
            os.environ.pop(key, None)
        os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                          ROS_LOG_DIR=str(evidence / "ros_log"))
        if live and any(os.environ.get(key) for key in (
            "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
        )):
            raise RuntimeError("CUSTOM_DDS_PROFILE_REQUIRES_REVIEW")
        summary["source_commit"] = run_text(["git", "rev-parse", "HEAD"], cwd=source)
        if live:
            clean_paths = ["ros2/sie_ros2/sie_ros2", "ros2/sie_ros2/config/wheel_odometry_v1.yaml",
                           "ros2/sie_ros2/config/diagnostics/robot_geometry_footprint_v1.json",
                           "firmware/esp32/tools", "vision_core/tools"]
            if run_text(["git", "status", "--porcelain", "--", *clean_paths], cwd=source):
                raise RuntimeError("PRODUCTION_SOURCE_OR_GEOMETRY_NOT_CLEAN")
            baseline = "2a00bd80348e8d10d4577e886f354d30e24057c1"
            checked = subprocess.run(["git", "diff", "--quiet", baseline, "HEAD", "--", *clean_paths],
                                     cwd=source, timeout=5)
            if checked.returncode:
                raise RuntimeError("PRODUCTION_SOURCE_OR_GEOMETRY_DIFFERS_FROM_STEP_75_4")
            for path in (source, source / "ros2/sie_ros2"):
                sys.path.insert(0, str(path))
            os.environ["PYTHONPATH"] = os.pathsep.join([
                str(source / "ros2/sie_ros2"), str(source), os.environ.get("PYTHONPATH", ""),
            ])
            uart_check_enabled = True
        mapper_yaml, timeout, mount, summary["slam_toolbox"] = config(source, evidence)
        summary["mapper_tf_timeout_s"] = timeout
        rclpy.init(args=[])
        check = Check(args.mode, mount, evidence, args.expected_boot, bool(args.tf_audit_executable))
        if args.tf_audit_executable:
            binary = args.tf_audit_executable.resolve(strict=True)
            with binary.open("rb") as stream:
                summary["tf_audit_executable_sha256"] = hashlib.file_digest(stream, "sha256").hexdigest()
            processes.start("tf_audit", [str(binary), str(evidence / "tf_receipts.txt")])
            if not spin(check, processes, 5, lambda: all(
                any(full_name(ep) == TF_AUDIT for ep in check.get_subscriptions_info_by_topic(topic))
                for topic in TF_AUTHORITIES
            )):
                raise RuntimeError("NATIVE_TF_RECEIPT_SUBSCRIBER_UNAVAILABLE")
        run(args.mode, source, check, processes, mapper_yaml, timeout)
    except (Exception, KeyboardInterrupt) as exc:
        refusal.append(f"{type(exc).__name__}:{exc}" if str(exc) else type(exc).__name__)
    finally:
        if check is not None and rclpy.ok() and check.mapper_transition.service_is_ready():
            try:
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
                future = check.mapper_transition.call_async(request)
                rclpy.spin_until_future_complete(check, future, timeout_sec=3)
                summary["mapper_lifecycle_shutdown"] = bool(future.done() and future.result().success)
                if not summary["mapper_lifecycle_shutdown"]:
                    refusal.append("MAPPER_LIFECYCLE_SHUTDOWN_UNVERIFIED")
            except (Exception, KeyboardInterrupt) as exc:
                refusal.append(f"MAPPER_LIFECYCLE_SHUTDOWN_UNVERIFIED:{exc}")
        summary["shutdown"] = processes.stop()
        if any(result["signals"] != ["SIGINT"] or result["exit_code"] not in (0, -signal.SIGINT)
               for result in summary["shutdown"].values()):
            refusal.append("CHILD_SHUTDOWN_NOT_CLEAN")
        if check is not None:
            if check.relay is not None:
                check.destroy_publisher(check.relay)
                check.relay = None
            # Stop local input publishers too. Keep the node for graph inspection.
            for publisher in list(check.publishers):
                if publisher.topic_name in (SCAN, WHEEL_TF):
                    check.destroy_publisher(publisher)
            if rclpy.ok():
                for _ in range(40):
                    rclpy.spin_once(check, timeout_sec=0.05)
                after = check.inventory()
                json_file(evidence / "graph_shutdown.json", after)
                if any(endpoints for topic, endpoints in after.items() if topic != SCAN):
                    refusal.append("ORPHAN_GRAPH_PUBLISHER")
                if any(ep["node"] != LD19 or not live for ep in after[SCAN]):
                    refusal.append("ORPHAN_SCAN_PUBLISHER")
            else:
                refusal.append("GRAPH_SHUTDOWN_UNVERIFIED")
            if check.native_tf_audit and "tf_audit" in summary["shutdown"]:
                try:
                    check.reconcile_native_tf()
                except (OSError, ValueError) as exc:
                    refusal.append(f"NATIVE_TF_RECEIPTS_UNVERIFIED:{exc}")
            if check.failure and not any(check.failure in reason for reason in refusal):
                refusal.append(check.failure)
            check.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        if uart_check_enabled:
            try:
                from sie_ros2.wheel_odometry_uart import owners
                summary["uart_owners_after_shutdown"] = sorted(owners(Path(UART).resolve(strict=True)))
                if summary["uart_owners_after_shutdown"]:
                    refusal.append("UART_NOT_RELEASED")
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                refusal.append(f"UART_RELEASE_UNVERIFIED:{exc}")

    if check is not None:
        summary.update(summarize(check, evidence, refusal))
    summary.update(result="PASS" if not refusal else "FAIL" if summary["shutdown"] else "BLOCKED", refusal_reasons=refusal, evidence=str(evidence))
    json_file(evidence / "qualification.json", summary)
    with (evidence / "SHA256SUMS.txt").open("w") as manifest:
        for path in sorted(evidence.rglob("*")):
            if path.is_file() and path != evidence / "SHA256SUMS.txt":
                with path.open("rb") as stream:
                    digest = hashlib.file_digest(stream, "sha256").hexdigest()
                manifest.write(f"{digest}  {path.relative_to(evidence)}\n")
    print(json.dumps(summary, sort_keys=True))
    return int(bool(refusal))


def summarize(check, evidence, refusal):
    map_stamps = check.tf_attributed_stamps[MAP_EDGE]
    first_map_stamp = min(map_stamps, default=None)
    map_misses = []
    for stamp, record in check.scans.items():
        if not record["map_tf"]:
            before_map = first_map_stamp is not None and stamp < first_map_stamp
            reason = ("INPUT_TF_UNAVAILABLE" if record["input_tf"] != "AVAILABLE"
                      else "BEFORE_FIRST_ATTRIBUTED_MAP_TF" if before_map else "UNRESOLVED_MAP_TF")
            record["map_tf_miss_reason"] = reason
            map_misses.append({"scan_stamp_ns": stamp, "reason": reason})
    with (evidence / "scan_timing.jsonl").open("w") as journal:
        for stamp, record in check.scans.items():
            journal.write(json.dumps({**record, "matched_slam_pose": stamp in check.pose_stamps}) + "\n")
    received = set(check.scans)
    exact_poses = received & check.pose_stamps
    mapper_log = evidence / "mapper.log"
    lines = mapper_log.read_text(errors="replace").splitlines() if mapper_log.is_file() else []
    drops = Counter()
    for line in lines:
        if "Message Filter dropping message" in line:
            drops[line.partition("for reason")[2].strip(" '") or "UNCLASSIFIED_MESSAGE_FILTER_DROP"] += 1
    timing = Counter(record["input_tf"] or "UNRESOLVED" for record in check.scans.values())
    connected = sum(bool(record["map_tf"]) for record in check.scans.values())
    if not refusal:
        if not received or MOUNT_EDGE not in check.edge_owners:
            refusal.append("NO_SCAN_OR_MOUNT_EVIDENCE")
        elif check.mode == "missing-tf":
            if (len(received) != 32 or timing["MISSING_WITHIN_TIMEOUT"] != len(received) or check.map_summaries
                    or check.pose_stamps or MAP_EDGE in check.edge_owners or not drops):
                refusal.append("MISSING_TF_NEGATIVE_CASE_NOT_DEMONSTRATED")
        else:
            if (not exact_poses or not check.map_summaries or not connected
                    or ODOM_EDGE not in check.edge_owners or MAP_EDGE not in check.edge_owners):
                refusal.append("MAPPING_OR_SCAN_TIME_CONNECTIVITY_NOT_DEMONSTRATED")
            if check.mode == "synthetic" and (len(received) != 64 or timing["AVAILABLE"] != len(received)):
                refusal.append("SYNTHETIC_SCAN_COUNT_OR_TF_MISMATCH")
    return {
        "observer_received_scans": len(received), "observer_scan_time_input_tf": dict(timing),
        "observer_scan_time_map_chain_available": connected,
        "first_attributed_map_tf_stamp_ns": first_map_stamp,
        "map_chain_miss_counts": dict(Counter(row["reason"] for row in map_misses)),
        "first_map_chain_misses": map_misses[:8],
        "verified_mapper_pose_stamps": len(exact_poses),
        "observed_without_matching_pose": len(received - exact_poses),
        "mapper_message_filter_drop_log_counts": dict(drops),
        "last_map": check.map_summaries[-1] if check.map_summaries else None,
        "map_messages": len(check.map_summaries), "wheel_tf_forwarded": check.forwarded,
        "tf_edge_owners": {" -> ".join(pair): sorted(owners) for pair, owners in check.edge_owners.items()},
        "tf_edge_publisher_gids": {" -> ".join(pair): sorted(gids) for pair, gids in check.edge_gids.items()},
    }


if __name__ == "__main__":
    sys.exit(main())
