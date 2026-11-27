"""Opt-in ROS 2 wheel odometry, with no actuator or navigation-admission path."""
from contextlib import ExitStack
from dataclasses import asdict, dataclass, fields
import json
from math import cos, isfinite, sin
import os
from pathlib import Path
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from .wheel_odometry import WheelGeometry, WheelOdometry
from .wheel_odometry_uart import ReadOnlyUart, lease, open_uart

NS = 1_000_000_000
PARENT = "odom"
CHILD = "base_link"


@dataclass(frozen=True)
class Covariance:
    pose_xy_variance: float = 0.01
    pose_yaw_variance: float = 0.025
    twist_linear_variance: float = 0.04
    twist_angular_variance: float = 0.10
    unobserved_variance: float = 1e6
    xy_variance_per_wheel_meter: float = 0.01
    yaw_variance_per_radian: float = 0.025
    discontinuity_xy_variance: float = 0.25
    discontinuity_yaw_variance: float = 0.25

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) not in (float, int) or not isfinite(value) or value <= 0:
                raise ValueError(f"Invalid odometry covariance: {name}")

    def diagonals(self, estimate):
        xy = (self.pose_xy_variance + self.xy_variance_per_wheel_meter * estimate.distance_travelled
              + self.discontinuity_xy_variance * estimate.discontinuities)
        yaw = (self.pose_yaw_variance + self.yaw_variance_per_radian * estimate.heading_travelled
               + self.discontinuity_yaw_variance * estimate.discontinuities)
        unknown = self.unobserved_variance
        return ((xy, xy, unknown, unknown, unknown, yaw),
                (self.twist_linear_variance, unknown, unknown, unknown, unknown, self.twist_angular_variance))


def diagonal_matrix(diagonal):
    return [value if column == row else 0.0
            for row, value in enumerate(diagonal) for column in range(6)]


def ros_messages(estimate, stamp_ns, covariance):
    diagonal_pose, diagonal_twist = covariance.diagonals(estimate)
    if not all(isfinite(value) for value in diagonal_pose):
        raise ValueError("POSE_COVARIANCE_NONFINITE")
    odom = Odometry()
    odom.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    odom.header.frame_id = PARENT
    odom.child_frame_id = CHILD
    odom.pose.pose.position.x = estimate.pose.x
    odom.pose.pose.position.y = estimate.pose.y
    odom.pose.pose.orientation.z = sin(estimate.pose.yaw / 2)
    odom.pose.pose.orientation.w = cos(estimate.pose.yaw / 2)
    odom.twist.twist.linear.x = estimate.linear_x
    odom.twist.twist.angular.z = estimate.angular_z
    odom.pose.covariance = diagonal_matrix(diagonal_pose)
    odom.twist.covariance = diagonal_matrix(diagonal_twist)

    transform = TransformStamped()
    transform.header = odom.header
    transform.child_frame_id = CHILD
    transform.transform.translation.x = estimate.pose.x
    transform.transform.translation.y = estimate.pose.y
    transform.transform.rotation = odom.pose.pose.orientation
    return odom, transform


class ReceiptClock:
    def __init__(self, max_age_s, max_offset_step_s):
        self.max_age_ns = round(max_age_s * NS)
        self.max_offset_step_ns = round(max_offset_step_s * NS)
        self.last_stamp = 0
        self.offset = None

    def check(self, attempt, now_ns):
        receipt = attempt.received_ns
        stamp = attempt.ros_receipt_ns
        if not attempt.started_ns <= receipt <= now_ns:
            return "HOST_MONOTONIC_INVALID"
        if now_ns - receipt > self.max_age_ns or receipt - attempt.started_ns > self.max_age_ns:
            return "HOST_TELEMETRY_STALE"
        if not self.last_stamp < stamp < (1 << 31) * NS:
            return "ROS_STAMP_INVALID_OR_NONINCREASING"
        offset = stamp - receipt
        previous = self.offset
        self.offset = offset
        self.last_stamp = stamp
        if previous is not None and abs(offset - previous) > self.max_offset_step_ns:
            return "ROS_CLOCK_STEP"
        return None


class WheelOdometryNode(Node):
    def __init__(self, *, state_dir=None, parameter_overrides=None):
        super().__init__("sie_wheel_odometry", parameter_overrides=parameter_overrides)
        self.resources = ExitStack()
        try:
            self.configure(state_dir or Path.home() / ".local/state/sie")
        except BaseException:
            self.destroy_node()
            raise

    def fixed(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def configure(self, state_dir):
        geometry = WheelGeometry(**{
            f.name: self.fixed(f.name, f.default) for f in fields(WheelGeometry)
        })
        self.covariance = Covariance(**{
            f.name: self.fixed(f.name, f.default) for f in fields(Covariance)
        })
        rate = self.fixed("acquisition_hz", 10.0)
        self.timeout_s = self.fixed("uart_timeout_s", 0.25)
        host_age = self.fixed("max_host_age_s", 0.35)
        max_interval = self.fixed("max_sample_interval_s", 0.35)
        max_offset_step = self.fixed("max_ros_offset_step_s", 0.05)
        values = (rate, self.timeout_s, host_age, max_interval, max_offset_step)
        if (not all(isfinite(value) and value > 0 for value in values)
                or not 0.1 <= rate <= 20 or self.timeout_s > 3
                or max_interval < 0.000001):
            raise ValueError("Invalid wheel odometry acquisition or freshness limits")
        self.period_ns = round(NS / rate)
        self.engine = WheelOdometry(geometry, max_interval_us=round(max_interval * 1_000_000))
        self.receipt_clock = ReceiptClock(host_age, max_offset_step)
        self.enable_tf = self.fixed("publish_tf", False)
        self.port = self.fixed("uart_port", "/dev/ttyAMA0")
        journal_path = self.fixed("journal_path", "")
        if not journal_path or self.get_parameter("use_sim_time").value:
            raise ValueError("A new journal_path and use_sim_time=false are required")
        self.add_on_set_parameters_callback(self.refuse_parameter_changes)

        self.state_dir = state_dir
        self.resources.enter_context(lease(state_dir / "odom_base_link_authority.lock"))
        journal_fd = os.open(Path(journal_path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.journal = self.resources.enter_context(os.fdopen(journal_fd, "w", encoding="utf-8"))
        self.record({
            "event": "configuration", "wheel_geometry": asdict(geometry), "covariance": asdict(self.covariance),
            "acquisition_hz": rate, "uart_timeout_s": self.timeout_s, "max_host_age_s": host_age,
            "max_sample_interval_s": max_interval, "max_ros_offset_step_s": max_offset_step,
            "uart_port": self.port, "publish_tf": self.enable_tf, "navigation_admitted": False,
        })
        self.serial = None
        self.verified_boot = None
        self.status_required = True
        self.next_request_ns = 0
        self.blocked = None
        self.odom_publisher = None
        self.broadcaster = None
        self.diagnostic = self.create_publisher(DiagnosticArray, "/sie/diagnostics/wheel_odometry", 10)
        self.static_tf_listener = self.create_subscription(
            TFMessage, "/tf_static", self.check_static_tf,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.graph_ready_ns = time.monotonic_ns() + NS
        self.timer = self.create_timer(
            min(0.01, 0.2 / rate), self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    @staticmethod
    def refuse_parameter_changes(_):
        return SetParametersResult(successful=False, reason="Restart to establish a new odometry epoch")

    def check_static_tf(self, message):
        if any(transform.child_frame_id.lstrip("/") == CHILD for transform in message.transforms):
            self.block("FOREIGN_STATIC_BASE_LINK_TF")

    def check_graph(self):
        # Jazzy rclpy does not expose publisher GIDs on received TF messages.
        # In this stand-alone diagnostic graph refuse any other dynamic TF source.
        for topic, own in (("/odom", self.odom_publisher), ("/tf", self.broadcaster)):
            if self.count_publishers(self.resolve_topic_name(topic)) > int(own is not None):
                self.block("FOREIGN_ODOM_PUBLISHER" if topic == "/odom" else "FOREIGN_DYNAMIC_TF_PUBLISHER")
                return False
        return True

    def block(self, reason):
        if self.blocked is None:
            self.blocked = reason

    def record(self, row):
        self.journal.write(json.dumps(row, allow_nan=False) + "\n")
        self.journal.flush()

    def report(self, update, *, boot=None):
        good = update.estimate is not None
        status = DiagnosticStatus()
        status.name = "sie/wheel_odometry"
        status.hardware_id = self.port
        status.level = DiagnosticStatus.OK if good else DiagnosticStatus.WARN
        status.message = update.reason
        status.values = [KeyValue(key=key, value=value) for key, value in {
            "telemetry_valid": str(good).lower(), "navigation_admitted": "false",
            "boot_session_id": boot or self.verified_boot or "unknown",
            "continuity_disposition": update.disposition or "none",
            "motion_continuity_lost": str(bool(self.engine.discontinuities)).lower(),
            "discontinuities": str(self.engine.discontinuities),
        }.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic.publish(message)

    def tick(self):
        now = time.monotonic_ns()
        if now < self.graph_ready_ns or now < self.next_request_ns:
            return
        self.next_request_ns = now + self.period_ns
        if not self.blocked:
            self.check_graph()
        if self.blocked:
            self.report(self.engine.interrupt(self.blocked))
            return
        if self.serial is None:
            try:
                fd = self.resources.enter_context(open_uart(self.port, self.state_dir))
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.block(str(exc))
                self.report(self.engine.interrupt(self.blocked))
                return
            self.serial = ReadOnlyUart(fd, self.timeout_s, lambda: self.get_clock().now().nanoseconds)
            self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
            if self.enable_tf:
                self.broadcaster = TransformBroadcaster(self)

        self.next_request_ns = time.monotonic_ns() + self.period_ns
        self.poll_uart()

    def poll_uart(self):
        attempt = None
        sample = None
        status = None
        messages = None
        try:
            attempt = self.serial.make_attempt(status=self.status_required)
            received = self.serial.exchange(attempt)
            timing_refusal = self.receipt_clock.check(attempt, time.monotonic_ns())
            if self.status_required:
                status = received
                if not timing_refusal:
                    self.verified_boot = status["boot_session_id"]
                    self.status_required = False
                update = self.engine.interrupt(timing_refusal or "FIRMWARE_VERIFIED_AWAITING_SAMPLE")
            else:
                sample = received
                update = self.engine.observe(
                    sample, verified_boot=self.verified_boot, timing_refusal=timing_refusal,
                )
                if sample.boot_session_id != self.verified_boot and update.disposition != "RETIRED_BOOT":
                    self.status_required = True
                if update.estimate is not None:
                    messages = ros_messages(update.estimate, attempt.ros_receipt_ns, self.covariance)
        except (OSError, ValueError) as exc:
            # A failed transaction can leave a late reply. This session sends no retry.
            self.block(str(exc))
            update = self.engine.interrupt(self.blocked)
        finally:
            if attempt is not None and attempt.started_ns is not None:
                self.next_request_ns = attempt.started_ns + self.period_ns

        if messages is not None and not self.check_graph():
            messages = None
            update = self.engine.interrupt(self.blocked)
        try:
            self.record({
                "event": "uart_attempt", "attempt": asdict(attempt) if attempt else None,
                "status": status, "sample": asdict(sample) if sample else None,
                "result": asdict(update), "verified_boot": self.verified_boot,
                "navigation_admitted": False,
            })
        except (OSError, ValueError):
            self.block("EVIDENCE_WRITE_FAILED")
            self.report(self.engine.interrupt(self.blocked))
            return
        if messages is not None and time.monotonic_ns() - attempt.received_ns > self.receipt_clock.max_age_ns:
            update = self.engine.interrupt("PUBLICATION_STALE")
            try:
                self.record({"event": "publication_refused", "request_id": attempt.request_id, "reason": update.reason})
            except OSError:
                self.block("EVIDENCE_WRITE_FAILED")
            messages = None
        if messages is not None:
            odom, transform = messages
            self.odom_publisher.publish(odom)
            if self.broadcaster is not None:
                self.broadcaster.sendTransform(transform)
        self.report(update, boot=sample.boot_session_id if sample else None)

    def destroy_node(self):
        try:
            result = super().destroy_node()
        finally:
            self.resources.close()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
