"""Synthetic ROS graph and PTY tests. No sensor or actuator device is opened."""
from collections import deque
import json
import os
import pty
import select
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from sie_ros2.wheel_odometry import Estimate, Pose
from sie_ros2.wheel_odometry_node import Covariance, ReceiptClock, WheelOdometryNode, ros_messages
from sie_ros2.wheel_odometry_uart import FIRMWARE_BUILD, Attempt, frame_crc, lease

BOOT = "0123456789ABCDEF"
REBOOT = "1123456789ABCDEF"


def test_message_frames_stamps_and_covariance_consistency():
    estimate = Estimate(Pose(1.2, -0.3, .7), -0.6, 1.2, 4.0, 1.0, 2)
    odom, tf = ros_messages(estimate, 4_000_000_123, Covariance())
    assert odom.header == tf.header
    assert odom.header.stamp.sec == 4 and odom.header.stamp.nanosec == 123
    assert odom.header.frame_id == "odom" and odom.child_frame_id == tf.child_frame_id == "base_link"
    assert odom.pose.pose.position.x == tf.transform.translation.x == 1.2
    assert odom.pose.pose.position.y == tf.transform.translation.y == -.3
    assert odom.pose.pose.orientation == tf.transform.rotation
    assert odom.twist.twist.linear.x == -.6 and odom.twist.twist.angular.z == 1.2
    assert odom.pose.covariance[0] == pytest.approx(.55)
    assert odom.pose.covariance[35] == pytest.approx(.55)
    assert odom.pose.covariance[14] == 1e6
    assert all(odom.twist.covariance[index] > 0 for index in range(0, 36, 7))


def test_receipt_clock_refuses_stale_rollback_and_step():
    clock = ReceiptClock(.35, .05)
    attempt = Attempt("WHEEL_ENCODERS_V1", "00000001", "", started_ns=1_000_000_000,
                      received_ns=1_040_000_000, ros_receipt_ns=101_040_000_000)
    assert clock.check(attempt, 1_050_000_000) is None
    assert clock.check(attempt, 1_050_000_000) == "ROS_STAMP_INVALID_OR_NONINCREASING"
    assert clock.check(attempt, 1_500_000_000) == "HOST_TELEMETRY_STALE"
    attempt.started_ns = 1_100_000_000
    attempt.received_ns = 1_140_000_000
    attempt.ros_receipt_ns = 102_140_000_000
    assert clock.check(attempt, 1_150_000_000) == "ROS_CLOCK_STEP"
    attempt.started_ns = 1_200_000_000
    attempt.received_ns = 1_240_000_000
    attempt.ros_receipt_ns = 102_240_000_000
    assert clock.check(attempt, 1_250_000_000) is None


class SimulatedController:
    def __init__(self, master):
        self.master = master
        self.boot = BOOT
        self.trace = deque([
            (BOOT, 51, 0), (BOOT, 52, 10), (BOOT, 54, 45), (BOOT, 55, 55),
            (REBOOT, 1, 0), (REBOOT, 2, 2), (REBOOT, 3, 2),
        ])
        self.requests = []
        self.done = threading.Event()

    def run(self):
        buffer = b""
        while not self.done.is_set():
            if not select.select([self.master], [], [], 0.02)[0]:
                continue
            try:
                buffer += os.read(self.master, 4096)
            except OSError:
                time.sleep(0.01)
                continue
            while b"\n" in buffer:
                request, buffer = buffer.split(b"\n", 1)
                core, crc = request.rsplit(b"|", 1)
                assert frame_crc(core) == crc
                _, identity, operation = core.split(b"|")
                self.requests.append((identity, operation))
                if operation == b"STATUS":
                    body = dict(firmware_build_id=FIRMWARE_BUILD, boot_session_id=self.boot,
                                state="READY", fault=None, bounded_fault_latched=False,
                                steered_safety_latched=False, remote_stop_pending=False)
                else:
                    assert operation == b"WHEEL_ENCODERS_V1"
                    if not self.trace:
                        continue
                    self.boot, seq, ticks = self.trace.popleft()
                    body = dict(version=1, boot_session_id=self.boot, sample_sequence=str(seq),
                                sample_monotonic_us=str(seq * 100_000),
                                right_total_signed_ticks=str(ticks), left_total_signed_ticks=str(ticks),
                                controller_state="READY", fault_latched=False,
                                steered_safety_latched=False, right_counter_overflow=False,
                                left_counter_overflow=False, sequence_exhausted=False)
                response = b"@1|" + identity + b"|200|" + json.dumps(body).encode()
                os.write(self.master, response + b"|" + frame_crc(response) + b"\r\n")


@pytest.fixture
def graph():
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    def spin_until(predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        assert predicate()

    def remove(node):
        executor.remove_node(node)
        nodes.remove(node)
        node.destroy_node()

    try:
        yield add, spin_until, remove
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


def parameters(path, port="/dev/pts/invalid", enable_tf=True):
    return [Parameter("journal_path", value=str(path)), Parameter("uart_port", value=port),
            Parameter("publish_tf", value=enable_tf)]


@pytest.mark.parametrize("publish_tf", [True, False])
def test_ros_end_to_end_pty_gap_reboot_timeout(tmp_path, graph, publish_tf):
    add, spin_until, remove = graph
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    simulator = SimulatedController(master)
    thread = threading.Thread(target=simulator.run)
    thread.start()
    node = None
    try:
        journal = tmp_path / "probe.jsonl"
        node = add(WheelOdometryNode(state_dir=tmp_path, parameter_overrides=parameters(journal, port, publish_tf)))
        client = add(Node("wheel_test_receiver"))
        odom, tf, diagnostics = [], [], []
        client.create_subscription(Odometry, "/odom", odom.append, 10)
        client.create_subscription(TFMessage, "/tf", lambda msg: tf.extend(msg.transforms), 10)
        client.create_subscription(DiagnosticArray, "/sie/diagnostics/wheel_odometry",
                                   lambda msg: diagnostics.extend(msg.status), 10)
        static_pub = client.create_publisher(
            TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        unrelated = TransformStamped(child_frame_id="ld19_sensor")
        unrelated.header.frame_id = "base_link"
        static_pub.publish(TFMessage(transforms=[unrelated]))
        assert not node.set_parameters([Parameter("track_width_m", value=.5)])[0].successful
        assert not node.set_parameters([Parameter("use_sim_time", value=True)])[0].successful
        spin_until(lambda: any(item.message == "UART_REPLY_TIMEOUT" for item in diagnostics))
        assert len(odom) == 3
        if publish_tf:
            assert [item.header for item in odom] == [item.header for item in tf]
        else:
            assert not tf and node.count_publishers("/tf") == 0
        assert odom[2].pose.pose == odom[1].pose.pose
        assert odom[2].twist.twist.linear.x == 0.0
        assert all(key.value == "false" for item in diagnostics for key in item.values
                   if key.key == "navigation_admitted")
        assert len(simulator.requests) == 10
        assert len({identity for identity, _ in simulator.requests}) == len(simulator.requests)
        rows = [json.loads(line) for line in journal.read_text().splitlines()]
        assert [row["result"]["reason"] for row in rows[1:]] == [
            "FIRMWARE_VERIFIED_AWAITING_SAMPLE", "BASELINE", "VALID", "SAMPLE_GAP", "VALID",
            "FIRMWARE_STATUS_REQUIRED", "FIRMWARE_VERIFIED_AWAITING_SAMPLE",
            "RECOVERY_BASELINE", "VALID", "UART_REPLY_TIMEOUT",
        ]
        assert rows[6]["result"]["disposition"] == "REBOOT_BASELINE"
        assert rows[-1]["attempt"]["received_ns"] is None
        assert rows[-1]["attempt"]["written_hex"]
    finally:
        if node is not None:
            remove(node)
        simulator.done.set()
        thread.join(timeout=1)
        os.close(master)


@pytest.mark.parametrize("foreign", ["/odom", "/tf", "/tf_static"])
def test_foreign_authority_refused_before_uart(tmp_path, graph, foreign):
    add, spin_until, _ = graph
    node = add(WheelOdometryNode(state_dir=tmp_path, parameter_overrides=parameters(tmp_path / "journal")))
    peer = add(Node("conflicting_publisher"))
    if foreign == "/tf_static":
        publisher = peer.create_publisher(
            TFMessage, foreign, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        transform = TransformStamped(child_frame_id="base_link")
        transform.header.frame_id = "map"
        publisher.publish(TFMessage(transforms=[transform]))
    else:
        peer.create_publisher(Odometry if foreign == "/odom" else TFMessage, foreign, 10)
    spin_until(lambda: node.blocked is not None)
    assert node.blocked.startswith("FOREIGN_") and node.serial is None


def test_second_local_authority_refused(tmp_path, graph):
    with lease(tmp_path / "odom_base_link_authority.lock"):
        with pytest.raises(RuntimeError, match="AUTHORITY_ALREADY_OWNED"):
            WheelOdometryNode(state_dir=tmp_path, parameter_overrides=parameters(tmp_path / "journal"))
