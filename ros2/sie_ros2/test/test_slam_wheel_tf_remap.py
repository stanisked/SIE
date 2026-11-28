"""Check the Step 75.5a TF route against the unchanged odometry authority guard."""
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

from sie_ros2.wheel_odometry_node import WheelOdometryNode


def test_wheel_tf_guard_accepts_mapper_topic_and_rejects_private_duplicate(tmp_path):
    private = "/sie/step75_5a/wheel_tf"
    rclpy.init(args=["--ros-args", "-r", f"sie_wheel_odometry:/tf:={private}"])
    nodes = []
    try:
        wheel = WheelOdometryNode(
            state_dir=tmp_path,
            parameter_overrides=[Parameter("journal_path", value=str(tmp_path / "wheel.jsonl"))],
        )
        nodes.append(wheel)
        # Drive the guard directly with graph endpoints. The timer never opens UART.
        peer = Node("test_slam_mapper")
        nodes.append(peer)
        peer.create_publisher(TFMessage, "/tf", 10)
        assert wheel.resolve_topic_name("/tf") == private
        assert peer.count_publishers("/tf") == 1
        assert wheel.check_graph()
        wheel.broadcaster = SimpleNamespace(publisher=wheel.create_publisher(TFMessage, "/tf", 10))
        assert wheel.check_graph()
        peer.create_publisher(TFMessage, private, 10)
        assert not wheel.check_graph()
        assert wheel.blocked == "FOREIGN_DYNAMIC_TF_PUBLISHER"
        assert wheel.serial is None
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()
