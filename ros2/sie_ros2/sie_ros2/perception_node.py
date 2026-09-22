"""Validate and relay canonical perception measurements into the ROS graph."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import ContractError, decode_json, encode, validate_perception


class PerceptionContractNode(Node):
    def __init__(self) -> None:
        super().__init__("sie_perception_contract")
        self.declare_parameter("input_topic", "/sie/perception/raw_measurement")
        self.declare_parameter("output_topic", "/sie/perception/measurement")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.subscription = self.create_subscription(String, input_topic, self._callback, 10)
        self.get_logger().info(f"validating {input_topic} -> {output_topic}")

    def _callback(self, message: String) -> None:
        try:
            measurement = validate_perception(decode_json(message.data))
        except ContractError as error:
            self.get_logger().warning(f"rejected raw perception: {error}")
            return
        output = String()
        output.data = encode(measurement)
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = PerceptionContractNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
