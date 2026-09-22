"""Publish phase-1 supervisor state; this node never calls an actuator bridge."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import ContractError, decode_json, encode, supervisor_state


class SupervisorContractNode(Node):
    def __init__(self) -> None:
        super().__init__("sie_supervisor_contract")
        self.declare_parameter("input_topic", "/sie/navigation/decision")
        self.declare_parameter("output_topic", "/sie/supervisor/state")
        self.publisher = self.create_publisher(String, self.get_parameter("output_topic").value, 10)
        self.subscription = self.create_subscription(
            String, self.get_parameter("input_topic").value, self._callback, 10
        )
        self.get_logger().info("phase 1: actuator bridge disabled")

    def _callback(self, message: String) -> None:
        try:
            state = supervisor_state(decode_json(message.data))
        except ContractError as error:
            self.get_logger().warning(f"rejected navigation decision: {error}")
            return
        output = String()
        output.data = encode(state)
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = SupervisorContractNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
