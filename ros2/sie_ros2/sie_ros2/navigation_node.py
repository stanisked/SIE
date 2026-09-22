"""Convert validated SIE measurements into non-executing navigation recommendations."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import ContractError, decode_json, encode, navigation_decision, validate_perception


class NavigationContractNode(Node):
    def __init__(self) -> None:
        super().__init__("sie_navigation_contract")
        self.declare_parameter("input_topic", "/sie/perception/measurement")
        self.declare_parameter("output_topic", "/sie/navigation/decision")
        self.declare_parameter("safe_distance_m", 2.0)
        self.declare_parameter("bearing_deadband_deg", 2.0)
        self.declare_parameter("min_confidence", 0.5)
        self.publisher = self.create_publisher(String, self.get_parameter("output_topic").value, 10)
        self.subscription = self.create_subscription(
            String, self.get_parameter("input_topic").value, self._callback, 10
        )

    def _callback(self, message: String) -> None:
        try:
            measurement = validate_perception(decode_json(message.data))
            decision = navigation_decision(
                measurement,
                safe_distance_m=float(self.get_parameter("safe_distance_m").value),
                bearing_deadband_deg=float(self.get_parameter("bearing_deadband_deg").value),
                min_confidence=float(self.get_parameter("min_confidence").value),
            )
        except ContractError as error:
            self.get_logger().warning(f"rejected perception measurement: {error}")
            return
        output = String()
        output.data = encode(decision)
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = NavigationContractNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
