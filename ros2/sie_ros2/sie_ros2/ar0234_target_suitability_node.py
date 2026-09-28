"""Interpret AR0234 target geometry without producing a metric measurement."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import (
    ContractError,
    ar0234_target_suitability,
    decode_json,
    encode,
)


class Ar0234TargetSuitabilityNode(Node):
    """Blocks edge-truncated targets from any later metric-measurement path."""

    def __init__(self) -> None:
        super().__init__("sie_ar0234_target_suitability")
        self.declare_parameter(
            "input_topic", "/sie/observations/ar0234_person_upper_body/validated"
        )
        self.declare_parameter(
            "output_topic", "/sie/interpretations/ar0234_target_suitability"
        )
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._publisher = self.create_publisher(String, output_topic, 10)
        self._subscription = self.create_subscription(
            String, input_topic, self._on_observation, 10
        )
        self.get_logger().info(
            f"interpreting target geometry {input_topic} -> {output_topic}; "
            "metric_measurement_authorized remains false"
        )

    def _on_observation(self, message: String) -> None:
        try:
            interpretation = ar0234_target_suitability(decode_json(message.data))
        except ContractError as error:
            self.get_logger().warning(f"rejected AR0234 observation: {error}")
            return
        output = String()
        output.data = encode(interpretation)
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Ar0234TargetSuitabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
