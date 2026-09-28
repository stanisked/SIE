"""Validate and republish AR0234 YOLO observations without creating metric measurements."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import ContractError, decode_json, encode, validate_ar0234_observation


class Ar0234ObservationValidationNode(Node):
    """The Observation boundary: validates pixels-only data and makes no control decision."""

    def __init__(self) -> None:
        super().__init__("sie_ar0234_observation_validation")
        self.declare_parameter(
            "input_topic", "/sie/observations/ar0234_person_upper_body"
        )
        self.declare_parameter(
            "output_topic", "/sie/observations/ar0234_person_upper_body/validated"
        )
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._publisher = self.create_publisher(String, output_topic, 10)
        self._subscription = self.create_subscription(
            String, input_topic, self._on_observation, 10
        )
        self.get_logger().info(
            f"validating {input_topic} -> {output_topic}; "
            "this node never creates a Measurement or actuator command"
        )

    def _on_observation(self, message: String) -> None:
        try:
            observation = validate_ar0234_observation(decode_json(message.data))
        except ContractError as error:
            self.get_logger().warning(f"rejected AR0234 observation: {error}")
            return
        output = String()
        output.data = encode(observation)
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Ar0234ObservationValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
