"""Track a moving AR0234 person target without imposing a static-scene rule.

This node publishes 2D target-track evidence only. It never produces a metric
Measurement, navigation decision, HTTP request, or actuator command.
"""

from __future__ import annotations

import secrets
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import (
    ContractError,
    ar0234_dynamic_target_candidate,
    decode_json,
    encode,
)


class Ar0234DynamicTargetTrackerNode(Node):
    """Assign a run-local track identifier to consecutive dynamic observations."""

    def __init__(self) -> None:
        super().__init__("sie_ar0234_dynamic_target_tracker")
        self.declare_parameter(
            "input_topic", "/sie/observations/ar0234_person_upper_body"
        )
        self.declare_parameter(
            "output_topic", "/sie/interpretations/ar0234_dynamic_target_track"
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(String, output_topic, 10)
        self._subscription = self.create_subscription(
            String, input_topic, self._on_observation, 10
        )
        self._track_epoch = 0
        self._active_track_id: str | None = None
        self._previous_center_x_px: float | None = None
        self._previous_center_y_px: float | None = None
        self._run_id = secrets.token_hex(8)
        self.get_logger().info(
            f"dynamic 2D target tracking {input_topic} -> {output_topic}; "
            "metric measurement and actuator access remain disabled"
        )

    def _new_track_id(self) -> str:
        self._track_epoch += 1
        return f"ar0234-dynamic-track:{self._run_id}:{self._track_epoch}"

    def _on_observation(self, message: String) -> None:
        try:
            candidate = ar0234_dynamic_target_candidate(decode_json(message.data))
        except ContractError as error:
            self.get_logger().warning(f"rejected AR0234 observation: {error}")
            return

        if candidate["disposition"] != "OBSERVED":
            self._active_track_id = None
            self._previous_center_x_px = None
            self._previous_center_y_px = None
            output = {
                **candidate,
                "track_state": "LOST_OR_AMBIGUOUS",
                "track_id": None,
                "center_delta_px": None,
            }
        else:
            target = candidate["candidate"]
            if type(target) is not dict:
                raise RuntimeError("observed dynamic candidate must be an object")
            bbox = target["bbox_xyxy_px"]
            center_x = float(target["center_x_px"])
            center_y = (float(bbox[1]) + float(bbox[3])) / 2.0
            if self._active_track_id is None:
                self._active_track_id = self._new_track_id()
                track_state = "ACQUIRED"
                center_delta_px: float | None = None
            else:
                track_state = "TRACKING"
                center_delta_px = (
                    (center_x - float(self._previous_center_x_px)) ** 2
                    + (center_y - float(self._previous_center_y_px)) ** 2
                ) ** 0.5
            self._previous_center_x_px = center_x
            self._previous_center_y_px = center_y
            output = {
                **candidate,
                "track_state": track_state,
                "track_id": self._active_track_id,
                "center_delta_px": center_delta_px,
            }

        ros_message = String()
        ros_message.data = encode(output)
        self._publisher.publish(ros_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Ar0234DynamicTargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
