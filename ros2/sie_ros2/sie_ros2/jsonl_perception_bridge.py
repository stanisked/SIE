"""Read canonical SIE perception evidence from an append-only JSONL file."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contracts import ContractError, decode_json, encode, validate_perception


class PerceptionJsonlBridge(Node):
    """Follow one evidence log without altering it and publish only valid records."""

    def __init__(self) -> None:
        super().__init__("sie_perception_jsonl_bridge")
        self.declare_parameter("input_path", "")
        self.declare_parameter("output_topic", "/sie/perception/raw_measurement")
        self.declare_parameter("poll_interval_s", 0.2)
        self.declare_parameter("initial_position", "end")

        raw_path = self.get_parameter("input_path").value
        if not isinstance(raw_path, str) or not raw_path:
            raise ValueError("input_path must be a non-empty JSONL path")
        self.path = Path(raw_path).expanduser()
        self.publisher = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.offset = self._initial_offset(self.get_parameter("initial_position").value)
        interval = float(self.get_parameter("poll_interval_s").value)
        if interval <= 0:
            raise ValueError("poll_interval_s must be positive")
        self.timer = self.create_timer(interval, self._poll)
        self.published = 0
        self.get_logger().info(
            f"following {self.path} -> {self.get_parameter('output_topic').value}; "
            f"offset={self.offset}"
        )

    def _initial_offset(self, position: object) -> int:
        if position not in {"beginning", "end"}:
            raise ValueError("initial_position must be beginning or end")
        try:
            return self.path.stat().st_size if position == "end" else 0
        except FileNotFoundError:
            return 0

    def _poll(self) -> None:
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            return
        if size < self.offset:
            self.get_logger().warning("JSONL file was truncated; restarting at offset 0")
            self.offset = 0
        try:
            with self.path.open("r", encoding="utf-8") as stream:
                stream.seek(self.offset)
                lines = stream.readlines()
                self.offset = stream.tell()
        except OSError as error:
            self.get_logger().warning(f"cannot read JSONL: {error}")
            return
        for line in lines:
            text = line.strip()
            if not text:
                continue
            try:
                measurement = validate_perception(decode_json(text))
            except ContractError as error:
                self.get_logger().warning(f"rejected JSONL record: {error}")
                continue
            message = String()
            message.data = encode(measurement)
            self.publisher.publish(message)
            self.published += 1


def main() -> None:
    rclpy.init()
    node = PerceptionJsonlBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
