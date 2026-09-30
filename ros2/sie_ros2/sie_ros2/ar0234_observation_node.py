"""AR0234 + YOLO ROS 2 Observation producer with append-only evidence output.

This node deliberately emits an image Observation only. It does not produce a
metric depth Measurement, a navigation decision, or an actuator command.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_AR0234_DEVICE = (
    "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
)


class Ar0234ObservationNode(Node):
    def __init__(self) -> None:
        super().__init__("sie_ar0234_observation")
        self.declare_parameter("project_root", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_device", DEFAULT_AR0234_DEVICE)
        self.declare_parameter(
            "output_jsonl",
            "~/.local/state/sie/streams/ar0234_yolo_observations_v1.jsonl",
        )
        self.declare_parameter(
            "output_topic", "/sie/observations/ar0234_person_upper_body"
        )
        self.declare_parameter("confidence_threshold", 0.40)
        self.declare_parameter("frame_rate_hz", 5.0)
        self.declare_parameter("auto_exposure", 3)

        self.project_root = self._absolute_path("project_root")
        self.model_path = self._absolute_path("model_path")
        self.output_path = Path(
            str(self.get_parameter("output_jsonl").value)
        ).expanduser()
        self.camera_device = str(self.get_parameter("camera_device").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        rate_hz = float(self.get_parameter("frame_rate_hz").value)
        confidence = float(self.get_parameter("confidence_threshold").value)
        if rate_hz <= 0.0:
            raise ValueError("frame_rate_hz must be positive")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")

        if str(self.project_root) not in sys.path:
            sys.path.insert(0, str(self.project_root))
        try:
            import cv2
            from vision_core.person_localization.yolo11_person_upper_body_runtime import (
                OnnxRuntimeYolo11PersonUpperBodyObserver,
            )
        except ImportError as error:
            raise RuntimeError(
                "required CV runtime is unavailable; run this node from sie_mvp "
                "after sourcing /opt/ros/jazzy/setup.bash"
            ) from error

        self.cv2 = cv2
        self.observer = OnnxRuntimeYolo11PersonUpperBodyObserver(
            self.model_path, confidence_threshold=confidence
        )
        exposure = int(self.get_parameter("auto_exposure").value)
        subprocess.run(
            ["v4l2-ctl", "-d", self.camera_device, "-c", f"auto_exposure={exposure}"],
            check=True,
            text=True,
            capture_output=True,
        )
        self.capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open AR0234 device: {self.camera_device}")

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.sequence = 0
        self.timer = self.create_timer(1.0 / rate_hz, self._observe)
        self.get_logger().info(
            f"AR0234 observation -> {self.output_topic}; "
            f"evidence={self.output_path}; actuator access disabled"
        )

    def _absolute_path(self, parameter_name: str) -> Path:
        value = str(self.get_parameter(parameter_name).value)
        path = Path(value).expanduser()
        if not value or not path.is_absolute():
            raise ValueError(f"{parameter_name} must be an absolute path")
        return path

    def _observe(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning("AR0234 frame read failed")
            return
        self.sequence += 1
        captured_at = datetime.now(timezone.utc)
        cycle_id = f"ar0234-ros2-{self.sequence:08d}"
        try:
            observation = self.observer.observe(
                frame, captured_at_utc=captured_at, cycle_id=cycle_id
            )
            self._append_evidence(observation)
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().warning(f"AR0234 observation rejected: {error}")
            return

        message = String()
        message.data = json.dumps(observation, allow_nan=False, sort_keys=True)
        self.publisher.publish(message)

    def _append_evidence(self, observation: dict[str, Any]) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        line = json.dumps(observation, allow_nan=False, sort_keys=True) + "\n"
        with self.output_path.open("a", encoding="utf-8") as stream:
            stream.write(line)
            stream.flush()
            os.fsync(stream.fileno())

    def destroy_node(self) -> bool:
        if hasattr(self, "capture"):
            self.capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ar0234ObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
