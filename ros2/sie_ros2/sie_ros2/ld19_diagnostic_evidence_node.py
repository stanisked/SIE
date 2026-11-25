"""Bounded motor-off LD19 diagnostic evidence capture.

This node SUBSCRIBES to raw LaserScan and writes a local append-only journal.
It publishes no canonical SIE Observation/Measurement, modifies no World State
and has no motor/ESP32 interfaces. The draft record schema is not approved.
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .ld19_diagnostic_evidence import make_ld19_diagnostic_record


class Ld19DiagnosticEvidenceNode(Node):
    def __init__(self) -> None:
        super().__init__("siee_ld19_diagnostic_evidence")
        self.declare_parameter("output_jsonl", "~/.local/state/sie/streams/ld19_diagnostic_v0.jsonl")
        self.declare_parameter("max_records", 100)

        target = str(self.get_parameter("output_jsonl").value)
        self.path = Path(target).expanduser()
        if not self.path.is_absolute():
            raise ValueError("output_jsonl must be an absolute path")

        self.max_records = int(self.get_parameter("max_records").value)
        if not 1 <= self.max_records <= 5000:
            raise ValueError("max_records must be in [1, 5000]")

        self.record_count = 0
        self.done = False
        self.subscription = self.create_subscription(
            LaserScan, "/sie/lidar/scan_raw", self.on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"LD19 draft evidence capture: {self.path}; limit={self.max_records}; "
            "DIAGNOSTIC_ONLY; no downstream publisher or actuator"
        )

    def on_scan(self, scan: LaserScan) -> None:
        received_at_utc = datetime.now(timezone.utc).isoformat()
        try:
            record = make_ld19_diagnostic_record(
                scan, received_at_utc=received_at_utc
            )
            encoded = json.dumps(record, allow_nan=False, sort_keys=True, separators=(",", ":"))
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(
                str(self.path),
                os.O_WRONLY | os.O_CREAT | os.O_APPEND,
                0o600,
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    stream.write(encoded + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
            except Exception:
                # fdopen owns the descriptor once successfully constructed.
                raise
        except (ValueError, OSError, OverflowError) as error:
            self.get_logger().warning(f"LD19 diagnostic scan rejected: {error}")
            return

        self.record_count += 1
        if self.record_count >= self.max_records:
            self.done = True
            self.get_logger().info(
                f"LD19 diagnostic capture complete: {self.record_count} records; {self.path}"
            )


def main() -> None:
    rclpy.init()
    node = Ld19DiagnosticEvidenceNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
