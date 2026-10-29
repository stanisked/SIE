#!/usr/bin/env python3
"""Collect raw control-preview audits for Shadow sensor integration, no UART.

Run while the existing camera-only preview publishes its diagnostic topic.
This tool creates a JSONL record and a small field inventory. It does not
create ShadowObservation or infer person range from a diagnostic depth field.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import secrets
import time


RANGE_WORDS = ("depth", "distance", "range", "track", "association", "frame")


def paths_with_sensor_words(value, prefix="", *, max_paths=200):
    paths = set()
    def walk(obj, path):
        if len(paths) >= max_paths:
            return
        if isinstance(obj, dict):
            for key, child in obj.items():
                child_path = path + "." + key if path else key
                if any(word in key.lower() for word in RANGE_WORDS):
                    paths.add(child_path)
                walk(child, child_path)
        elif isinstance(obj, list) and obj:
            walk(obj[0], path + "[]")
    walk(value, prefix)
    return sorted(paths)


def collect(*, topic, duration_s, max_records, output, rclpy, node_class,
            string_class, qos):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    counts = Counter()
    schemas = Counter()
    keys = set()
    paths = set()
    calibrations = set()
    samples = []
    malformed = 0
    rclpy.init()
    node = node_class("sie_shadow_sensor_audit_" + secrets.token_hex(3))
    journal_path = output / "audit.jsonl"
    try:
        with journal_path.open("x") as journal:
            def callback(message):
                nonlocal malformed
                if sum(counts.values()) >= max_records:
                    return
                try:
                    record = json.loads(message.data)
                    if not isinstance(record, dict):
                        raise ValueError("AUDIT_NOT_OBJECT")
                except (ValueError, TypeError):
                    malformed += 1
                    return
                journal.write(json.dumps(record, ensure_ascii=False,
                                         allow_nan=False) + "\n")
                journal.flush()
                counts[str(record.get("status"))] += 1
                schemas[str(record.get("schema_version"))] += 1
                keys.update(record)
                paths.update(paths_with_sensor_words(record))
                calibration = record.get("calibration")
                if isinstance(calibration, dict):
                    calibrations.add(json.dumps(calibration, sort_keys=True))
                if len(samples) < 3:
                    targets = record.get("ar0234_dynamic_target") or {}
                    segmentation = record.get("segmentation_observation") or {}
                    instances = segmentation.get("instances")
                    samples.append({
                        "cycle_id": record.get("cycle_id"),
                        "status": record.get("status"),
                        "target_disposition": targets.get("disposition"),
                        "target_reason": targets.get("reason"),
                        "eligible_count": targets.get("current_eligible_count"),
                        "segmentation_instances": (len(instances)
                                                   if isinstance(instances, list) else None),
                    })
            node.create_subscription(string_class, topic, callback, qos)
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline and sum(counts.values()) < max_records:
                rclpy.spin_once(node, timeout_sec=min(.2, max(0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    summary = {
        "schema_version": "sie.shadow_sensor_audit_inventory.v1",
        "purpose": "RAW_PREVIEW_FIELD_INVENTORY_DIAGNOSTIC_ONLY",
        "motor_posts": 0, "uart_requests": 0,
        "duration_s": duration_s, "records": sum(counts.values()),
        "malformed_messages": malformed,
        "status_counts": dict(counts), "schema_counts": dict(schemas),
        "top_level_keys": sorted(keys),
        "sensor_related_paths": sorted(paths),
        "calibrations": [json.loads(item) for item in sorted(calibrations)],
        "samples": samples,
        "audit_jsonl": str(journal_path),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2,
                                                    ensure_ascii=False) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--max-records", type=int, default=300)
    args = parser.parse_args()
    if not 1 <= args.duration_s <= 60 or not 1 <= args.max_records <= 1000:
        parser.error("duration 1..60 s and max-records 1..1000 required")

    from vision_core.tools.run_v7_supervised_one_step import TOPIC
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    output = Path.home() / ".local/state/sie/evidence" / (
        "shadow_sensor_audit_" + str(int(time.time())) + "_" + secrets.token_hex(4))
    report = collect(topic=TOPIC, duration_s=args.duration_s,
                     max_records=args.max_records, output=output,
                     rclpy=rclpy, node_class=Node, string_class=String,
                     qos=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                    reliability=ReliabilityPolicy.RELIABLE))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["records"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
