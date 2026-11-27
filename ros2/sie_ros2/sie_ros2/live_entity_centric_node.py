"""Explicit opt-in ROS observer for independent Step 74 diagnostics."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import threading
import time

from .latest_diagnostic_writer import LatestDiagnosticWriter
from .live_entity_centric_v1 import (
    BOUNDARY, CAMERA_INPUT, CAMERA_OUTPUT, LD19_OUTPUT, POLICY, SCHEMA,
    camera_event, camera_view, evidence, obstacle_event, scan_view,
)
from .ld19_diagnostic_evidence import SOURCE_TOPIC
from .live_temporal_association_v1 import FRESHNESS, PAIRING, RECENT
from .temporal_association_freshness_v1 import classify_source_age


class Rate:
    """Count and average cadence on an explicit monotonic endpoint."""

    def __init__(self):
        self.count = 0
        self.first_ns = self.last_ns = None

    def add(self, instant):
        if instant is None or (self.last_ns is not None and instant < self.last_ns):
            return
        if self.first_ns is None:
            self.first_ns = instant
        self.last_ns = instant
        self.count += 1

    def report(self):
        hz = (self.count - 1) * 1e9 / (self.last_ns - self.first_ns) if (
            self.count > 1 and self.last_ns > self.first_ns) else None
        return {"count": self.count, "first_monotonic_ns": self.first_ns,
                "last_monotonic_ns": self.last_ns, "hz": hz}


class Lane:
    """One pending item via the existing diagnostic latest-wins writer."""

    def __init__(self, directory, adapter, publish, clock, *, use_utc=False):
        self.adapter, self.publish, self.clock = adapter, publish, clock
        self.use_utc = use_utc
        self.input_rate = Rate()
        self.rates = {}
        self.dispositions = Counter()
        self.durations = {}
        self.max_pending = self.publish_errors = 0
        self.writer = LatestDiagnosticWriter(directory, write=self._process)
        try:
            self.journal = (directory / "events.jsonl").open("x", encoding="utf-8")
        except OSError:
            self.writer.close()
            raise

    def submit(self, message):
        receipt = self.clock()
        self.input_rate.add(receipt)
        utc_ns = time.time_ns() if self.use_utc else None
        self.writer.submit((message, receipt, self.input_rate.count, utc_ns))
        pending = self.writer.pending.qsize()
        self.max_pending = max(self.max_pending, pending, int(self.writer.dropped > 0))

    def _process(self, job):
        message, receipt, sequence, utc_ns = job
        started = self.clock()
        row, published_ns, rates, statuses, ages = self.adapter(message, receipt, sequence, utc_ns)
        row.update(schema_version=SCHEMA, policy_id=POLICY, receipt_sequence=sequence,
                   subscriber_received_monotonic_ns=receipt, publication_endpoint_monotonic_ns=published_ns,
                   timing_ns={"queue_wait": started - receipt, "preview_processing": published_ns - started,
                              "callback_to_preview": published_ns - receipt, **ages})
        for name, instant in rates.items():
            self.rates.setdefault(name, Rate()).add(instant)
        self.dispositions.update(statuses)
        for name, duration in row["timing_ns"].items():
            if duration is not None:
                stats = self.durations.setdefault(name, {"count": 0, "sum_ns": 0, "max_ns": duration})
                stats["count"] += 1
                stats["sum_ns"] += duration
                stats["max_ns"] = max(stats["max_ns"], duration)
        encoded = json.dumps(row, separators=(",", ":"), allow_nan=False)
        if self.publish is not None:
            try:
                self.publish(encoded)
            except Exception:
                self.publish_errors += 1
            else:
                self.rates.setdefault("ros_output", Rate()).add(published_ns)
        self.journal.write(encoded + "\n")
        self.journal.flush()

    def close(self):
        self.writer.close()
        self.journal.close()

    def report(self):
        return {"input": self.input_rate.report(),
                "rates": {name: value.report() for name, value in self.rates.items()},
                "dispositions": dict(self.dispositions), "timing_ns": self.durations,
                "dropped_pending": self.writer.dropped, "max_pending_observed": self.max_pending,
                "pending": self.writer.pending.qsize(), "processing_or_journal_errors": self.writer.errors,
                "last_error": self.writer.last_error, "publish_errors": self.publish_errors}


class DiagnosticSession:
    def __init__(self, directory, mode, decode_camera, decode_scan, *, publish_camera=None,
                 publish_ld19=None, clock=time.monotonic_ns):
        if mode not in ("baseline", "enabled"):
            raise ValueError("explicit diagnostic mode required")
        self.directory, self.mode, self.clock = directory, mode, clock
        self.decode_camera, self.decode_scan = decode_camera, decode_scan
        self.camera_stamps, self.last_scan_header = {}, None
        self.enable = mode == "enabled"
        directory.mkdir(mode=0o700)
        self.camera = Lane(directory / "camera", self.camera_adapter,
                           publish_camera if self.enable else None, clock)
        try:
            self.ld19 = Lane(directory / "ld19", self.ld19_adapter,
                             publish_ld19 if self.enable else None, clock, use_utc=True)
        except OSError:
            self.camera.close()
            raise
        self.camera.rates.update({key: Rate() for key in ("rgb_producer", "rgb_stereo_spatial_producer",
                                                         "entity_preview", "ros_output")})
        self.ld19.rates.update({key: Rate() for key in ("obstacle_updates", "ros_output")})

    def camera_adapter(self, raw, _receipt, _sequence, _utc):
        audit = self.decode_camera(raw)
        cycle, semantic, spatial, timing, producer, now = camera_view(audit, self.clock, self.camera_stamps)
        if self.enable:
            row = camera_event(cycle, semantic, spatial)
            row["source_timing"] = timing
        else:
            row = {**BOUNDARY, "cycle_id": cycle, "semantic_status": semantic["status"],
                   "spatial_status": spatial["status"]}
        row.update(stream="camera", mode=self.mode, producer=producer)
        rgb, stereo = timing["ar0234"], timing["ov9281"]
        rates = {}
        if producer["rgb_inference_completed"] and rgb["status"] in ("AVAILABLE", "STALE"):
            rates["rgb_producer"] = rgb["source_timestamp_ns"]
        if producer["spatial_status"] == "AVAILABLE" and stereo["status"] in ("AVAILABLE", "STALE"):
            rates["rgb_stereo_spatial_producer"] = stereo["source_timestamp_ns"]
        if row.get("provisional_entity") is not None:
            rates["entity_preview"] = now
        ages = {}
        for key, source in (("rgb", rgb), ("stereo", stereo)):
            ages[f"{key}_driver_declared_age"] = source.get("age_ns")
            packet_receipt = source.get("packet_received_monotonic_ns")
            ages[f"{key}_packet_receipt_to_preview"] = None if packet_receipt is None else now - packet_receipt
        statuses = (f"semantic:{semantic['status']}", f"spatial:{spatial['status']}",
                    f"rgb_packet:{rgb['status']}", f"stereo_packet:{stereo['status']}",
                    f"camera_pair:{timing['actual_camera_pair']['status']}")
        return row, now, rates, statuses, ages

    def ld19_adapter(self, raw, receipt, sequence, utc_ns):
        if not self.enable:
            now = self.clock()
            row = {**BOUNDARY, "stream": "ld19", "mode": self.mode}
            return row, now, {}, (), {"ld19_receipt_age": now - receipt}
        message = self.decode_scan(raw)
        utc = datetime.fromtimestamp(utc_ns / 1e9, timezone.utc).isoformat()
        try:
            scan, self.last_scan_header = scan_view(message, utc, sequence, receipt, self.last_scan_header)
        except (ValueError, OverflowError, OSError) as error:
            scan = evidence("REJECTED", str(error))
        now = self.clock()
        age = now - receipt
        freshness = classify_source_age(age, FRESHNESS.max_ld19_ros_header_age_ns, FRESHNESS.max_future_skew_ns)
        if scan["status"] == "AVAILABLE" and freshness != RECENT:
            scan = scan | {"status": "STALE" if freshness == "SOURCE_STALE" else "REJECTED",
                           "reason": f"HOST_RECEIPT_{freshness}"}
        scan["host_receipt_freshness"] = freshness
        row = obstacle_event(scan) | {"stream": "ld19", "mode": self.mode}
        rates = {"obstacle_updates": now} if scan["status"] == "AVAILABLE" else {}
        return row, now, rates, (f"ld19:{scan['status']}",), {"ld19_receipt_age": age}

    def close(self):
        self.camera.close()
        self.ld19.close()
        summary = {**BOUNDARY, "schema_version": SCHEMA, "policy_id": POLICY, "mode": self.mode,
                   "camera": self.camera.report(), "ld19": self.ld19.report(),
                   "timing_policy_ids": {"pairing": PAIRING.policy_id, "freshness": FRESHNESS.policy_id},
                   "pending_capacity_per_stream": 1, "same_host_camera_required": True,
                   "ld19_header_age_at_publication": "UNKNOWN"}
        with (self.directory / "summary.json").open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, sort_keys=True, separators=(",", ":"), allow_nan=False)
            stream.write("\n")


def main():
    # Keep ROS out of the source adapters and software-only tests.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("baseline", "enabled"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    os.umask(0o077)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("sie_live_entity_centric_diagnostic")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = ({topic: node.create_publisher(String, topic, qos) for topic in (CAMERA_OUTPUT, LD19_OUTPUT)}
                  if args.mode == "enabled" else {})
    session = None
    try:
        session = DiagnosticSession(
            args.output, args.mode, lambda raw: json.loads(deserialize_message(raw, String).data),
            lambda raw: deserialize_message(raw, LaserScan),
            publish_camera=(lambda data: publishers[CAMERA_OUTPUT].publish(String(data=data))),
            publish_ld19=(lambda data: publishers[LD19_OUTPUT].publish(String(data=data))),
        )
        node.create_subscription(String, CAMERA_INPUT, session.camera.submit, qos, raw=True)
        node.create_subscription(LaserScan, SOURCE_TOPIC, session.ld19.submit, qos, raw=True)
        (args.output / "ready").touch(exist_ok=False)
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        # Drain the workers while the publishers are still valid.
        try:
            if session is not None:
                session.close()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
