"""Bounded runtime plumbing for the read-only temporal shadow side stream."""

from collections import deque
import json
import os
from pathlib import Path
import queue
import threading
import time

from .host_timebase_bridge_v1 import make_host_timebase_bridge
from .host_timebase_clock_probe_v1 import collect_host_clock_brackets
from .ld19_timestamp_policy_v1 import EXPECTED_FRAME, EXPECTED_TOPIC, NANOSECONDS
from .latest_v4l2_capture import CameraTiming
from .live_temporal_association_v1 import (
    BOUNDARY, BRIDGE_POLICY, SCHEMA, CycleTiming, ScanTiming, evaluate_cycle, nanoseconds,
)

CAMERA_HISTORY_SIZE = 128
LD19_HISTORY_SIZE = 64
CLOCK_HISTORY_SIZE = 64
JOURNAL_QUEUE_SIZE = 8
CLOCK_PERIOD_NS = 100_000_000


class LiveTemporalShadowMonitor:
    def __init__(self, path, session, cameras, context, ld19_clock_evidence):
        self.path = path
        self.session = session
        self.cameras = cameras
        self.context = context
        self.ld19_clock_evidence = ld19_clock_evidence or None
        self.boot_id = None
        self._stop = threading.Event()
        self._journal_unavailable = threading.Event()
        self._history_lock = threading.Lock()
        self._scans = deque(maxlen=LD19_HISTORY_SIZE)
        self._scan_evicted_through = None
        self._windows = deque(maxlen=CLOCK_HISTORY_SIZE)
        self._pending = queue.Queue(maxsize=JOURNAL_QUEUE_SIZE)
        self._receipt_sequence = 0
        self.dropped = self.written = self.write_errors = self.evaluation_errors = 0
        self.hook_errors = self.clock_errors = self.subscriber_errors = 0
        self.subscriber_error = None
        self.last_journal_error = None
        self._writer = threading.Thread(target=self._write, name="sie-temporal-journal", daemon=True)
        self._subscriber = threading.Thread(target=self._receive, name="sie-temporal-ld19", daemon=True)
        self._writer.start()
        self._subscriber.start()

    def submit(self, cycle_id, preview_status, selected, capture_metadata, result_ready_ns, publication_ns):
        """Snapshot metadata only after the primary preview path has published."""
        if self._stop.is_set() or self._journal_unavailable.is_set() or self._pending.full():
            self.dropped += 1
            return
        ar = selected.get("ar0234")
        ov = selected.get("ov9281")
        anchor = ar or {}
        cycle = CycleTiming(
            cycle_id, preview_status,
            CameraTiming.from_packet(ar) if ar else None,
            CameraTiming.from_packet(ov) if ov else None,
            anchor.get("rgb_trigger_policy"),
            nanoseconds(anchor.get("pair_request_monotonic_s")),
            nanoseconds(anchor.get("rgb_packet_frozen_monotonic_s")),
            nanoseconds(ov["pair_selected_monotonic_s"]) if ov else None,
            result_ready_ns, publication_ns,
            {name: {"status": row.get("status", "PACKET_READ"),
                    "reason": row.get("reason")}
             for name, row in capture_metadata.items()},
        )
        cameras = {name: camera.timing_history() for name, camera in self.cameras.items()}
        with self._history_lock:
            job = (cycle, cameras, tuple(self._scans), self._scan_evicted_through, tuple(self._windows))
        try:
            self._pending.put_nowait(job)
        except queue.Full:
            self.dropped += 1

    def _on_scan(self, scan, node):
        received = time.monotonic_ns()
        header = scan.header
        sec, nsec = header.stamp.sec, header.stamp.nanosec
        stamp = sec * NANOSECONDS + nsec
        source_status = "HEADER_PRESENT"
        if sec < 0 or not 0 <= nsec < NANOSECONDS or stamp <= 0:
            stamp = None
            source_status = "INVALID_ROS_HEADER_STAMP"
        if header.frame_id != EXPECTED_FRAME:
            source_status = "UNEXPECTED_LD19_FRAME"
        clock_status = "ROS_CLOCK_DOMAIN_UNVERIFIED"
        if self.ld19_clock_evidence:
            clock_status = ("OPERATOR_ATTESTED_SAME_HOST_REALTIME"
                            if node.count_publishers(EXPECTED_TOPIC) == 1
                            else "LD19_PUBLISHER_NOT_UNIQUE")
        with self._history_lock:
            self._receipt_sequence += 1
            if len(self._scans) == self._scans.maxlen:
                self._scan_evicted_through = self._scans[0].received_monotonic_ns
            self._scans.append(ScanTiming(
                self._receipt_sequence, stamp, received, header.frame_id[:128],
                clock_status, source_status,
            ))

    def _receive(self):
        # Run on its own executor so a camera/segmentation callback cannot
        # defer LD19 receipt stamping until the camera result is ready.
        try:
            self._spin_subscriber()
        except Exception as error:
            self.subscriber_errors += 1
            self.subscriber_error = type(error).__name__

    def _spin_subscriber(self):
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        node = executor = None
        try:
            try:
                self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip() or None
            except OSError:
                self.clock_errors += 1
            node = Node("siee_temporal_shadow_ld19", context=self.context, use_global_arguments=False)
            node.create_subscription(
                LaserScan, EXPECTED_TOPIC, lambda scan: self._on_scan(scan, node),
                qos_profile_sensor_data,
            )
            executor = SingleThreadedExecutor(context=self.context)
            executor.add_node(node)
            previous = None
            next_sample = 0
            while self.context.ok() and not self._stop.is_set():
                if time.monotonic_ns() >= next_sample and self.boot_id:
                    try:
                        bridge = make_host_timebase_bridge(
                            collect_host_clock_brackets(count=3, interval_ms=0),
                            boot_id=self.boot_id, policy=BRIDGE_POLICY, previous_bridge=previous,
                        )
                        if bridge["status"] == "DIAGNOSTIC_BRIDGE_READY":
                            previous = bridge
                    except (OSError, ValueError):
                        self.clock_errors += 1
                        bridge = {"status": "CLOCK_SAMPLING_ERROR"}
                    now = time.monotonic_ns()
                    with self._history_lock:
                        self._windows.append((now, bridge))
                    next_sample = now + CLOCK_PERIOD_NS
                executor.spin_once(timeout_sec=.05)
        except ExternalShutdownException:
            pass
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()

    def counters(self):
        return {
            "written": self.written, "dropped": self.dropped, "pending": self._pending.qsize(),
            "write_errors": self.write_errors, "evaluation_errors": self.evaluation_errors,
            "hook_errors": self.hook_errors, "clock_errors": self.clock_errors,
            "subscriber_errors": self.subscriber_errors, "subscriber_error": self.subscriber_error,
            "last_journal_error": self.last_journal_error,
        }

    def _append(self, stream, row):
        try:
            stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
            stream.flush()
            return True
        except (OSError, ValueError) as error:
            self.write_errors += 1
            self.last_journal_error = type(error).__name__
            return False

    def _write(self):
        try:
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                self._write_pending(stream)
        except OSError as error:
            self.write_errors += 1
            self.last_journal_error = type(error).__name__
            self._journal_unavailable.set()

    def _write_pending(self, stream):
        while not self._stop.is_set() or not self._pending.empty():
            try:
                job = self._pending.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                row = evaluate_cycle(
                    *job, boot_id=self.boot_id, session=self.session,
                    ld19_clock_evidence=self.ld19_clock_evidence,
                )
            except Exception as error:
                self.evaluation_errors += 1
                self.last_journal_error = type(error).__name__
                continue
            row["journal"] = self.counters()
            if not self._append(stream, row):
                self._journal_unavailable.set()
                return
            self.written += 1
        self._append(stream, {
            **BOUNDARY, "schema_version": SCHEMA, "record_type": "SESSION_SUMMARY",
            "session_id": self.session, "boot_id": self.boot_id, "journal": self.counters(),
        })

    def close(self):
        self._stop.set()
        self._subscriber.join(timeout=1)
        self._writer.join(timeout=2)
        return not (self._subscriber.is_alive() or self._writer.is_alive())
