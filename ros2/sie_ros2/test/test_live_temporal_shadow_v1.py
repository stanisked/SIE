"""Synthetic coverage for causal live metadata association and bounded journaling."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace

import pytest

from sie_ros2.host_timebase_bridge_v1 import make_host_timebase_bridge
from sie_ros2.latest_v4l2_capture import CameraTiming
from sie_ros2 import live_temporal_shadow_monitor as runtime
from sie_ros2.live_temporal_association_v1 import (
    BRIDGE_POLICY, MS, CycleTiming, ScanTiming, evaluate_cycle,
)

BOOT = "synthetic-boot"
T = 10_000 * MS
OFFSET = 1_800_000_000_000_000_000
ATTESTED = "OPERATOR_ATTESTED_SAME_HOST_REALTIME"
MATCH = "MATCH_DIAGNOSTIC_ONLY"


def bridge_window(last_ms, boot_id=BOOT):
    newest = T + last_ms * MS
    bridge = make_host_timebase_bridge([
        {"monotonic_before_ns": now - 20, "realtime_ns": now + OFFSET,
         "monotonic_after_ns": now + 20}
        for now in (newest - 2 * MS, newest - MS, newest)
    ], boot_id=boot_id, policy=BRIDGE_POLICY)
    assert bridge["status"] == "DIAGNOSTIC_BRIDGE_READY"
    return newest + 100, bridge


def camera(sequence, stamp_ms, received_ms):
    return CameraTiming(
        sequence, (T + stamp_ms * MS) / 1e9, (T + received_ms * MS) / 1e9,
        (T + received_ms * MS) / 1e9,
    )


def scan(sequence, stamp_ms, received_ms, clock_status=ATTESTED):
    return ScanTiming(sequence, T + OFFSET + stamp_ms * MS, T + received_ms * MS,
                      "ld19_sensor", clock_status)


def cycle(**changes):
    original = CycleTiming(
        "preview-cycle", "SYNTHETIC_PREVIEW", camera(10, -14, 0), camera(31, -9, 18),
        "NEW_RGB_THEN_FRESH_STEREO_DURING_SEGMENTATION",
        T - MS, T + MS, T + 19 * MS, T + 100 * MS, T + 102 * MS, {},
    )
    return replace(original, **changes)


def evaluate(*, timing=None, stereo=(), scans=(), windows=None, stereo_evicted=None, scan_evicted=None):
    timing = timing or cycle()
    windows = (bridge_window(-5), bridge_window(90)) if windows is None else windows
    return evaluate_cycle(
        timing,
        {"ar0234": ((timing.rgb,) if timing.rgb else (), None),
         "ov9281": (stereo, stereo_evicted)},
        scans, scan_evicted, windows, boot_id=BOOT, session="preview-run",
        ld19_clock_evidence="synthetic-clock-evidence",
    )


def test_causal_cutoff_excludes_later_arrivals_and_preserves_actual_pair():
    row = evaluate(stereo=(camera(30, -27, -3), camera(31, -9, 18)),
                   scans=(scan(1, -36, -1), scan(2, -14, 1)))
    assert row["endpoint_monotonic_ns"] == {
        "zero_wait": T, "selection": T + 19 * MS,
        "result_ready": T + 100 * MS, "publication": T + 102 * MS,
    }
    stereo = row["zero_wait"]["rgb_stereo"]
    ld19 = row["zero_wait"]["rgb_ld19_ros_header"]
    assert stereo["status"] == ld19["status"] == MATCH
    assert stereo["candidate"]["sequence"] == 30
    assert stereo["available_count"] == ld19["available_count"] == 1
    assert ld19["candidate"]["receipt_sequence"] == 1
    assert ld19["candidate"]["sequence"] is None
    assert ld19["candidate"]["evidence_ref"].endswith("ld19:receipt:1")
    actual = row["actual_camera_pair"]
    assert actual["stereo"]["sequence"] == 31
    assert actual["stereo_minus_rgb_ns"] == 5 * MS
    assert actual["stereo_received_by_rgb_receipt"] is False
    assert actual["same_stereo_as_zero_wait"] is False
    assert row["freshness_by_endpoint"]["publication"]["actual_stereo"]["age_ns"] == 111 * MS
    assert row["physical_laser_acquisition_time"] == "UNKNOWN"
    assert all(row[key] is False for key in (
        "measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))


def test_stereo_and_ld19_have_independent_gates():
    # Stereo to LD19 is 65 ms, but both independent links are within their bounds.
    row = evaluate(stereo=(camera(1, -34, -2),), scans=(scan(1, 31, 0),),
                   timing=cycle(rgb=camera(10, -14, 40)))
    assert row["zero_wait"]["rgb_stereo"]["status"] == MATCH
    assert row["zero_wait"]["rgb_ld19_ros_header"]["status"] == "STAMP_AHEAD_OF_RECEIPT"
    row = evaluate(stereo=(camera(1, -34, -2),), scans=(scan(1, 31, 32),),
                   timing=cycle(rgb=camera(10, -14, 40)))
    assert row["zero_wait"]["rgb_stereo"]["status"] == MATCH
    assert row["zero_wait"]["rgb_ld19_ros_header"]["status"] == MATCH
    row = evaluate(stereo=(camera(1, -60, -2),), scans=(scan(1, -20, -3),))
    assert row["zero_wait"]["rgb_stereo"]["status"] == "DELTA_EXCEEDED"
    assert row["zero_wait"]["rgb_ld19_ros_header"]["status"] == MATCH


@pytest.mark.parametrize(("stamp_ms", "expected"), [
    (-251, "SOURCE_STALE"),
    (2, "STAMP_FROM_FUTURE"),
    (1, "FUTURE_WITHIN_TRIAL_SKEW"),
])
def test_freshness_refuses_stale_and_future_sources(stamp_ms, expected):
    row = evaluate(stereo=(camera(1, stamp_ms, 0),), scans=(scan(1, stamp_ms, 0),))
    for link in row["zero_wait"].values():
        assert link["status"] == expected
        assert link["candidate_freshness"]["age_ns"] == -stamp_ms * MS


def test_freshness_is_reevaluated_at_result_and_publication():
    row = evaluate(
        timing=cycle(result_ready_ns=T + 230 * MS, publication_ns=T + 240 * MS),
        stereo=(camera(1, -20, 0),), windows=(bridge_window(-5), bridge_window(220)),
    )
    assert row["zero_wait"]["rgb_stereo"]["status"] == MATCH
    assert row["freshness_by_endpoint"]["result_ready"]["zero_wait_stereo"]["age_ns"] == 250 * MS
    assert row["freshness_by_endpoint"]["publication"]["zero_wait_stereo"]["status"] == "SOURCE_STALE"


def test_missing_source_and_clock_unverified_keep_explicit_refusals():
    row = evaluate(stereo=(camera(1, -10, -1),))
    assert row["zero_wait"]["rgb_stereo"]["status"] == MATCH
    assert row["zero_wait"]["rgb_ld19_ros_header"]["status"] == "SOURCE_UNAVAILABLE"
    row = evaluate(scans=(scan(1, -14, 0, "ROS_CLOCK_DOMAIN_UNVERIFIED"),))
    link = row["zero_wait"]["rgb_ld19_ros_header"]
    assert link["status"] == "ROS_CLOCK_DOMAIN_UNVERIFIED"
    assert link["candidate"]["timestamp_ns"] == T + OFFSET - 14 * MS
    assert link["candidate_freshness"]["age_ns"] is link["candidate_minus_rgb_ns"] is None
    row = evaluate(timing=cycle(rgb=None, stereo=None, pair_selected_ns=None, publication_ns=None))
    assert row["zero_wait"]["rgb_stereo"]["status"] == "RGB_SOURCE_UNAVAILABLE"
    assert row["actual_camera_pair"]["stereo"] is None
    assert row["endpoint_monotonic_ns"]["publication"] is None


@pytest.mark.parametrize(("windows", "status"), [
    ((bridge_window(-1_100), bridge_window(1)), "BRIDGE_EXPIRED"),
    ((bridge_window(1),), "BRIDGE_UNAVAILABLE"),
    ((bridge_window(-5, "earlier-boot"),), "BOOT_ID_MISMATCH"),
    (((T - MS, {"status": "CLOCK_STEP_SUSPECTED"}),), "CLOCK_STEP_SUSPECTED"),
])
def test_refuses_invalid_bridge_at_cutoff(windows, status):
    row = evaluate(stereo=(camera(1, -15, -1),), scans=(scan(1, -15, -2),), windows=windows)
    assert row["clock"]["zero_wait"]["status"] == status
    assert row["clock"]["zero_wait"]["clock_valid"] is False
    for link in row["zero_wait"].values():
        assert link["rgb_freshness"]["age_ns"] is None
        assert link["candidate_freshness"]["age_ns"] is None
        assert link["status"] == "RGB_" + status


def test_history_eviction_inside_freshness_horizon_refuses_incomplete_search():
    row = evaluate(
        stereo=(camera(1, -15, -1),), stereo_evicted=(T - MS) / 1e9,
        scans=(scan(1, -15, -1),), scan_evicted=T - 251 * MS,
    )
    assert row["zero_wait"]["rgb_stereo"]["status"] == "CAUSAL_HISTORY_TRUNCATED"
    assert row["zero_wait"]["rgb_ld19_ros_header"]["status"] == MATCH
    assert row["history"]["ov9281"]["evicted_through_monotonic_ns"] == T - MS


@pytest.fixture
def monitor_factory(tmp_path, monkeypatch):
    monkeypatch.setattr(runtime.LiveTemporalShadowMonitor, "_spin_subscriber", lambda self: None)
    actors = []

    def create():
        cameras = {name: SimpleNamespace(timing_history=lambda: ((), None)) for name in ("ar0234", "ov9281")}
        monitor = runtime.LiveTemporalShadowMonitor(
            tmp_path / "journal.jsonl", "preview-run", cameras, None, "synthetic-evidence",
        )
        actors.append(monitor)
        return monitor

    yield create
    for monitor in actors:
        assert monitor.close()


def test_ld19_keeps_only_bounded_header_metadata(monitor_factory):
    monitor = monitor_factory()

    class HeaderOnly:
        header = SimpleNamespace(frame_id="ld19_sensor", stamp=SimpleNamespace(sec=1, nanosec=1))

        @property
        def ranges(self):
            raise AssertionError("diagnostic accessed a scan payload")

    multiple_publishers = SimpleNamespace(count_publishers=lambda topic: 2)
    for _ in range(runtime.LD19_HISTORY_SIZE + 1):
        monitor._on_scan(HeaderOnly(), multiple_publishers)
    assert len(monitor._scans) == runtime.LD19_HISTORY_SIZE
    assert monitor._scans[0].receipt_sequence == 2
    assert monitor._scans[-1].clock_status == "LD19_PUBLISHER_NOT_UNIQUE"
    assert monitor._scan_evicted_through <= monitor._scans[0].received_monotonic_ns


def test_journal_has_bounded_queue_drops_and_private_summary(monitor_factory, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original_evaluate = runtime.evaluate_cycle

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return original_evaluate(*args, **kwargs)

    monkeypatch.setattr(runtime, "evaluate_cycle", blocked)
    monitor = monitor_factory()

    def submit(i):
        monitor.submit(str(i), "SYNTHETIC", {}, {}, T, None)

    try:
        submit(0)
        assert entered.wait(2)
        for i in range(1, runtime.JOURNAL_QUEUE_SIZE + 4):
            submit(i)
        assert monitor.counters()["dropped"] == 3
        assert monitor.counters()["pending"] == runtime.JOURNAL_QUEUE_SIZE
    finally:
        release.set()
    assert monitor.close()
    rows = [json.loads(line) for line in monitor.path.read_text().splitlines()]
    assert len(rows) == runtime.JOURNAL_QUEUE_SIZE + 2
    assert rows[-1]["record_type"] == "SESSION_SUMMARY"
    assert rows[-1]["journal"]["dropped"] == 3
    assert rows[-1]["journal"]["write_errors"] == 0
    assert monitor.path.stat().st_mode & 0o777 == 0o600


def test_journal_create_error_is_counted_without_overwriting(monitor_factory, tmp_path):
    path = tmp_path / "journal.jsonl"
    path.write_text("preexisting evidence")
    monitor = monitor_factory()
    assert monitor._journal_unavailable.wait(2)
    monitor.submit("failed-journal", "SYNTHETIC", {}, {}, T, None)
    assert monitor.counters()["write_errors"] == 1
    assert monitor.counters()["dropped"] == 1
    assert path.read_text() == "preexisting evidence"


def test_launcher_rejects_argument_passthrough_before_loading_ros():
    root = Path(__file__).resolve().parents[3]
    launcher = root / "vision_core/tools/run_live_temporal_shadow_v1.sh"
    result = subprocess.run([launcher, "--execute", "--supervised-experimental"],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert "accepts no positional or passthrough arguments" in result.stderr
    script = launcher.read_text()
    assert "control_preview_topic:=/sie/diagnostics/ar0234_ov9281/temporal_shadow_preview" in script
    for forbidden in ("shadow_live_bridge", "run_shadow_uart", "sie_pipeline.launch", "ttyUSB", "cmd_vel"):
        assert forbidden not in script
