import io
import json
import math
import sys
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np
import pytest

from sie_core.geometry.camera_lidar_mechanical_geometry_v1 import (
    LD19_FRAME, OPTICAL_FRAME, RIM_FRAME, mechanical_transform,
)
from sie_core.geometry.camera_lidar_optical_datum_v1 import estimated_ld19_transform
from sie_ros2.host_timebase_bridge_v1 import make_host_timebase_bridge
from sie_ros2.ld19_diagnostic_evidence import make_ld19_diagnostic_record
from vision_core.stereo.stereo_lidar_static_v1 import compare_scan, fit_stereo_surface, target_mask
from vision_core.tools import stereo_lidar_joint_static_v1 as tool


MONO = 10_000_000_000
OFFSET = 1_800_000_000_000_000_000
BOOT = "test-boot"


def geometry():
    return [json.loads((tool.CONFIG / filename).read_text()) for filename in (
        "camera_lidar_mechanical_geometry_v1.json", "camera_lidar_optical_datum_v1.json",
    )]


def matrices(f=100, cx=50, cy=40):
    p = np.array([[f, 0, cx, 0], [0, f, cy, 0], [0, 0, 1, 0]], dtype=float)
    q = np.array([[1, 0, 0, -cx], [0, 1, 0, -cy], [0, 0, 0, f],
                  [0, 0, .01, 0]], dtype=float)
    return p, q


def test_estimated_datum_sign_and_mechanical_refusal():
    mech, datum = geometry()
    rotation, rim = mechanical_transform(mech, source_frame=LD19_FRAME, destination_frame=RIM_FRAME)
    shifted_rotation, optical, axial = estimated_ld19_transform(mech, datum)
    assert axial == pytest.approx(.0125 / math.cos(math.radians(20)))
    np.testing.assert_array_equal(shifted_rotation, rotation)
    np.testing.assert_allclose(optical - rim, [0, 0, axial])
    with pytest.raises(ValueError, match="optical-center transform unavailable"):
        mechanical_transform(mech, source_frame=LD19_FRAME, destination_frame=OPTICAL_FRAME)


@pytest.mark.parametrize("field,value", [
    ("measurement_authorized", True), ("destination_frame", OPTICAL_FRAME),
    ("pitch_up_deg", 15), ("optical_center_to_rim_robot_forward_component_m", float("nan")),
    ("uncertainty_m", 0),
])
def test_optical_datum_refuses_changed_assumptions(field, value):
    mech, datum = geometry()
    datum[field] = value
    with pytest.raises(ValueError):
        estimated_ld19_transform(mech, datum)


def test_q_metres_and_r1_transpose():
    _, q = matrices()
    shape = (80, 100)
    mask = target_mask(shape, [[30, 20], [70, 20], [70, 60], [30, 60]])
    r1, _ = cv2.Rodrigues(np.array([.12, .06, 0]))
    # f * B / d = 100 px * 0.1 m / 5 px = 2 m.
    surface = fit_stereo_surface(np.full(shape, 5.0), np.ones(shape, bool), q, r1, mask)
    np.testing.assert_allclose(surface.normal, r1.T @ [0, 0, 1], atol=1e-12)
    assert surface.centroid_m @ surface.normal == pytest.approx(2)


def test_missing_stereo_support_and_invalid_polygon_refuse():
    _, q = matrices()
    shape = (80, 100)
    with pytest.raises(ValueError):
        target_mask(shape, [[30, 20], [70, 60], [70, 20], [30, 60]])
    mask = target_mask(shape, [[30, 20], [70, 20], [70, 60], [30, 60]])
    with pytest.raises(ValueError, match="INSUFFICIENT_STEREO_TARGET_SUPPORT"):
        fit_stereo_surface(np.full(shape, 5.0), np.zeros(shape, bool), q, np.eye(3), mask)


def test_finite_overlap_missing_returns_and_untrimmed_residuals():
    p, q = matrices()
    shape = (80, 100)
    lr = np.ones(shape, bool)
    lr[:, 56:64] = False
    mask = target_mask(shape, [[32, 20], [68, 20], [68, 60], [32, 60]])
    surface = fit_stereo_surface(np.full(shape, 5.0), lr, q, np.eye(3), mask)
    scan = {"angle_min_rad": -.3, "angle_increment_rad": .1,
            "range_min_m": .1, "range_max_m": 10,
            "ranges_m": [2, 2, 2, 5, "+Infinity", 2, 2]}
    rotation = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    result = compare_scan(scan, surface, np.eye(3), p, rotation, np.zeros(3), (-15, 15))
    statuses = [row["status"] for row in result["beams"]]
    assert statuses == [
        "OUTSIDE_DECLARED_SECTOR", "OUTSIDE_OBSERVED_TARGET", "AMBIGUOUS_FINITE_CORRESPONDENCE",
        "FINITE_SURFACE_CANDIDATE", "MISSING_RETURN_IN_STEREO_SUPPORTED_REGION",
        "OUTSIDE_OBSERVED_TARGET", "OUTSIDE_DECLARED_SECTOR",
    ]
    assert result["infinite_plane_in_declared_sector"]["count"] == 4
    assert result["finite_surface_candidates"]["median_mm"] == pytest.approx(3000)


def bridge():
    brackets = [{"monotonic_before_ns": t - 100, "realtime_ns": t + OFFSET,
                 "monotonic_after_ns": t + 100}
                for t in (MONO - 8_000_000, MONO - 6_000_000, MONO - 4_000_000)]
    return make_host_timebase_bridge(brackets, boot_id=BOOT, policy=tool.BRIDGE_POLICY)


def frame_row():
    return {
        "status": "PAYLOAD_RECORDED", "frame_index": 1,
        "camera": {"host_packet_received_monotonic_s": MONO / 1e9,
                   "driver_timestamp_s": (MONO - 12_000_000) / 1e9},
        "clock": {"clock_valid": True, "bridge": bridge()},
    }


def scan_row(sequence=1, received_ms=-5, stamp_ms=-20, clock="OPERATOR_ATTESTED_SAME_HOST_REALTIME"):
    return {"status": "PAYLOAD_RECORDED", "clock_status": clock,
            "receipt_monotonic_ns": MONO + received_ms * 1_000_000,
            "header_ns": MONO + OFFSET + stamp_ms * 1_000_000, "receipt_sequence": sequence}


def test_causal_timing_excludes_late_closer_scan():
    assessment, source = tool.select_scan(
        frame_row(), [scan_row(), scan_row(2, received_ms=1, stamp_ms=-12)], BOOT,
    )
    assert source["receipt_sequence"] == 1
    assert assessment["header_minus_stereo_stamp_ns"] == -8_000_000


@pytest.mark.parametrize(("scan", "reason"), [
    (scan_row(clock="ROS_CLOCK_DOMAIN_UNVERIFIED"), "ROS_CLOCK_DOMAIN_UNVERIFIED"),
    (scan_row(clock="LD19_PUBLISHER_CHANGED"), "LD19_PUBLISHER_CHANGED"),
    (scan_row(stamp_ms=-300), "SOURCE_STALE"),
    (scan_row(stamp_ms=10), "STAMP_FROM_FUTURE"),
    (scan_row(stamp_ms=-2), "HEADER_AHEAD_OF_CALLBACK_RECEIPT"),
    (scan_row(stamp_ms=-90), "HEADER_DELTA_EXCEEDED"),
])
def test_refused_scan_reasons(scan, reason):
    assessment, source = tool.select_scan(frame_row(), [scan], BOOT)
    assert source is None
    assert assessment["candidate_refusals"] == {reason: 1}


def test_invalid_bridge_and_missing_sources_refuse():
    frame = frame_row()
    assessment, source = tool.select_scan(frame, [], BOOT)
    assert source is None and assessment["status"] == "NO_ELIGIBLE_CAUSAL_SCAN"
    frame["clock"] = {"clock_valid": False, "status": "BRIDGE_EXPIRED"}
    assert tool.select_scan(frame, [scan_row()], BOOT) == ({"status": "BRIDGE_EXPIRED"}, None)


def archive_synthetic_pair(directory, monkeypatch):
    directory.mkdir()
    w, h = tool.SIZE
    cv2.setNumThreads(1)
    p, q = matrices(640, 640, 400)
    cal = io.BytesIO()
    np.savez(cal, size=tool.SIZE, camera_1_semantics="physical_left", camera_2_semantics="physical_right",
             captured_stream_identity="captured_second_is_physical_left",
             K1=p[:, :3], K2=p[:, :3], D1=np.zeros(5), D2=np.zeros(5),
             R1=np.eye(3), R2=np.eye(3), P1=p, P2=p, Q=q)
    monkeypatch.setattr(tool, "ACTIVE_V7_SHA256", tool.digest(cal.getvalue()))
    inputs = {"calibration": tool.save_bytes(directory, "v7.npz", cal.getvalue())}
    for name, data in zip(("mechanical", "optical_datum"), geometry()):
        inputs[name] = tool.save_bytes(directory, f"{name}.json", tool.encoded(data).encode())
    tool.write_json(directory / "session.json", dict(tool.BOUNDARY, schema_version=tool.SCHEMA,
                                                    boot_id=BOOT, inputs=inputs))
    left = np.random.default_rng(2).integers(0, 256, (h, w), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-32] = left[:, 32:]
    ok, jpeg = cv2.imencode(".jpg", np.hstack((right, left)), [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    frame = frame_row()
    frame["payload_mjpg"] = tool.save_bytes(directory, "frame.mjpg", jpeg.tobytes())
    signature = scan_row()
    stamp = SimpleNamespace(sec=signature["header_ns"] // 10**9, nanosec=signature["header_ns"] % 10**9)
    message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id="ld19_sensor"),
                              angle_min=0., angle_increment=.01, angle_max=.02,
                              time_increment=.001, scan_time=.1, range_min=.1, range_max=10.,
                              ranges=[1.89, math.inf, 1.9], intensities=[1., 1., 1.])
    signature["record"] = make_ld19_diagnostic_record(message, received_at_utc="2026-11-26T00:00:00+00:00")
    signature["payload_cdr"] = tool.save_bytes(directory, "scan.cdr", b"synthetic CDR fixture")
    for filename, row in (("scans.jsonl", signature), ("frames.jsonl", frame)):
        with (directory / filename).open("x") as stream:
            tool.append_row(stream, row)


def test_offline_end_to_end_with_real_matcher_and_payload_integrity(tmp_path, monkeypatch):
    capture = tmp_path / "capture"
    archive_synthetic_pair(capture, monkeypatch)
    args = SimpleNamespace(capture=capture, output=tmp_path / "analysis",
                           polygon=[500, 510, 750, 510, 750, 690, 500, 690],
                           sector_deg=(-3, 3), target_label="synthetic panel")
    assert tool.analyze(args) == 0
    row, = tool.read_rows(args.output / "comparisons.jsonl")
    result = row["comparison"]
    assert result["stereo_plane_centroid_physical_left_optical_m"][2] == pytest.approx(2, abs=.004)
    assert result["finite_surface_candidates"]["count"] == 2
    assert result["beam_status_counts"]["MISSING_RETURN_IN_STEREO_SUPPORTED_REGION"] == 1
    summary = json.loads((args.output / "summary.json").read_text())
    assert summary["distinct_compared_ld19_scans"] == 1
    assert not any(summary[key] for key in (
        "measurement_authorized", "world_state_update_authorized", "motor_control_authorized"))
    (capture / "scan.cdr").write_bytes(b"changed")
    args.output = tmp_path / "refused"
    with pytest.raises(ValueError, match="digest mismatch"):
        tool.analyze(args)
    assert not args.output.exists()


def test_capture_archives_callback_payload_and_releases_fake_camera(tmp_path, monkeypatch):
    fixtures = tmp_path / "fixtures"
    archive_synthetic_pair(fixtures, monkeypatch)
    jpeg = (fixtures / "frame.mjpg").read_bytes()
    now = MONO
    released, ros_shutdown, callbacks = [], [], []

    def spin_once(node, *, timeout_sec):
        nonlocal now
        now += 500_000_000
        for callback in callbacks:
            header = now + OFFSET - 10_000_000
            callback(SimpleNamespace(
                header=SimpleNamespace(frame_id="ld19_sensor", stamp=SimpleNamespace(
                    sec=header // 10**9, nanosec=header % 10**9)),
                angle_min=0., angle_max=0., angle_increment=.01,
                time_increment=0., scan_time=.1, range_min=.1, range_max=10.,
                ranges=[math.inf], intensities=[],
            ))

    class FakeNode:
        def __init__(self, name, **kwargs):
            assert kwargs["use_global_arguments"] is False

        def create_subscription(self, message_type, topic, callback, qos):
            assert topic == "/sie/lidar/scan_raw"
            callbacks.append(callback)

        def get_publishers_info_by_topic(self, topic):
            return [SimpleNamespace(endpoint_gid=b"test-publisher")]

        def destroy_node(self):
            ros_shutdown.append("node")

    modules = {
        "rclpy": {"init": lambda **kw: None, "ok": lambda: True, "spin_once": spin_once,
                  "shutdown": lambda: ros_shutdown.append("context")},
        "rclpy.executors": {"ExternalShutdownException": type("ExternalShutdownException", (Exception,), {})},
        "rclpy.node": {"Node": FakeNode},
        "rclpy.qos": {"qos_profile_sensor_data": object()},
        "rclpy.serialization": {"serialize_message": lambda message: b"raw scan"},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"LaserScan": object},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    def clock_brackets(**kwargs):
        return [{"monotonic_before_ns": v - 100, "realtime_ns": v + OFFSET,
                 "monotonic_after_ns": v + 100} for v in (now - 3000, now - 2000, now - 1000)]

    def camera_snapshot(last_sequence):
        return jpeg, {"sequence": 30, "driver_timestamp_s": (now - 1_000_000) / 1e9,
                      "host_packet_received_monotonic_s": now / 1e9}

    monkeypatch.setattr(tool, "time", SimpleNamespace(monotonic_ns=lambda: now, time_ns=lambda: now + OFFSET))
    monkeypatch.setattr(tool, "collect_host_clock_brackets", clock_brackets)
    monkeypatch.setattr(tool, "LatestV4L2Capture", lambda *a, **kw: SimpleNamespace(
        snapshot=camera_snapshot, release=lambda: released.append(True)))
    helper = tmp_path / "fake-executable"
    helper.write_bytes(b"fixture")
    helper.chmod(0o700)
    args = SimpleNamespace(native_capture=helper, calibration=fixtures / "v7.npz", device="/dev/fake",
                           frames=1, interval_s=.2, output=tmp_path / "archived",
                           ld19_clock_evidence="sha256:" + "0" * 64)

    assert tool.capture(args) == 0
    assert released == [True] and ros_shutdown == ["node", "context"]
    frame, = tool.read_rows(args.output / "frames.jsonl")
    assert tool.checked_bytes(args.output, frame["payload_mjpg"]) == jpeg
    assert (args.output / "target-reference-left.png").is_file()
    scans = tool.read_rows(args.output / "scans.jsonl")
    assert len(scans) == 4
    assert scans[0]["record"]["payload"]["scan"]["ranges_m"] == ["+Infinity"]
    assert tool.checked_bytes(args.output, scans[0]["payload_cdr"]) == b"raw scan"
    assert scans[0]["clock_status"] == "OPERATOR_ATTESTED_SAME_HOST_REALTIME"
