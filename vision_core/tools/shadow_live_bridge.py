"""Live Shadow adapter for the existing dynamic RGB/stereo audit topic.

This is an experimental control input, not an SIE Measurement. The camera
process remains the owner of capture, model verification and association.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from vision_core.tools.run_shadow_uart import ShadowObservation, run_session
from vision_core.tools.shadow_range_gate import (
    ACTIVE_STEREO_SHA256, FRONT_FRAME, RgbTarget, TargetRange,
    select_target_range,
)
from vision_core.tools.uart_http_transport import UartHttp


AUDIT_TOPIC = "/sie/diagnostics/ar0234_ov9281/dynamic_association"
AR_INTRINSIC_SHA256 = "e9454ed3d93ab36a6b159aa8c8e2356353cca8a6b0480e73c371199937ae7096"


def observation_from_audit(audit: dict, *, rim_forward_of_optical_center_m: float,
                           received_mono_s: float, received_utc: datetime,
                           max_capture_age_s: float = .5) -> ShadowObservation:
    """Convert only a same-cycle confirmed target and 3D association.

    The mounting offset is an explicit operator input. The output remains an
    experimental range; this function cannot certify its physical accuracy.
    """
    empty = ShadowObservation(None, ())
    if (type(rim_forward_of_optical_center_m) not in (float, int) or
            not math.isfinite(rim_forward_of_optical_center_m) or
            not 0 <= rim_forward_of_optical_center_m <= .10):
        raise ValueError("FRONT_OPTICAL_DATUM_REQUIRED")
    if (audit.get("status") != "ASSOCIATED_DIAGNOSTIC_ONLY" or
            audit.get("reference_frame") != "ov9281_physical_left_optical_frame" or
            audit.get("calibration", {}).get("ov9281_stereo_sha256") != ACTIVE_STEREO_SHA256 or
            audit.get("calibration", {}).get("ar0234_intrinsic_sha256") != AR_INTRINSIC_SHA256):
        return empty
    association = audit.get("association") or {}
    confirmation = (audit.get("ar_target_track") or {}).get("confirmation") or {}
    track_id = confirmation.get("track_id") if isinstance(confirmation, dict) else None
    cycle_id = audit.get("cycle_id")
    if (not isinstance(confirmation, dict) or
            confirmation.get("state") != "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION"
            or not isinstance(track_id, str) or not track_id
            or not isinstance(cycle_id, str) or not cycle_id
            or not isinstance(audit.get("audit_id"), str)
            or not audit["audit_id"].endswith(cycle_id)):
        return empty
    try:
        captured = datetime.fromisoformat(audit["timestamp"])
        if captured.tzinfo is None:
            return empty
        age = (received_utc - captured.astimezone(timezone.utc)).total_seconds()
        x, y, z = (float(association[key]) for key in ("x_m", "y_m", "z_m"))
        confidence = float(association["confidence"])
        if (not all(map(math.isfinite, (age, x, y, z, confidence))) or
                not 0 <= age <= max_capture_age_s or z <= 0 or
                not 0 <= confidence <= 1):
            return empty
    except (KeyError, TypeError, ValueError, OverflowError):
        return empty
    # The operator reported a 15 degree upward mount and a 0.210 m setback
    # from the front rim. The optical-to-rim offset is provided explicitly.
    pitch = math.radians(15.0)
    forward = y * math.sin(pitch) + z * math.cos(pitch)
    distance = forward - .210 - rim_forward_of_optical_center_m
    if not math.isfinite(distance) or distance <= 0:
        return empty
    observed_s = received_mono_s - age
    bearing = math.degrees(math.atan2(x, forward))
    target = RgbTarget(track_id, observed_s, bearing, confidence,
                       AR_INTRINSIC_SHA256)
    rng = TargetRange("OV9281", track_id, observed_s, distance, confidence,
                      FRONT_FRAME, ACTIVE_STEREO_SHA256, audit["audit_id"])
    return ShadowObservation(target, (rng,))


def live_audits(node, *, duration_s: float):
    """Yield latest audit or a no-target tick; never replay a consumed cycle."""
    import rclpy
    from std_msgs.msg import String

    pending = None
    def on_audit(message):
        nonlocal pending
        pending = (message.data, time.monotonic(), datetime.now(timezone.utc))
    subscription = node.create_subscription(String, AUDIT_TOPIC, on_audit, 1)
    deadline = time.monotonic() + duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            current, pending = pending, None
            yield current
    finally:
        node.destroy_subscription(subscription)


def preview_record(audit: dict | None, observation: ShadowObservation,
                   *, now_s: float, received_utc: datetime | None = None,
                   minimum_confidence: float = .40) -> dict:
    """Expose the upstream refusal instead of hiding it as a missing RGB target."""
    decision = select_target_range(observation.target, list(observation.ranges),
        now_s, expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
        minimum_confidence=minimum_confidence)
    audit = audit or {}
    geometry = audit.get("stereo_geometry") or {}
    pre_range = geometry.get("pre_range_depth_diagnostic") or {}
    timestamp = audit.get("timestamp")
    age_s = None
    if received_utc is not None and isinstance(timestamp, str):
        try:
            captured = datetime.fromisoformat(timestamp)
            if captured.tzinfo is not None:
                age_s = round((received_utc - captured).total_seconds(), 3)
        except ValueError:
            pass
    return {
        "cycle_id": audit.get("cycle_id"),
        "audit_status": audit.get("status"), "audit_reason": audit.get("reason"),
        "rgb_reason": (audit.get("ar0234_dynamic_target") or {}).get("reason"),
        "candidate_geometry": (audit.get("ar0234_candidate_geometry") or {}).get("reason"),
        "lr_samples": geometry.get("lr_consistent_disparity_sample_count"),
        "in_range_samples": pre_range.get("in_range_sample_count", geometry.get("in_range_depth_sample_count")),
        "reprojected_samples": geometry.get("reprojected_inside_ar_inner_roi_count"),
        "plausible_components": geometry.get("plausible_spatial_component_count"),
        "depth_clusters": geometry.get("plausible_depth_cluster_count"),
        "depth_percentiles_m": pre_range.get("depth_percentiles_m"),
        "capture_to_receive_s": age_s,
        "host_processing_s": (audit.get("processing_timing") or {}).get("host_capture_start_to_result_ready_s"),
        "decision": decision.action, "decision_reason": decision.reason,
        "distance_m": None if decision.selected is None else decision.selected.distance_m,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--front-rim-forward-of-optical-m", type=float, required=True,
                        help="Measured optical-centre to front-rim forward component, in metres")
    parser.add_argument("--duration-s", type=float, default=30.)
    parser.add_argument("--segment-m", type=float, default=.10)
    parser.add_argument("--minimum-confidence", type=float, default=.40)
    parser.add_argument("--max-segments", type=int, default=None)
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument("--serial-port", default="/dev/ttyAMA0")
    parser.add_argument("--execute", action="store_true",
                        help="Enable physical motion; without this flag only decisions are printed")
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s <= 0:
        parser.error("duration must be positive")
    if not .02 <= args.segment_m <= .30:
        parser.error("segment must be in 0.02..0.30 m")
    if not math.isfinite(args.minimum_confidence) or not 0 < args.minimum_confidence <= 1:
        parser.error("minimum confidence must be in (0, 1]")
    # Validate the physical datum before opening serial or subscribing.
    observation_from_audit({}, rim_forward_of_optical_center_m=args.front_rim_forward_of_optical_m,
                           received_mono_s=time.monotonic(), received_utc=datetime.now(timezone.utc))

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("sie_shadow_live_bridge")
    try:
        def observations():
            for entry in live_audits(node, duration_s=args.duration_s):
                if entry is None:
                    yield None
                    continue
                data, mono, utc = entry
                audit = None
                try:
                    audit = json.loads(data)
                    if not isinstance(audit, dict):
                        raise ValueError("audit is not an object")
                    observation = observation_from_audit(
                        audit, rim_forward_of_optical_center_m=args.front_rim_forward_of_optical_m,
                        received_mono_s=mono, received_utc=utc)
                except (ValueError, TypeError, KeyError, AttributeError):
                    observation = ShadowObservation(None, ())
                if not args.execute:
                    row = preview_record(audit, observation, now_s=time.monotonic(),
                                         received_utc=utc,
                                         minimum_confidence=args.minimum_confidence)
                    print(json.dumps(row, allow_nan=False), flush=True)
                yield observation
        stream = observations()
        if args.execute:
            with UartHttp(args.serial_port) as request:
                report = run_session(request=request, observations=stream,
                    journal_path=args.journal, rgb_calibration_id=AR_INTRINSIC_SHA256,
                    segment_m=args.segment_m, max_segments=args.max_segments,
                    minimum_confidence=args.minimum_confidence)
                print(json.dumps(report, allow_nan=False), flush=True)
        else:
            for _ in stream:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
