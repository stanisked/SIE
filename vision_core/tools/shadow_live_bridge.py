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
from vision_core.tools.shadow_fast_adapter import front_profile_sha256, from_control_preview


AUDIT_TOPIC = "/sie/diagnostics/ar0234_ov9281/dynamic_association"
FAST_TOPIC = "/sie/diagnostics/ar0234_ov9281/control_preview"
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


def live_audits(node, *, duration_s: float, topic: str = AUDIT_TOPIC):
    """Yield latest audit or a no-target tick; never replay a consumed cycle."""
    import rclpy
    from std_msgs.msg import String

    pending = None
    def on_audit(message):
        nonlocal pending
        pending = (message.data, time.monotonic(), datetime.now(timezone.utc))
    subscription = node.create_subscription(String, topic, on_audit, 1)
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
    shape = audit.get("shape_diagnostic") or {}
    masked = shape.get("mask_depth") or {}
    quality = audit.get("control_depth_quality_shadow") or {}
    local = quality.get("local_consistency") or {}
    local_residual = local.get("held_out_residual") or {}
    front = shape.get("front_geometry") or {}
    candidate_front = shape.get("candidate_layer_front_geometry") or {}
    local_candidate_front = shape.get("local_candidate_front_geometry") or {}
    stable_front = shape.get("stable_candidate_front_geometry") or {}
    selected_instance = (audit.get("ar0234_dynamic_target") or {}).get("candidate") or {}
    v3_diagnostic = audit.get("shadow_v3_observation_diagnostic") or {}
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
        "current_eligible_count": (audit.get("ar0234_dynamic_target") or {}).get("current_eligible_count"),
        "segmentation_instance_count": len((audit.get("segmentation_observation") or {}).get("instances") or []),
        "candidate_geometry": (audit.get("ar0234_candidate_geometry") or {}).get("reason"),
        "lr_samples": geometry.get("lr_consistent_disparity_sample_count"),
        "in_range_samples": pre_range.get("in_range_sample_count", geometry.get("in_range_depth_sample_count")),
        "reprojected_samples": geometry.get("reprojected_inside_ar_inner_roi_count"),
        "plausible_components": geometry.get("plausible_spatial_component_count"),
        "depth_clusters": geometry.get("plausible_depth_cluster_count"),
        "selected_depth_mad_m": geometry.get("selected_depth_mad_m",
                                             (geometry.get("depth_clusters") or [{}])[0].get("mad_m")),
        "associated_optical_range_m": (audit.get("association") or {}).get("range_m"),
        "associated_confidence": (audit.get("association") or {}).get("confidence"),
        "mask_status": masked.get("status"),
        "mask_candidate_samples": masked.get("candidate_sample_count"),
        "mask_depth_layers": masked.get("depth_layers"),
        "mask_candidate_mad_m": masked.get("candidate_mad_m"),
        # Same-cycle local fit is independent of whole-body depth MAD. Its
        # residual describes consistency, not physical accuracy or motion admission.
        "shape_aware_quality_status": quality.get("status"),
        "shape_aware_quality_refusals": quality.get("refusal_reasons"),
        "local_surface_status": local.get("status"),
        "local_surface_binding": local.get("fast_binding"),
        "local_surface_residual_samples": local_residual.get("sample_count"),
        "local_surface_residual_mad_m": local_residual.get("mad_m"),
        "local_surface_residual_p90_m": local_residual.get("absolute_p90_m"),
        "local_surface_fitted_fraction": local.get("fitted_sample_fraction"),
        "mask_sha256": shape.get("mask_sha256"),
        "segmentation_model_sha256": shape.get("model_sha256"),
        "track_id": (observation.target.track_id if observation.target is not None else
                     (audit.get("shadow_track_confirmation") or {}).get("track_id")),
        "rgb_continuity_track_id": (audit.get("shadow_rgb_continuity") or {}).get("track_id"),
        "rgb_continuity_status": (audit.get("shadow_rgb_continuity") or {}).get("status"),
        "v3_mask_match": (audit.get("shadow_v3_mask_match") or {}).get("status"),
        "v3_detection_count": (audit.get("shadow_v3_mask_match") or {}).get("v3_detection_count"),
        "v3_raw_detection_count": v3_diagnostic.get("detection_count"),
        "v3_detections": v3_diagnostic.get("detections"),
        "semantic_source": audit.get("shadow_semantic_source"),
        "v3_inference_s": audit.get("shadow_v3_inference_s"),
        "selected_mask_instance_bbox_xyxy_px": selected_instance.get("bbox_xyxy_px"),
        "selected_mask_instance_index": selected_instance.get("instance_index"),
        "selected_mask_instance_confidence": selected_instance.get("confidence"),
        "front_known_p05_m": (front.get("known_term_m") or {}).get("p05"),
        "front_known_median_m": (front.get("known_term_m") or {}).get("median"),
        "front_point_set": front.get("point_set"),
        "candidate_front_known_p05_m": (candidate_front.get("known_term_m") or {}).get("p05"),
        "candidate_front_known_median_m": (candidate_front.get("known_term_m") or {}).get("median"),
        "candidate_front_point_count": candidate_front.get("point_count"),
        "candidate_front_point_set": candidate_front.get("point_set"),
        "local_candidate_front_status": local_candidate_front.get("status"),
        "local_candidate_front_point_count": local_candidate_front.get("point_count"),
        "local_candidate_front_point_set": local_candidate_front.get("point_set"),
        "local_candidate_front_provenance": local_candidate_front.get("provenance"),
        "local_candidate_front_known_p05_m": (local_candidate_front.get("known_term_m") or {}).get("p05"),
        "local_candidate_front_residual_p90_m": (local_candidate_front.get("local_fit_residual") or {}).get("absolute_p90_m"),
        "local_candidate_front_fitted_fraction": local_candidate_front.get("local_fit_fitted_fraction"),
        "stable_front_status": stable_front.get("status"),
        "stable_front_point_count": stable_front.get("point_count"),
        "stable_front_patch_count": (stable_front.get("stable_local_patches") or {}).get("selected_patch_count"),
        "stable_front_known_p05_m": (stable_front.get("known_term_m") or {}).get("p05"),
        "stable_front_policy": (stable_front.get("stable_local_patches") or {}).get("policy"),
        "stable_front_provenance": stable_front.get("provenance"),
        "source_snapshot_evidence": audit.get("source_snapshot_evidence"),
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
    parser.add_argument("--source", choices=("dynamic_audit", "fast_mask"),
                        default="dynamic_audit")
    parser.add_argument("--segmentation-model-sha256", default="")
    parser.add_argument("--semantic-source", choices=("v3_mask_binding", "segmentation_mask"),
                        default="v3_mask_binding")
    parser.add_argument("--execute", action="store_true",
                        help="Enable physical motion; without this flag only decisions are printed")
    args = parser.parse_args()
    if args.source == "fast_mask" and args.execute:
        parser.error("fast-mask motor execution requires a reviewed Pi end-to-end run")
    if (args.source == "fast_mask" and
            (len(args.segmentation_model_sha256) != 64 or
             any(c not in "0123456789abcdef" for c in args.segmentation_model_sha256))):
        parser.error("fast-mask requires the exact reviewed segmentation model SHA-256")
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
    profile_sha = front_profile_sha256() if args.source == "fast_mask" else None
    try:
        def observations():
            for entry in live_audits(node, duration_s=args.duration_s,
                                     topic=FAST_TOPIC if args.source == "fast_mask" else AUDIT_TOPIC):
                if entry is None:
                    if not args.execute:
                        row = {"source": args.source, "cycle_id": None,
                               "decision_monotonic_s": time.monotonic(),
                               "decision": "PAUSE", "decision_reason": "NO_NEW_CAMERA_CYCLE"}
                        journal.write(json.dumps(row) + "\n")
                        journal.flush()
                    yield None
                    continue
                data, mono, utc = entry
                audit = None
                refusal = None
                try:
                    audit = json.loads(data)
                    if not isinstance(audit, dict):
                        raise ValueError("audit is not an object")
                    if args.source == "fast_mask":
                        observation, refusal = from_control_preview(
                            audit, now_s=time.monotonic(),
                            rim_forward_of_optical_center_m=args.front_rim_forward_of_optical_m,
                            expected_front_profile_sha256=profile_sha,
                            expected_segmentation_model_sha256=args.segmentation_model_sha256,
                            semantic_source=args.semantic_source,
                            local_preview=not args.execute)
                    else:
                        observation = observation_from_audit(
                            audit, rim_forward_of_optical_center_m=args.front_rim_forward_of_optical_m,
                            received_mono_s=mono, received_utc=utc)
                except (ValueError, TypeError, KeyError, AttributeError):
                    observation = ShadowObservation(None, ())
                    refusal = "INVALID_AUDIT_PAYLOAD"
                if not args.execute:
                    now = time.monotonic()
                    row = preview_record(audit, observation, now_s=now,
                        received_utc=utc, minimum_confidence=args.minimum_confidence)
                    row.update(source=args.source, adapter_reason=refusal,
                        semantic_source=(args.semantic_source if args.source == "fast_mask"
                                         else "dynamic_audit"),
                        decision_monotonic_s=now,
                        driver_timestamp_s={k: v.get("driver_timestamp_s") for k, v in
                            ((audit or {}).get("frame_freshness") or {}).get("frames", {}).items()
                            if isinstance(v, dict)},
                        driver_to_decision_s={k: round(now - v["driver_timestamp_s"], 6)
                            for k, v in ((audit or {}).get("frame_freshness") or {}).get("frames", {}).items()
                            if isinstance(v, dict) and type(v.get("driver_timestamp_s")) in (int, float)
                            and math.isfinite(v["driver_timestamp_s"])})
                    print(json.dumps(row, allow_nan=False), flush=True)
                    journal.write(json.dumps(row, allow_nan=False) + "\n")
                    journal.flush()
                yield observation
        if args.execute:
            with UartHttp(args.serial_port) as request:
                report = run_session(request=request, observations=observations(),
                    journal_path=args.journal, rgb_calibration_id=AR_INTRINSIC_SHA256,
                    segment_m=args.segment_m, max_segments=args.max_segments,
                    minimum_confidence=args.minimum_confidence)
                print(json.dumps(report, allow_nan=False), flush=True)
        else:
            args.journal.parent.mkdir(parents=True, exist_ok=True)
            with args.journal.open("x") as journal:
                for _ in observations():
                    pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
