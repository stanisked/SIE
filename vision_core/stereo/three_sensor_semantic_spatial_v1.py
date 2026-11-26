"""Pure source-bound semantic stereo ownership and diagnostic LD19 comparisons."""
from dataclasses import asdict
import hashlib

import cv2
import numpy as np

from sie_ros2.live_temporal_association_v1 import BRIDGE_POLICY, FRESHNESS, PAIRING, RECENT, nanoseconds
from sie_ros2.host_timebase_bridge_v1 import convert_monotonic_stamp
from sie_ros2.temporal_association_freshness_v1 import classify_source_age
from vision_core.person_localization.diagnostic_instance_selection import has_unique_eligible_instance_binding
from vision_core.person_localization.instance_mask_stereo_fusion import depth_from_owned_samples_with_candidate_indices
from vision_core.person_localization.torso_core_geometry import TORSO_CORE_POLICY, projected_in_torso_core
from vision_core.stereo.local_stereo_lidar_correspondence_v1 import CANDIDATE, compare_local_scan
from vision_core.stereo.stereo_lidar_static_v1 import fit_stereo_surface

SCHEMA = "sie.three_sensor_semantic_spatial.v1.diagnostic"
RGB_SHAPE = (1200, 1920)
MATCH = "MATCH_DIAGNOSTIC_ONLY"


def semantic_snapshot_metadata(audit, mask_payload):
    """Copy producer-native evidence after primary publication, without a new decision."""
    seg = audit.get("segmentation_observation") or {}
    shape = audit.get("shape_diagnostic") or {}
    return {
        "capture_mask_cycle_id": mask_payload.get("cycle_id"),
        "capture_mask_sha256": mask_payload.get("mask_sha256"),
        "segmentation": {key: seg.get(key) for key in ("cycle_id", "status", "instances",
            "instance_mask_sha256", "arrays_sha256", "model_sha256", "input_transport")},
        "target": audit.get("ar0234_dynamic_target"),
        "geometry_accepted": (audit.get("target_geometry") or {}).get("accepted"),
        "geometry_override": ("dynamic_shadow_edge_override" in audit
                              or "dynamic_shadow_geometry_override" in audit),
        "track_confirmation": audit.get("shadow_track_confirmation"),
        "rgb_track_continuity": audit.get("shadow_rgb_continuity"),
        "shape": {key: shape.get(key) for key in ("cycle_id", "mask", "mask_sha256",
                                                  "model_sha256", "arrays_sha256")},
        "mask_coordinates": {"mode": "ORIGINAL_DISTORTED_RGB"},
    }


def bind_semantic_target(manifest, mask):
    """Validate the exact current observation, mask and track from the preview."""
    meta = manifest.get("step71_semantic") or {}
    seg, target = meta.get("segmentation") or {}, meta.get("target") or {}
    cycle = manifest.get("cycle_id")
    instances = seg.get("instances")
    if not has_unique_eligible_instance_binding(instances, target):
        raise ValueError("UNVERIFIED_OR_MULTIPLE_ELIGIBLE_RGB_TARGETS")
    candidate = target["candidate"]
    index = candidate["instance_index"]
    if index != manifest.get("selected_instance_index") or not cycle:
        raise ValueError("RGB_INSTANCE_IDENTITY_MISMATCH")
    observation_id = f"segmentation-preview:{cycle}:{index}"
    confirmation = meta.get("track_confirmation") or {}
    continuity = meta.get("rgb_track_continuity") or {}
    track = manifest.get("track_id")
    if not isinstance(track, str) or not track:
        raise ValueError("CURRENT_RGB_TRACK_UNAVAILABLE")
    confirmed = (confirmation.get("state") == "CONFIRMED_BY_CURRENT_RGB_STEREO_ASSOCIATION"
                 and confirmation.get("confirmed_observation_id") == observation_id
                 and confirmation.get("track_id") == track)
    continuous = (not confirmation and continuity.get("status") == "MATCHED_CURRENT_RGB_DIAGNOSTIC_ONLY"
                  and continuity.get("track_id") == track)
    if not (confirmed or continuous) or (continuity and continuity.get("track_id") != track):
        raise ValueError("CURRENT_RGB_TRACK_BINDING_MISMATCH")
    if meta.get("geometry_accepted") is not True or meta.get("geometry_override") is not False:
        raise ValueError("CURRENT_RGB_GEOMETRY_REFUSED")
    if mask.dtype != np.bool_ or mask.ndim != 2:
        raise ValueError("INVALID_SEMANTIC_MASK")
    mask_hash = hashlib.sha256(mask.tobytes()).hexdigest()
    masks = seg.get("instance_mask_sha256")
    shape = meta.get("shape") or {}
    shape_mask = shape.get("mask") or {}
    if (seg.get("status") != "CURRENT_INSTANCES_DIAGNOSTIC_ONLY"
            or cycle != seg.get("cycle_id") or cycle != shape.get("cycle_id")
            or cycle != meta.get("capture_mask_cycle_id")
            or not isinstance(masks, list) or len(masks) != len(instances)
            or masks[index] != mask_hash or shape.get("mask_sha256") != mask_hash
            or meta.get("capture_mask_sha256") != mask_hash
            or manifest.get("instance_mask_sha256") != mask_hash
            or shape_mask.get("status") != "CURRENT_FRAME_MASK_DIAGNOSTIC_ONLY"
            or shape_mask.get("instance_index") != index
            or not seg.get("arrays_sha256") or shape_mask.get("source_rgb_arrays_sha256") != seg["arrays_sha256"]
            or not seg.get("model_sha256") or shape.get("model_sha256") != seg["model_sha256"]
            or seg["model_sha256"] != manifest.get("segmentation_model_sha256")
            or not shape.get("arrays_sha256") or shape["arrays_sha256"] != manifest.get("fusion_arrays_sha256")):
        raise ValueError("CURRENT_RGB_MASK_PROVENANCE_MISMATCH")
    if candidate.get("truncated_left") or candidate.get("truncated_right"):
        raise ValueError("SIDE_TRUNCATED_TARGET_REFUSED")
    torso = None
    if candidate.get("truncated_top") or candidate.get("truncated_bottom"):
        torso = manifest.get("stable_torso_core_front_geometry") or {}
        provenance = {
            "source_cycle_id": cycle, "mask_sha256": mask_hash, "instance_index": index,
            "model_sha256": seg["model_sha256"], "rgb_arrays_sha256": seg["arrays_sha256"],
            "fusion_arrays_sha256": shape["arrays_sha256"],
        }
        if (torso.get("status") != "STABLE_TORSO_CORE_FRONT_DIAGNOSTIC_ONLY"
                or torso.get("provenance") != provenance
                or torso.get("torso_core_policy") != TORSO_CORE_POLICY
                or type(torso.get("point_count")) is not int
                or torso["point_count"] < TORSO_CORE_POLICY["minimum_stable_points"]):
            raise ValueError("TRUNCATED_TORSO_WITHOUT_CURRENT_STABLE_CORE")
    return {"observation_id": observation_id, "track_id": track, "instance_index": index,
            "track_basis": "CURRENT_CONFIRMATION" if confirmed else "CURRENT_RGB_CONTINUITY",
            "mask_sha256": mask_hash, "model_sha256": seg["model_sha256"],
            "rgb_transport_binding_sha256": seg["arrays_sha256"],
            "fusion_arrays_sha256": shape["arrays_sha256"],
            "mask_coordinates": meta.get("mask_coordinates"),
            "bbox_xyxy_px": candidate.get("bbox_xyxy_px"),
            "torso_only": torso is not None}


def temporal_eligibility(temporal, frame_freshness):
    """Preserve independent Step 57 dispositions and screen the actual third edge."""
    rows = (frame_freshness or {}).get("frames") or {}
    rgb, stereo = temporal.get("rgb") or {}, (temporal.get("actual_camera_pair") or {}).get("stereo") or {}
    links = temporal.get("zero_wait") or {}
    a, b = links.get("rgb_stereo") or {}, links.get("rgb_ld19_ros_header") or {}
    scan = b.get("candidate") or {}
    details = {"rgb": rgb, "actual_stereo": stereo, "ld19": scan,
               "rgb_stereo": a, "rgb_ld19": b, "stereo_ld19": {"status": "UNAVAILABLE"}}
    for name, source in (("ar0234", rgb), ("ov9281", stereo)):
        packet = rows.get(name) or {}
        if (not packet or source.get("source_id") != name or source.get("sequence") != packet.get("sequence")
                or source.get("timestamp_ns") != nanoseconds(packet.get("driver_timestamp_s"))
                or source.get("received_monotonic_ns") != nanoseconds(packet.get("host_packet_received_monotonic_s"))):
            return details | {"status": "CAMERA_PACKET_BINDING_MISMATCH"}
    if temporal.get("association_policy") != asdict(PAIRING) or temporal.get("freshness_policy") != asdict(FRESHNESS):
        return details | {"status": "STEP57_TEMPORAL_POLICY_MISMATCH"}
    cutoff = (temporal.get("endpoint_monotonic_ns") or {}).get("zero_wait")
    clock = (temporal.get("clock") or {}).get("zero_wait") or {}
    if cutoff != rgb["received_monotonic_ns"] or clock.get("clock_valid") is not True:
        return details | {"status": "RGB_CUTOFF_OR_CLOCK_UNVERIFIED"}
    mapped = convert_monotonic_stamp(clock["bridge"], source_monotonic_ns=stereo["timestamp_ns"],
                                     evaluation_monotonic_ns=cutoff, boot_id=temporal["boot_id"],
                                     policy=BRIDGE_POLICY)
    estimate = mapped["estimated_realtime_ns"]
    if estimate is not None and type(scan.get("timestamp_ns")) is int:
        if scan.get("ros_clock_status") != "OPERATOR_ATTESTED_SAME_HOST_REALTIME":
            details["stereo_ld19"] = {"status": "LD19_CLOCK_UNVERIFIED"}
        else:
            delta = scan["timestamp_ns"] - estimate
            direct = MATCH if abs(delta) <= PAIRING.max_rgb_ld19_header_delta_ns else "STEREO_LD19_HEADER_DELTA_EXCEEDED"
            details["stereo_ld19"] = {"status": direct, "ld19_header_minus_stereo_ns": delta,
                                     "max_absolute_delta_ns": PAIRING.max_rgb_ld19_header_delta_ns,
                                     "policy": "STEP66_EXISTING_50MS_TRIAL"}
    if a.get("status") != MATCH or b.get("status") != MATCH:
        return details | {"status": "INDEPENDENT_RGB_TEMPORAL_LINK_REFUSED"}
    if stereo != a.get("candidate") or stereo["received_monotonic_ns"] > cutoff or scan["received_monotonic_ns"] > cutoff:
        return details | {"status": "ACTUAL_STEREO_OR_LD19_NOT_CAUSALLY_BOUND"}
    offset = clock["bridge"]["offset_ns"]
    rgb_stereo = stereo["timestamp_ns"] - rgb["timestamp_ns"]
    rgb_ld19 = scan["timestamp_ns"] - (rgb["timestamp_ns"] + offset)
    if (rgb_stereo != a.get("candidate_minus_rgb_ns") or rgb_ld19 != b.get("candidate_minus_rgb_ns")
            or abs(rgb_stereo) > PAIRING.max_rgb_stereo_delta_ns
            or abs(rgb_ld19) > PAIRING.max_rgb_ld19_header_delta_ns
            or scan["timestamp_ns"] > scan["received_monotonic_ns"] + offset):
        return details | {"status": "INDEPENDENT_LINK_PROVENANCE_MISMATCH"}
    rgb_age = classify_source_age(cutoff - rgb["timestamp_ns"], FRESHNESS.max_rgb_age_ns,
                                  FRESHNESS.max_future_skew_ns)
    stereo_age = classify_source_age(cutoff - stereo["timestamp_ns"], FRESHNESS.max_stereo_age_ns,
                                     FRESHNESS.max_future_skew_ns)
    scan_age = classify_source_age(cutoff + offset - scan["timestamp_ns"],
                                   FRESHNESS.max_ld19_ros_header_age_ns, FRESHNESS.max_future_skew_ns)
    if (rgb_age, stereo_age, scan_age) != (RECENT, RECENT, RECENT):
        return details | {"status": "SOURCE_FRESHNESS_REFUSED",
                          "source_freshness": {"rgb": rgb_age, "stereo": stereo_age, "ld19_header": scan_age}}
    return details | {"status": ("THREE_SOURCE_TIMING_DIAGNOSTIC_ONLY" if details["stereo_ld19"]["status"] == MATCH
                                 else details["stereo_ld19"]["status"])}


def project_ar(physical_left_m, calibration):
    """Invert the calibrated AR→left transform and use the original RGB distortion model."""
    ar = (physical_left_m - calibration["T"]) @ calibration["R"]
    uv = np.full((len(ar), 2), np.nan)
    front = np.isfinite(ar).all(axis=1) & (ar[:, 2] > 0)
    if front.any():
        xy, _ = cv2.projectPoints(ar[front], np.zeros(3), np.zeros(3), calibration["K"], calibration["D"])
        uv[front] = xy.reshape(-1, 2)
    return uv, ar[:, 2]


def mask_coordinates(uv, mapping, mask_shape, rgb_shape=RGB_SHAPE):
    """Map original pixel centers into an explicitly declared resize/letterbox grid."""
    if mapping == {"mode": "ORIGINAL_DISTORTED_RGB"} and mask_shape == rgb_shape:
        return uv.copy()
    if not isinstance(mapping, dict) or mapping.get("mode") != "AFFINE_RESIZED_OR_LETTERBOXED_RGB":
        raise ValueError("UNDECLARED_SEMANTIC_MASK_COORDINATES")
    if mapping.get("rgb_shape") != list(rgb_shape) or mapping.get("mask_shape") != list(mask_shape):
        raise ValueError("SEMANTIC_MASK_GRID_MISMATCH")
    scale = np.asarray(mapping.get("scale_xy"), dtype=float)
    pad = np.asarray(mapping.get("pad_xy_px"), dtype=float)
    if (scale.shape != (2,) or pad.shape != (2,) or not np.isfinite([scale, pad]).all()
            or (scale <= 0).any() or (pad < 0).any()
            or (pad + scale * rgb_shape[::-1] > np.asarray(mask_shape[::-1]) + 1e-6).any()):
        raise ValueError("INVALID_SEMANTIC_MASK_AFFINE")
    return (uv + .5) * scale - .5 + pad


def _beam_rgb_projection(point, mask, mapping, calibration):
    uv, _ = project_ar(np.asarray([point]), calibration)
    if not np.isfinite(uv).all():
        return None, False
    mapped = mask_coordinates(uv, mapping, mask.shape)[0]
    u, v = (int(np.floor(value + .5)) for value in mapped)
    inside = 0 <= u < mask.shape[1] and 0 <= v < mask.shape[0]
    return uv[0].tolist(), bool(inside and mask[v, u])


def semantic_stereo_component(grid, mask, binding, calibration, policy):
    """Find one plausible visible RGB-owned stereo component before LD19 is read."""
    xyz, _ = grid
    y, x = np.nonzero(np.isfinite(xyz[:, :, 2]))
    points = xyz[y, x]
    uv, ar_z = project_ar(points, calibration)
    original = (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= -.5) & (uv[:, 0] < RGB_SHAPE[1] - .5)
                & (uv[:, 1] >= -.5) & (uv[:, 1] < RGB_SHAPE[0] - .5))
    ix = np.flatnonzero(original)
    rgb_pixels = np.floor(uv[ix] + .5).astype(int)
    pixel_id = rgb_pixels[:, 1] * RGB_SHAPE[1] + rgb_pixels[:, 0]
    nearest = np.full(np.prod(RGB_SHAPE), np.inf)
    np.minimum.at(nearest, pixel_id, ar_z[ix])
    visible = ar_z[ix] == nearest[pixel_id]
    mapped = mask_coordinates(uv[ix], binding["mask_coordinates"], mask.shape)
    rounded = np.floor(mapped + .5).astype(int)
    u, v = rounded.T
    margin = policy["mask_interior_margin_px"]
    interior = cv2.erode(mask.astype(np.uint8), np.ones((2 * margin + 1,) * 2, np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    within = (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
    mask_owned = np.zeros(len(ix), bool)
    mask_owned[within] = interior[v[within], u[within]]
    torso_roi = None
    if binding["torso_only"]:
        core, torso_roi = projected_in_torso_core(uv[ix], binding["bbox_xyxy_px"])
        mask_owned &= core
    owned = ix[mask_owned & visible]
    report = {"stereo_valid_points": len(points), "rgb_projected_points": len(ix),
              "rgb_occluded_projection_samples": int((~visible).sum()),
              "rgb_mask_owned_visible_points": len(owned), "torso_core_roi_px": torso_roi}
    empty = np.zeros(xyz.shape[:2], bool)
    layer, selected = depth_from_owned_samples_with_candidate_indices(
        points[owned, 2], int(mask.sum()), int(interior.sum()),
        minimum_samples=policy["minimum_stereo_component_points"],
        depth_gap_m=policy["minimum_depth_layer_gap_m"], relative_depth_gap=policy["relative_depth_layer_gap"],
        minimum_layer_support_fraction=policy["minimum_competing_component_fraction"],
    )
    report["stereo_depth_layers"] = layer
    if not selected.size:
        status = ("AMBIGUOUS_RGB_STEREO_DEPTH_LAYERS" if layer.get("plausible_layer_count", 0) > 1
                  else "NO_SUPPORTED_RGB_OWNED_STEREO_LAYER")
        return empty, report | {"status": status}
    ids = owned[selected]
    occupancy = empty.copy()
    occupancy[y[ids], x[ids]] = True
    count, labels, stats, _ = cv2.connectedComponentsWithStats(occupancy.astype(np.uint8), connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA]
    minimum = max(policy["minimum_stereo_component_points"],
                  policy["minimum_competing_component_fraction"] * int(areas.max()))
    plausible = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= minimum]
    report["spatial_components"] = [{"component_id": i, "support": int(stats[i, cv2.CC_STAT_AREA]),
                                    "plausible": i in plausible} for i in range(1, count)]
    if len(plausible) != 1:
        return empty, report | {"status": ("AMBIGUOUS_RGB_STEREO_COMPONENTS" if plausible
                                           else "NO_SUPPORTED_RGB_STEREO_COMPONENT")}
    component_id, = plausible
    component = labels == component_id
    identity = hashlib.sha256(np.packbits(component).tobytes()).hexdigest()
    return component, report | {"status": "SINGLE_RGB_OWNED_STEREO_COMPONENT_DIAGNOSTIC_ONLY",
                                "component_id": component_id, "component_mask_sha256": identity}


def associate_cycle(disparity, lr, grid, mask, binding, raw_scan, calibration,
                    rotation, translation, axial, local_policy, policy):
    """LD19 comparisons happen only after semantic stereo ownership is frozen."""
    component, semantic = semantic_stereo_component(grid, mask, binding, calibration, policy)
    independent_global = {"status": "NO_SEMANTIC_STEREO_COMPONENT"}
    if component.any():
        try:
            plane = fit_stereo_surface(disparity, lr, calibration["Q"], calibration["R1"], component)
            independent_global = {"status": "GLOBAL_PLANE_DIAGNOSTIC_ONLY", "stereo_plane_fit": plane.fit_stats}
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            independent_global = {"status": "GLOBAL_PLANE_REFUSED", "reason": str(error)}
    beams, _ = compare_local_scan(
        raw_scan, grid if component.any() else None, np.ones_like(component),
        calibration["R1"], calibration["P1"], rotation, translation, axial, (-180, 180), local_policy,
    )
    for beam in beams:
        beam["rgb_owned_stereo_component_at_projection"] = False
        if beam["inside_image"]:
            u, v = np.floor(np.asarray(beam["projected_rectified_left_px"]) + .5).astype(int)
            beam["rgb_owned_stereo_component_at_projection"] = bool(component[v, u])
            rgb_uv, mask_hit = _beam_rgb_projection(
                beam["hit_estimated_optical_m"], mask, binding["mask_coordinates"], calibration,
            )
            beam["ld19_projected_original_rgb_px_diagnostic"] = rgb_uv
            beam["ld19_projected_rgb_mask_membership_diagnostic"] = mask_hit
            stereo_uv, stereo_mask_hit = _beam_rgb_projection(
                grid[0][v, u], mask, binding["mask_coordinates"], calibration,
            )
            beam["stereo_anchor_projected_original_rgb_px"] = stereo_uv
            beam["stereo_anchor_rgb_mask_membership"] = stereo_mask_hit
            if not component.any():
                beam["semantic_status"] = semantic["status"]
            elif not component[v, u]:
                beam["semantic_status"] = "LD19_PROJECTION_OUTSIDE_SELECTED_RGB_STEREO_COMPONENT"
            elif beam["status"] != CANDIDATE:
                beam["semantic_status"] = beam["status"]
            else:
                pixels = np.asarray(beam["local_stereo"]["selected_pixels_xy_px"])
                if not component[pixels[:, 1], pixels[:, 0]].all():
                    beam["semantic_status"] = "AMBIGUOUS_LOCAL_SUPPORT_CROSSES_SEMANTIC_COMPONENT"
                else:
                    # Retain untrimmed Step 69 residuals as independent evidence.
                    beam["semantic_status"] = "THREE_SENSOR_LOCAL_CANDIDATE_DIAGNOSTIC_ONLY"
                    beam["stereo_component_id"] = semantic["component_id"]
                    localized_rgb, _ = project_ar(grid[0][pixels[:, 1], pixels[:, 0]], calibration)
                    beam["selected_support_original_rgb_px"] = localized_rgb.tolist()
        else:
            beam["semantic_status"] = beam["status"]
    return {"semantic_stereo": semantic, "independent_global_plane": independent_global, "beams": beams}
