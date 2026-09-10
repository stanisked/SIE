"""Offline-only forensic geometry checks for AR0234 to OV9281 extrinsics."""
from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from vision_core.ar0234_ov9281_static_extrinsic import (
    AR_SHAPE,
    CHECKERBOARD_CORNER_COUNT,
    CHECKERBOARD_SIZE,
    DATASET_SCHEMA_VERSION,
    OV_LEFT_SHAPE,
    SQUARE_SIZE_MM,
    CameraIntrinsics,
    StaticExtrinsicError,
    _audit_camera_local_pose,
    _audit_file_integrity,
    _audit_reprojection_metrics,
    _checkerboard,
    _json_payload,
    _json_safe,
    _load_ar_intrinsics,
    _load_candidate_transform,
    _load_ov_intrinsics,
    _mean_rotation,
    _project_board,
    _rotation_angle_deg,
    _sha256,
    _validate_dataset_manifest,
    _validate_dataset_session_evidence,
    _write_new,
)


AR_FRAME = "ar0234_rgb_optical_frame"
OV_RAW_LEFT_FRAME = "ov9281_physical_left_optical_frame"
OV_RECTIFIED_LEFT_FRAME = "rectified_left_optical_frame"


@dataclass(frozen=True)
class RigidTransform:
    name: str
    source_frame: str
    target_frame: str
    rotation: np.ndarray
    translation_mm: np.ndarray


def compose_ov_from_ar(
    ar_rotation_from_board: np.ndarray,
    ar_translation_from_board_mm: np.ndarray,
    ov_rotation_from_board: np.ndarray,
    ov_translation_from_board_mm: np.ndarray,
) -> RigidTransform:
    """Return T_ov_from_ar using OpenCV X_camera = R * X_object + t."""
    rotation = ov_rotation_from_board @ ar_rotation_from_board.T
    translation_mm = ov_translation_from_board_mm - rotation @ ar_translation_from_board_mm
    return RigidTransform("direct_pnp_composition", AR_FRAME, OV_RAW_LEFT_FRAME, rotation, translation_mm)


def invert_transform(transform: RigidTransform, *, name: str) -> RigidTransform:
    rotation = transform.rotation.T
    translation_mm = -rotation @ transform.translation_mm
    return RigidTransform(name, transform.target_frame, transform.source_frame, rotation, translation_mm)


def require_raw_frame_contract(*, source_frame: str, target_frame: str, ov_intrinsics_frame: str) -> None:
    if source_frame != AR_FRAME or target_frame != OV_RAW_LEFT_FRAME:
        raise StaticExtrinsicError("T_stereo_left_rgb must map ar0234_rgb_optical_frame to ov9281_physical_left_optical_frame")
    if ov_intrinsics_frame != OV_RAW_LEFT_FRAME:
        raise StaticExtrinsicError("raw physical-left K/D cannot be used as rectified_left_optical_frame intrinsics")


def cross_reprojection_errors_as_required(
    transform: RigidTransform,
    *,
    ar_rotation_from_board: np.ndarray,
    ar_translation_from_board_mm: np.ndarray,
    ov_corners: np.ndarray,
    ov_intrinsics: CameraIntrinsics,
) -> np.ndarray:
    projected = _project_board(
        transform.rotation @ ar_rotation_from_board,
        transform.rotation @ ar_translation_from_board_mm + transform.translation_mm,
        ov_intrinsics,
    )
    return np.linalg.norm(projected.reshape(-1, 2) - ov_corners.reshape(-1, 2), axis=1)


def _native_direction_errors(
    transform: RigidTransform,
    *,
    ar_rotation: np.ndarray,
    ar_translation_mm: np.ndarray,
    ar_corners: np.ndarray,
    ov_rotation: np.ndarray,
    ov_translation_mm: np.ndarray,
    ov_corners: np.ndarray,
    ar_intrinsics: CameraIntrinsics,
    ov_intrinsics: CameraIntrinsics,
) -> np.ndarray:
    if transform.source_frame == AR_FRAME and transform.target_frame == OV_RAW_LEFT_FRAME:
        return cross_reprojection_errors_as_required(
            transform,
            ar_rotation_from_board=ar_rotation,
            ar_translation_from_board_mm=ar_translation_mm,
            ov_corners=ov_corners,
            ov_intrinsics=ov_intrinsics,
        )
    if transform.source_frame == OV_RAW_LEFT_FRAME and transform.target_frame == AR_FRAME:
        projected = _project_board(
            transform.rotation @ ov_rotation,
            transform.rotation @ ov_translation_mm + transform.translation_mm,
            ar_intrinsics,
        )
        return np.linalg.norm(projected.reshape(-1, 2) - ar_corners.reshape(-1, 2), axis=1)
    raise StaticExtrinsicError("unsupported forensic transform direction")


def _transform_record(transform: RigidTransform) -> dict[str, Any]:
    return _json_safe({
        "name": transform.name,
        "source_reference_frame": transform.source_frame,
        "target_reference_frame": transform.target_frame,
        "rotation_3x3": transform.rotation.tolist(),
        "translation_mm": transform.translation_mm.tolist(),
    })


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise StaticExtrinsicError(f"cannot read {label} JSON") from error
    if not isinstance(value, dict):
        raise StaticExtrinsicError(f"{label} JSON must be an object")
    return value


def _raw_calibration_metadata(path: Path) -> dict[str, Any]:
    values = np.load(path, allow_pickle=False)
    required = ("K1", "D1", "R1", "P1", "size", "camera_1_semantics", "board_size", "square_size_mm")
    if any(key not in values.files for key in required):
        raise StaticExtrinsicError("OV calibration lacks raw/rectification forensic metadata")
    size = [int(value) for value in np.asarray(values["size"]).reshape(-1)]
    board_size = [int(value) for value in np.asarray(values["board_size"]).reshape(-1)]
    return _json_safe({
        "camera_1_semantics": str(values["camera_1_semantics"].item()),
        "raw_image_size": size,
        "board_size": board_size,
        "square_size_mm": float(values["square_size_mm"].item()),
        "raw_intrinsics": {"matrix_key": "K1", "distortion_key": "D1", "K_shape": list(values["K1"].shape), "D_shape": list(values["D1"].shape), "reference_frame": OV_RAW_LEFT_FRAME},
        "rectification_metadata": {"rotation_key": "R1", "projection_key": "P1", "R1_shape": list(values["R1"].shape), "P1_shape": list(values["P1"].shape), "reference_frame": OV_RECTIFIED_LEFT_FRAME, "used_for_pnp": False},
    })


def _aggregate_errors(error_sets: list[np.ndarray]) -> dict[str, Any]:
    values = np.concatenate(error_sets) if error_sets else np.asarray([], dtype=np.float64)
    return _audit_reprojection_metrics(values)


def forensic_geometry_review(*, capture_manifest: Path, ar_intrinsic: Path, ov_calibration: Path, candidate: Path) -> dict[str, Any]:
    manifest = _load_json(capture_manifest, "capture manifest")
    _validate_dataset_manifest(manifest)
    _validate_dataset_session_evidence(capture_manifest.parent, manifest)
    if manifest.get("schema_version") != DATASET_SCHEMA_VERSION:
        raise StaticExtrinsicError("forensic review requires the aggregated static-only dataset schema")
    candidate_json = _load_json(candidate, "candidate")
    _, candidate_rotation, candidate_translation_mm = _load_candidate_transform(candidate)
    ar_intrinsics = _load_ar_intrinsics(ar_intrinsic)
    ov_intrinsics = _load_ov_intrinsics(ov_calibration)
    source_frame = candidate_json.get("transform", {}).get("source_reference_frame")
    target_frame = candidate_json.get("transform", {}).get("target_reference_frame")
    require_raw_frame_contract(source_frame=source_frame, target_frame=target_frame, ov_intrinsics_frame=ov_intrinsics.frame)
    calibration_metadata = _raw_calibration_metadata(ov_calibration)
    if calibration_metadata["camera_1_semantics"] != "physical_left":
        raise StaticExtrinsicError("K1/D1 are not declared for physical_left")
    if calibration_metadata["raw_image_size"] != [OV_LEFT_SHAPE[1], OV_LEFT_SHAPE[0]]:
        raise StaticExtrinsicError("OV raw intrinsic image size differs from physical-left raw frames")
    if calibration_metadata["board_size"] != list(CHECKERBOARD_SIZE) or calibration_metadata["square_size_mm"] != SQUARE_SIZE_MM:
        raise StaticExtrinsicError("OV calibration checkerboard geometry differs from static extrinsic geometry")
    candidate_square = candidate_json.get("checkerboard", {}).get("square_size_mm")
    if candidate_square != SQUARE_SIZE_MM:
        raise StaticExtrinsicError("candidate square size differs from static extrinsic geometry")
    root = capture_manifest.parent
    observations: list[dict[str, Any]] = []
    pair_transforms: list[RigidTransform] = []
    raw_shapes: dict[str, set[tuple[int, ...]]] = {"ar0234": set(), "ov9281_physical_left": set()}
    for pair in manifest.get("pairs", []):
        pair_checkerboard = pair.get("checkerboard", {})
        if (
            pair_checkerboard.get("inner_corners") != list(CHECKERBOARD_SIZE)
            or pair_checkerboard.get("square_size_mm") != SQUARE_SIZE_MM
            or pair_checkerboard.get("ar0234_corner_count") != CHECKERBOARD_CORNER_COUNT
            or pair_checkerboard.get("ov9281_physical_left_corner_count") != CHECKERBOARD_CORNER_COUNT
        ):
            raise StaticExtrinsicError(f"checkerboard provenance differs for pair {pair.get('pair_id')}")
        integrity = {
            camera: _audit_file_integrity(root, pair.get(camera, {}).get("file", {}))
            for camera in ("ar0234", "ov9281_combined", "ov9281_physical_left")
        }
        if not all(row["status"] == "VERIFIED" for row in integrity.values()):
            raise StaticExtrinsicError(f"raw integrity failed for pair {pair.get('pair_id')}")
        ar_image = cv2.imread(str(root / integrity["ar0234"]["filename"]), cv2.IMREAD_COLOR)
        ov_image = cv2.imread(str(root / integrity["ov9281_physical_left"]["filename"]), cv2.IMREAD_COLOR)
        if ar_image is None or ov_image is None:
            raise StaticExtrinsicError(f"raw decode failed for pair {pair.get('pair_id')}")
        raw_shapes["ar0234"].add(tuple(ar_image.shape))
        raw_shapes["ov9281_physical_left"].add(tuple(ov_image.shape))
        if tuple(ar_image.shape) != AR_SHAPE or tuple(ov_image.shape) != OV_LEFT_SHAPE:
            raise StaticExtrinsicError(f"raw image dimensions differ for pair {pair.get('pair_id')}")
        ar_corners, ov_corners = _checkerboard(ar_image), _checkerboard(ov_image)
        if ar_corners is None or ov_corners is None:
            raise StaticExtrinsicError(f"54/54 checkerboard detection failed for pair {pair.get('pair_id')}")
        ar_rotation, ar_translation_mm, ar_local = _audit_camera_local_pose(ar_corners, ar_intrinsics)
        ov_rotation, ov_translation_mm, ov_local = _audit_camera_local_pose(ov_corners, ov_intrinsics)
        direct = compose_ov_from_ar(ar_rotation, ar_translation_mm, ov_rotation, ov_translation_mm)
        pair_transforms.append(direct)
        observations.append({
            "pair_id": pair["pair_id"],
            "session_id": pair["session_id"],
            "corner_counts": {"ar0234": CHECKERBOARD_CORNER_COUNT, "ov9281_physical_left": CHECKERBOARD_CORNER_COUNT},
            "camera_local_pnp_reprojection": {"ar0234": ar_local, "ov9281_physical_left": ov_local},
            "ar_rotation": ar_rotation,
            "ar_translation_mm": ar_translation_mm,
            "ar_corners": ar_corners,
            "ov_rotation": ov_rotation,
            "ov_translation_mm": ov_translation_mm,
            "ov_corners": ov_corners,
            "direct_transform": direct,
        })
    if not observations:
        raise StaticExtrinsicError("forensic review requires at least one valid pair")
    direct_consensus = RigidTransform(
        "direct_pnp_composition_consensus",
        AR_FRAME,
        OV_RAW_LEFT_FRAME,
        _mean_rotation([item.rotation for item in pair_transforms]),
        np.median(np.asarray([item.translation_mm for item in pair_transforms]), axis=0),
    )
    existing = RigidTransform("existing_candidate", AR_FRAME, OV_RAW_LEFT_FRAME, candidate_rotation, candidate_translation_mm)
    comparisons = [
        existing,
        invert_transform(existing, name="inverse_existing_candidate"),
        direct_consensus,
        invert_transform(direct_consensus, name="inverse_direct_pnp_composition_consensus"),
    ]
    comparison_reports: list[dict[str, Any]] = []
    per_pair: list[dict[str, Any]] = [
        {
            "pair_id": row["pair_id"],
            "session_id": row["session_id"],
            "corner_counts": row["corner_counts"],
            "camera_local_pnp_reprojection": row["camera_local_pnp_reprojection"],
            "direct_pnp_composition": _transform_record(row["direct_transform"]),
            "comparisons": {},
        }
        for row in observations
    ]
    for transform in comparisons:
        required_sets: list[np.ndarray] = []
        native_sets: list[np.ndarray] = []
        for index, row in enumerate(observations):
            required = cross_reprojection_errors_as_required(transform, ar_rotation_from_board=row["ar_rotation"], ar_translation_from_board_mm=row["ar_translation_mm"], ov_corners=row["ov_corners"], ov_intrinsics=ov_intrinsics)
            native = _native_direction_errors(transform, ar_rotation=row["ar_rotation"], ar_translation_mm=row["ar_translation_mm"], ar_corners=row["ar_corners"], ov_rotation=row["ov_rotation"], ov_translation_mm=row["ov_translation_mm"], ov_corners=row["ov_corners"], ar_intrinsics=ar_intrinsics, ov_intrinsics=ov_intrinsics)
            required_sets.append(required)
            native_sets.append(native)
            per_pair[index]["comparisons"][transform.name] = {"as_required_T_ov_from_ar": _audit_reprojection_metrics(required), "native_direction": _audit_reprojection_metrics(native)}
        comparison_reports.append({
            "transform": _transform_record(transform),
            "compatible_with_required_T_ov_from_ar": transform.source_frame == AR_FRAME and transform.target_frame == OV_RAW_LEFT_FRAME,
            "as_required_T_ov_from_ar_cross_reprojection": _aggregate_errors(required_sets),
            "native_direction_cross_reprojection": _aggregate_errors(native_sets),
        })
    direct_residual = {
        "translation_mm": float(np.linalg.norm(direct_consensus.translation_mm - existing.translation_mm)),
        "rotation_deg": _rotation_angle_deg(direct_consensus.rotation, existing.rotation),
    }
    existing_rms = comparison_reports[0]["as_required_T_ov_from_ar_cross_reprojection"]["rms_px"]
    inverse_as_required_rms = comparison_reports[1]["as_required_T_ov_from_ar_cross_reprojection"]["rms_px"]
    formula_verdict = "NO_FORMULA_OR_DIRECTION_INVERSION_DEFECT_PROVEN" if existing_rms <= inverse_as_required_rms else "DIRECTION_INVERSION_DEFECT_INDICATED"
    session_ids = sorted({row["session_id"] for row in per_pair})
    session_reports = []
    for session_id in session_ids:
        rows = [row for row in per_pair if row["session_id"] == session_id]
        session_reports.append({
            "session_id": session_id,
            "pair_ids": [row["pair_id"] for row in rows],
            "existing_candidate_pair_rms_distribution_px": _audit_reprojection_metrics(
                np.asarray([row["comparisons"]["existing_candidate"]["as_required_T_ov_from_ar"]["rms_px"] for row in rows])
            ),
        })
    return _json_safe({
        "schema_version": "sie.ar0234_ov9281_extrinsic_forensic_geometry.v1",
        "status": "FORENSIC_DIAGNOSTIC_ONLY_COMPLETE",
        "candidate": {"path": str(candidate), "sha256": _sha256(candidate), "status": candidate_json["status"], "status_unchanged": True},
        "inputs": {"capture_manifest": {"path": str(capture_manifest), "sha256": _sha256(capture_manifest)}, "ar0234_intrinsic": {"path": str(ar_intrinsic), "sha256": _sha256(ar_intrinsic)}, "ov9281_calibration": {"path": str(ov_calibration), "sha256": _sha256(ov_calibration)}},
        "opencv_pose_convention": {"equation": "X_camera = R * X_object + t", "relative_transform": "T_ov_from_ar = T_ov_from_board * inverse(T_ar_from_board)", "rotation_formula": "R_ov_from_ar = R_ov_from_board * transpose(R_ar_from_board)", "translation_formula": "t_ov_from_ar = t_ov_from_board - R_ov_from_ar * t_ar_from_board", "verified_by_synthetic_test": True},
        "frame_and_intrinsic_consistency": {"ar0234_raw_image_shapes": [list(shape) for shape in sorted(raw_shapes["ar0234"])], "ov9281_physical_left_raw_image_shapes": [list(shape) for shape in sorted(raw_shapes["ov9281_physical_left"])], "ar0234_intrinsic_image_size": [AR_SHAPE[1], AR_SHAPE[0]], "ov9281": calibration_metadata, "raw_K_D_used_for_pnp": True, "rectified_R1_P1_used_for_pnp": False, "raw_and_rectified_frames_kept_distinct": True, "square_size_mm_consistent": True},
        "camera_local_pnp_reprojection": {
            "ar0234": _aggregate_errors([
                np.asarray([row["camera_local_pnp_reprojection"]["ar0234"]["rms_px"]]) for row in observations
            ]),
            "ov9281_physical_left": _aggregate_errors([
                np.asarray([row["camera_local_pnp_reprojection"]["ov9281_physical_left"]["rms_px"]]) for row in observations
            ]),
            "aggregation_note": "Distribution of per-pair camera-local RMS values.",
        },
        "transform_comparisons": comparison_reports,
        "direct_pnp_consensus_residual_to_existing_candidate": direct_residual,
        "per_pair": per_pair,
        "per_session": session_reports,
        "diagnosis": {"formula_and_direction_verdict": formula_verdict, "numeric_evidence": {"existing_candidate_as_T_ov_from_ar_rms_px": existing_rms, "inverse_candidate_as_T_ov_from_ar_rms_px": inverse_as_required_rms, "direct_consensus_to_candidate_translation_mm": direct_residual["translation_mm"], "direct_consensus_to_candidate_rotation_deg": direct_residual["rotation_deg"]}, "statement": "High cross-camera reprojection alone does not prove a formula, direction, hardware, or frame-root-cause defect."},
        "facts": ["OpenCV PnP poses use X_camera = R * X_object + t.", "Physical-left raw K1/D1 are used for PnP; R1/P1 rectification metadata are inspected but not used.", "All comparison transforms are diagnostic and the candidate is unchanged."],
        "hypotheses": ["Inverse transforms are evaluated both in their native direction and, comparison-only, as the required T_ov_from_ar.", "Direct PnP composition consensus is recomputed without replacing the candidate."],
        "unresolved_questions": ["Whether residual error originates in static pairing, calibration applicability, camera rigidity, or another unmeasured source remains unresolved.", "No hardware root cause or numeric acceptance policy is established by this forensic review."],
        "limitation": "Offline forensic diagnostic only. No camera capture, runtime integration, calibration activation, automatic transform application, or hardware action is performed.",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
    })


def _summary_markdown(report: dict[str, Any]) -> str:
    diagnosis = report["diagnosis"]
    evidence = diagnosis["numeric_evidence"]
    return "\n".join([
        "# Forensic geometry AR0234 ↔ OV9281",
        "",
        f"- Вердикт: `{diagnosis['formula_and_direction_verdict']}`",
        f"- Existing candidate as T_ov_from_ar RMS: `{evidence['existing_candidate_as_T_ov_from_ar_rms_px']:.6f} px`",
        f"- Inverse candidate as T_ov_from_ar RMS: `{evidence['inverse_candidate_as_T_ov_from_ar_rms_px']:.6f} px`",
        f"- Direct consensus residual: `{evidence['direct_consensus_to_candidate_translation_mm']:.6f} mm`, `{evidence['direct_consensus_to_candidate_rotation_deg']:.6f} deg`",
        "- Candidate status remains `PROVISIONAL_DIAGNOSTIC_ONLY`.",
        "",
        "## Факты",
        "",
        *[f"- {item}" for item in report["facts"]],
        "",
        "## Гипотезы",
        "",
        *[f"- {item}" for item in report["hypotheses"]],
        "",
        "## Нерешённые вопросы",
        "",
        *[f"- {item}" for item in report["unresolved_questions"]],
        "",
    ])


def write_forensic_outputs(*, output_dir: Path, report: dict[str, Any]) -> None:
    if output_dir.exists():
        raise StaticExtrinsicError(f"forensic output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=False)
    _write_new(output_dir / "forensic_geometry_report.json", _json_payload(report))
    _write_new(output_dir / "forensic_geometry_summary.md", _summary_markdown(report).encode("utf-8"))
