"""Fail-closed offline person-depth fusion for immutable paired PNG inputs.

The module is playback-only: it does not open cameras, validate live
temperatures, update World State, select navigation targets, or control hardware.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import tempfile
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

import cv2
import numpy as np

from vision_core.person_localization.models import BoundingBox, PersonLocalizationResult, PersonLocalizationStatus
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.rgb_stereo_extrinsic.solve import (
    AR_INTRINSIC, AR_INTRINSIC_SHA256, ARTIFACT_PATH as EXTRINSIC_CANDIDATE_PATH,
    STEREO_CALIBRATION, STEREO_CALIBRATION_SHA256,
    VALIDATION_ARTIFACT_PATH as EXTRINSIC_VALIDATION_PATH, ExtrinsicSolveError,
    load_ar_intrinsics,
)
from vision_core.stereo.guarded_runtime_v6_v2 import (
    REQUIRED_NUM_DISPARITIES, GuardedStereoDepthProcessor, StereoDepthFrame,
)
from vision_core.stereo.stereo_calibration_guard_v6 import (
    GuardedCalibration, StereoCalibrationGuard, TemperatureGateResult, file_sha256,
)

AR_SHAPE = (1200, 1920, 3)
COMBINED_SHAPE = (800, 2560, 3)
RECTIFIED_LEFT_FRAME = "rectified_left_optical_frame"
EXPECTED_CANDIDATE_SHA256 = "0d9b4a2f4110eecbc4994eddc0f7be267868520920988cfb426fdf034041689e"
EXPECTED_VALIDATION_SHA256 = "4dc4b9dbb0265a6765f8f368e709538d33813aaa0f91a10f051d0ac39ca37a1a"
DEFAULT_POLICY_PATH = Path("vision_core/config/runtime/stereo_calibration_v6_runtime_policy_v2.json")


class PersonDepthFusionError(RuntimeError):
    pass


class PersonMeasurementStatus(str, Enum):
    SUCCESS = "SUCCESS"
    PERSON_LOST = "PERSON_LOST"
    MULTIPLE_PERSONS = "MULTIPLE_PERSONS"
    DEPTH_UNAVAILABLE = "DEPTH_UNAVAILABLE"
    INSUFFICIENT_SUPPORT = "INSUFFICIENT_SUPPORT"
    CALIBRATION_INVALID = "CALIBRATION_INVALID"
    INVALID_INPUT = "INVALID_INPUT"


@dataclass(frozen=True)
class FusionPolicy:
    minimum_selected_points: int = 100
    minimum_inliers: int = 60

    def __post_init__(self) -> None:
        if type(self.minimum_selected_points) is not int or self.minimum_selected_points <= 0:
            raise ValueError("minimum_selected_points must be a positive integer")
        if type(self.minimum_inliers) is not int or not 0 < self.minimum_inliers <= self.minimum_selected_points:
            raise ValueError("minimum_inliers must be positive and not exceed minimum_selected_points")


@dataclass(frozen=True)
class FusionCalibration:
    candidate_path: Path
    candidate_sha256: str
    validation_path: Path
    validation_sha256: str
    calibration_id: str
    raw_from_ar_m: np.ndarray
    ar_from_rectified_left_m: np.ndarray
    ar_camera_matrix: np.ndarray
    ar_distortion: np.ndarray
    rectified_p1: np.ndarray
    stereo_calibration_path: Path
    stereo_calibration_sha256: str
    stereo_policy_path: Path
    stereo_policy_id: str
    stereo_depth_range_m: tuple[float, float]


@dataclass(frozen=True)
class PersonMeasurement:
    status: PersonMeasurementStatus
    timestamp: str
    detail: str | None = None
    measurement_id: str | None = None
    person_observation_id: str | None = None
    person_evidence_id: str | None = None
    reference_frame: str | None = None
    units: str | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    range_m: float | None = None
    confidence: float | None = None
    selected_point_count: int = 0
    support_ratio: float | None = None
    initial_median_z_m: float | None = None
    depth_mad_m: float | None = None
    inlier_count: int = 0
    rgb_roi_xyxy_px: list[int] | None = None
    rgb_seed_roi_xyxy_px: list[int] | None = None
    detector_provenance: dict[str, Any] | None = None
    calibration: dict[str, Any] | None = None
    stereo_policy_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        value = {key: getattr(self, key) for key in self.__dataclass_fields__}
        value["status"] = self.status.value
        _assert_json_finite(value)
        return value


class StereoDepthKernel(Protocol):
    def process(self, combined_bgr: np.ndarray) -> StereoDepthFrame: ...


def _sha256(path: Path) -> str:
    if not path.is_file() or path.is_symlink():
        raise PersonDepthFusionError(f"not a regular file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise PersonDepthFusionError(f"invalid JSON: {path}") from error
    if type(value) is not dict:
        raise PersonDepthFusionError(f"JSON root must be object: {path}")
    return value


def _matrix_m(value: object, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], (0., 0., 0., 1.)):
        raise PersonDepthFusionError(f"invalid {name} matrix")
    rotation = matrix[:3, :3]
    if abs(float(np.linalg.det(rotation)) - 1.) > 1e-6 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise PersonDepthFusionError(f"non-rigid {name} rotation")
    return matrix


def load_fusion_calibration(*, candidate_path: Path = EXTRINSIC_CANDIDATE_PATH, validation_path: Path = EXTRINSIC_VALIDATION_PATH, ar_intrinsic_path: Path = AR_INTRINSIC, stereo_calibration_path: Path = STEREO_CALIBRATION, stereo_policy_path: Path = DEFAULT_POLICY_PATH, expected_candidate_sha256: str = EXPECTED_CANDIDATE_SHA256, expected_validation_sha256: str = EXPECTED_VALIDATION_SHA256) -> FusionCalibration:
    if _sha256(candidate_path) != expected_candidate_sha256:
        raise PersonDepthFusionError("candidate extrinsic SHA mismatch")
    if _sha256(validation_path) != expected_validation_sha256:
        raise PersonDepthFusionError("physical extrinsic validation SHA mismatch")
    candidate, validation = _json_object(candidate_path), _json_object(validation_path)
    if candidate.get("status") != "CANDIDATE_OFFLINE_SOLVED":
        raise PersonDepthFusionError("candidate status is not CANDIDATE_OFFLINE_SOLVED")
    if validation.get("status") != "PHYSICAL_EXTRINSIC_VALIDATION_PASS" or validation.get("no_refit_performed") is not True:
        raise PersonDepthFusionError("physical validation status/no_refit gate failed")
    candidate_ref = validation.get("candidate")
    if type(candidate_ref) is not dict or candidate_ref.get("sha256") != expected_candidate_sha256 or candidate_ref.get("calibration_id") != candidate.get("calibration_id"):
        raise PersonDepthFusionError("validation candidate ID/SHA binding mismatch")
    if candidate.get("source_frame") != "ar0234_optical_frame" or candidate.get("target_frame") != "stereo_left_raw_optical_frame" or candidate.get("translation_units") != "m":
        raise PersonDepthFusionError("candidate frame or translation-unit contract mismatch")
    raw, inverse = candidate.get("raw_transform"), candidate.get("rectified_inverse_transform")
    if type(raw) is not dict or raw.get("name") != "T_stereo_left_raw_from_ar0234":
        raise PersonDepthFusionError("candidate raw transform missing")
    if type(inverse) is not dict or inverse.get("name") != "T_ar0234_from_rectified_left":
        raise PersonDepthFusionError("candidate rectified inverse missing")
    raw_from_ar_m, ar_from_rectified_left_m = _matrix_m(raw.get("matrix_4x4"), "raw_from_ar"), _matrix_m(inverse.get("matrix_4x4"), "ar_from_rectified_left")
    try:
        ar = load_ar_intrinsics(ar_intrinsic_path, AR_INTRINSIC_SHA256)
    except ExtrinsicSolveError as error:
        raise PersonDepthFusionError(str(error)) from error
    if _sha256(stereo_calibration_path) != STEREO_CALIBRATION_SHA256:
        raise PersonDepthFusionError("Stereo V6 calibration SHA mismatch")
    policy = _json_object(stereo_policy_path)
    if policy.get("status") != "ENABLED" or policy.get("calibration_id") != "stereo_calibration_v6" or policy.get("calibration_sha256") != STEREO_CALIBRATION_SHA256 or policy.get("hidden_depth_scale_or_offset_correction_allowed") is not False or policy.get("num_disparities") != REQUIRED_NUM_DISPARITIES:
        raise PersonDepthFusionError("Stereo V6 policy contract mismatch")
    depth_range = policy.get("measured_depth_acceptance_range_m")
    if type(depth_range) is not list or len(depth_range) != 2:
        raise PersonDepthFusionError("Stereo V6 policy depth range missing")
    lo, hi = float(depth_range[0]), float(depth_range[1])
    if not all(math.isfinite(v) for v in (lo, hi)) or not 0. < lo <= hi:
        raise PersonDepthFusionError("Stereo V6 policy depth range invalid")
    try:
        with np.load(stereo_calibration_path, allow_pickle=False) as archive:
            p1 = np.asarray(archive["P1"], dtype=np.float64)
    except (KeyError, OSError, ValueError) as error:
        raise PersonDepthFusionError("cannot load Stereo V6 P1") from error
    if p1.shape != (3, 4) or not np.isfinite(p1).all():
        raise PersonDepthFusionError("Stereo V6 P1 is invalid")
    return FusionCalibration(candidate_path, expected_candidate_sha256, validation_path, expected_validation_sha256, str(candidate["calibration_id"]), raw_from_ar_m, ar_from_rectified_left_m, ar.camera_matrix, ar.distortion, p1, stereo_calibration_path, STEREO_CALIBRATION_SHA256, stereo_policy_path, str(policy["policy_id"]), (lo, hi))


class _OfflinePlaybackGuard:
    """Playback adapter: no live temperature state is fabricated or reported as valid."""
    def check_before_measurement(self) -> TemperatureGateResult:
        return TemperatureGateResult(datetime.now(timezone.utc).isoformat(), 0.0, {})


def build_offline_stereo_kernel(calibration: FusionCalibration, *, project_root: Path) -> GuardedStereoDepthProcessor:
    """Use the validated V6 processor/maps/SGBM on immutable PNG playback only."""
    try:
        guard = StereoCalibrationGuard.from_policy(calibration.stereo_policy_path, project_root=project_root)
        activation = guard._validate_activation_record()  # policy/activation linkage, deliberately without live /tmp temperature state
        if file_sha256(guard.calibration_path) != calibration.stereo_calibration_sha256:
            raise PersonDepthFusionError("guarded V6 calibration SHA mismatch")
        with np.load(str(guard.calibration_path), allow_pickle=False) as archive:
            parameters = {key: np.array(archive[key], copy=True) for key in archive.files}
        guarded = GuardedCalibration(str(parameters["calibration_id"].reshape(-1)[0]), calibration.stereo_calibration_sha256, str(guard.policy["activation_record_sha256"]), parameters, _OfflinePlaybackGuard().check_before_measurement())
        if guarded.calibration_id != "stereo_calibration_v6" or activation.get("status") != "ACTIVE_CONDITIONAL":
            raise PersonDepthFusionError("offline guarded calibration/activation mismatch")
        return GuardedStereoDepthProcessor(guard=_OfflinePlaybackGuard(), calibration=guarded, num_disparities=REQUIRED_NUM_DISPARITIES)
    except (KeyError, OSError, ValueError, RuntimeError) as error:
        if isinstance(error, PersonDepthFusionError):
            raise
        raise PersonDepthFusionError(f"cannot initialize offline Stereo V6 kernel: {error}") from error


def inner_seed_roi(roi: BoundingBox) -> BoundingBox:
    return BoundingBox(roi.x_min + math.floor(roi.width * .35), roi.y_min + math.floor(roi.height * .20), roi.x_min + math.ceil(roi.width * .65), roi.y_min + math.ceil(roi.height * .70))


def rectified_xyz(depth_m: np.ndarray, valid_mask: np.ndarray, p1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    depth, mask, p1 = np.asarray(depth_m, dtype=np.float64), np.asarray(valid_mask, dtype=bool), np.asarray(p1, dtype=np.float64)
    if depth.shape != (800, 1280) or mask.shape != depth.shape or p1.shape != (3, 4):
        raise PersonDepthFusionError("invalid rectified depth/mask/P1 shape")
    fx, fy, cx, cy = float(p1[0,0]), float(p1[1,1]), float(p1[0,2]), float(p1[1,2])
    if not all(math.isfinite(v) and v > 0 for v in (fx,fy)) or not all(math.isfinite(v) for v in (cx,cy)):
        raise PersonDepthFusionError("invalid rectified projection")
    v,u=np.indices(depth.shape,dtype=np.float64); xyz=np.stack(((u-cx)*depth/fx,(v-cy)*depth/fy,depth),axis=-1)
    return xyz, mask & np.isfinite(xyz).all(axis=-1) & (xyz[...,2]>0)


def transform_and_project_to_ar(xyz_rect_m: np.ndarray, ar_from_rectified_m: np.ndarray, ar_camera_matrix: np.ndarray, ar_distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz=np.asarray(xyz_rect_m,dtype=np.float64).reshape(-1,3)
    if not len(xyz): return np.empty((0,3)),np.empty((0,2))
    ar=(ar_from_rectified_m[:3,:3]@xyz.T+ar_from_rectified_m[:3,3:4]).T
    pixels,_=cv2.projectPoints(ar,np.zeros((3,1)),np.zeros((3,1)),ar_camera_matrix,ar_distortion)
    return ar,pixels.reshape(-1,2)


def robust_xyz(points_m: np.ndarray) -> tuple[np.ndarray,float,float,np.ndarray]:
    points=np.asarray(points_m,dtype=np.float64).reshape(-1,3)
    if not len(points) or not np.isfinite(points).all() or np.any(points[:,2]<=0): raise PersonDepthFusionError("no finite positive-depth support")
    initial=float(np.median(points[:,2])); mad=float(np.median(np.abs(points[:,2]-initial)))
    if not all(math.isfinite(v) for v in (initial,mad)): raise PersonDepthFusionError("non-finite median/MAD")
    inliers=np.abs(points[:,2]-initial)<=min(max(3*mad,.10),.30)
    return np.median(points[inliers],axis=0),initial,mad,inliers


def _assert_json_finite(value: Any) -> None:
    if type(value) is float and not math.isfinite(value): raise PersonDepthFusionError("JSON payload contains NaN or infinity")
    if type(value) is dict:
        for item in value.values(): _assert_json_finite(item)
    elif type(value) is list:
        for item in value: _assert_json_finite(item)


def _simple(status: PersonMeasurementStatus, detail: str, timestamp: str) -> PersonMeasurement:
    return PersonMeasurement(status, timestamp, detail=detail)


class PersonDepthFusionOffline:
    def __init__(self, person_pipeline: PersonLocalizationPipeline, depth_kernel: StereoDepthKernel, calibration: FusionCalibration, policy: FusionPolicy = FusionPolicy()) -> None:
        self.person_pipeline,self.depth_kernel,self.calibration,self.policy=person_pipeline,depth_kernel,calibration,policy

    def _p1(self) -> np.ndarray:
        return self.calibration.rectified_p1

    def process_with_localization(self, ar_frame_bgr: np.ndarray, combined_stereo_bgr: np.ndarray, *, captured_at_utc: datetime, cycle_id: str) -> tuple[PersonMeasurement, PersonLocalizationResult | None]:
        timestamp=captured_at_utc.astimezone(timezone.utc).isoformat() if isinstance(captured_at_utc,datetime) and captured_at_utc.tzinfo else datetime.now(timezone.utc).isoformat()
        if type(ar_frame_bgr) is not np.ndarray or ar_frame_bgr.dtype!=np.uint8 or ar_frame_bgr.shape!=AR_SHAPE or type(combined_stereo_bgr) is not np.ndarray or combined_stereo_bgr.dtype!=np.uint8 or combined_stereo_bgr.shape!=COMBINED_SHAPE:
            return _simple(PersonMeasurementStatus.INVALID_INPUT,"expected AR0234 1200x1920x3 and combined stereo 800x2560x3 uint8 images",timestamp), None
        localization=self.person_pipeline.process(ar_frame_bgr,captured_at_utc=captured_at_utc,cycle_id=cycle_id)
        return self.fuse_localization(localization, combined_stereo_bgr, captured_at_utc=captured_at_utc, cycle_id=cycle_id), localization

    def process(self, ar_frame_bgr: np.ndarray, combined_stereo_bgr: np.ndarray, *, captured_at_utc: datetime, cycle_id: str) -> PersonMeasurement:
        return self.process_with_localization(ar_frame_bgr, combined_stereo_bgr, captured_at_utc=captured_at_utc, cycle_id=cycle_id)[0]

    def fuse_localization(self, localization: PersonLocalizationResult, combined_stereo_bgr: np.ndarray, *, captured_at_utc: datetime, cycle_id: str, measurement_mode: str = "offline") -> PersonMeasurement:
        timestamp=captured_at_utc.astimezone(timezone.utc).isoformat() if isinstance(captured_at_utc,datetime) and captured_at_utc.tzinfo else datetime.now(timezone.utc).isoformat()
        if localization.status is PersonLocalizationStatus.PERSON_LOST: return _simple(PersonMeasurementStatus.PERSON_LOST,localization.detail or "person not found",timestamp)
        if localization.status is PersonLocalizationStatus.MULTIPLE_PERSONS: return _simple(PersonMeasurementStatus.MULTIPLE_PERSONS,"exactly one person is required; no target was selected",timestamp)
        if localization.status is not PersonLocalizationStatus.SINGLE_PERSON or localization.observation is None or localization.bounding_box is None: return _simple(PersonMeasurementStatus.INVALID_INPUT,f"person localization blocked: {localization.status.value}",timestamp)
        roi,seed=localization.bounding_box,inner_seed_roi(localization.bounding_box)
        try: frame=self.depth_kernel.process(combined_stereo_bgr)
        except Exception as error: return _simple(PersonMeasurementStatus.DEPTH_UNAVAILABLE,f"Stereo V6 offline kernel rejected input: {error}",timestamp)
        if frame.reference_frame!=RECTIFIED_LEFT_FRAME: return _simple(PersonMeasurementStatus.CALIBRATION_INVALID,"Stereo depth frame reference frame mismatch",timestamp)
        try: xyz,valid=rectified_xyz(frame.depth_m,frame.valid_mask,self._p1())
        except PersonDepthFusionError as error: return _simple(PersonMeasurementStatus.DEPTH_UNAVAILABLE,str(error),timestamp)
        disparity=np.asarray(frame.disparity_px,dtype=np.float64)
        if disparity.shape != valid.shape:
            return _simple(PersonMeasurementStatus.DEPTH_UNAVAILABLE,"Stereo V6 disparity shape mismatch",timestamp)
        valid &= np.isfinite(disparity) & (disparity > 0.)
        lo,hi=self.calibration.stereo_depth_range_m; valid &= (xyz[...,2]>=lo)&(xyz[...,2]<=hi)
        if not np.any(valid): return _simple(PersonMeasurementStatus.DEPTH_UNAVAILABLE,"no finite valid depth inside Stereo V6 policy range",timestamp)
        rect=xyz[valid]; ar,pixels=transform_and_project_to_ar(rect,self.calibration.ar_from_rectified_left_m,self.calibration.ar_camera_matrix,self.calibration.ar_distortion)
        inside=np.isfinite(ar).all(axis=1)&(ar[:,2]>0)&np.isfinite(pixels).all(axis=1)&(pixels[:,0]>=seed.x_min)&(pixels[:,0]<seed.x_max)&(pixels[:,1]>=seed.y_min)&(pixels[:,1]<seed.y_max)
        selected=rect[inside]; ratio=float(len(selected)/len(rect))
        common={"timestamp":timestamp,"selected_point_count":int(len(selected)),"support_ratio":ratio,"rgb_roi_xyxy_px":roi.to_xyxy(),"rgb_seed_roi_xyxy_px":seed.to_xyxy()}
        if len(selected)<self.policy.minimum_selected_points: return PersonMeasurement(PersonMeasurementStatus.INSUFFICIENT_SUPPORT,detail="too few depth points project into the documented torso seed ROI",**common)
        try: median,initial,mad,inliers=robust_xyz(selected)
        except PersonDepthFusionError as error: return PersonMeasurement(PersonMeasurementStatus.INSUFFICIENT_SUPPORT,detail=str(error),**common)
        count=int(np.count_nonzero(inliers)); common.update(initial_median_z_m=initial,depth_mad_m=mad,inlier_count=count)
        if count<self.policy.minimum_inliers: return PersonMeasurement(PersonMeasurementStatus.INSUFFICIENT_SUPPORT,detail="too few robust torso-seed inliers",**common)
        if not np.isfinite(median).all() or median[2]<lo or median[2]>hi: return _simple(PersonMeasurementStatus.DEPTH_UNAVAILABLE,"robust depth lies outside the Stereo V6 policy range",timestamp)
        detector=localization.observation.payload.get("detector")
        if type(detector) is not dict: return _simple(PersonMeasurementStatus.INVALID_INPUT,"person observation has no detector provenance",timestamp)
        confidence=float(min(1.,(count/len(selected))*min(1.,len(selected)/self.policy.minimum_selected_points)))
        if measurement_mode not in ("offline", "live"):
            raise PersonDepthFusionError("unknown measurement mode")
        return PersonMeasurement(status=PersonMeasurementStatus.SUCCESS,measurement_id=f"measurement.person_depth.{measurement_mode}.{cycle_id}.{uuid.uuid4()}",person_observation_id=localization.observation.observation_id,person_evidence_id=localization.observation.evidence_ids[0],reference_frame=RECTIFIED_LEFT_FRAME,units="m",x_m=float(median[0]),y_m=float(median[1]),z_m=float(median[2]),range_m=float(np.linalg.norm(median)),confidence=confidence,detector_provenance=detector,calibration={"extrinsic_calibration_id":self.calibration.calibration_id,"candidate_sha256":self.calibration.candidate_sha256,"physical_validation_sha256":self.calibration.validation_sha256,"ar_intrinsic_sha256":AR_INTRINSIC_SHA256,"stereo_v6_calibration_sha256":self.calibration.stereo_calibration_sha256,"offline_temperature_eligibility_evaluated":False,"rgb_seed_semantics":"MVP_TORSO_SEED_FRACTIONAL_ROI_NOT_SEMANTIC_MASK"},stereo_policy_id=self.calibration.stereo_policy_id,**common)


def write_offline_report(path: Path, measurement: PersonMeasurement, calibration: FusionCalibration, *, ar_image_path: Path, combined_image_path: Path) -> None:
    if path.exists() or path.is_symlink(): raise PersonDepthFusionError(f"refusing to overwrite report: {path}")
    payload={"schema_version":"sie.person_depth_offline_report.v1","mode":"offline_static_png_playback","no_live_temperature_eligibility_evaluated":True,"inputs":{"ar0234_png":{"path":str(ar_image_path),"sha256":_sha256(ar_image_path)},"stereo_combined_png":{"path":str(combined_image_path),"sha256":_sha256(combined_image_path)}},"calibration":{"candidate_sha256":calibration.candidate_sha256,"physical_validation_sha256":calibration.validation_sha256,"stereo_policy_id":calibration.stereo_policy_id},"measurement":measurement.to_dict()}
    _assert_json_finite(payload); content=(json.dumps(payload,indent=2,sort_keys=True,allow_nan=False)+"\n").encode(); path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix=f".{path.name}.",dir=path.parent); temporary=Path(name)
    try:
        with os.fdopen(fd,"wb") as stream: stream.write(content); stream.flush(); os.fsync(stream.fileno())
        os.link(temporary,path); directory=os.open(path.parent,os.O_RDONLY|getattr(os,"O_DIRECTORY",0))
        try: os.fsync(directory)
        finally: os.close(directory)
    except FileExistsError as error: raise PersonDepthFusionError(f"refusing to overwrite report: {path}") from error
    finally:
        try: temporary.unlink()
        except FileNotFoundError: pass
