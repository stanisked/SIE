from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from vision_core.person_depth_fusion.offline import (
    AR_SHAPE, COMBINED_SHAPE, FusionCalibration, PersonDepthFusionError,
    PersonDepthFusionOffline, PersonMeasurementStatus, inner_seed_roi,
    load_fusion_calibration, rectified_xyz, robust_xyz,
    transform_and_project_to_ar, write_offline_report,
)
from vision_core.rgb_stereo_extrinsic.solve import ARTIFACT_PATH, VALIDATION_ARTIFACT_PATH
from vision_core.person_localization.detector import (
    ARTIFACT_SCHEMA_VERSION, CONFIDENCE_SEMANTICS, OUTPUT_CONTRACT_VERSION,
    DetectorArtifact,
)
from vision_core.person_localization.models import BoundingBox, PersonDetection
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.stereo.guarded_runtime_v6_v2 import StereoDepthFrame
from vision_core.stereo.stereo_calibration_guard_v6 import TemperatureGateResult


class Detector:
    artifact = DetectorArtifact(ARTIFACT_SCHEMA_VERSION, "test", "1", "test_model", "1", "0" * 64, OUTPUT_CONTRACT_VERSION, "person", CONFIDENCE_SEMANTICS, 0.5, {})
    def __init__(self, detections): self.detections = detections
    def detect(self, frame): return self.detections


class Kernel:
    def __init__(self, depth, valid): self.depth, self.valid = depth, valid
    def process(self, combined):
        return StereoDepthFrame(1, "now", "rectified_left_optical_frame", "stereo_calibration_v6", "x", "y", "profile", TemperatureGateResult("now", 0.0, {}), np.empty((800,1280,3),np.uint8), np.empty((800,1280,3),np.uint8), np.zeros((800,1280),np.int16), np.ones((800,1280),np.float32), self.valid, self.depth)


def calibration() -> FusionCalibration:
    return FusionCalibration(Path("candidate"), "a"*64, Path("validation"), "b"*64, "candidate", np.eye(4), np.eye(4), np.array([[1000.,0.,960.],[0.,1000.,600.],[0.,0.,1.]]), np.zeros((1,5)), np.array([[1000.,0.,640.,0.],[0.,1000.,400.,0.],[0.,0.,1.,0.]]), Path("stereo.npz"), "c"*64, Path("policy.json"), "policy", (.38,2.55))


def pipeline(detections):
    now=datetime.now(timezone.utc)
    return PersonLocalizationPipeline(Detector(detections), now_utc=lambda: now), now


def test_inner_seed_is_documented_fraction() -> None:
    assert inner_seed_roi(BoundingBox(100,200,500,600)).to_xyxy()==[240,280,360,480]


def test_rectified_xyz_and_raw_to_rectified_inverse_projection_direction() -> None:
    depth=np.full((800,1280),2.,np.float32); valid=np.ones_like(depth,bool); p1=np.array([[1000.,0.,640.,0.],[0.,1000.,400.,0.],[0.,0.,1.,0.]])
    xyz,mask=rectified_xyz(depth,valid,p1)
    assert mask[400,640] and np.allclose(xyz[400,640],[0,0,2])
    inverse=np.eye(4); inverse[0,3]=.1
    ar,pixels=transform_and_project_to_ar(np.array([[0.,0.,2.]]),inverse,np.array([[1000.,0.,960.],[0.,1000.,600.],[0.,0.,1.]]),np.zeros((1,5)))
    assert np.allclose(ar,[[.1,0,2]]) and np.allclose(pixels,[[1010,600]])


def test_robust_median_mad_rejects_background_outlier() -> None:
    points=np.vstack((np.tile([.1,.2,1.],(100,1)),np.tile([3.,3.,2.5],(10,1))))
    median,initial,mad,inliers=robust_xyz(points)
    assert np.allclose(median,[.1,.2,1.]) and initial==1. and mad==0. and np.count_nonzero(inliers)==100


def test_background_dominated_support_stays_at_median_not_nearest_point() -> None:
    points=np.vstack((np.tile([0.,0.,2.],(100,1)),np.tile([0.,0.,.7],(10,1))))
    median,initial,mad,inliers=robust_xyz(points)
    assert np.allclose(median,[0,0,2]) and initial==2. and mad==0. and np.count_nonzero(inliers)==100


def test_calibration_sha_and_status_gates_fail_closed(tmp_path: Path) -> None:
    candidate=tmp_path/'candidate.json'; candidate.write_text(ARTIFACT_PATH.read_text())
    validation=tmp_path/'validation.json'; document=json.loads(VALIDATION_ARTIFACT_PATH.read_text()); document['status']='PHYSICAL_EXTRINSIC_VALIDATION_FAIL'; validation.write_text(json.dumps(document))
    candidate_sha=hashlib.sha256(candidate.read_bytes()).hexdigest(); validation_sha=hashlib.sha256(validation.read_bytes()).hexdigest()
    with pytest.raises(PersonDepthFusionError,match='status'):
        load_fusion_calibration(candidate_path=candidate,validation_path=validation,expected_candidate_sha256=candidate_sha,expected_validation_sha256=validation_sha)
    with pytest.raises(PersonDepthFusionError,match='candidate extrinsic SHA mismatch'):
        load_fusion_calibration(candidate_path=candidate,validation_path=validation,expected_candidate_sha256='0'*64,expected_validation_sha256=validation_sha)


@pytest.mark.parametrize("detections,status", [([],PersonMeasurementStatus.PERSON_LOST),([PersonDetection(BoundingBox(500,300,1400,1000),.9),PersonDetection(BoundingBox(10,10,20,20),.9)],PersonMeasurementStatus.MULTIPLE_PERSONS)])
def test_zero_one_two_policy_blocks_without_target_selection(detections,status) -> None:
    person,now=pipeline(detections); fusion=PersonDepthFusionOffline(person,Kernel(np.full((800,1280),1.,np.float32),np.ones((800,1280),bool)),calibration())
    result=fusion.process(np.zeros(AR_SHAPE,np.uint8),np.zeros(COMBINED_SHAPE,np.uint8),captured_at_utc=now,cycle_id="x")
    assert result.status is status


def test_empty_and_invalid_depth_statuses() -> None:
    person,now=pipeline([PersonDetection(BoundingBox(500,300,1400,1000),.9)])
    fusion=PersonDepthFusionOffline(person,Kernel(np.full((800,1280),np.nan,np.float32),np.zeros((800,1280),bool)),calibration())
    assert fusion.process(np.zeros(AR_SHAPE,np.uint8),np.zeros(COMBINED_SHAPE,np.uint8),captured_at_utc=now,cycle_id="x").status is PersonMeasurementStatus.DEPTH_UNAVAILABLE
    assert fusion.process(np.zeros((1,1,3),np.uint8),np.zeros(COMBINED_SHAPE,np.uint8),captured_at_utc=now,cycle_id="x").status is PersonMeasurementStatus.INVALID_INPUT


def test_success_contract_and_report_has_no_pixels_or_nonfinite(tmp_path: Path) -> None:
    person,now=pipeline([PersonDetection(BoundingBox(500,300,1400,1000),.9)])
    fusion=PersonDepthFusionOffline(person,Kernel(np.full((800,1280),1.,np.float32),np.ones((800,1280),bool)),calibration())
    result=fusion.process(np.zeros(AR_SHAPE,np.uint8),np.zeros(COMBINED_SHAPE,np.uint8),captured_at_utc=now,cycle_id="x")
    assert result.status is PersonMeasurementStatus.SUCCESS and result.reference_frame=="rectified_left_optical_frame" and result.units=="m" and result.selected_point_count>=100
    ar=tmp_path/'ar.png'; stereo=tmp_path/'stereo.png'; ar.write_bytes(b'ar'); stereo.write_bytes(b'stereo')
    report=tmp_path/'report.json'; write_offline_report(report,result,calibration(),ar_image_path=ar,combined_image_path=stereo)
    text=report.read_text(); assert 'NaN' not in text and 'Infinity' not in text and 'pixels' not in text
