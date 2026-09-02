from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import pytest

from vision_core.person_depth_fusion.live import LiveFusionError, LivePersonDepthFusion, build_live_runtime
from vision_core.person_depth_fusion.offline import AR_SHAPE, COMBINED_SHAPE, FusionCalibration, PersonDepthFusionOffline
from vision_core.person_localization.detector import ARTIFACT_SCHEMA_VERSION, CONFIDENCE_SEMANTICS, OUTPUT_CONTRACT_VERSION, DetectorArtifact
from vision_core.person_localization.models import BoundingBox, PersonDetection
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.stereo.guarded_runtime_v6_v2 import StereoDepthFrame
from vision_core.stereo.stereo_calibration_guard_v6 import TemperatureGateResult


class Detector:
    def __init__(self, outputs, threshold=.5):
        self.artifact = DetectorArtifact(ARTIFACT_SCHEMA_VERSION, "test", "1", "model", "1", "0" * 64, OUTPUT_CONTRACT_VERSION, "person", CONFIDENCE_SEMANTICS, threshold, {"score_threshold": threshold})
        self.outputs, self.calls = outputs, 0
    def detect(self, _):
        value = self.outputs[min(self.calls, len(self.outputs)-1)]
        self.calls += 1
        return value


class Camera:
    def __init__(self, frame, *, fail_at=None): self.frame, self.fail_at, self.reads, self.closed, self.opens = frame, fail_at, 0, False, 0
    def open(self, _): self.opens += 1; return {}
    def read(self, _):
        self.reads += 1
        if self.fail_at is not None and self.reads >= self.fail_at: raise RuntimeError("read failed")
        return self.frame
    def close(self): self.closed = True


class Kernel:
    def process(self, _):
        return StereoDepthFrame(1, "now", "rectified_left_optical_frame", "stereo_calibration_v6", "x", "y", "profile", TemperatureGateResult("now", 0., {}), np.empty((800,1280,3),np.uint8), np.empty((800,1280,3),np.uint8), np.ones((800,1280),np.int16), np.ones((800,1280),np.float32), np.ones((800,1280),bool), np.ones((800,1280),np.float32))


def calibration():
    return FusionCalibration(Path("candidate"), "a"*64, Path("validation"), "b"*64, "candidate", np.eye(4), np.eye(4), np.array([[1000.,0.,960.],[0.,1000.,600.],[0.,0.,1.]]), np.zeros((1,5)), np.array([[1000.,0.,640.,0.],[0.,1000.,400.,0.],[0.,0.,1.,0.]]), Path("stereo"), "c"*64, Path("policy"), "policy", (.38,2.55))


def controls(*_):
    import subprocess
    arg = _[0][-1]; name = arg.split("=")[-1]
    return subprocess.CompletedProcess(_[0], 0, f"{name}: {1 if name == 'white_balance_automatic' else 3}")


def runtime(outputs, *, clock=None, threshold=.5):
    now = datetime(2026, 9, 2, tzinfo=timezone.utc)
    detector = Detector(outputs, threshold)
    fusion = PersonDepthFusionOffline(PersonLocalizationPipeline(detector, now_utc=lambda: now), Kernel(), calibration())
    ar, stereo = Camera(np.zeros(AR_SHAPE,np.uint8)), Camera(np.zeros(COMBINED_SHAPE,np.uint8))
    values = iter(clock or [0.] * 10000)
    value = LivePersonDepthFusion(fusion, ar_camera=ar, stereo_camera=stereo, control_runner=controls, now_utc=lambda: now, monotonic=lambda: next(values))
    return value, ar, stereo, detector


def one(): return [PersonDetection(BoundingBox(500,300,1400,1000),.9)]


def test_success_json_is_pixel_free_and_reuses_constructed_runtime():
    value, ar, stereo, detector = runtime([one(), one()])
    value.start(); first, second = value.cycle("one"), value.cycle("two"); value.close()
    assert first["status"] == "SUCCESS" and second["status"] == "SUCCESS"
    assert detector.calls == 2 and ar.opens == stereo.opens == 1 and ar.reads == stereo.reads == 62
    payload = json.dumps(first, allow_nan=False)
    assert "ndarray" not in payload and "pixels" not in payload and first["temperature_eligibility_evaluated"] is False
    assert first["person_threshold"] == .5 and second["person_threshold"] == .5
    assert ar.closed and stereo.closed


@pytest.mark.parametrize("detections,status", [([], "PERSON_LOST"), ([*one(), *one()], "MULTIPLE_PERSONS")])
def test_zero_one_two_policy_blocks_without_measurement(detections, status):
    value, *_ = runtime([detections]); value.start(); result = value.cycle("x"); value.close()
    assert result["status"] == status and result["measurement"] is None and result["person_threshold"] == .5


def test_excessive_skew_skips_perception():
    value, ar, stereo, detector = runtime([one()], clock=[0., 0., 1., 1.])
    value.start(); result = value.cycle("x"); value.close()
    assert result["status"] == "PAIR_SKEW_TOO_HIGH" and detector.calls == 0


def test_camera_failure_releases_both():
    value, ar, stereo, _ = runtime([one()])
    stereo.fail_at = 61
    value.start()
    with pytest.raises(RuntimeError): value.cycle("x")
    assert ar.closed and stereo.closed


def test_model_and_processor_factories_are_called_once(tmp_path):
    counts = {"detector": 0, "kernel": 0, "camera": 0}
    class FactoryDetector(Detector):
        def __init__(self, *args): counts["detector"] += 1; super().__init__([one()], args[2])
    def kernel(_): counts["kernel"] += 1; return Kernel()
    cameras = [Camera(np.zeros(AR_SHAPE,np.uint8)), Camera(np.zeros(COMBINED_SHAPE,np.uint8))]
    def camera(*_): counts["camera"] += 1; return cameras.pop(0)
    value = build_live_runtime(model=Path("/tmp/model.onnx"), reference=Path("/tmp/ref.py"), project_root=Path("/tmp"), person_threshold=.4, detector_factory=FactoryDetector, calibration_loader=calibration, kernel_factory=kernel, camera_factory=camera)
    assert counts == {"detector": 1, "kernel": 1, "camera": 2}
    assert value.person_threshold == .4
