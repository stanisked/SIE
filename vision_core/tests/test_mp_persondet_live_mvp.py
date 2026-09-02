from __future__ import annotations
from datetime import datetime, timezone
from pathlib import Path
import numpy as np, pytest
from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV, MODEL_SHA256
from vision_core.person_localization.pipeline import PersonLocalizationPipeline
from vision_core.person_localization.models import BoundingBox, PersonDetection, PersonLocalizationStatus
from vision_core.person_localization.mp_persondet import artifact
from vision_core.person_localization.mp_persondet import _decode_landmarks, _full_body_roi
def test_sha_mismatch_blocks_before_loader(tmp_path):
 p=tmp_path/'x.onnx'; p.write_bytes(b'x')
 with pytest.raises(ValueError,match='MODEL_SHA256_MISMATCH'): MPPersonDetOpenCV(p,Path('/missing'))
def test_pipeline_zero_one_many():
 class D:
  def __init__(self,x): self.artifact=artifact(); self.x=x
  def detect(self,_): return self.x
 frame=np.zeros((20,20,3),np.uint8); now=datetime.now(timezone.utc); one=PersonDetection(BoundingBox(1,1,10,10),.9)
 assert PersonLocalizationPipeline(D([])).process(frame,captured_at_utc=now,cycle_id='x').status==PersonLocalizationStatus.PERSON_LOST
 assert PersonLocalizationPipeline(D([one])).process(frame,captured_at_utc=now,cycle_id='x').status==PersonLocalizationStatus.SINGLE_PERSON
 assert PersonLocalizationPipeline(D([one,one])).process(frame,captured_at_utc=now,cycle_id='x').status==PersonLocalizationStatus.MULTIPLE_PERSONS
def test_live_runner_remains_explicit_and_bounded():
 source=Path('vision_core/tools/run_person_localization_ar0234.py').read_text()
 assert "BACKEND_NOT_EXPLICITLY_CONFIGURED" in source and "--max-frames" in source and "capture.close()" in source
 assert "AR0234CaptureConfig(device=AR0234_BY_ID,width=1920,height=1200,fps=30.0,fourcc='MJPG',buffer_size=1)" in source
 assert "for _ in range(60): capture.read()" in source
 assert "cv2.VideoCapture" not in source
def test_virtual_landmarks_and_full_body_roi_not_face_box():
 points=_decode_landmarks(np.array([112,112,224,112,112,0,112,224],np.float32),np.array([0.,0.]),1.,0,0)
 assert points.shape==(4,2) and np.allclose(points[0],[112,112]) and np.allclose(points[1],[224,112])
 roi=_full_body_roi(points,300,300)
 assert roi.to_xyxy()==[0,0,224,224]
def test_full_body_roi_clips_and_rejects_invalid_radius():
 assert _full_body_roi(np.array([[5.,5.],[25.,5.],[0.,0.],[0.,0.]]),20,20).to_xyxy()==[0,0,20,20]
 with pytest.raises(ValueError): _full_body_roi(np.zeros((4,2)),20,20)
 with pytest.raises(ValueError): _full_body_roi(np.full((4,2),np.nan),20,20)
