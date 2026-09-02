from pathlib import Path
import subprocess

import numpy as np
import pytest

import vision_core.rgb_stereo_extrinsic.capture as capture


def test_split_mapping_returns_combined_right_half():
    frame=np.zeros(capture.STEREO_SHAPE,np.uint8); frame[:, :1280]=1; frame[:,1280:]=2
    assert np.all(capture.split_physical_left(frame)==2)


def test_exact_device_rejects_other_path():
    with pytest.raises(capture.ExtrinsicCaptureError): capture._exact_device(Path('/dev/video2'),capture.AR0234_BY_ID)


def test_mode_validation_releases_on_mismatch():
    class Fake:
        def isOpened(self): return True
        def set(self,*_): return True
        def get(self,prop): return 1
        def release(self): self.released=True
    fake=Fake(); camera=capture.CheckedCamera(capture.AR0234_BY_ID,capture.AR_MODE,factory=lambda *_:fake,resolver=lambda *_:Path('/dev/video7'))
    with pytest.raises(capture.ExtrinsicCaptureError): camera.open(capture.AR0234_BY_ID)
    assert fake.released


def test_control_timeout_and_failure():
    def timeout(*_): raise subprocess.TimeoutExpired('v4l2-ctl',5)
    with pytest.raises(capture.ExtrinsicCaptureError): capture.set_control(capture.AR0234_BY_ID,'auto_exposure',3,timeout)
    with pytest.raises(capture.ExtrinsicCaptureError): capture.set_control(capture.AR0234_BY_ID,'auto_exposure',3,lambda *_:subprocess.CompletedProcess([],1,'bad'))


def test_acceptance_needs_all_corners(monkeypatch):
    frame=np.zeros(capture.AR_SHAPE,np.uint8); left=np.zeros(capture.LEFT_SHAPE,np.uint8)
    monkeypatch.setattr(capture,'checkerboard_corners',lambda x:np.zeros((54,1,2),np.float32) if x is frame else None)
    assert capture.pair_acceptable(frame,left) is None


def test_preview_does_not_mutate_raw(monkeypatch):
    frame=np.zeros(capture.AR_SHAPE,np.uint8); original=frame.copy(); corners=np.zeros((54,1,2),np.float32)
    assert capture._preview(frame,corners,'LIVE') is not frame
    assert np.array_equal(frame,original)


def test_paired_preview_is_display_only_and_has_common_height():
    ar=np.zeros(capture.AR_SHAPE,np.uint8); left=np.zeros(capture.LEFT_SHAPE,np.uint8)
    view=capture._paired_preview(ar,None,left,None,'LIVE','LIVE')
    assert view.shape[0] == 600
    assert not ar.any() and not left.any()


def test_pair_filename_is_deterministic():
    assert capture.pair_filename(0)=='pair_000.png' and capture.pair_filename(12)=='pair_012.png'


def test_resume_rejects_missing_recorded_file(tmp_path):
    path=tmp_path/'pair_records.jsonl'; path.write_text('{"pair_id":"pair_000","files":{}}\n')
    with pytest.raises(capture.ExtrinsicCaptureError): capture.next_pair_index(tmp_path)


def test_resume_rejects_digest_mismatch(tmp_path):
    filename='pair_000.png'; payload=b'not a png'
    for section in ('ar0234','stereo_left_raw','stereo_combined'):
        path=tmp_path/section/filename; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(payload)
    files={section:{'filename':filename,'bytes':len(payload),'sha256':'0'*64} for section in ('ar0234','stereo_left_raw','stereo_combined')}
    (tmp_path/'pair_records.jsonl').write_text(__import__('json').dumps({'pair_id':'pair_000','files':files})+'\n')
    with pytest.raises(capture.ExtrinsicCaptureError): capture.next_pair_index(tmp_path)


def test_cameras_release_on_read_error():
    class Fake:
        def __init__(self): self.released=False
        def isOpened(self): return True
        def read(self): return False,None
        def release(self): self.released=True
    fake=Fake(); camera=capture.CheckedCamera(capture.AR0234_BY_ID,capture.AR_MODE); camera._cap=fake
    with pytest.raises(capture.ExtrinsicCaptureError): camera.read(capture.AR_SHAPE)
    assert fake.released


def test_live_defers_detector_until_space_and_review_does_not_repeat(monkeypatch, tmp_path):
    class Camera:
        def __init__(self, shape): self.shape=shape; self.closed=False; self.n=0; self.target=Path('/dev/video9')
        def open(self, _expected): return {'width':self.shape[1], 'height':self.shape[0], 'fps':30.0 if self.shape == capture.AR_SHAPE else 60.0, 'fourcc':'MJPG', 'buffer_size':1}
        def read(self, _shape): self.n+=1; return np.zeros(self.shape,np.uint8)
        def close(self): self.closed=True
    ar, stereo = Camera(capture.AR_SHAPE), Camera(capture.STEREO_SHAPE)
    corners=np.zeros((54,1,2),np.float32); calls=[]
    monkeypatch.setattr(capture, '_sha256', lambda path: capture.AR_INTRINSIC_SHA256 if path == capture.AR_INTRINSIC else capture.STEREO_CALIBRATION_SHA256)
    monkeypatch.setattr(capture, 'set_control', lambda *args, **kwargs: {'requested':1,'actual':1})
    monkeypatch.setattr(capture.cv2, 'namedWindow', lambda *_: None)
    monkeypatch.setattr(capture.cv2, 'resizeWindow', lambda *_: None)
    monkeypatch.setattr(capture.cv2, 'imshow', lambda *_: None)
    monkeypatch.setattr(capture.cv2, 'destroyAllWindows', lambda: None)
    keys=iter([ord(' '), ord('a')]); monkeypatch.setattr(capture, 'checkerboard_corners', lambda _frame: calls.append(1) or corners)
    monkeypatch.setattr(capture, 'save_pair', lambda *args, **kwargs: None)
    monkeypatch.setattr(capture.cv2, 'waitKey', lambda *_: next(keys))
    capture.capture_runtime(root=tmp_path, pair_limit=1, ar_camera=ar, stereo_camera=stereo, key_reader=lambda *_: next(keys))
    assert len(calls) == 2
    assert ar.closed and stereo.closed
