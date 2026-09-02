from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from vision_core.person_depth_fusion.capture_pair import (
    AR_SHAPE, STEREO_SHAPE, PersonDepthPairCaptureError, _axis_preview,
    _principal_points, _show_positioning, capture_pair,
)


class FakeCamera:
    def __init__(self, frame, *, target="/dev/video-test", fail_after=None):
        self.frame, self.target, self.fail_after, self.reads, self.closed = frame, Path(target), fail_after, 0, False
    def open(self, expected): return {"width": self.frame.shape[1], "height": self.frame.shape[0], "fps": 30.0, "fourcc": "MJPG", "buffer_size": 1}
    def read(self, shape):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after: raise RuntimeError("read failure")
        return self.frame
    def close(self): self.closed = True


def controls(argv, timeout):
    import subprocess
    if "--set-ctrl" in argv[-1]: return subprocess.CompletedProcess(argv, 0, "")
    name = argv[-1].split("=")[-1]
    value = 1 if name == "white_balance_automatic" else 3
    return subprocess.CompletedProcess(argv, 0, f"{name}: {value}")


def run(tmp_path, *, countdown=0.0, ar_frame=None, stereo_frame=None, times=None, fail_after=None):
    ar=FakeCamera(ar_frame if ar_frame is not None else np.zeros(AR_SHAPE,np.uint8), fail_after=fail_after)
    st=FakeCamera(stereo_frame if stereo_frame is not None else np.zeros(STEREO_SHAPE,np.uint8), fail_after=fail_after)
    clock=iter(times or [0.0]*1000)
    return capture_pair(output_root=tmp_path, countdown_s=countdown, ar_camera=ar, stereo_camera=st, control_runner=controls, monotonic=lambda: next(clock)), ar, st


def test_success_save_and_json(tmp_path: Path):
    record, ar, st = run(tmp_path)
    assert record["status"] == "PAIR_SAVED" and (tmp_path/"ar0234.png").is_file() and (tmp_path/"stereo_combined.png").is_file()
    assert json.loads((tmp_path/"capture_pair.json").read_text())["receive_skew_s"] == 0.0 and ar.closed and st.closed


@pytest.mark.parametrize("kind", ["ar", "stereo"])
def test_invalid_shape_dtype(tmp_path: Path, kind: str):
    bad=np.zeros((2,2,3),np.uint8) if kind=="ar" else np.zeros(STEREO_SHAPE,np.float32)
    with pytest.raises(PersonDepthPairCaptureError, match="INVALID_FRAME"): run(tmp_path, ar_frame=bad if kind=="ar" else None, stereo_frame=bad if kind=="stereo" else None)


def test_excessive_skew_saves_nothing(tmp_path: Path):
    with pytest.raises(PersonDepthPairCaptureError, match="PAIR_SKEW_TOO_HIGH"):
        run(tmp_path, times=[0.0, 0.0, 0.0, 1.0])
    assert not (tmp_path/"ar0234.png").exists() and not (tmp_path/"stereo_combined.png").exists()


def test_no_overwrite(tmp_path: Path):
    (tmp_path/"ar0234.png").write_bytes(b"existing")
    with pytest.raises(PersonDepthPairCaptureError, match="OUTPUT_NOT_EMPTY"): run(tmp_path)


def test_release_on_error(tmp_path: Path):
    ar=FakeCamera(np.zeros(AR_SHAPE,np.uint8), fail_after=60); st=FakeCamera(np.zeros(STEREO_SHAPE,np.uint8), fail_after=60)
    with pytest.raises(RuntimeError): capture_pair(output_root=tmp_path, countdown_s=0, ar_camera=ar, stereo_camera=st, control_runner=controls)
    assert ar.closed and st.closed


def test_countdown_reads_both_streams(tmp_path: Path):
    clock=iter([0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.6])
    ar=FakeCamera(np.zeros(AR_SHAPE,np.uint8)); st=FakeCamera(np.zeros(STEREO_SHAPE,np.uint8))
    capture_pair(output_root=tmp_path, countdown_s=.5, ar_camera=ar, stereo_camera=st, control_runner=controls, monotonic=lambda: next(clock))
    assert ar.reads > 61 and st.reads > 61


def test_preview_uses_calibration_principal_points_and_preserves_raw(tmp_path, monkeypatch):
    ar_path = tmp_path / "ar.json"
    ar_path.write_text(json.dumps({"schema": "ar-v3", "image": {"width": 1920, "height": 1200},
                                   "camera_matrix": [[1000, 0, 997.25], [0, 1001, 693.5], [0, 0, 1]]}))
    stereo_path = tmp_path / "stereo.npz"
    np.savez(stereo_path, K1=np.array([[500, 0, 652.25], [0, 501, 356.5], [0, 0, 1]], float),
             size=np.array([1280, 800]), calibration_id=np.array("stereo_calibration_v6"))
    ar_cal, st_cal = _principal_points(ar_path, stereo_path)
    assert (ar_cal["cx"], ar_cal["cy"]) == (997.25, 693.5)
    assert (st_cal["cx"], st_cal["cy"]) == (652.25, 356.5)
    raw = np.zeros(AR_SHAPE, np.uint8)
    annotated = _axis_preview(raw, ar_cal, "AR0234")
    assert np.array_equal(raw, np.zeros_like(raw)) and not np.array_equal(raw, annotated)
    shown = []
    monkeypatch.setattr("cv2.imshow", lambda name, image: shown.append((name, image.copy())))
    monkeypatch.setattr("cv2.putText", lambda image, *args, **kwargs: image)
    _show_positioning(raw, np.zeros(STEREO_SHAPE, np.uint8), ar_cal, st_cal, state="LIVE")
    assert shown and shown[0][0] == "Person Depth Pair" and shown[0][1].shape[1] <= 1280


def test_physical_left_is_right_half():
    combined = np.zeros(STEREO_SHAPE, np.uint8)
    combined[:, :1280] = 7
    combined[:, 1280:] = 19
    from vision_core.rgb_stereo_extrinsic.capture import split_physical_left
    left = split_physical_left(combined)
    assert left.shape == (800, 1280, 3) and np.all(left == 19)
