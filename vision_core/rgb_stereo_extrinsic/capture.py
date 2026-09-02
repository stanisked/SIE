"""Fail-closed paired raw capture for AR0234-to-physical-left extrinsics.

This module stores raw observations only.  It performs neither rectification
nor an extrinsic solve, and it emits no SIE Measurement.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
import stat
import subprocess
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import cv2
import numpy as np

from vision_core.person_localization.ar0234 import (
    VideoCaptureLike,
    check_v4l2_capture_capability,
)

AR0234_BY_ID = Path("/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0")
STEREO_BY_ID = Path("/dev/v4l/by-id/usb-TSTC_Web_Camera_TSTC_Web_Camera-video-index0")
DATASET_ROOT = Path("/home/stanislav/dev_ws/datasets/ar0234_stereo_extrinsic_v1")
AR_INTRINSIC = Path("/home/stanislav/sie_rgb_stereo_fusion/ar0234_intrinsic/dataset_v3_daylight/ar0234_intrinsic_fullres_v3/calibration_fullres.json")
AR_INTRINSIC_SHA256 = "d4515076497f54b6c306e12ae0e9c50ab357480d3a6d61dd77927b3aefe1e381"
STEREO_CALIBRATION = Path("/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/solution_joint_refine_corner_order_filtered_freeze_v2_run07/stereo_params_v6.npz")
STEREO_CALIBRATION_SHA256 = "bb8fb665c6e06e2cbb633cf4c3c61aa74933dd253c9c7950a8420591975dd5e7"
CHECKERBOARD_SIZE = (9, 6)
CHECKERBOARD_CORNERS = 54
SQUARE_SIZE_MM = 24.5
AR_SHAPE = (1200, 1920, 3)
STEREO_SHAPE = (800, 2560, 3)
LEFT_SHAPE = (800, 1280, 3)
WARMUP_FRAMES = 60
V4L2_TIMEOUT_S = 5.0


class ExtrinsicCaptureError(RuntimeError):
    pass


class ControlRunner(Protocol):
    def __call__(self, argv: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]: ...


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _exact_device(device: Path, expected: Path) -> Path:
    if device != expected:
        raise ExtrinsicCaptureError(f"only exact stable by-id device is allowed: {expected}")
    if not device.is_symlink():
        raise ExtrinsicCaptureError(f"stable by-id symlink is absent: {device}")
    target = device.resolve(strict=True)
    if not target.name.startswith("video") or not target.name[5:].isdigit():
        raise ExtrinsicCaptureError(f"by-id device does not resolve to a video node: {target}")
    if not stat.S_ISCHR(target.stat().st_mode):
        raise ExtrinsicCaptureError(f"resolved device is not a character device: {target}")
    try:
        check_v4l2_capture_capability(target)
    except RuntimeError as error:
        raise ExtrinsicCaptureError(str(error)) from error
    return target


def default_control_runner(argv: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout_s, check=False)


def set_control(device: Path, name: str, value: int, runner: ControlRunner = default_control_runner) -> dict[str, int]:
    try:
        result = runner(["v4l2-ctl", "--device", str(device), f"--set-ctrl={name}={value}"], V4L2_TIMEOUT_S)
    except subprocess.TimeoutExpired as error:
        raise ExtrinsicCaptureError(f"v4l2 control timed out: {name}") from error
    if result.returncode != 0:
        raise ExtrinsicCaptureError(f"v4l2 control failed for {name}: {result.stdout}")
    try:
        result = runner(["v4l2-ctl", "--device", str(device), f"--get-ctrl={name}"], V4L2_TIMEOUT_S)
    except subprocess.TimeoutExpired as error:
        raise ExtrinsicCaptureError(f"v4l2 confirmation timed out: {name}") from error
    if result.returncode != 0:
        raise ExtrinsicCaptureError(f"v4l2 confirmation failed for {name}: {result.stdout}")
    rows = [row.strip() for row in result.stdout.splitlines() if row.strip()]
    prefix = f"{name}: "
    if len(rows) != 1 or not rows[0].startswith(prefix):
        raise ExtrinsicCaptureError(f"malformed v4l2 confirmation for {name}")
    token = rows[0][len(prefix):].split(" ", 1)[0]
    try:
        actual = int(token)
    except ValueError as error:
        raise ExtrinsicCaptureError(f"non-integer v4l2 confirmation for {name}") from error
    if actual != value:
        raise ExtrinsicCaptureError(f"v4l2 confirmation mismatch for {name}: {actual}")
    return {"requested": value, "actual": actual}


def _fourcc(value: float) -> str:
    return "".join(chr((int(value) >> (8 * index)) & 0xff) for index in range(4))


@dataclass(frozen=True)
class CaptureMode:
    width: int
    height: int
    fps: float
    fourcc: str = "MJPG"
    buffer_size: int = 1


AR_MODE = CaptureMode(1920, 1200, 30.0)
STEREO_MODE = CaptureMode(2560, 800, 60.0)


class CheckedCamera:
    def __init__(self, device: Path, mode: CaptureMode, *, factory: Callable[[str, int], VideoCaptureLike] = cv2.VideoCapture, resolver: Callable[[Path, Path], Path] = _exact_device) -> None:
        self.device, self.mode, self._factory, self._resolver, self._cap, self.target = device, mode, factory, resolver, None, None

    def open(self, expected: Path) -> dict[str, object]:
        self.target = self._resolver(self.device, expected)
        cap = self._factory(str(self.target), cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise ExtrinsicCaptureError(f"camera did not open: {self.device}")
            for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.mode.fourcc)), (cv2.CAP_PROP_FRAME_WIDTH, self.mode.width), (cv2.CAP_PROP_FRAME_HEIGHT, self.mode.height), (cv2.CAP_PROP_FPS, self.mode.fps), (cv2.CAP_PROP_BUFFERSIZE, self.mode.buffer_size)):
                if not cap.set(prop, value):
                    raise ExtrinsicCaptureError(f"camera rejected mode property: {prop}")
            actual = {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), "fps": float(cap.get(cv2.CAP_PROP_FPS)), "fourcc": _fourcc(cap.get(cv2.CAP_PROP_FOURCC)), "buffer_size": self.mode.buffer_size}
            if (actual["width"], actual["height"], actual["fourcc"]) != (self.mode.width, self.mode.height, self.mode.fourcc) or abs(actual["fps"] - self.mode.fps) > 0.5:
                raise ExtrinsicCaptureError(f"camera mode mismatch: {actual}")
        except BaseException:
            cap.release()
            raise
        self._cap = cap
        return actual

    def read(self, shape: tuple[int, int, int]) -> np.ndarray:
        if self._cap is None:
            raise ExtrinsicCaptureError("camera is not open")
        try:
            ok, frame = self._cap.read()
            if not ok or type(frame) is not np.ndarray or frame.dtype != np.uint8 or frame.shape != shape:
                raise ExtrinsicCaptureError(f"invalid camera frame: expected {shape}/uint8")
            return frame
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        cap, self._cap = self._cap, None
        if cap is not None:
            cap.release()


def split_physical_left(combined: np.ndarray) -> np.ndarray:
    if type(combined) is not np.ndarray or combined.dtype != np.uint8 or combined.shape != STEREO_SHAPE:
        raise ExtrinsicCaptureError("combined frame must be uint8 800x2560x3")
    return combined[:, 1280:]


def checkerboard_corners(frame: np.ndarray) -> np.ndarray | None:
    if type(frame) is not np.ndarray or frame.dtype != np.uint8 or frame.ndim != 3:
        raise ExtrinsicCaptureError("checkerboard frame must be uint8 BGR")
    found, corners = cv2.findChessboardCornersSB(frame, CHECKERBOARD_SIZE, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found or type(corners) is not np.ndarray or corners.shape != (CHECKERBOARD_CORNERS, 1, 2) or not np.isfinite(corners).all():
        return None
    return corners


def pair_acceptable(ar_frame: np.ndarray, stereo_left_raw: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    ar_corners, left_corners = checkerboard_corners(ar_frame), checkerboard_corners(stereo_left_raw)
    if ar_corners is None or left_corners is None:
        return None
    return ar_corners, left_corners


def _png(frame: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise ExtrinsicCaptureError("PNG encoding failed")
    return encoded.tobytes()


def _exclusive(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise ExtrinsicCaptureError(f"refusing overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try: os.fsync(directory)
        finally: os.close(directory)
    except FileExistsError as error:
        raise ExtrinsicCaptureError(f"refusing overwrite: {path}") from error
    finally:
        try: temporary.unlink()
        except FileNotFoundError: pass


def pair_filename(index: int) -> str:
    if type(index) is not int or index < 0:
        raise ExtrinsicCaptureError("pair index must be non-negative integer")
    return f"pair_{index:03d}.png"


def _record_lines(path: Path) -> list[dict[str, Any]]:
    if not path.exists(): return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text().splitlines():
        if not line: continue
        try: row = json.loads(line)
        except json.JSONDecodeError as error: raise ExtrinsicCaptureError("invalid pair records JSONL") from error
        if type(row) is not dict or type(row.get("pair_id")) is not str: raise ExtrinsicCaptureError("invalid pair record")
        rows.append(row)
    return rows


def next_pair_index(root: Path) -> int:
    rows = _record_lines(root / "pair_records.jsonl")
    for index, row in enumerate(rows):
        if row["pair_id"] != f"pair_{index:03d}": raise ExtrinsicCaptureError("pair records are not contiguous")
        for section in ("ar0234", "stereo_left_raw", "stereo_combined"):
            item = row.get("files", {}).get(section, {})
            filename = item.get("filename")
            target = root / section / filename if type(filename) is str else root
            if (filename != pair_filename(index) or target.is_symlink() or not target.is_file()
                    or type(item.get("bytes")) is not int or type(item.get("sha256")) is not str):
                raise ExtrinsicCaptureError("resume record files are invalid")
            payload = target.read_bytes()
            if len(payload) != item["bytes"] or hashlib.sha256(payload).hexdigest() != item["sha256"]:
                raise ExtrinsicCaptureError("resume record file digest mismatch")
    return len(rows)


def _preview(frame: np.ndarray, corners: np.ndarray | None, label: str) -> np.ndarray:
    preview = frame.copy()
    if corners is not None: cv2.drawChessboardCorners(preview, CHECKERBOARD_SIZE, corners, True)
    cv2.putText(preview, label, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
    return preview


def _paired_preview(ar_frame: np.ndarray, ar_corners: np.ndarray | None, left_frame: np.ndarray, left_corners: np.ndarray | None, ar_label: str, left_label: str) -> np.ndarray:
    """Compose display-only copies; capture pixels are never resized or annotated."""
    ar_preview = _preview(ar_frame, ar_corners, ar_label)
    left_preview = _preview(left_frame, left_corners, left_label)
    height = 600
    ar_width = round(ar_preview.shape[1] * height / ar_preview.shape[0])
    left_width = round(left_preview.shape[1] * height / left_preview.shape[0])
    return np.hstack((cv2.resize(ar_preview, (ar_width, height)), cv2.resize(left_preview, (left_width, height))))


def save_pair(root: Path, index: int, ar_frame: np.ndarray, combined: np.ndarray, ar_stamp: float, stereo_stamp: float, metadata: dict[str, Any], corners: tuple[np.ndarray, np.ndarray] | None = None) -> dict[str, Any]:
    left = split_physical_left(combined)
    accepted = corners if corners is not None else pair_acceptable(ar_frame, left)
    if accepted is None: raise ExtrinsicCaptureError("pair requires 54 checkerboard corners in both cameras")
    name = pair_filename(index); blobs = {"ar0234": _png(ar_frame), "stereo_left_raw": _png(left), "stereo_combined": _png(combined)}
    files: dict[str, dict[str, Any]] = {}
    for section, blob in blobs.items():
        path = root / section / name; _exclusive(path, blob)
        files[section] = {"filename": name, "sha256": hashlib.sha256(blob).hexdigest(), "bytes": len(blob)}
    record = {"schema_version":"sie.ar0234_stereo_extrinsic_capture_record.v1", "pair_id":f"pair_{index:03d}", "files":files, "ar0234_host_monotonic_s":ar_stamp, "stereo_host_monotonic_s":stereo_stamp, "receive_skew_s":abs(ar_stamp-stereo_stamp), "checkerboard":{"type":"checkerboard","inner_corners":[9,6],"square_size_mm":SQUARE_SIZE_MM,"units":"mm"}, "corner_counts":{"ar0234":54,"stereo_left_raw":54}, "combined_frame_mapping":{"combined_left_half":"physical_right","combined_right_half":"physical_left","saved_stereo":"physical_left_from_combined_right_half"}, **metadata}
    records = root / "pair_records.jsonl"
    if records.exists():
        current = records.read_bytes()
    else: current = b""
    _exclusive(records, current + json.dumps(record, sort_keys=True, allow_nan=False).encode() + b"\n") if not records.exists() else _append_record(records, record)
    return record


def _append_record(path: Path, record: dict[str, Any]) -> None:
    payload = json.dumps(record, sort_keys=True, allow_nan=False).encode() + b"\n"
    with path.open("ab") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())


def write_manifest(root: Path, count: int, metadata: dict[str, Any]) -> None:
    path = root / "capture_manifest.json"
    if path.exists(): return
    _exclusive(path, (json.dumps({"schema_version":"sie.ar0234_stereo_extrinsic_capture_manifest.v1", "status":"IN_PROGRESS", "accepted_pair_count":count, "pair_limit":12, **metadata}, sort_keys=True, indent=2, allow_nan=False) + "\n").encode())


def capture_runtime(*, root: Path = DATASET_ROOT, pair_limit: int = 12, ar_device: Path = AR0234_BY_ID, stereo_device: Path = STEREO_BY_ID, ar_camera: CheckedCamera | None = None, stereo_camera: CheckedCamera | None = None, control_runner: ControlRunner = default_control_runner, monotonic: Callable[[], float] = time.monotonic, key_reader: Callable[[int], int] = cv2.waitKey) -> None:
    if pair_limit <= 0: raise ExtrinsicCaptureError("pair limit must be positive")
    if _sha256(AR_INTRINSIC) != AR_INTRINSIC_SHA256 or _sha256(STEREO_CALIBRATION) != STEREO_CALIBRATION_SHA256: raise ExtrinsicCaptureError("pinned calibration SHA mismatch")
    index = next_pair_index(root)
    ar = ar_camera or CheckedCamera(ar_device, AR_MODE); stereo = stereo_camera or CheckedCamera(stereo_device, STEREO_MODE)
    try:
        ar_actual = ar.open(AR0234_BY_ID); stereo_actual = stereo.open(STEREO_BY_ID)
        controls = {"ar0234_auto_exposure":set_control(ar_device,"auto_exposure",3,control_runner), "ar0234_white_balance_automatic":set_control(ar_device,"white_balance_automatic",1,control_runner), "stereo_auto_exposure":set_control(stereo_device,"auto_exposure",3,control_runner)}
        metadata = {"devices":{"ar0234_stable_by_id":str(ar_device),"ar0234_resolved":str(ar.target),"stereo_stable_by_id":str(stereo_device),"stereo_resolved":str(stereo.target)}, "requested_modes":{"ar0234":AR_MODE.__dict__,"stereo_combined":STEREO_MODE.__dict__}, "actual_modes":{"ar0234":ar_actual,"stereo_combined":stereo_actual}, "controls":controls, "ar0234_intrinsic":{"path":str(AR_INTRINSIC),"sha256":AR_INTRINSIC_SHA256}, "stereo_v6_calibration":{"path":str(STEREO_CALIBRATION),"sha256":STEREO_CALIBRATION_SHA256}}
        write_manifest(root, index, metadata)
        for _ in range(WARMUP_FRAMES): ar.read(AR_SHAPE); stereo.read(STEREO_SHAPE)
        cv2.namedWindow("AR0234 Stereo Extrinsic Capture", cv2.WINDOW_NORMAL); cv2.resizeWindow("AR0234 Stereo Extrinsic Capture", 1280, 600)
        frozen: tuple[np.ndarray,np.ndarray,float,float,np.ndarray|None,np.ndarray|None] | None = None
        while index < pair_limit:
            if frozen is None:
                ar_frame = ar.read(AR_SHAPE); ar_stamp = monotonic(); combined = stereo.read(STEREO_SHAPE); stereo_stamp = monotonic(); left = split_physical_left(combined)
                view = _paired_preview(ar_frame, None, left, None, "LIVE AR0234: SPACE freeze pair", "LIVE physical LEFT")
            else:
                ar_frame, combined, ar_stamp, stereo_stamp, ar_corners, left_corners = frozen; left = split_physical_left(combined); view = _paired_preview(ar_frame, ar_corners, left, left_corners, "REVIEW: A accept, R retake", "REVIEW physical LEFT")
            cv2.imshow("AR0234 Stereo Extrinsic Capture", view); key = key_reader(20) & 0xff
            if key in (27, ord("q")): break
            if frozen is None and key == ord(" "):
                print(f"CHECKING_CORNERS pair_{index:03d}", flush=True)
                ar_copy, combined_copy = ar_frame.copy(), combined.copy()
                left_copy = split_physical_left(combined_copy)
                ar_corners, left_corners = checkerboard_corners(ar_copy), checkerboard_corners(left_copy)
                ar_count = 0 if ar_corners is None else len(ar_corners)
                left_count = 0 if left_corners is None else len(left_corners)
                if ar_count == CHECKERBOARD_CORNERS and left_count == CHECKERBOARD_CORNERS:
                    print(f"REVIEW_VALID AR={ar_count}/{CHECKERBOARD_CORNERS} STEREO_LEFT={left_count}/{CHECKERBOARD_CORNERS}", flush=True)
                else:
                    print(f"REVIEW_INVALID AR={ar_count} STEREO_LEFT={left_count}", flush=True)
                frozen = (ar_copy, combined_copy, ar_stamp, stereo_stamp, ar_corners, left_corners)
            elif frozen is not None and key == ord("r"): frozen = None
            elif frozen is not None and key == ord("a"):
                if ar_corners is None or left_corners is None:
                    continue
                save_pair(root,index,ar_frame,combined,ar_stamp,stereo_stamp,metadata,(ar_corners,left_corners)); index += 1; frozen = None
    finally:
        ar.close(); stereo.close(); cv2.destroyAllWindows()
