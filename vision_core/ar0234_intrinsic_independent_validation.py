"""Independent AR0234 intrinsic V3 evidence capture and offline validation.

This module is intentionally AR0234-only.  It captures immutable checkerboard
observations and evaluates an existing intrinsic candidate; it never recalibrates
the camera, activates a calibration, or emits an SIE Measurement.
"""
from __future__ import annotations

import ctypes
import errno
import hashlib
import json
import math
import mmap
import os
import re
import select
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


AR_DEVICE = Path("/dev/video4")
AR_WIDTH = 1920
AR_HEIGHT = 1200
AR_FPS = 30
AR_SHAPE = (AR_HEIGHT, AR_WIDTH, 3)
AR_REFERENCE_FRAME = "ar0234_rgb_optical_frame"
CHECKERBOARD_SIZE = (9, 6)
CHECKERBOARD_CORNER_COUNT = 54
SQUARE_SIZE_MM = 24.5
CLOCK_DOMAIN = "CLOCK_MONOTONIC"
CAPTURE_BACKEND = "DIRECT_V4L2_MMAP_DQBUF"
DATASET_SCHEMA_VERSION = "sie.ar0234_intrinsic_independent_validation_dataset.v1"
SESSION_SCHEMA_VERSION = "sie.ar0234_intrinsic_independent_validation_session.v1"
FRAME_SCHEMA_VERSION = "sie.ar0234_intrinsic_independent_validation_frame.v1"
REPORT_SCHEMA_VERSION = "sie.ar0234_intrinsic_independent_validation_report.v1"
_SESSION_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x2000


class IntrinsicValidationError(RuntimeError):
    """Raised for an invalid independent-intrinsic evidence operation."""


def _ioc(direction: int, ioctl_type: str, number: int, size: int) -> int:
    return (direction << 30) | (ord(ioctl_type) << 8) | (number << 0) | (size << 16)


def _iorw(ioctl_type: str, number: int, structure: type[ctypes.Structure]) -> int:
    return _ioc(3, ioctl_type, number, ctypes.sizeof(structure))


def _ior(ioctl_type: str, number: int, structure: type[ctypes.Structure]) -> int:
    return _ioc(2, ioctl_type, number, ctypes.sizeof(structure))


def _iow(ioctl_type: str, number: int, scalar: type[ctypes._SimpleCData]) -> int:
    return _ioc(1, ioctl_type, number, ctypes.sizeof(scalar))


class _Capability(ctypes.Structure):
    _fields_ = [
        ("driver", ctypes.c_uint8 * 16), ("card", ctypes.c_uint8 * 32),
        ("bus_info", ctypes.c_uint8 * 32), ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32), ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class _PixFormat(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
        ("pixelformat", ctypes.c_uint32), ("field", ctypes.c_uint32),
        ("bytesperline", ctypes.c_uint32), ("sizeimage", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32), ("priv", ctypes.c_uint32),
        ("flags", ctypes.c_uint32), ("ycbcr_enc", ctypes.c_uint32),
        ("quantization", ctypes.c_uint32), ("xfer_func", ctypes.c_uint32),
    ]


class _FormatUnion(ctypes.Union):
    _fields_ = [("pix", _PixFormat), ("raw_data", ctypes.c_uint8 * 200), ("alignment", ctypes.c_uint64)]


class _Format(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("fmt", _FormatUnion)]


class _Fract(ctypes.Structure):
    _fields_ = [("numerator", ctypes.c_uint32), ("denominator", ctypes.c_uint32)]


class _CaptureParm(ctypes.Structure):
    _fields_ = [("capability", ctypes.c_uint32), ("capturemode", ctypes.c_uint32), ("timeperframe", _Fract), ("extendedmode", ctypes.c_uint32), ("readbuffers", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 4)]


class _StreamParmUnion(ctypes.Union):
    _fields_ = [("capture", _CaptureParm), ("raw_data", ctypes.c_uint8 * 200)]


class _StreamParm(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("parm", _StreamParmUnion)]


class _RequestBuffers(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32), ("memory", ctypes.c_uint32), ("capabilities", ctypes.c_uint32), ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("flags", ctypes.c_uint32), ("frames", ctypes.c_uint8), ("seconds", ctypes.c_uint8), ("minutes", ctypes.c_uint8), ("hours", ctypes.c_uint8), ("userbits", ctypes.c_uint8 * 4)]


class _BufferMemory(ctypes.Union):
    _fields_ = [("offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong), ("fd", ctypes.c_int32)]


class _BufferRequest(ctypes.Union):
    _fields_ = [("request_fd", ctypes.c_int32), ("reserved", ctypes.c_uint32)]


class _Buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32), ("type", ctypes.c_uint32), ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32), ("field", ctypes.c_uint32), ("timestamp", _Timeval),
        ("timecode", _Timecode), ("sequence", ctypes.c_uint32), ("memory", ctypes.c_uint32),
        ("m", _BufferMemory), ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
        ("request", _BufferRequest),
    ]


VIDIOC_QUERYCAP = _ior("V", 0, _Capability)
VIDIOC_S_FMT = _iorw("V", 5, _Format)
VIDIOC_REQBUFS = _iorw("V", 8, _RequestBuffers)
VIDIOC_QUERYBUF = _iorw("V", 9, _Buffer)
VIDIOC_QBUF = _iorw("V", 15, _Buffer)
VIDIOC_DQBUF = _iorw("V", 17, _Buffer)
VIDIOC_STREAMON = _iow("V", 18, ctypes.c_int)
VIDIOC_STREAMOFF = _iow("V", 19, ctypes.c_int)
VIDIOC_S_PARM = _iorw("V", 22, _StreamParm)
_LIBC = ctypes.CDLL(None, use_errno=True)
_LIBC.ioctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_void_p]
_LIBC.ioctl.restype = ctypes.c_int


@dataclass(frozen=True)
class ArIntrinsics:
    matrix: np.ndarray
    distortion: np.ndarray
    sha256: str


@dataclass(frozen=True)
class KernelFrame:
    image: np.ndarray
    timestamp_ns: int
    sequence: int


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_safe(value: object) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
    except (TypeError, ValueError) as error:
        raise IntrinsicValidationError("record must be JSON-safe and finite") from error


def _json_payload(value: object) -> bytes:
    return (json.dumps(_json_safe(value), ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2) + "\n").encode("utf-8")


def _require_persistent_root(path: Path) -> None:
    resolved = path.expanduser().resolve()
    if resolved == Path("/tmp") or Path("/tmp") in resolved.parents:
        raise IntrinsicValidationError("independent intrinsic evidence must not be stored in /tmp")


def _validate_session_id(session_id: str) -> None:
    if not isinstance(session_id, str) or _SESSION_ID_PATTERN.fullmatch(session_id) is None:
        raise IntrinsicValidationError("session_id must be 1..64 ASCII letters, digits, '.', '_' or '-'")


def _fourcc_mjpg() -> int:
    return sum(ord(char) << (8 * index) for index, char in enumerate("MJPG"))


class DirectAr0234Camera:
    """Direct AR0234 MMAP/DQBUF capture with required monotonic timestamps."""

    def __init__(self, device: Path = AR_DEVICE) -> None:
        self.device = device
        self.fd: int | None = None
        self.buffers: list[mmap.mmap] = []
        self.streamed = False

    def _ioctl(self, request: int, value: ctypes.Structure | ctypes._SimpleCData) -> None:
        if self.fd is None:
            raise IntrinsicValidationError("V4L2 device is not open")
        while True:
            result = _LIBC.ioctl(self.fd, request, ctypes.byref(value))
            if result >= 0:
                return
            error_number = ctypes.get_errno()
            if error_number != errno.EINTR:
                raise IntrinsicValidationError(f"V4L2 ioctl failed for {self.device}: {os.strerror(error_number)}")

    def open(self) -> dict[str, Any]:
        try:
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
            capability = _Capability(); self._ioctl(VIDIOC_QUERYCAP, capability)
            caps = capability.device_caps if capability.capabilities & V4L2_CAP_DEVICE_CAPS else capability.capabilities
            if caps & V4L2_CAP_VIDEO_CAPTURE == 0 or caps & V4L2_CAP_STREAMING == 0:
                raise IntrinsicValidationError(f"device lacks V4L2 capture/streaming: {self.device}")
            fmt = _Format(); fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            fmt.fmt.pix.width, fmt.fmt.pix.height = AR_WIDTH, AR_HEIGHT
            fmt.fmt.pix.pixelformat, fmt.fmt.pix.field = _fourcc_mjpg(), V4L2_FIELD_NONE
            self._ioctl(VIDIOC_S_FMT, fmt)
            if (fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat) != (AR_WIDTH, AR_HEIGHT, _fourcc_mjpg()):
                raise IntrinsicValidationError("AR0234 V4L2 mode mismatch; required MJPG 1920x1200")
            parm = _StreamParm(); parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.timeperframe = _Fract(1, AR_FPS); self._ioctl(VIDIOC_S_PARM, parm)
            fraction = parm.parm.capture.timeperframe
            if fraction.numerator == 0 or fraction.denominator == 0:
                raise IntrinsicValidationError("AR0234 returned invalid FPS fraction")
            actual_fps = fraction.denominator / fraction.numerator
            if abs(actual_fps - AR_FPS) > 0.5:
                raise IntrinsicValidationError(f"AR0234 FPS mismatch: {actual_fps}")
            request = _RequestBuffers(); request.count, request.type, request.memory = 3, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
            self._ioctl(VIDIOC_REQBUFS, request)
            if request.count < 2:
                raise IntrinsicValidationError("AR0234 supplied fewer than two MMAP buffers")
            for index in range(request.count):
                buffer = _Buffer(); buffer.index, buffer.type, buffer.memory = index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
                self._ioctl(VIDIOC_QUERYBUF, buffer)
                self.buffers.append(mmap.mmap(self.fd, buffer.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buffer.m.offset))
                self._ioctl(VIDIOC_QBUF, buffer)
            self._ioctl(VIDIOC_STREAMON, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)); self.streamed = True
            return {"device": str(self.device), "pixel_format": "MJPG", "width": AR_WIDTH, "height": AR_HEIGHT, "fps": actual_fps, "buffer_count": len(self.buffers)}
        except BaseException:
            self.close()
            raise

    def read(self, timeout_s: float = 2.0) -> KernelFrame:
        if self.fd is None or not self.streamed:
            raise IntrinsicValidationError("AR0234 V4L2 stream is not active")
        ready, _, _ = select.select([self.fd], [], [], timeout_s)
        if not ready:
            raise IntrinsicValidationError(f"AR0234 frame timeout: {self.device}")
        buffer = _Buffer(); buffer.type, buffer.memory = V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
        self._ioctl(VIDIOC_DQBUF, buffer)
        try:
            if buffer.index >= len(self.buffers) or buffer.bytesused == 0 or buffer.bytesused > buffer.length:
                raise IntrinsicValidationError("invalid AR0234 V4L2 buffer payload")
            if not buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
                raise IntrinsicValidationError("AR0234 kernel timestamp is not CLOCK_MONOTONIC")
            frame = cv2.imdecode(np.frombuffer(self.buffers[buffer.index][:buffer.bytesused], dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None or frame.dtype != np.uint8 or frame.shape != AR_SHAPE:
                raise IntrinsicValidationError(f"decoded AR0234 frame mismatch; expected {AR_SHAPE}")
            timestamp_ns = int(buffer.timestamp.tv_sec) * 1_000_000_000 + int(buffer.timestamp.tv_usec) * 1_000
            return KernelFrame(frame, timestamp_ns, int(buffer.sequence))
        finally:
            self._ioctl(VIDIOC_QBUF, buffer)

    def close(self) -> None:
        if self.fd is None:
            return
        if self.streamed:
            try:
                self._ioctl(VIDIOC_STREAMOFF, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
            except IntrinsicValidationError:
                pass
        self.streamed = False
        for buffer in self.buffers:
            buffer.close()
        self.buffers.clear()
        os.close(self.fd)
        self.fd = None


def _load_intrinsics(path: Path) -> ArIntrinsics:
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise IntrinsicValidationError("cannot read AR0234 intrinsic JSON") from error
    image = record.get("image", {})
    board = record.get("checkerboard", {})
    matrix = np.asarray(record.get("camera_matrix"), dtype=np.float64)
    distortion = np.asarray(record.get("distortion_coefficients"), dtype=np.float64).reshape(1, -1)
    if (
        record.get("camera") != "AR0234"
        or image.get("width") != AR_WIDTH
        or image.get("height") != AR_HEIGHT
        or board.get("inner_corners") != list(CHECKERBOARD_SIZE)
        or board.get("square_size_mm") != SQUARE_SIZE_MM
        or matrix.shape != (3, 3)
        or distortion.shape[1] < 4
        or not np.isfinite(matrix).all()
        or not np.isfinite(distortion).all()
    ):
        raise IntrinsicValidationError("AR0234 intrinsic is incompatible with 1920x1200 9x6 24.5mm evidence")
    return ArIntrinsics(matrix=matrix, distortion=distortion, sha256=_sha256(path))


def _checkerboard(frame: np.ndarray) -> np.ndarray | None:
    found, corners = cv2.findChessboardCornersSB(
        frame, CHECKERBOARD_SIZE,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
    )
    if not found or corners is None or corners.shape != (CHECKERBOARD_CORNER_COUNT, 1, 2) or not np.isfinite(corners).all():
        return None
    return corners.astype(np.float64)


def _object_points_mm() -> np.ndarray:
    return np.asarray([[column * SQUARE_SIZE_MM, row * SQUARE_SIZE_MM, 0.0] for row in range(CHECKERBOARD_SIZE[1]) for column in range(CHECKERBOARD_SIZE[0])], dtype=np.float64)


def _pose_reprojection(corners: np.ndarray, intrinsics: ArIntrinsics) -> dict[str, Any]:
    ok, rvec, tvec = cv2.solvePnP(_object_points_mm(), corners, intrinsics.matrix, intrinsics.distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise IntrinsicValidationError("solvePnP failed")
    projected, _ = cv2.projectPoints(_object_points_mm(), rvec, tvec, intrinsics.matrix, intrinsics.distortion)
    errors = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    if not np.isfinite(errors).all():
        raise IntrinsicValidationError("PnP reprojection is non-finite")
    return _metrics(errors)


def _metrics(values: np.ndarray | list[float]) -> dict[str, Any]:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if not array.size:
        return {"count": 0, "mean_px": None, "median_px": None, "p95_px": None, "max_px": None, "rms_px": None}
    if not np.isfinite(array).all():
        raise IntrinsicValidationError("metrics require finite values")
    return _json_safe({"count": int(array.size), "mean_px": float(np.mean(array)), "median_px": float(np.median(array)), "p95_px": float(np.percentile(array, 95)), "max_px": float(np.max(array)), "rms_px": float(math.sqrt(np.mean(array ** 2)))})


def _write_new(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise IntrinsicValidationError(f"refusing to overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.link(temporary, path)
    except FileExistsError as error:
        raise IntrinsicValidationError(f"refusing to overwrite: {path}") from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _write_json_replace(path: Path, record: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(_json_payload(record)); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _write_png_new(path: Path, image: np.ndarray) -> dict[str, Any]:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise IntrinsicValidationError(f"cannot encode PNG: {path}")
    _write_new(path, encoded.tobytes())
    return {"filename": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _evidence_file_path(root: Path, filename: object) -> Path:
    if not isinstance(filename, str):
        raise IntrinsicValidationError("raw evidence filename is invalid")
    relative = Path(filename)
    if relative.is_absolute() or ".." in relative.parts:
        raise IntrinsicValidationError("raw evidence filename must remain inside dataset root")
    return root / relative


def _capture_configuration(intrinsic: Path, opened: dict[str, Any]) -> dict[str, Any]:
    return _json_safe({
        "capture_backend": CAPTURE_BACKEND,
        "timestamp_clock_domain": CLOCK_DOMAIN,
        "device": str(AR_DEVICE),
        "capture_configuration": {"pixel_format": "MJPG", "width": AR_WIDTH, "height": AR_HEIGHT, "fps": AR_FPS, "actual": opened},
        "intrinsic": {"path": str(intrinsic), "sha256": _sha256(intrinsic), "status": "CANDIDATE_NOT_ACTIVATED"},
        "checkerboard": {"inner_corners": list(CHECKERBOARD_SIZE), "square_size_mm": SQUARE_SIZE_MM},
        "reference_frame": AR_REFERENCE_FRAME,
    })


def _configuration_identity(configuration: dict[str, Any]) -> dict[str, Any]:
    try:
        return _json_safe({
            "capture_backend": configuration["capture_backend"],
            "timestamp_clock_domain": configuration["timestamp_clock_domain"],
            "device": configuration["device"],
            "capture_configuration": {key: configuration["capture_configuration"][key] for key in ("pixel_format", "width", "height", "fps")},
            "intrinsic_sha256": configuration["intrinsic"]["sha256"],
            "checkerboard": configuration["checkerboard"],
            "reference_frame": configuration["reference_frame"],
        })
    except (KeyError, TypeError) as error:
        raise IntrinsicValidationError("capture configuration is incomplete") from error


def _new_dataset(configuration: dict[str, Any]) -> dict[str, Any]:
    return _json_safe({
        "schema_version": DATASET_SCHEMA_VERSION,
        "status": "INDEPENDENT_INTRINSIC_EVIDENCE_CAPTURED",
        "static_target_affirmed": True,
        "configuration": configuration,
        "sessions": [],
        "frames": [],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "updated_utc": datetime.now(timezone.utc).isoformat(),
        "limitation": "Independent evidence only. This dataset neither recalibrates nor activates the AR0234 intrinsic.",
    })


def _validate_dataset(dataset: dict[str, Any]) -> None:
    if (
        dataset.get("schema_version") != DATASET_SCHEMA_VERSION
        or dataset.get("status") != "INDEPENDENT_INTRINSIC_EVIDENCE_CAPTURED"
        or dataset.get("static_target_affirmed") is not True
        or not isinstance(dataset.get("configuration"), dict)
        or not isinstance(dataset.get("sessions"), list)
        or not isinstance(dataset.get("frames"), list)
    ):
        raise IntrinsicValidationError("capture manifest is not an independent AR0234 intrinsic dataset")


def _load_dataset(output_root: Path) -> dict[str, Any] | None:
    manifest_path = output_root / "capture_manifest.json"
    if not manifest_path.exists():
        if (output_root / "sessions").exists():
            raise IntrinsicValidationError("dataset root contains session evidence but capture_manifest.json is absent; refusing reconstruction")
        return None
    try:
        dataset = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise IntrinsicValidationError("cannot read existing capture manifest") from error
    _validate_dataset(dataset)
    _validate_dataset_session_evidence(output_root, dataset)
    return _json_safe(dataset)


def _validate_dataset_session_evidence(output_root: Path, dataset: dict[str, Any]) -> None:
    sessions_root = output_root / "sessions"
    if not sessions_root.is_dir():
        raise IntrinsicValidationError("dataset manifest lacks sessions directory")
    declared_ids: set[str] = set()
    expected_frames: list[dict[str, Any]] = []
    for entry in dataset["sessions"]:
        if not isinstance(entry, dict):
            raise IntrinsicValidationError("dataset session entry is invalid")
        session_id = entry.get("session_id")
        _validate_session_id(session_id)
        if session_id in declared_ids:
            raise IntrinsicValidationError("dataset contains duplicate session_id")
        declared_ids.add(session_id)
        reference = entry.get("session_manifest")
        expected_name = f"sessions/{session_id}/session_manifest.json"
        if not isinstance(reference, dict) or reference.get("filename") != expected_name:
            raise IntrinsicValidationError("dataset session manifest reference is incomplete")
        manifest_path = output_root / expected_name
        if not manifest_path.is_file() or _sha256(manifest_path) != reference.get("sha256"):
            raise IntrinsicValidationError("immutable session manifest is absent or checksum differs")
        try:
            session = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise IntrinsicValidationError("immutable session manifest is invalid JSON") from error
        if (
            session.get("schema_version") != SESSION_SCHEMA_VERSION
            or session.get("status") != "INDEPENDENT_INTRINSIC_SESSION_CAPTURED"
            or session.get("session_id") != session_id
            or session.get("static_target_affirmed") is not True
            or not isinstance(session.get("frames"), list)
            or session.get("frame_ids") != [frame.get("frame_id") for frame in session["frames"]]
            or any(frame.get("session_id") != session_id for frame in session["frames"])
            or _configuration_identity(session.get("configuration")) != _configuration_identity(dataset["configuration"])
        ):
            raise IntrinsicValidationError("immutable session manifest is incomplete or inconsistent")
        for frame in session["frames"]:
            file_record = frame.get("image", {}).get("file", {}) if isinstance(frame.get("image"), dict) else {}
            filename, expected_sha = file_record.get("filename"), file_record.get("sha256")
            expected_prefix = f"sessions/{session_id}/frames/"
            if not isinstance(filename, str) or not filename.startswith(expected_prefix) or not isinstance(expected_sha, str):
                raise IntrinsicValidationError("session raw frame provenance is invalid")
            path = _evidence_file_path(output_root, filename)
            if not path.is_file() or _sha256(path) != expected_sha:
                raise IntrinsicValidationError("session raw frame is absent or checksum differs")
        if (
            entry.get("schema_version") != SESSION_SCHEMA_VERSION
            or entry.get("status") != session["status"]
            or entry.get("operator_pose_label") != session.get("operator_pose_label")
            or entry.get("frame_ids") != session["frame_ids"]
            or _configuration_identity(entry.get("configuration")) != _configuration_identity(session["configuration"])
        ):
            raise IntrinsicValidationError("dataset session entry differs from immutable session manifest")
        expected_frames.extend(session["frames"])
    actual_ids = {path.name for path in sessions_root.iterdir() if path.is_dir()}
    if actual_ids != declared_ids or dataset["frames"] != expected_frames:
        raise IntrinsicValidationError("dataset manifest and immutable session evidence are inconsistent")


def _frame_record(*, session_id: str, frame_id: str, frame: KernelFrame, output_root: Path) -> dict[str, Any]:
    path = output_root / "sessions" / session_id / "frames" / f"{frame_id}.png"
    file_record = _write_png_new(path, frame.image)
    file_record["filename"] = str(path.relative_to(output_root))
    return _json_safe({
        "schema_version": FRAME_SCHEMA_VERSION,
        "frame_id": frame_id,
        "session_id": session_id,
        "timestamp_clock_domain": CLOCK_DOMAIN,
        "timestamp_ns": frame.timestamp_ns,
        "sequence": frame.sequence,
        "image": {"width": AR_WIDTH, "height": AR_HEIGHT, "pixel_format": "BGR8", "file": file_record},
        "checkerboard": {"inner_corners": list(CHECKERBOARD_SIZE), "corner_count": CHECKERBOARD_CORNER_COUNT, "square_size_mm": SQUARE_SIZE_MM},
        "reference_frame": AR_REFERENCE_FRAME,
    })


def _session_manifest(session_id: str, pose_label: str, configuration: dict[str, Any], frames: list[dict[str, Any]]) -> dict[str, Any]:
    if not isinstance(pose_label, str) or not pose_label.strip() or len(pose_label) > 160:
        raise IntrinsicValidationError("operator_pose_label must be a non-empty string up to 160 characters")
    return _json_safe({
        "schema_version": SESSION_SCHEMA_VERSION,
        "status": "INDEPENDENT_INTRINSIC_SESSION_CAPTURED",
        "session_id": session_id,
        "operator_pose_label": pose_label,
        "static_target_affirmed": True,
        "configuration": configuration,
        "frame_count": len(frames),
        "frame_ids": [frame["frame_id"] for frame in frames],
        "frames": frames,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "limitation": "Immutable independent validation evidence. It is neither recalibration nor activation evidence by itself.",
    })


def _append_session(dataset: dict[str, Any], *, configuration: dict[str, Any], session: dict[str, Any], session_manifest_file: dict[str, Any]) -> dict[str, Any]:
    _validate_dataset(dataset)
    if _configuration_identity(dataset["configuration"]) != _configuration_identity(configuration):
        raise IntrinsicValidationError("capture configuration or intrinsic hash differs from existing dataset")
    session_id = session.get("session_id")
    _validate_session_id(session_id)
    if any(row.get("session_id") == session_id for row in dataset["sessions"]):
        raise IntrinsicValidationError(f"duplicate session_id: {session_id}")
    existing_ids = {row.get("frame_id") for row in dataset["frames"]}
    frames = session.get("frames")
    frame_ids = [row.get("frame_id") for row in frames] if isinstance(frames, list) else []
    if not frame_ids or len(set(frame_ids)) != len(frame_ids) or any(not isinstance(value, str) or value in existing_ids for value in frame_ids):
        raise IntrinsicValidationError("session frame IDs are missing or duplicated")
    if any(row.get("session_id") != session_id for row in frames):
        raise IntrinsicValidationError("session frame provenance is inconsistent")
    updated = _json_safe(dataset)
    updated["sessions"].append(_json_safe({
        "schema_version": SESSION_SCHEMA_VERSION,
        "session_id": session_id,
        "status": session["status"],
        "operator_pose_label": session["operator_pose_label"],
        "configuration": session["configuration"],
        "frame_ids": session["frame_ids"],
        "session_manifest": session_manifest_file,
        "created_utc": session["created_utc"],
    }))
    updated["frames"].extend(_json_safe(frames))
    updated["updated_utc"] = datetime.now(timezone.utc).isoformat()
    return _json_safe(updated)


def _persist_session(*, output_root: Path, session_id: str, pose_label: str, configuration: dict[str, Any], frames: list[dict[str, Any]], existing: dict[str, Any] | None) -> dict[str, Any]:
    session_root = output_root / "sessions" / session_id
    if not session_root.is_dir() or (session_root / "session_manifest.json").exists():
        raise IntrinsicValidationError(f"session output is not a new immutable session directory: {session_root}")
    session = _session_manifest(session_id, pose_label, configuration, frames)
    manifest_path = session_root / "session_manifest.json"
    _write_new(manifest_path, _json_payload(session))
    file_record = {"filename": str(manifest_path.relative_to(output_root)), "sha256": _sha256(manifest_path), "bytes": manifest_path.stat().st_size}
    dataset = existing if existing is not None else _new_dataset(configuration)
    updated = _append_session(dataset, configuration=configuration, session=session, session_manifest_file=file_record)
    _write_json_replace(output_root / "capture_manifest.json", updated)
    return updated


def capture_independent_evidence(*, output_root: Path, session_id: str, intrinsic: Path, operator_pose_label: str, static_target_affirmed: bool, frame_count: int = 12) -> dict[str, Any]:
    if not static_target_affirmed:
        raise IntrinsicValidationError("capture requires --static-target-affirmed")
    if frame_count < 1:
        raise IntrinsicValidationError("frame_count must be positive")
    _require_persistent_root(output_root)
    _validate_session_id(session_id)
    _load_intrinsics(intrinsic)
    if output_root.exists() and not output_root.is_dir():
        raise IntrinsicValidationError(f"output root is not a directory: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    existing = _load_dataset(output_root)
    if existing is not None and any(row.get("session_id") == session_id for row in existing["sessions"]):
        raise IntrinsicValidationError(f"duplicate session_id: {session_id}")
    if (output_root / "sessions" / session_id).exists():
        raise IntrinsicValidationError(f"session output already exists: {output_root / 'sessions' / session_id}")
    camera = DirectAr0234Camera(AR_DEVICE)
    accepted: list[KernelFrame] = []
    try:
        opened = camera.open()
        configuration = _capture_configuration(intrinsic, opened)
        if existing is not None and _configuration_identity(existing["configuration"]) != _configuration_identity(configuration):
            raise IntrinsicValidationError("capture configuration or intrinsic hash differs from existing dataset")
        for _ in range(30):
            camera.read()
        for _ in range(frame_count * 4):
            candidate = camera.read()
            if _checkerboard(candidate.image) is not None:
                accepted.append(candidate)
                if len(accepted) == frame_count:
                    break
    finally:
        camera.close()
    if len(accepted) < frame_count:
        raise IntrinsicValidationError(f"insufficient checkerboard frames: {len(accepted)}/{frame_count}")
    session_root = output_root / "sessions" / session_id
    session_root.mkdir(parents=True, exist_ok=False)
    records = [_frame_record(session_id=session_id, frame_id=f"{session_id}__frame_{index:03d}", frame=frame, output_root=output_root) for index, frame in enumerate(accepted)]
    return _persist_session(output_root=output_root, session_id=session_id, pose_label=operator_pose_label, configuration=configuration, frames=records, existing=existing)


def _read_dataset_manifest(path: Path) -> dict[str, Any]:
    try:
        dataset = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise IntrinsicValidationError("cannot read independent validation dataset manifest") from error
    _validate_dataset(dataset)
    return _json_safe(dataset)


def _frame_diagnostic(*, root: Path, frame: dict[str, Any], intrinsics: ArIntrinsics) -> dict[str, Any]:
    result: dict[str, Any] = {"frame_id": frame.get("frame_id"), "session_id": frame.get("session_id"), "status": None, "reason": None, "raw_integrity": None, "corner_count": None, "coverage": None, "pnp_reprojection_px": None}
    image_metadata = frame.get("image", {})
    file_record = image_metadata.get("file", {}) if isinstance(image_metadata, dict) else {}
    filename = file_record.get("filename")
    expected_sha = file_record.get("sha256")
    if not isinstance(filename, str) or not isinstance(expected_sha, str):
        result.update(status="INVALID_FRAME_RECORD", reason="missing_file_provenance")
        return _json_safe(result)
    try:
        path = _evidence_file_path(root, filename)
    except IntrinsicValidationError:
        result.update(status="INVALID_FRAME_RECORD", reason="unsafe_file_provenance")
        return _json_safe(result)
    if not path.is_file():
        result.update(status="MISSING_RAW_FILE", reason="raw_file_absent", raw_integrity={"filename": filename, "status": "MISSING"})
        return _json_safe(result)
    actual_sha = _sha256(path)
    if actual_sha != expected_sha:
        result.update(status="RAW_SHA256_MISMATCH", reason="raw_file_checksum_differs", raw_integrity={"filename": filename, "status": "SHA256_MISMATCH", "expected_sha256": expected_sha, "actual_sha256": actual_sha})
        return _json_safe(result)
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        result.update(status="RAW_DECODE_FAILED", reason="cv2_imread_returned_none", raw_integrity={"filename": filename, "status": "VERIFIED"})
        return _json_safe(result)
    if image.shape != AR_SHAPE or image.dtype != np.uint8:
        result.update(status="WRONG_IMAGE_DIMENSIONS", reason="expected_1920x1200_bgr", raw_integrity={"filename": filename, "status": "VERIFIED"}, observed_image_shape=list(image.shape))
        return _json_safe(result)
    corners = _checkerboard(image)
    if corners is None:
        result.update(status="CHECKERBOARD_NOT_FOUND", reason="expected_54_corners_9x6", raw_integrity={"filename": filename, "status": "VERIFIED"}, observed_image_shape=list(image.shape))
        return _json_safe(result)
    center = np.mean(corners.reshape(-1, 2), axis=0)
    minimum = np.min(corners.reshape(-1, 2), axis=0)
    maximum = np.max(corners.reshape(-1, 2), axis=0)
    column = min(2, int(center[0] / (AR_WIDTH / 3)))
    row = min(2, int(center[1] / (AR_HEIGHT / 3)))
    result.update(
        status="VALIDATED",
        reason=None,
        raw_integrity={"filename": filename, "status": "VERIFIED", "sha256": actual_sha, "bytes": path.stat().st_size},
        corner_count=CHECKERBOARD_CORNER_COUNT,
        coverage={"center_px": [float(center[0]), float(center[1])], "span_px": [float(maximum[0] - minimum[0]), float(maximum[1] - minimum[1])], "grid_3x3_bin": {"row": row, "column": column}},
        pnp_reprojection_px=_pose_reprojection(corners, intrinsics),
    )
    return _json_safe(result)


def _coverage_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    bins: list[dict[str, Any]] = []
    valid = [record for record in records if record.get("status") == "VALIDATED"]
    for row in range(3):
        for column in range(3):
            selected = [record for record in valid if record["coverage"]["grid_3x3_bin"] == {"row": row, "column": column}]
            bins.append({"row": row, "column": column, "validated_frame_count": len(selected), "pnp_reprojection_px": _metrics([record["pnp_reprojection_px"]["rms_px"] for record in selected])})
    return _json_safe({"grid": {"rows": 3, "columns": 3, "image_width_px": AR_WIDTH, "image_height_px": AR_HEIGHT}, "validated_frame_count": len(valid), "bins": bins})


def _session_summary(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    sessions: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        sessions.setdefault(record.get("session_id", "UNKNOWN"), []).append(record)
    return _json_safe([{
        "session_id": session_id,
        "frame_count": len(rows),
        "status_counts": {status: sum(1 for row in rows if row.get("status") == status) for status in sorted({row.get("status") for row in rows})},
        "pnp_reprojection_px": _metrics([row["pnp_reprojection_px"]["rms_px"] for row in rows if row.get("status") == "VALIDATED"]),
    } for session_id, rows in sorted(sessions.items())])


def build_independent_validation_report(*, dataset: dict[str, Any], dataset_manifest: Path, intrinsic: Path, records: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [record for record in records if record.get("status") == "VALIDATED"]
    status_counts = {status: sum(1 for record in records if record.get("status") == status) for status in sorted({record.get("status") for record in records})}
    return _json_safe({
        "schema_version": REPORT_SCHEMA_VERSION,
        "status": "INDEPENDENT_INTRINSIC_VALIDATION_DIAGNOSTIC_COMPLETE" if len(valid) == len(records) else "INDEPENDENT_INTRINSIC_VALIDATION_DIAGNOSTIC_WITH_FRAME_FAILURES",
        "dataset_manifest": {"path": str(dataset_manifest), "sha256": _sha256(dataset_manifest)},
        "intrinsic": {"path": str(intrinsic), "sha256": _sha256(intrinsic), "activation_status_changed": False},
        "geometry": {"reference_frame": AR_REFERENCE_FRAME, "image_size_px": {"width": AR_WIDTH, "height": AR_HEIGHT}, "checkerboard": {"inner_corners": list(CHECKERBOARD_SIZE), "corner_count": CHECKERBOARD_CORNER_COUNT, "square_size_mm": SQUARE_SIZE_MM}, "opencv_pose_convention": "X_camera = R * X_object + t"},
        "dataset": {"schema_version": dataset["schema_version"], "session_count": len(dataset["sessions"]), "declared_frame_count": len(dataset["frames"]), "configuration_identity": _configuration_identity(dataset["configuration"])},
        "frame_status_counts": status_counts,
        "aggregate_pnp_reprojection_px": _metrics([record["pnp_reprojection_px"]["rms_px"] for record in valid]),
        "coverage": _coverage_summary(records),
        "sessions": _session_summary(records),
        "frames": records,
        "facts": ["Raw PNG files are checked against manifest SHA-256 before corner and PnP evaluation.", "Only frames with a newly detected finite 54-corner checkerboard contribute PnP and coverage metrics."],
        "limitations": ["This report independently validates an existing intrinsic candidate on new raw evidence; it does not recalibrate or activate it.", "No numeric acceptance policy, runtime policy, AR0234-to-OV9281 transform, or hardware conclusion is changed."],
        "generated_utc": datetime.now(timezone.utc).isoformat(),
    })


def validate_independent_dataset(*, dataset_manifest: Path, intrinsic: Path) -> dict[str, Any]:
    dataset = _read_dataset_manifest(dataset_manifest)
    intrinsics = _load_intrinsics(intrinsic)
    root = dataset_manifest.parent
    records = [_frame_diagnostic(root=root, frame=frame, intrinsics=intrinsics) for frame in dataset["frames"]]
    return build_independent_validation_report(dataset=dataset, dataset_manifest=dataset_manifest, intrinsic=intrinsic, records=records)


def _summary_markdown(report: dict[str, Any]) -> str:
    aggregate = report["aggregate_pnp_reprojection_px"]
    return "\n".join([
        "# AR0234 intrinsic V3 independent validation",
        "",
        f"- Статус diagnostic: `{report['status']}`",
        f"- Проверено кадров: `{report['dataset']['declared_frame_count']}`; valid: `{report['coverage']['validated_frame_count']}`.",
        f"- Aggregate PnP RMS: `{aggregate['rms_px']}` px.",
        "- Отчёт не выполняет recalibration и не меняет activation status intrinsic.",
        "- AR↔OV extrinsic candidate не изменён и остаётся `PROVISIONAL_DIAGNOSTIC_ONLY`.",
        "",
    ])


def _write_bytes_replace(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def write_validation_outputs(*, output_root: Path, report: dict[str, Any]) -> None:
    _require_persistent_root(output_root)
    diagnostics = output_root / "diagnostics"
    _write_json_replace(diagnostics / "independent_validation_report.json", report)
    _write_bytes_replace(diagnostics / "independent_validation_summary.md", _summary_markdown(report).encode("utf-8"))
