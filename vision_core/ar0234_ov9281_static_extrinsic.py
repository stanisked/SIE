"""Static-only AR0234 to physical-left OV9281 extrinsic evidence tools.

The capture path uses direct V4L2 MMAP/DQBUF timestamps.  It is deliberately
limited to an affirmed stationary checkerboard and never supplies a runtime
transform or an SIE Measurement.
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
from typing import Any, Iterable

import cv2
import numpy as np


AR_DEVICE = Path("/dev/video4")
OV_DEVICE = Path("/dev/video2")
AR_SHAPE = (1200, 1920, 3)
OV_COMBINED_SHAPE = (800, 2560, 3)
OV_LEFT_SHAPE = (800, 1280, 3)
AR_MODE = (1920, 1200, 30)
OV_MODE = (2560, 800, 60)
CHECKERBOARD_SIZE = (9, 6)
CHECKERBOARD_CORNER_COUNT = 54
SQUARE_SIZE_MM = 24.5
MAX_PAIR_SKEW_NS = 5_000_000
OV_CALIBRATION_SHA256 = "bb8fb665c6e06e2cbb633cf4c3c61aa74933dd253c9c7950a8420591975dd5e7"
OV_CALIBRATION_DEFAULT = Path(
    "vision_core/vision_benchmark/hardware_audit/stereo_calibration_v6/"
    "solution_joint_refine_corner_order_filtered_freeze_v2_run07/stereo_params_v6.npz"
)
CLOCK_DOMAIN = "CLOCK_MONOTONIC"
CAPTURE_BACKEND = "DIRECT_V4L2_MMAP_DQBUF"
DATASET_SCHEMA_VERSION = "sie.ar0234_ov9281_static_capture_dataset.v1"
SESSION_SCHEMA_VERSION = "sie.ar0234_ov9281_static_capture_session.v1"
_SESSION_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x2000


class StaticExtrinsicError(RuntimeError):
    """Raised for a fail-closed static extrinsic evidence failure."""


def _ioc(direction: int, ioctl_type: str, number: int, size: int) -> int:
    return (direction << 30) | (ord(ioctl_type) << 8) | number | (size << 16)


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


def _fourcc(text: str) -> int:
    if text != "MJPG":
        raise StaticExtrinsicError("only MJPG is supported")
    return sum(ord(char) << (8 * index) for index, char in enumerate(text))


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
        raise StaticExtrinsicError("record must be JSON-safe and finite") from error


def split_physical_left(combined: np.ndarray) -> np.ndarray:
    """Return physical-left OV9281 from the combined right half."""
    if not isinstance(combined, np.ndarray) or combined.dtype != np.uint8 or combined.shape != OV_COMBINED_SHAPE:
        raise StaticExtrinsicError("OV combined frame must be uint8 2560x800 BGR")
    return combined[:, 1280:].copy()


def require_acceptable_skew(ar_timestamp_ns: int, ov_timestamp_ns: int) -> int:
    if type(ar_timestamp_ns) is not int or type(ov_timestamp_ns) is not int:
        raise StaticExtrinsicError("kernel timestamps must be integer nanoseconds")
    skew_ns = abs(ar_timestamp_ns - ov_timestamp_ns)
    if skew_ns > MAX_PAIR_SKEW_NS:
        raise StaticExtrinsicError(f"pair skew exceeds 5 ms: {skew_ns} ns")
    return skew_ns


def _checkerboard(frame: np.ndarray) -> np.ndarray | None:
    found, corners = cv2.findChessboardCornersSB(
        frame,
        CHECKERBOARD_SIZE,
        flags=(cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY),
    )
    if not found or corners is None or corners.shape != (CHECKERBOARD_CORNER_COUNT, 1, 2) or not np.isfinite(corners).all():
        return None
    return corners.astype(np.float64)


@dataclass(frozen=True)
class KernelFrame:
    image: np.ndarray
    timestamp_ns: int
    sequence: int


class DirectV4L2Camera:
    """Small direct MMAP/DQBUF reader that accepts only monotonic timestamps."""

    def __init__(self, device: Path, width: int, height: int, fps: int) -> None:
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self.fd: int | None = None
        self.buffers: list[mmap.mmap] = []
        self.streamed = False

    def _ioctl(self, request: int, value: ctypes.Structure | ctypes._SimpleCData) -> None:
        if self.fd is None:
            raise StaticExtrinsicError("V4L2 device is not open")
        while True:
            result = _LIBC.ioctl(self.fd, request, ctypes.byref(value))
            if result >= 0:
                return
            error_number = ctypes.get_errno()
            if error_number != errno.EINTR:
                raise StaticExtrinsicError(
                    f"V4L2 ioctl failed for {self.device}: {os.strerror(error_number)}"
                )

    def open(self) -> dict[str, Any]:
        try:
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
            capability = _Capability(); self._ioctl(VIDIOC_QUERYCAP, capability)
            caps = capability.device_caps if capability.capabilities & V4L2_CAP_DEVICE_CAPS else capability.capabilities
            if caps & V4L2_CAP_VIDEO_CAPTURE == 0 or caps & V4L2_CAP_STREAMING == 0:
                raise StaticExtrinsicError(f"device lacks V4L2 capture/streaming: {self.device}")
            fmt = _Format(); fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            fmt.fmt.pix.width, fmt.fmt.pix.height = self.width, self.height
            fmt.fmt.pix.pixelformat, fmt.fmt.pix.field = _fourcc("MJPG"), V4L2_FIELD_NONE
            self._ioctl(VIDIOC_S_FMT, fmt)
            if (fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat) != (self.width, self.height, _fourcc("MJPG")):
                raise StaticExtrinsicError(f"V4L2 mode mismatch on {self.device}")
            parm = _StreamParm(); parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.timeperframe = _Fract(1, self.fps); self._ioctl(VIDIOC_S_PARM, parm)
            actual_fraction = parm.parm.capture.timeperframe
            if actual_fraction.numerator == 0 or actual_fraction.denominator == 0:
                raise StaticExtrinsicError(f"V4L2 returned invalid FPS fraction on {self.device}")
            actual_fps = actual_fraction.denominator / actual_fraction.numerator
            if abs(actual_fps - self.fps) > 0.5:
                raise StaticExtrinsicError(f"V4L2 FPS mismatch on {self.device}: {actual_fps}")
            req = _RequestBuffers(); req.count, req.type, req.memory = 3, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
            self._ioctl(VIDIOC_REQBUFS, req)
            if req.count < 2:
                raise StaticExtrinsicError("V4L2 supplied fewer than two MMAP buffers")
            for index in range(req.count):
                buf = _Buffer(); buf.index, buf.type, buf.memory = index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
                self._ioctl(VIDIOC_QUERYBUF, buf)
                self.buffers.append(mmap.mmap(self.fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset))
                self._ioctl(VIDIOC_QBUF, buf)
            stream_type = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE); self._ioctl(VIDIOC_STREAMON, stream_type); self.streamed = True
            return {"width": self.width, "height": self.height, "fps": actual_fps, "fourcc": "MJPG", "buffer_count": len(self.buffers)}
        except BaseException:
            self.close(); raise

    def read(self, timeout_s: float = 2.0) -> KernelFrame:
        if self.fd is None or not self.streamed:
            raise StaticExtrinsicError("V4L2 stream is not active")
        ready, _, _ = select.select([self.fd], [], [], timeout_s)
        if not ready:
            raise StaticExtrinsicError(f"V4L2 frame timeout: {self.device}")
        buf = _Buffer(); buf.type, buf.memory = V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP
        self._ioctl(VIDIOC_DQBUF, buf)
        try:
            if buf.index >= len(self.buffers) or buf.bytesused == 0 or buf.bytesused > buf.length:
                raise StaticExtrinsicError("invalid V4L2 buffer payload")
            if not buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
                raise StaticExtrinsicError("V4L2 kernel timestamp is not CLOCK_MONOTONIC")
            payload = self.buffers[buf.index][:buf.bytesused]
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            expected = (self.height, self.width, 3)
            if frame is None or frame.dtype != np.uint8 or frame.shape != expected:
                raise StaticExtrinsicError(f"decoded frame mismatch: expected {expected}")
            return KernelFrame(frame, int(buf.timestamp.tv_sec) * 1_000_000_000 + int(buf.timestamp.tv_usec) * 1_000, int(buf.sequence))
        finally:
            self._ioctl(VIDIOC_QBUF, buf)

    def close(self) -> None:
        if self.fd is None:
            return
        if self.streamed:
            try: self._ioctl(VIDIOC_STREAMOFF, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
            except StaticExtrinsicError: pass
        self.streamed = False
        for buffer in self.buffers:
            buffer.close()
        self.buffers.clear(); os.close(self.fd); self.fd = None


def _nearest_one_to_one(ar_frames: Iterable[KernelFrame], ov_frames: Iterable[KernelFrame]) -> list[tuple[KernelFrame, KernelFrame]]:
    available = sorted(list(ov_frames), key=lambda item: (item.timestamp_ns, item.sequence))
    pairs: list[tuple[KernelFrame, KernelFrame]] = []
    for ar in sorted(ar_frames, key=lambda item: (item.timestamp_ns, item.sequence)):
        if not available:
            break
        index = min(range(len(available)), key=lambda item: (abs(available[item].timestamp_ns - ar.timestamp_ns), available[item].timestamp_ns, available[item].sequence))
        pairs.append((ar, available.pop(index)))
    return pairs


def _write_new(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise StaticExtrinsicError(f"refusing to overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        try: temporary.unlink()
        except FileNotFoundError: pass


def _png(frame: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise StaticExtrinsicError("PNG encoding failed")
    return encoded.tobytes()


def _frame_file(root: Path, section: str, pair_id: str, frame: np.ndarray, *, manifest_prefix: str = "") -> dict[str, Any]:
    payload = _png(frame); filename = f"{pair_id}.png"; _write_new(root / section / filename, payload)
    relative_name = f"{section}/{filename}"
    if manifest_prefix:
        relative_name = f"{manifest_prefix}/{relative_name}"
    return {"filename": relative_name, "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}


def _capture_counters() -> dict[str, int]:
    return {
        "candidate_pairs_seen": 0,
        "rejected_skew": 0,
        "rejected_ar_checkerboard_not_found": 0,
        "rejected_ov_left_checkerboard_not_found": 0,
        "rejected_decode_or_frame_error": 0,
        "accepted_pairs": 0,
    }


def _classify_static_candidates(
    ar_frames: Iterable[KernelFrame],
    ov_frames: Iterable[KernelFrame],
) -> tuple[list[tuple[KernelFrame, KernelFrame, np.ndarray, int]], dict[str, int]]:
    """Classify candidate pairs without changing capture or pairing semantics."""
    accepted: list[tuple[KernelFrame, KernelFrame, np.ndarray, int]] = []
    counters = _capture_counters()
    for ar, ov in _nearest_one_to_one(ar_frames, ov_frames):
        counters["candidate_pairs_seen"] += 1
        try:
            skew_ns = require_acceptable_skew(ar.timestamp_ns, ov.timestamp_ns)
        except StaticExtrinsicError:
            counters["rejected_skew"] += 1
            continue
        try:
            left = split_physical_left(ov.image)
            ar_corners = _checkerboard(ar.image)
            if ar_corners is None:
                counters["rejected_ar_checkerboard_not_found"] += 1
                continue
            left_corners = _checkerboard(left)
        except (StaticExtrinsicError, cv2.error):
            counters["rejected_decode_or_frame_error"] += 1
            continue
        if left_corners is None:
            counters["rejected_ov_left_checkerboard_not_found"] += 1
            continue
        accepted.append((ar, ov, left, skew_ns))
    counters["accepted_pairs"] = len(accepted)
    return accepted, counters


def _write_jpeg_preview(root: Path, filename: str, frame: np.ndarray) -> dict[str, Any]:
    """Persist supplementary decoded-frame evidence, never a calibration pair."""
    metadata: dict[str, Any] = {
        "filename": filename,
        "saved": False,
        "role": "last successfully decoded preview frame only; not a calibration pair",
    }
    try:
        ok, encoded = cv2.imencode(".jpg", frame)
    except cv2.error:
        ok, encoded = False, None
    if ok and encoded is not None:
        payload = encoded.tobytes()
        _write_new(root / filename, payload)
        metadata.update({"saved": True, "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)})
    else:
        metadata["reason"] = "jpeg_encoding_failed"
    return metadata


def _diagnostic_preview_frames(root: Path, ar_frames: list[KernelFrame], ov_frames: list[KernelFrame]) -> dict[str, Any]:
    ar_preview: dict[str, Any] = {
        "filename": "diagnostic_last_ar0234.jpg",
        "saved": False,
        "role": "last successfully decoded preview frame only; not a calibration pair",
    }
    left_preview: dict[str, Any] = {
        "filename": "diagnostic_last_ov9281_physical_left.jpg",
        "saved": False,
        "role": "last successfully decoded preview frame only; not a calibration pair",
    }
    if ar_frames:
        ar_preview = _write_jpeg_preview(root, "diagnostic_last_ar0234.jpg", ar_frames[-1].image)
    for frame in reversed(ov_frames):
        try:
            left_preview = _write_jpeg_preview(root, "diagnostic_last_ov9281_physical_left.jpg", split_physical_left(frame.image))
        except StaticExtrinsicError:
            continue
        break
    return {"ar0234": ar_preview, "ov9281_physical_left": left_preview}


def _capture_summary(
    *,
    result: str,
    configuration: dict[str, Any],
    counters: dict[str, int],
    preview_frames: dict[str, Any],
) -> dict[str, Any]:
    return _json_safe({
        "schema_version": "sie.ar0234_ov9281_static_capture_rejection_summary.v1",
        "result": result,
        "configuration": configuration,
        "counters": counters,
        "diagnostic_preview_frames": preview_frames,
        "diagnostic_preview_limitation": "Preview images are supplementary decoded-frame evidence only and are not calibration pairs.",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "limitation": "Static target only. Kernel timestamp proximity is not proof of hardware synchronization and is not approved for dynamic scene pairing.",
    })


def _write_capture_summary(root: Path, summary: dict[str, Any]) -> None:
    _write_new(root / "capture_rejection_summary.json", _json_payload(summary))


def _json_payload(record: dict[str, Any]) -> bytes:
    return (json.dumps(_json_safe(record), indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _raise_insufficient_static_pairs(
    *,
    root: Path,
    pair_count: int,
    accepted_count: int,
    configuration: dict[str, Any],
    counters: dict[str, int],
    preview_frames: dict[str, Any],
) -> None:
    _write_capture_summary(root, _capture_summary(
        result="INSUFFICIENT_STATIC_VALID_PAIRS",
        configuration=configuration,
        counters=counters,
        preview_frames=preview_frames,
    ))
    raise StaticExtrinsicError(f"insufficient static valid pairs: {accepted_count}/{pair_count}")


def _validate_session_id(session_id: str) -> None:
    if not isinstance(session_id, str) or _SESSION_ID_PATTERN.fullmatch(session_id) is None:
        raise StaticExtrinsicError("session_id must be 1..64 ASCII letters, digits, '.', '_' or '-'")


def _capture_configuration(
    *,
    ar_device: Path,
    ov_device: Path,
    ar_mode: dict[str, Any],
    ov_mode: dict[str, Any],
    ar_intrinsic: Path,
    ov_calibration: Path,
) -> dict[str, Any]:
    return {
        "capture_backend": CAPTURE_BACKEND,
        "timestamp_clock_domain": CLOCK_DOMAIN,
        "maximum_pair_skew_ns": MAX_PAIR_SKEW_NS,
        "devices": {"ar0234": str(ar_device), "ov9281_combined": str(ov_device)},
        "capture_configuration": {"ar0234": ar_mode, "ov9281_combined": ov_mode},
        "calibrations": {
            "ar0234_intrinsic_candidate": {
                "path": str(ar_intrinsic),
                "sha256": _sha256(ar_intrinsic),
                "status": "CANDIDATE_NOT_VALIDATED",
            },
            "ov9281_stereo_v6": {"path": str(ov_calibration), "sha256": OV_CALIBRATION_SHA256},
        },
    }


def _configuration_identity(configuration: dict[str, Any]) -> dict[str, Any]:
    """Return the immutable dataset identity; calibration paths may move, hashes may not."""
    if not isinstance(configuration, dict):
        raise StaticExtrinsicError("capture provenance configuration is invalid")
    calibrations = configuration.get("calibrations", {})
    if not isinstance(calibrations, dict):
        raise StaticExtrinsicError("capture calibration provenance is invalid")
    ar_calibration = calibrations.get("ar0234_intrinsic_candidate", {})
    ov_calibration = calibrations.get("ov9281_stereo_v6", {})
    if not isinstance(ar_calibration, dict) or not isinstance(ov_calibration, dict):
        raise StaticExtrinsicError("capture calibration provenance is invalid")
    return {
        "capture_backend": configuration.get("capture_backend"),
        "timestamp_clock_domain": configuration.get("timestamp_clock_domain"),
        "maximum_pair_skew_ns": configuration.get("maximum_pair_skew_ns"),
        "devices": configuration.get("devices"),
        "capture_configuration": configuration.get("capture_configuration"),
        "calibration_sha256": {
            "ar0234_intrinsic_candidate": ar_calibration.get("sha256"),
            "ov9281_stereo_v6": ov_calibration.get("sha256"),
        },
    }


def _new_dataset_manifest(configuration: dict[str, Any]) -> dict[str, Any]:
    return _json_safe({
        "schema_version": DATASET_SCHEMA_VERSION,
        "status": "STATIC_ONLY_DATASET_CAPTURED",
        "static_target_affirmed": True,
        "dynamic_pairing_permitted": False,
        "configuration": configuration,
        "sessions": [],
        "pairs": [],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "updated_utc": datetime.now(timezone.utc).isoformat(),
        "limitation": "Static target only. Kernel timestamp proximity is not proof of hardware synchronization and is not approved for dynamic scene pairing.",
    })


def _validate_dataset_manifest(manifest: dict[str, Any]) -> None:
    if (
        manifest.get("schema_version") != DATASET_SCHEMA_VERSION
        or manifest.get("status") != "STATIC_ONLY_DATASET_CAPTURED"
        or manifest.get("static_target_affirmed") is not True
        or manifest.get("dynamic_pairing_permitted") is not False
        or not isinstance(manifest.get("configuration"), dict)
        or not isinstance(manifest.get("sessions"), list)
        or not isinstance(manifest.get("pairs"), list)
    ):
        raise StaticExtrinsicError("capture manifest is not a supported static-only dataset")


def _append_session_to_dataset(
    dataset: dict[str, Any],
    *,
    configuration: dict[str, Any],
    session: dict[str, Any],
    pairs: list[dict[str, Any]],
) -> dict[str, Any]:
    _validate_dataset_manifest(dataset)
    if _configuration_identity(dataset["configuration"]) != _configuration_identity(configuration):
        raise StaticExtrinsicError("capture provenance differs from existing dataset")
    session_id = session.get("session_id")
    _validate_session_id(session_id)
    if any(row.get("session_id") == session_id for row in dataset["sessions"]):
        raise StaticExtrinsicError(f"duplicate session_id: {session_id}")
    known_pair_ids = {row.get("pair_id") for row in dataset["pairs"]}
    pair_ids = [row.get("pair_id") for row in pairs]
    if any(not isinstance(pair_id, str) or pair_id in known_pair_ids for pair_id in pair_ids) or len(set(pair_ids)) != len(pair_ids):
        raise StaticExtrinsicError("pair_id is missing, duplicated, or already present in dataset")
    if session.get("pair_ids") != pair_ids or any(row.get("session_id") != session_id for row in pairs):
        raise StaticExtrinsicError("session and flat pair provenance are inconsistent")
    updated = _json_safe(dataset)
    updated["sessions"].append(_json_safe(session))
    updated["pairs"].extend(_json_safe(pairs))
    updated["updated_utc"] = datetime.now(timezone.utc).isoformat()
    return _json_safe(updated)


def _load_dataset_manifest(output_root: Path) -> dict[str, Any] | None:
    manifest_path = output_root / "capture_manifest.json"
    if not manifest_path.exists():
        sessions_root = output_root / "sessions"
        if sessions_root.exists():
            raise StaticExtrinsicError(
                "dataset root contains session evidence but capture_manifest.json is absent; refusing reconstruction"
            )
        return None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise StaticExtrinsicError("cannot read existing capture dataset manifest") from error
    _validate_dataset_manifest(manifest)
    _validate_dataset_session_evidence(output_root, manifest)
    return _json_safe(manifest)


def _write_json_replace(path: Path, record: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(_json_payload(record))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _session_manifest_record(
    *,
    session_id: str,
    configuration: dict[str, Any],
    pairs: list[dict[str, Any]],
) -> dict[str, Any]:
    return _json_safe({
        "schema_version": SESSION_SCHEMA_VERSION,
        "session_id": session_id,
        "status": "STATIC_ONLY_CAPTURED",
        "static_target_affirmed": True,
        "configuration": configuration,
        "pair_count": len(pairs),
        "pair_ids": [record["pair_id"] for record in pairs],
        "pairs": pairs,
        "capture_rejection_summary": f"sessions/{session_id}/capture_rejection_summary.json",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "limitation": "Static target only. This session is not valid for dynamic scene pairing.",
    })


def _write_session_manifest(session_root: Path, record: dict[str, Any]) -> dict[str, Any]:
    payload = _json_payload(record)
    _write_new(session_root / "session_manifest.json", payload)
    return {
        "filename": f"sessions/{record['session_id']}/session_manifest.json",
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _session_entry(
    *,
    session_manifest: dict[str, Any],
    session_manifest_file: dict[str, Any],
) -> dict[str, Any]:
    return _json_safe({
        "schema_version": SESSION_SCHEMA_VERSION,
        "session_id": session_manifest["session_id"],
        "status": session_manifest["status"],
        "static_target_affirmed": True,
        "configuration": session_manifest["configuration"],
        "pair_ids": session_manifest["pair_ids"],
        "capture_rejection_summary": session_manifest["capture_rejection_summary"],
        "session_manifest": session_manifest_file,
        "created_utc": session_manifest["created_utc"],
        "limitation": session_manifest["limitation"],
    })


def _validate_pair_file_evidence(output_root: Path, session_id: str, pair: dict[str, Any]) -> None:
    for camera in ("ar0234", "ov9281_combined", "ov9281_physical_left"):
        file_record = pair.get(camera, {}).get("file") if isinstance(pair.get(camera), dict) else None
        if not isinstance(file_record, dict):
            raise StaticExtrinsicError("session pair file evidence is incomplete")
        filename, expected_sha256 = file_record.get("filename"), file_record.get("sha256")
        prefix = f"sessions/{session_id}/"
        if not isinstance(filename, str) or not filename.startswith(prefix) or not isinstance(expected_sha256, str):
            raise StaticExtrinsicError("session pair file provenance is invalid")
        path = output_root / filename
        if not path.is_file() or _sha256(path) != expected_sha256:
            raise StaticExtrinsicError("session raw pair file is absent or checksum differs")


def _validate_dataset_session_evidence(output_root: Path, dataset: dict[str, Any]) -> None:
    sessions_root = output_root / "sessions"
    if not sessions_root.is_dir():
        raise StaticExtrinsicError("dataset manifest lacks sessions directory")
    declared_ids: set[str] = set()
    expected_pairs: list[dict[str, Any]] = []
    for entry in dataset["sessions"]:
        if not isinstance(entry, dict):
            raise StaticExtrinsicError("dataset session entry is invalid")
        session_id = entry.get("session_id")
        _validate_session_id(session_id)
        if session_id in declared_ids:
            raise StaticExtrinsicError("dataset manifest contains duplicate session_id")
        declared_ids.add(session_id)
        reference = entry.get("session_manifest")
        if not isinstance(reference, dict) or reference.get("filename") != f"sessions/{session_id}/session_manifest.json":
            raise StaticExtrinsicError("dataset session manifest reference is incomplete")
        session_manifest_path = output_root / reference["filename"]
        if not session_manifest_path.is_file():
            raise StaticExtrinsicError("dataset session manifest is absent")
        payload = session_manifest_path.read_bytes()
        if reference.get("sha256") != hashlib.sha256(payload).hexdigest():
            raise StaticExtrinsicError("dataset session manifest checksum differs")
        try:
            session_manifest = json.loads(payload)
        except json.JSONDecodeError as error:
            raise StaticExtrinsicError("dataset session manifest is invalid JSON") from error
        if (
            session_manifest.get("schema_version") != SESSION_SCHEMA_VERSION
            or session_manifest.get("session_id") != session_id
            or session_manifest.get("status") != "STATIC_ONLY_CAPTURED"
            or session_manifest.get("static_target_affirmed") is not True
            or not isinstance(session_manifest.get("pairs"), list)
            or any(not isinstance(row, dict) for row in session_manifest["pairs"])
            or session_manifest.get("pair_ids") != [row.get("pair_id") for row in session_manifest["pairs"]]
            or any(row.get("session_id") != session_id for row in session_manifest["pairs"])
        ):
            raise StaticExtrinsicError("dataset session manifest is incomplete or inconsistent")
        for pair in session_manifest["pairs"]:
            if not isinstance(pair, dict):
                raise StaticExtrinsicError("dataset session pair is invalid")
            _validate_pair_file_evidence(output_root, session_id, pair)
        if _configuration_identity(session_manifest.get("configuration")) != _configuration_identity(dataset["configuration"]):
            raise StaticExtrinsicError("dataset session provenance differs from dataset provenance")
        if (
            entry.get("schema_version") != SESSION_SCHEMA_VERSION
            or entry.get("status") != session_manifest["status"]
            or entry.get("static_target_affirmed") is not True
            or _configuration_identity(entry.get("configuration")) != _configuration_identity(session_manifest["configuration"])
            or entry.get("pair_ids") != session_manifest["pair_ids"]
            or entry.get("capture_rejection_summary") != session_manifest["capture_rejection_summary"]
            or not (output_root / session_manifest["capture_rejection_summary"]).is_file()
        ):
            raise StaticExtrinsicError("dataset session entry differs from immutable session manifest")
        expected_pairs.extend(session_manifest["pairs"])
    actual_session_dirs = {path.name for path in sessions_root.iterdir() if path.is_dir()}
    if actual_session_dirs != declared_ids:
        raise StaticExtrinsicError("dataset sessions directory and manifest are incomplete or inconsistent")
    if dataset["pairs"] != expected_pairs:
        raise StaticExtrinsicError("dataset flat pairs differ from immutable session manifests")


def _persist_successful_session(
    *,
    output_root: Path,
    session_root: Path,
    existing_dataset: dict[str, Any] | None,
    configuration: dict[str, Any],
    session_id: str,
    pairs: list[dict[str, Any]],
) -> dict[str, Any]:
    session_manifest = _session_manifest_record(
        session_id=session_id,
        configuration=configuration,
        pairs=pairs,
    )
    session_manifest_file = _write_session_manifest(session_root, session_manifest)
    session = _session_entry(
        session_manifest=session_manifest,
        session_manifest_file=session_manifest_file,
    )
    dataset = existing_dataset if existing_dataset is not None else _new_dataset_manifest(configuration)
    manifest = _append_session_to_dataset(dataset, configuration=configuration, session=session, pairs=pairs)
    _write_json_replace(output_root / "capture_manifest.json", manifest)
    return manifest


def capture_static_pairs(*, output_root: Path, session_id: str, ar_intrinsic: Path, ov_calibration: Path, pair_count: int, static_target_affirmed: bool, ar_device: Path = AR_DEVICE, ov_device: Path = OV_DEVICE) -> dict[str, Any]:
    if not static_target_affirmed:
        raise StaticExtrinsicError("capture requires --static-target-affirmed")
    if pair_count < 3:
        raise StaticExtrinsicError("pair_count must be at least three")
    _validate_session_id(session_id)
    if output_root.exists() and not output_root.is_dir():
        raise StaticExtrinsicError(f"output root is not a directory: {output_root}")
    if not ar_intrinsic.is_file() or not ov_calibration.is_file():
        raise StaticExtrinsicError("intrinsic or OV calibration file is absent")
    _load_ar_intrinsics(ar_intrinsic)
    if _sha256(ov_calibration) != OV_CALIBRATION_SHA256:
        raise StaticExtrinsicError("OV calibration SHA-256 mismatch")
    output_root.mkdir(parents=True, exist_ok=True)
    existing_dataset = _load_dataset_manifest(output_root)
    if existing_dataset is not None and any(row.get("session_id") == session_id for row in existing_dataset["sessions"]):
        raise StaticExtrinsicError(f"duplicate session_id: {session_id}")
    session_root = output_root / "sessions" / session_id
    if session_root.exists():
        raise StaticExtrinsicError(f"session output already exists: {session_root}")
    ar_camera, ov_camera = DirectV4L2Camera(ar_device, *AR_MODE), DirectV4L2Camera(ov_device, *OV_MODE)
    capture_errors = 0
    try:
        ar_mode, ov_mode = ar_camera.open(), ov_camera.open()
        configuration = _capture_configuration(
            ar_device=ar_device,
            ov_device=ov_device,
            ar_mode=ar_mode,
            ov_mode=ov_mode,
            ar_intrinsic=ar_intrinsic,
            ov_calibration=ov_calibration,
        )
        if existing_dataset is not None and _configuration_identity(existing_dataset["configuration"]) != _configuration_identity(configuration):
            raise StaticExtrinsicError("capture provenance differs from existing dataset")
        session_root.mkdir(parents=True, exist_ok=False)
        for _ in range(30): ar_camera.read(); ov_camera.read()
        ar_frames: list[KernelFrame] = []; ov_frames: list[KernelFrame] = []
        for _ in range(pair_count * 4):
            for camera, frames in ((ov_camera, ov_frames), (ar_camera, ar_frames), (ov_camera, ov_frames)):
                try:
                    frames.append(camera.read())
                except StaticExtrinsicError:
                    capture_errors += 1
    finally:
        ar_camera.close(); ov_camera.close()
    accepted, counters = _classify_static_candidates(ar_frames, ov_frames)
    if len(accepted) > pair_count:
        accepted = accepted[:pair_count]
    counters["accepted_pairs"] = len(accepted)
    counters["rejected_decode_or_frame_error"] += capture_errors
    configuration = _json_safe({**configuration, "static_target_affirmed": True, "requested_pair_count": pair_count})
    preview_frames = _diagnostic_preview_frames(session_root, ar_frames, ov_frames)
    if len(accepted) < pair_count:
        _raise_insufficient_static_pairs(
            root=session_root,
            pair_count=pair_count,
            accepted_count=len(accepted),
            configuration=configuration,
            counters=counters,
            preview_frames=preview_frames,
        )
    records: list[dict[str, Any]] = []
    for index, (ar, ov, left, skew_ns) in enumerate(accepted):
        pair_id = f"{session_id}__pair_{index:03d}"
        record = {
            "schema_version": "sie.ar0234_ov9281_static_pair.v1", "pair_id": pair_id,
            "session_id": session_id,
            "static_target_affirmed": True,
            "timestamp_clock_domain": CLOCK_DOMAIN,
            "capture_backend": CAPTURE_BACKEND,
            "ar0234": {"timestamp_ns": ar.timestamp_ns, "sequence": ar.sequence, "file": _frame_file(session_root, "ar0234", pair_id, ar.image, manifest_prefix=f"sessions/{session_id}")},
            "ov9281_combined": {"timestamp_ns": ov.timestamp_ns, "sequence": ov.sequence, "file": _frame_file(session_root, "ov9281_combined", pair_id, ov.image, manifest_prefix=f"sessions/{session_id}")},
            "ov9281_physical_left": {"file": _frame_file(session_root, "ov9281_physical_left", pair_id, left, manifest_prefix=f"sessions/{session_id}")},
            "skew_ns": skew_ns,
            "checkerboard": {"inner_corners": [9, 6], "square_size_mm": SQUARE_SIZE_MM, "ar0234_corner_count": 54, "ov9281_physical_left_corner_count": 54},
            "reference_frames": {"ar0234": "ar0234_rgb_optical_frame", "ov9281_physical_left": "ov9281_physical_left_optical_frame"},
        }
        records.append(record)
    _write_capture_summary(session_root, _capture_summary(
        result="STATIC_VALID_PAIRS_CAPTURED",
        configuration=configuration,
        counters=counters,
        preview_frames=preview_frames,
    ))
    return _persist_successful_session(
        output_root=output_root,
        session_root=session_root,
        existing_dataset=existing_dataset,
        configuration=configuration,
        session_id=session_id,
        pairs=records,
    )


@dataclass(frozen=True)
class CameraIntrinsics:
    matrix: np.ndarray
    distortion: np.ndarray
    frame: str


@dataclass(frozen=True)
class StaticPair:
    pair_id: str
    ar_corners: np.ndarray
    ov_left_corners: np.ndarray
    ar_timestamp_ns: int
    ov_timestamp_ns: int


def object_points_mm() -> np.ndarray:
    return np.asarray([[column * SQUARE_SIZE_MM, row * SQUARE_SIZE_MM, 0.0] for row in range(6) for column in range(9)], dtype=np.float64)


def _pose(corners: np.ndarray, intrinsics: CameraIntrinsics) -> tuple[np.ndarray, np.ndarray]:
    if corners.shape != (CHECKERBOARD_CORNER_COUNT, 1, 2) or not np.isfinite(corners).all():
        raise StaticExtrinsicError("solver requires 54 finite checkerboard corners")
    ok, rvec, tvec = cv2.solvePnP(object_points_mm(), corners, intrinsics.matrix, intrinsics.distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise StaticExtrinsicError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    if not np.isfinite(rotation).all() or not np.isfinite(tvec).all():
        raise StaticExtrinsicError("solvePnP returned non-finite pose")
    return rotation, tvec.reshape(3)


def _relative_transform(pair: StaticPair, ar: CameraIntrinsics, ov: CameraIntrinsics) -> tuple[np.ndarray, np.ndarray]:
    ar_rotation, ar_translation = _pose(pair.ar_corners, ar); ov_rotation, ov_translation = _pose(pair.ov_left_corners, ov)
    rotation = ov_rotation @ ar_rotation.T
    return rotation, ov_translation - rotation @ ar_translation


def _rotation_angle_deg(left: np.ndarray, right: np.ndarray) -> float:
    trace = float(np.trace(left @ right.T)); return float(math.degrees(math.acos(float(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))))


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    average = sum(rotations) / len(rotations); left, _, right = np.linalg.svd(average); result = left @ right
    if np.linalg.det(result) < 0: left[:, -1] *= -1; result = left @ right
    return result


def _metrics(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {"mean_px": float(np.mean(array)), "median_px": float(np.median(array)), "p95_px": float(np.percentile(array, 95)), "max_px": float(np.max(array)), "rms_px": float(np.sqrt(np.mean(array * array)))}


def build_static_transform_record(*, pairs: list[StaticPair], ar_intrinsics: CameraIntrinsics, ov_intrinsics: CameraIntrinsics, provenance: dict[str, Any]) -> dict[str, Any]:
    if len(pairs) < 3:
        raise StaticExtrinsicError("solver requires at least three static pairs")
    transforms = [(pair, *_relative_transform(pair, ar_intrinsics, ov_intrinsics)) for pair in pairs]
    initial_rotation = _mean_rotation([item[1] for item in transforms]); initial_translation = np.median(np.asarray([item[2] for item in transforms]), axis=0)
    translation_residuals = np.asarray([np.linalg.norm(item[2] - initial_translation) for item in transforms], dtype=np.float64)
    rotation_residuals = np.asarray([_rotation_angle_deg(item[1], initial_rotation) for item in transforms], dtype=np.float64)
    translation_limit = float(np.median(translation_residuals) + max(1e-9, 3.0 * 1.4826 * np.median(np.abs(translation_residuals - np.median(translation_residuals)))))
    rotation_limit = float(np.median(rotation_residuals) + max(1e-9, 3.0 * 1.4826 * np.median(np.abs(rotation_residuals - np.median(rotation_residuals)))))
    inliers = [item for item, translation_error, rotation_error in zip(transforms, translation_residuals, rotation_residuals) if translation_error <= translation_limit and rotation_error <= rotation_limit]
    if len(inliers) < 3:
        raise StaticExtrinsicError("robust rejection left fewer than three pairs")
    rotation = _mean_rotation([item[1] for item in inliers]); translation_mm = np.median(np.asarray([item[2] for item in inliers]), axis=0)
    reprojection_errors: list[float] = []; per_pair: list[dict[str, Any]] = []
    for pair, _, _ in inliers:
        ar_rotation, ar_translation = _pose(pair.ar_corners, ar_intrinsics)
        object_in_ar = (ar_rotation @ object_points_mm().T + ar_translation[:, None])
        object_in_ov = rotation @ object_in_ar + translation_mm[:, None]
        projected, _ = cv2.projectPoints(object_in_ov.T, np.zeros((3, 1)), np.zeros((3, 1)), ov_intrinsics.matrix, ov_intrinsics.distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - pair.ov_left_corners.reshape(-1, 2), axis=1)
        reprojection_errors.extend(float(value) for value in errors)
        per_pair.append({"pair_id": pair.pair_id, "ar_timestamp_ns": pair.ar_timestamp_ns, "ov_timestamp_ns": pair.ov_timestamp_ns, "reprojection": _metrics([float(value) for value in errors])})
    translation_m = [float(value / 1000.0) for value in translation_mm]
    record = {
        "schema_version": "sie.ar0234_ov9281_static_extrinsic_candidate.v1", "status": "PROVISIONAL_DIAGNOSTIC_ONLY",
        "static_only": True, "dynamic_pairing_permitted": False,
        "transform": {"name": "T_stereo_left_rgb", "source_reference_frame": "ar0234_rgb_optical_frame", "target_reference_frame": "ov9281_physical_left_optical_frame", "rotation_3x3": rotation.tolist(), "translation_m": translation_m},
        "checkerboard": {"inner_corners": [9, 6], "square_size_mm": SQUARE_SIZE_MM},
        "source_pairs": [item[0].pair_id for item in inliers], "rejected_pair_ids": [item[0].pair_id for item in transforms if item not in inliers],
        "quality_metrics": {"input_pair_count": len(pairs), "inlier_pair_count": len(inliers), "translation_residual_limit_mm": translation_limit, "rotation_residual_limit_deg": rotation_limit, "reprojection_consistency": _metrics(reprojection_errors), "per_pair": per_pair},
        "timestamps": {"clock_domain": CLOCK_DOMAIN, "source_pair_timestamps_ns": [{"pair_id": pair.pair_id, "ar0234": pair.ar_timestamp_ns, "ov9281": pair.ov_timestamp_ns} for pair, _, _ in inliers], "generated_utc": datetime.now(timezone.utc).isoformat()},
        "provenance": provenance,
        "limitation": "PROVISIONAL_DIAGNOSTIC_ONLY. Static checkerboard evidence does not prove hardware synchronization, dynamic pairing validity, runtime applicability, or motion safety.",
    }
    return _json_safe(record)


def _load_ar_intrinsics(path: Path) -> CameraIntrinsics:
    value = json.loads(path.read_text(encoding="utf-8")); image = value.get("image", {})
    if image.get("width") != 1920 or image.get("height") != 1200:
        raise StaticExtrinsicError("AR intrinsic must be 1920x1200")
    matrix, distortion = np.asarray(value.get("camera_matrix"), dtype=np.float64), np.asarray(value.get("distortion_coefficients"), dtype=np.float64).reshape(1, -1)
    if matrix.shape != (3, 3) or distortion.shape[1] < 4 or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise StaticExtrinsicError("invalid AR intrinsic candidate")
    return CameraIntrinsics(matrix, distortion, "ar0234_rgb_optical_frame")


def _load_ov_intrinsics(path: Path) -> CameraIntrinsics:
    if _sha256(path) != OV_CALIBRATION_SHA256:
        raise StaticExtrinsicError("OV calibration SHA-256 mismatch")
    values = np.load(path, allow_pickle=False); matrix, distortion = values["K1"].astype(np.float64), values["D1"].astype(np.float64).reshape(1, -1)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise StaticExtrinsicError("invalid OV physical-left intrinsic")
    return CameraIntrinsics(matrix, distortion, "ov9281_physical_left_optical_frame")


def solve_static_capture(*, capture_manifest: Path, ar_intrinsic: Path, ov_calibration: Path) -> dict[str, Any]:
    manifest = json.loads(capture_manifest.read_text(encoding="utf-8"))
    if (
        manifest.get("status") not in {"STATIC_ONLY_CAPTURED", "STATIC_ONLY_DATASET_CAPTURED"}
        or manifest.get("dynamic_pairing_permitted") is not False
        or (
            manifest.get("timestamp_clock_domain") != CLOCK_DOMAIN
            and manifest.get("configuration", {}).get("timestamp_clock_domain") != CLOCK_DOMAIN
        )
    ):
        raise StaticExtrinsicError("capture manifest is not static-only CLOCK_MONOTONIC evidence")
    pairs: list[StaticPair] = []
    root = capture_manifest.parent
    for row in manifest.get("pairs", []):
        if row.get("checkerboard", {}).get("ar0234_corner_count") != 54 or row.get("checkerboard", {}).get("ov9281_physical_left_corner_count") != 54:
            raise StaticExtrinsicError("capture record lacks 54/54 checkerboard corners")
        ar_path, left_path = root / row["ar0234"]["file"]["filename"], root / row["ov9281_physical_left"]["file"]["filename"]
        ar, left = cv2.imread(str(ar_path), cv2.IMREAD_COLOR), cv2.imread(str(left_path), cv2.IMREAD_COLOR)
        if ar is None or left is None: raise StaticExtrinsicError("captured raw frame is absent")
        ar_corners, left_corners = _checkerboard(ar), _checkerboard(left)
        if ar_corners is None or left_corners is None: raise StaticExtrinsicError("solver requires 54/54 corners in every source pair")
        require_acceptable_skew(int(row["ar0234"]["timestamp_ns"]), int(row["ov9281_combined"]["timestamp_ns"]))
        pairs.append(StaticPair(row["pair_id"], ar_corners, left_corners, int(row["ar0234"]["timestamp_ns"]), int(row["ov9281_combined"]["timestamp_ns"])))
    provenance = {"capture_manifest": {"path": str(capture_manifest), "sha256": _sha256(capture_manifest)}, "ar0234_intrinsic_candidate": {"path": str(ar_intrinsic), "sha256": _sha256(ar_intrinsic), "status": "CANDIDATE_NOT_VALIDATED"}, "ov9281_stereo_v6": {"path": str(ov_calibration), "sha256": _sha256(ov_calibration)}}
    return build_static_transform_record(pairs=pairs, ar_intrinsics=_load_ar_intrinsics(ar_intrinsic), ov_intrinsics=_load_ov_intrinsics(ov_calibration), provenance=provenance)


def write_json_new(path: Path, record: dict[str, Any]) -> None:
    _write_new(path, (json.dumps(_json_safe(record), indent=2, sort_keys=True, allow_nan=False) + "\n").encode())
