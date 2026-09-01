"""Stable-identity AR0234 capture adapter for the RGB-only MVP."""

from __future__ import annotations

import ctypes
import fcntl
import os
import re
import stat
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np


AR0234_BY_ID = Path(
    "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
)
_VIDEO_NODE_NAME = re.compile(r"video[0-9]+$")
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000


class _V4L2CapabilityLayout(ctypes.Structure):
    """Linux UAPI ``struct v4l2_capability`` from videodev2.h."""

    _fields_ = [
        ("driver", ctypes.c_uint8 * 16),
        ("card", ctypes.c_uint8 * 32),
        ("bus_info", ctypes.c_uint8 * 32),
        ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


@dataclass(frozen=True)
class V4L2Capabilities:
    capabilities: int
    device_caps: int

    @property
    def effective(self) -> int:
        if self.capabilities & V4L2_CAP_DEVICE_CAPS:
            return self.device_caps
        return self.capabilities


class VideoCaptureLike(Protocol):
    def isOpened(self) -> bool: ...
    def set(self, property_id: int, value: float) -> bool: ...
    def get(self, property_id: int) -> float: ...
    def read(self) -> tuple[bool, object]: ...
    def release(self) -> None: ...


def _vidioc_querycap_request() -> int:
    """Compute ``VIDIOC_QUERYCAP`` from Linux's generic _IOC ABI constants."""
    if os.name != "posix" or not hasattr(os, "uname") or os.uname().sysname != "Linux":
        raise RuntimeError("V4L2 capability verification requires Linux")
    size = ctypes.sizeof(_V4L2CapabilityLayout)
    if size != 104:
        raise RuntimeError(f"unexpected v4l2_capability size {size}, expected 104")
    return (2 << 30) | (ord("V") << 8) | (size << 16)


def query_v4l2_capabilities(
    device: Path,
    *,
    open_fn: Callable[[str, int], int] = os.open,
    ioctl_fn: Callable[[int, int, bytearray, bool], object] = fcntl.ioctl,
    close_fn: Callable[[int], None] = os.close,
) -> V4L2Capabilities:
    """Query a V4L2 node without starting streaming and always close its FD."""
    descriptor: int | None = None
    try:
        descriptor = open_fn(
            os.fspath(device),
            os.O_RDWR | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0),
        )
        buffer = bytearray(ctypes.sizeof(_V4L2CapabilityLayout))
        ioctl_fn(descriptor, _vidioc_querycap_request(), buffer, True)
        parsed = _V4L2CapabilityLayout.from_buffer_copy(buffer)
        return V4L2Capabilities(
            capabilities=int(parsed.capabilities),
            device_caps=int(parsed.device_caps),
        )
    except (OSError, TypeError, ValueError, ctypes.ArgumentError) as error:
        raise RuntimeError(f"V4L2 capability query failed for {device}: {error}") from error
    finally:
        if descriptor is not None:
            close_fn(descriptor)


def check_v4l2_capture_capability(
    device: Path,
    *,
    query_capabilities: Callable[[Path], V4L2Capabilities] = query_v4l2_capabilities,
) -> None:
    """Require a capture-capable and streaming-capable V4L2 interface."""
    reported = query_capabilities(device)
    capabilities = reported.effective
    capture_bits = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE
    if not capabilities & capture_bits:
        raise RuntimeError(f"AR0234 V4L2 node is not capture-capable: {device}")
    if not capabilities & V4L2_CAP_STREAMING:
        raise RuntimeError(f"AR0234 V4L2 node does not support streaming: {device}")


def resolve_ar0234_device(
    device: Path = AR0234_BY_ID,
    *,
    capability_checker: Callable[[Path], None] = check_v4l2_capture_capability,
) -> Path:
    """Resolve and verify the one approved AR0234 ``by-id`` capture interface."""
    if device != AR0234_BY_ID:
        raise ValueError("AR0234 must use the approved /dev/v4l/by-id/...-video-index0 path")
    if not device.is_symlink():
        raise FileNotFoundError(f"AR0234 stable device symlink is absent: {device}")
    target = device.resolve(strict=True)
    if not _VIDEO_NODE_NAME.fullmatch(target.name):
        raise RuntimeError(f"AR0234 by-id link does not resolve to a video node: {target}")
    if not stat.S_ISCHR(target.stat().st_mode):
        raise RuntimeError(f"AR0234 resolved target is not a character device: {target}")
    capability_checker(target)
    return target


@dataclass(frozen=True)
class AR0234CaptureConfig:
    device: Path = AR0234_BY_ID
    width: int = 1920
    height: int = 1200
    fps: float = 60.0
    fourcc: str = "MJPG"
    buffer_size: int = 1

    def __post_init__(self) -> None:
        if self.device != AR0234_BY_ID:
            raise ValueError("AR0234 capture does not permit /dev/video* fallbacks")


class AR0234Capture:
    """Injectable AR0234 capture adapter with fail-closed mode validation."""

    def __init__(
        self,
        config: AR0234CaptureConfig = AR0234CaptureConfig(),
        *,
        capture_factory: Callable[[str, int], VideoCaptureLike] = cv2.VideoCapture,
        device_resolver: Callable[[Path], Path] = resolve_ar0234_device,
    ) -> None:
        self.config = config
        self._capture_factory = capture_factory
        self._device_resolver = device_resolver
        self._capture: VideoCaptureLike | None = None

    def open(self) -> None:
        if self._capture is not None:
            raise RuntimeError("AR0234 capture is already open")
        target = self._device_resolver(self.config.device)
        capture = self._capture_factory(str(target), cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                raise RuntimeError(f"unable to open AR0234: {self.config.device}")
            for property_id, value in (
                (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.config.fourcc)),
                (cv2.CAP_PROP_FRAME_WIDTH, self.config.width),
                (cv2.CAP_PROP_FRAME_HEIGHT, self.config.height),
                (cv2.CAP_PROP_FPS, self.config.fps),
                (cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size),
            ):
                if not capture.set(property_id, value):
                    raise RuntimeError(f"AR0234 rejected required capture property {property_id}")
            actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
            if (actual_width, actual_height) != (self.config.width, self.config.height):
                raise RuntimeError(
                    "AR0234 mode mismatch: "
                    f"expected={self.config.width}x{self.config.height}, "
                    f"actual={actual_width}x{actual_height}"
                )
            if abs(actual_fps - self.config.fps) > 0.1:
                raise RuntimeError(
                    f"AR0234 FPS mismatch: expected={self.config.fps}, actual={actual_fps}"
                )
        except BaseException:
            capture.release()
            raise
        self._capture = capture

    def read(self) -> np.ndarray:
        if self._capture is None:
            raise RuntimeError("AR0234 capture is not open")
        try:
            ok, frame = self._capture.read()
            if not ok or not isinstance(frame, np.ndarray):
                raise RuntimeError("AR0234 frame read failed")
            expected_shape = (self.config.height, self.config.width, 3)
            if frame.shape != expected_shape or frame.dtype != np.uint8:
                raise RuntimeError(
                    "AR0234 frame mismatch: "
                    f"expected={expected_shape}/uint8, got={frame.shape}/{frame.dtype}"
                )
            return frame
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        capture = self._capture
        self._capture = None
        if capture is not None:
            capture.release()

    def __enter__(self) -> AR0234Capture:
        self.open()
        return self

    def __exit__(self, *_unused: object) -> None:
        self.close()
