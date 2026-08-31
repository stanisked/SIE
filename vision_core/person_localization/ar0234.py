"""Stable-identity AR0234 capture adapter for the RGB-only MVP."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


AR0234_BY_ID = Path(
    "/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
)


@dataclass(frozen=True)
class AR0234CaptureConfig:
    device: Path = AR0234_BY_ID
    width: int = 1920
    height: int = 1200
    fps: float = 60.0
    fourcc: str = "MJPG"
    buffer_size: int = 1


class AR0234Capture:
    """Open AR0234 by stable V4L2 identity, never by a fixed video number."""

    def __init__(self, config: AR0234CaptureConfig = AR0234CaptureConfig()) -> None:
        self.config = config
        self._capture: cv2.VideoCapture | None = None

    def open(self) -> None:
        if self._capture is not None:
            raise RuntimeError("AR0234 capture is already open")
        if not self.config.device.exists():
            raise FileNotFoundError(f"AR0234 stable device path is absent: {self.config.device}")

        capture = cv2.VideoCapture(str(self.config.device), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"unable to open AR0234: {self.config.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.config.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
        if (actual_width, actual_height) != (self.config.width, self.config.height):
            capture.release()
            raise RuntimeError(
                "AR0234 mode mismatch: "
                f"expected={self.config.width}x{self.config.height}, "
                f"actual={actual_width}x{actual_height}"
            )
        if abs(actual_fps - self.config.fps) > 0.1:
            capture.release()
            raise RuntimeError(
                f"AR0234 FPS mismatch: expected={self.config.fps}, actual={actual_fps}"
            )
        self._capture = capture

    def read(self) -> np.ndarray:
        if self._capture is None:
            raise RuntimeError("AR0234 capture is not open")
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("AR0234 frame read failed")
        expected_shape = (self.config.height, self.config.width, 3)
        if frame.shape != expected_shape or frame.dtype != np.uint8:
            raise RuntimeError(
                "AR0234 frame mismatch: "
                f"expected={expected_shape}/uint8, got={frame.shape}/{frame.dtype}"
            )
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def __enter__(self) -> AR0234Capture:
        self.open()
        return self

    def __exit__(self, *_unused: object) -> None:
        self.close()
