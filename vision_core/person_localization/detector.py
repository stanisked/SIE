"""Person-detector adapters used by the AR0234 RGB-only MVP."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

import cv2
import numpy as np

from .models import BoundingBox, PersonDetection


class PersonDetector(Protocol):
    """Replaceable implementation boundary for person detection."""

    detector_id: str

    def detect(self, frame_bgr: np.ndarray) -> Sequence[PersonDetection]:
        """Return all person candidates from one BGR uint8 image."""


class HOGPersonDetector:
    """OpenCV built-in HOG people detector with no model download.

    It supplies bounding boxes only. The pipeline derives an explicitly named
    ``bbox_mask`` for local visualization and testing; that mask is not a
    semantic person segmentation.
    """

    detector_id = "opencv_hog_default_people_detector_v1"

    def __init__(self) -> None:
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame_bgr: np.ndarray) -> list[PersonDetection]:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("person detector requires a BGR image with three channels")
        if frame_bgr.dtype != np.uint8:
            raise ValueError("person detector requires uint8 pixels")

        boxes, weights = self._hog.detectMultiScale(
            frame_bgr,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        return [
            PersonDetection(
                bounding_box=BoundingBox(
                    x_min=int(x),
                    y_min=int(y),
                    x_max=int(x + box_width),
                    y_max=int(y + box_height),
                ),
                confidence=float(weight),
            )
            for (x, y, box_width, box_height), weight in zip(
                boxes, weights, strict=True
            )
        ]
