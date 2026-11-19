"""Motor-off appearance diagnostics: central clothing-color signature.

Not biometric identification, not a person-ID or a motion admission source.
Do not use histogram distance as a hard ReID threshold without a labeled,
independent same/different-person validation dataset.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


HIST_BINS = 12
MIN_CLOTHING_PIXELS = 400


@dataclass(frozen=True)
class ClothingColorSignature:
    """Normalized 2D CIELAB a/b histogram of the mask-bound torso core."""

    histogram: tuple[float, ...]
    clothing_pixels: int
    coverage: float


def clothing_signature(rgb_bgr: np.ndarray, instance_mask: np.ndarray,
                       bbox_xyxy_px: list | tuple) -> ClothingColorSignature:
    """Use only same-frame selected mask, excluding the head/face and limbs.

    RGB image is OpenCV BGR. The torso crop is conservative; a current
    selected segmentation mask is mandatory even though the inner rectangle
    often lies entirely within the shirt. No frame or prior ID is cached.
    """
    if (not isinstance(rgb_bgr, np.ndarray) or rgb_bgr.ndim != 3
            or rgb_bgr.shape[2] != 3 or rgb_bgr.dtype != np.uint8
            or not isinstance(instance_mask, np.ndarray)
            or instance_mask.shape != rgb_bgr.shape[:2]
            or instance_mask.dtype != np.bool_
            or not isinstance(bbox_xyxy_px, (list, tuple))
            or len(bbox_xyxy_px) != 4):
        raise ValueError("INVALID_CURRENT_RGB_MASK_GEOMETRY")
    try:
        box = [float(v) for v in bbox_xyxy_px]
    except (ValueError, TypeError):
        raise ValueError("INVALID_CURRENT_RGB_MASK_GEOMETRY") from None
    height, width = rgb_bgr.shape[:2]
    x1, y1, x2, y2 = box
    if (not all(math.isfinite(v) for v in box)
            or not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height):
        raise ValueError("INVALID_CURRENT_RGB_MASK_GEOMETRY")

    # Central chest/abdomen; deliberately avoids the face. Standardized
    # relative to the CURRENT bbox, not to previous-target geometry.
    left = max(0, int(x1 + 0.25 * (x2 - x1)))
    right = min(width, int(x1 + 0.75 * (x2 - x1)))
    top = max(0, int(y1 + 0.24 * (y2 - y1)))
    bottom = min(height, int(y1 + 0.55 * (y2 - y1)))
    if left >= right or top >= bottom:
        raise ValueError("CLOTHING_CROP_EMPTY")
    current_mask = instance_mask[top:bottom, left:right]
    count = int(np.count_nonzero(current_mask))
    if count < MIN_CLOTHING_PIXELS:
        raise ValueError("INSUFFICIENT_CURRENT_CLOTHING_PIXELS")
    lab = cv2.cvtColor(rgb_bgr[top:bottom, left:right], cv2.COLOR_BGR2LAB)
    hist = cv2.calcHist([lab], [1, 2], current_mask.astype(np.uint8),
                        [HIST_BINS, HIST_BINS], [0, 256, 0, 256])
    hist = hist.reshape(-1).astype(np.float64)
    hist /= hist.sum()
    return ClothingColorSignature(tuple(map(float, hist)), count,
                                  count / current_mask.size)


def clothing_distance(first: ClothingColorSignature,
                      second: ClothingColorSignature) -> float:
    """Bhattacharyya distance in [0,1]; smaller = more similar COLOR only."""
    if (len(first.histogram) != HIST_BINS ** 2
            or len(second.histogram) != HIST_BINS ** 2):
        raise ValueError("INVALID_SIGNATURE_DIMENSIONS")
    a = np.asarray(first.histogram, dtype=np.float64)
    b = np.asarray(second.histogram, dtype=np.float64)
    if (not np.all(np.isfinite(a)) or not np.all(np.isfinite(b))
            or np.any(a < 0) or np.any(b < 0)
            or abs(float(a.sum()) - 1) > 1e-5
            or abs(float(b.sum()) - 1) > 1e-5):
        raise ValueError("INVALID_SIGNATURE_NORMALIZATION")
    affinity = float(np.sqrt(a * b).sum())
    return math.sqrt(max(0.0, 1.0 - min(1.0, affinity)))
