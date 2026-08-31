"""Data local to AR0234 person perception.

These types deliberately stop at Observation. They do not produce a stereo
Measurement, a Fact, a Decision, or a motor command.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from vision_core.observation import Observation


@dataclass(frozen=True)
class BoundingBox:
    """Pixel-aligned bounding box using exclusive right and bottom edges."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int

    def __post_init__(self) -> None:
        if self.x_min < 0 or self.y_min < 0:
            raise ValueError("bounding-box origin must be non-negative")
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("bounding box must have positive area")

    @property
    def width(self) -> int:
        return self.x_max - self.x_min

    @property
    def height(self) -> int:
        return self.y_max - self.y_min

    @property
    def area(self) -> int:
        return self.width * self.height

    def clip(self, *, width: int, height: int) -> BoundingBox | None:
        x_min = max(0, min(self.x_min, width))
        y_min = max(0, min(self.y_min, height))
        x_max = max(0, min(self.x_max, width))
        y_max = max(0, min(self.y_max, height))
        if x_max <= x_min or y_max <= y_min:
            return None
        return BoundingBox(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)

    def to_xyxy(self) -> list[int]:
        return [self.x_min, self.y_min, self.x_max, self.y_max]


@dataclass(frozen=True)
class PersonDetection:
    """One detector candidate. Confidence semantics belong to its backend."""

    bounding_box: BoundingBox
    confidence: float
    label: str = "person"

    def __post_init__(self) -> None:
        if self.label != "person":
            raise ValueError("person localization accepts only person detections")
        if not np.isfinite(self.confidence):
            raise ValueError("detection confidence must be finite")


class PersonLocalizationStatus(str, Enum):
    SINGLE_PERSON = "SINGLE_PERSON"
    PERSON_LOST = "PERSON_LOST"
    MULTIPLE_PERSONS = "MULTIPLE_PERSONS"
    STALE_FRAME = "STALE_FRAME"
    INVALID_FRAME = "INVALID_FRAME"


@dataclass(frozen=True)
class PersonLocalizationResult:
    """Internal perception result with at most one SIE Observation.

    ``bbox_mask`` is a binary mask of the selected bounding box. It is kept
    internal to Vision Core and is not serialized into the Observation payload
    as a substitute for a semantic segmentation mask.
    """

    status: PersonLocalizationStatus
    candidate_count: int
    captured_at_utc: str
    observation: Observation | None = None
    bounding_box: BoundingBox | None = None
    bbox_mask: np.ndarray | None = None
    detail: str | None = None

    def to_dict(self) -> dict:
        return {
            "status": self.status,
            "candidate_count": self.candidate_count,
            "captured_at_utc": self.captured_at_utc,
            "observation": None if self.observation is None else self.observation.to_dict(),
            "bounding_box_xyxy_px": (
                None if self.bounding_box is None else self.bounding_box.to_xyxy()
            ),
            "bbox_mask_kind": "bbox_mask" if self.bbox_mask is not None else None,
            "detail": self.detail,
        }
