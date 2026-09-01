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
        values = (self.x_min, self.y_min, self.x_max, self.y_max)
        if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
            raise ValueError("bounding-box coordinates must be integer pixels")
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

    def is_within(self, *, width: int, height: int) -> bool:
        return self.x_max <= width and self.y_max <= height

    def to_xyxy(self) -> list[int]:
        return [self.x_min, self.y_min, self.x_max, self.y_max]


@dataclass(frozen=True)
class PersonDetection:
    """One candidate under ``sie.person_detection_output.v1``."""

    bounding_box: BoundingBox
    confidence: float
    label: str = "person"

    def __post_init__(self) -> None:
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("detection label must be a non-empty string")
        if not isinstance(self.confidence, (int, float, np.floating)) or not np.isfinite(
            self.confidence
        ):
            raise ValueError("detection confidence must be finite")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("detection confidence must be in [0, 1]")


class PersonLocalizationStatus(str, Enum):
    SINGLE_PERSON = "SINGLE_PERSON"
    PERSON_LOST = "PERSON_LOST"
    MULTIPLE_PERSONS = "MULTIPLE_PERSONS"
    STALE_FRAME = "STALE_FRAME"
    FUTURE_TIMESTAMP = "FUTURE_TIMESTAMP"
    INVALID_FRAME = "INVALID_FRAME"
    MALFORMED_OUTPUT = "MALFORMED_OUTPUT"


@dataclass(frozen=True)
class PersonLocalizationResult:
    """Perception result with at most one SIE Observation.

    The current contract is bbox-only. Raw arrays and segmentation are outside
    this public result and require a future versioned contract.
    """

    status: PersonLocalizationStatus
    candidate_count: int
    captured_at_utc: str
    observation: Observation | None = None
    bounding_box: BoundingBox | None = None
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
            "detail": self.detail,
        }
