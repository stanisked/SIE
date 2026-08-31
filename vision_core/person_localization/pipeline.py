"""Single-person AR0234 Observation producer.

This module is intentionally RGB-only. Stereo association, depth sampling,
motion decisions, and ESP32 commands belong to later, separately validated
stages.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from vision_core.observation import Observation

from .detector import PersonDetector
from .models import (
    BoundingBox,
    PersonDetection,
    PersonLocalizationResult,
    PersonLocalizationStatus,
)


AR0234_OPTICAL_FRAME = "ar0234_optical_frame"
AR0234_SOURCE_ID = "ar0234_rgb"


@dataclass(frozen=True)
class PersonLocalizationPolicy:
    min_confidence: float = 0.0
    maximum_frame_age_s: float = 0.5

    def __post_init__(self) -> None:
        if not np.isfinite(self.min_confidence):
            raise ValueError("minimum confidence must be finite")
        if not np.isfinite(self.maximum_frame_age_s) or self.maximum_frame_age_s <= 0:
            raise ValueError("maximum frame age must be finite and positive")


class PersonLocalizationPipeline:
    """Emit one Observation only when exactly one person is usable."""

    def __init__(
        self,
        detector: PersonDetector,
        policy: PersonLocalizationPolicy = PersonLocalizationPolicy(),
        *,
        now_utc: Callable[[], datetime] | None = None,
    ) -> None:
        self.detector = detector
        self.policy = policy
        self._now_utc = now_utc or (lambda: datetime.now(timezone.utc))

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        captured_at_utc: datetime,
        cycle_id: str,
    ) -> PersonLocalizationResult:
        timestamp = self._normalize_timestamp(captured_at_utc)
        if not self._is_valid_frame(frame_bgr):
            return PersonLocalizationResult(
                status=PersonLocalizationStatus.INVALID_FRAME,
                candidate_count=0,
                captured_at_utc=timestamp.isoformat(),
                detail="AR0234 frame must be a non-empty BGR uint8 image",
            )
        age_s = (self._normalize_timestamp(self._now_utc()) - timestamp).total_seconds()
        if age_s > self.policy.maximum_frame_age_s:
            return PersonLocalizationResult(
                status=PersonLocalizationStatus.STALE_FRAME,
                candidate_count=0,
                captured_at_utc=timestamp.isoformat(),
                detail=(
                    f"frame age {age_s:.6f}s exceeds "
                    f"{self.policy.maximum_frame_age_s:.6f}s"
                ),
            )

        candidates = self._usable_candidates(self.detector.detect(frame_bgr), frame_bgr)
        if not candidates:
            return PersonLocalizationResult(
                status=PersonLocalizationStatus.PERSON_LOST,
                candidate_count=0,
                captured_at_utc=timestamp.isoformat(),
                detail="no usable person detection",
            )
        if len(candidates) != 1:
            return PersonLocalizationResult(
                status=PersonLocalizationStatus.MULTIPLE_PERSONS,
                candidate_count=len(candidates),
                captured_at_utc=timestamp.isoformat(),
                detail="movement-relevant localization requires exactly one person",
            )

        detection = candidates[0]
        bbox_mask = self._bbox_mask(frame_bgr.shape[:2], detection.bounding_box)
        observation = self._make_observation(
            detection=detection,
            frame_shape=frame_bgr.shape,
            captured_at_utc=timestamp,
            cycle_id=cycle_id,
        )
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.SINGLE_PERSON,
            candidate_count=1,
            captured_at_utc=timestamp.isoformat(),
            observation=observation,
            bounding_box=detection.bounding_box,
            bbox_mask=bbox_mask,
        )

    @staticmethod
    def _is_valid_frame(frame_bgr: np.ndarray) -> bool:
        return bool(
            isinstance(frame_bgr, np.ndarray)
            and frame_bgr.ndim == 3
            and frame_bgr.shape[0] > 0
            and frame_bgr.shape[1] > 0
            and frame_bgr.shape[2] == 3
            and frame_bgr.dtype == np.uint8
        )

    @staticmethod
    def _normalize_timestamp(value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("captured_at_utc must be timezone-aware")
        return value.astimezone(timezone.utc)

    def _usable_candidates(
        self,
        raw_candidates: object,
        frame_bgr: np.ndarray,
    ) -> list[PersonDetection]:
        height, width = frame_bgr.shape[:2]
        usable: list[PersonDetection] = []
        for candidate in raw_candidates:
            if not isinstance(candidate, PersonDetection):
                raise TypeError("person detector returned an invalid candidate type")
            if candidate.confidence < self.policy.min_confidence:
                continue
            clipped = candidate.bounding_box.clip(width=width, height=height)
            if clipped is None:
                continue
            usable.append(
                PersonDetection(
                    bounding_box=clipped,
                    confidence=candidate.confidence,
                    label=candidate.label,
                )
            )
        return usable

    def _make_observation(
        self,
        *,
        detection: PersonDetection,
        frame_shape: tuple[int, ...],
        captured_at_utc: datetime,
        cycle_id: str,
    ) -> Observation:
        height, width = frame_shape[:2]
        mask_metadata = {
            "kind": "bbox_mask",
            "semantic_segmentation": False,
            "shape_px": [width, height],
            "area_px": detection.bounding_box.area,
        }
        return Observation(
            observation_id=f"observation.person.ar0234.{cycle_id}",
            source_id=AR0234_SOURCE_ID,
            timestamp=captured_at_utc.isoformat(),
            cycle_id=cycle_id,
            observation_type="single_person_bbox",
            payload={
                "reference_frame": AR0234_OPTICAL_FRAME,
                "unit": "px",
                "image_size_px": [width, height],
                "person_count": 1,
                "bounding_box_xyxy_px": detection.bounding_box.to_xyxy(),
                "mask": mask_metadata,
                "detector_id": self.detector.detector_id,
            },
            confidence=float(detection.confidence),
            quality={
                "status": "PASS",
                "single_person_required": True,
                "candidate_count": 1,
                "detector_confidence": float(detection.confidence),
            },
            evidence_ids=(f"evidence.ar0234_frame.{cycle_id}",),
        )

    @staticmethod
    def _bbox_mask(shape: tuple[int, int], bounding_box: BoundingBox) -> np.ndarray:
        height, width = shape
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[
            bounding_box.y_min : bounding_box.y_max,
            bounding_box.x_min : bounding_box.x_max,
        ] = 255
        return mask
