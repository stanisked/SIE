"""Single-person AR0234 Observation producer.

This module is intentionally RGB-only. Stereo association, depth sampling,
motion decisions, and ESP32 commands belong to later, separately validated
stages.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from vision_core.observation import Observation

from .detector import DetectorArtifact, PersonDetector
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
    """Fail-closed confidence and wall-clock freshness limits for RGB input."""

    min_confidence: float = 0.0
    maximum_frame_age_s: float = 0.5
    allowed_future_skew_s: float = 0.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.min_confidence) or not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("minimum confidence must be finite and in [0, 1]")
        if not np.isfinite(self.maximum_frame_age_s) or self.maximum_frame_age_s <= 0:
            raise ValueError("maximum frame age must be finite and positive")
        if not np.isfinite(self.allowed_future_skew_s) or self.allowed_future_skew_s < 0:
            raise ValueError("allowed future skew must be finite and non-negative")


class PersonLocalizationPipeline:
    """Emit one Observation only when exactly one valid candidate is usable."""

    def __init__(
        self,
        detector: PersonDetector,
        policy: PersonLocalizationPolicy = PersonLocalizationPolicy(),
        *,
        now_utc: Callable[[], datetime] | None = None,
    ) -> None:
        artifact = detector.artifact
        if not isinstance(artifact, DetectorArtifact):
            raise ValueError("person detector must declare a validated DetectorArtifact")
        self.detector = detector
        self._artifact = artifact
        self._artifact_metadata = json.loads(
            json.dumps(artifact.metadata(), allow_nan=False, sort_keys=True, separators=(",", ":"))
        )
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
            return self._result(
                PersonLocalizationStatus.INVALID_FRAME,
                timestamp,
                detail="AR0234 frame must be a non-empty BGR uint8 image",
            )
        now = self._normalize_timestamp(self._now_utc())
        age_s = (now - timestamp).total_seconds()
        if age_s < -self.policy.allowed_future_skew_s:
            return self._result(
                PersonLocalizationStatus.FUTURE_TIMESTAMP,
                timestamp,
                detail=f"frame timestamp is {-age_s:.6f}s in the future",
            )
        if age_s > self.policy.maximum_frame_age_s:
            return self._result(
                PersonLocalizationStatus.STALE_FRAME,
                timestamp,
                detail=(
                    f"frame age {age_s:.6f}s exceeds "
                    f"{self.policy.maximum_frame_age_s:.6f}s"
                ),
            )

        try:
            raw_candidates = self.detector.detect(frame_bgr)
            if self.detector.artifact is not self._artifact:
                raise ValueError("detector artifact changed after pipeline initialization")
            candidates = self._validated_candidates(raw_candidates, frame_bgr)
        except Exception as error:
            return self._result(
                PersonLocalizationStatus.MALFORMED_OUTPUT,
                timestamp,
                detail=f"detector output rejected: {error}",
            )
        if not candidates:
            return self._result(
                PersonLocalizationStatus.PERSON_LOST,
                timestamp,
                detail="no usable person detection",
            )
        if len(candidates) != 1:
            return self._result(
                PersonLocalizationStatus.MULTIPLE_PERSONS,
                timestamp,
                candidate_count=len(candidates),
                detail="movement-relevant localization requires exactly one person detection",
            )

        detection = candidates[0]
        observation = self._make_observation(
            detection=detection,
            frame_shape=frame_bgr.shape,
            captured_at_utc=timestamp,
            cycle_id=cycle_id,
            candidate_count=len(candidates),
        )
        return PersonLocalizationResult(
            status=PersonLocalizationStatus.SINGLE_PERSON,
            candidate_count=1,
            captured_at_utc=timestamp.isoformat(),
            observation=observation,
            bounding_box=detection.bounding_box,
        )

    @staticmethod
    def _result(
        status: PersonLocalizationStatus,
        timestamp: datetime,
        *,
        candidate_count: int = 0,
        detail: str,
    ) -> PersonLocalizationResult:
        return PersonLocalizationResult(
            status=status,
            candidate_count=candidate_count,
            captured_at_utc=timestamp.isoformat(),
            detail=detail,
        )

    @staticmethod
    def _is_valid_frame(frame_bgr: object) -> bool:
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
        if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("captured_at_utc must be timezone-aware")
        return value.astimezone(timezone.utc)

    def _validated_candidates(
        self,
        raw_candidates: object,
        frame_bgr: np.ndarray,
    ) -> list[PersonDetection]:
        if not isinstance(raw_candidates, Sequence) or isinstance(raw_candidates, (str, bytes)):
            raise TypeError("detector must return a sequence of PersonDetection values")
        height, width = frame_bgr.shape[:2]
        validated: list[PersonDetection] = []
        for candidate in raw_candidates:
            if not isinstance(candidate, PersonDetection):
                raise TypeError("detector returned an invalid candidate type")
            self._validate_detection(candidate, width=width, height=height)
            if candidate.confidence >= max(
                self.policy.min_confidence,
                self._artifact.confidence_threshold,
            ):
                validated.append(candidate)
        return validated

    def _validate_detection(self, candidate: PersonDetection, *, width: int, height: int) -> None:
        if candidate.label != self._artifact.person_label:
            raise ValueError("detector label does not match artifact person_label")
        if not isinstance(candidate.bounding_box, BoundingBox):
            raise TypeError("detector bounding box has an invalid type")
        if not isinstance(candidate.confidence, (int, float, np.floating)) or not np.isfinite(
            candidate.confidence
        ):
            raise ValueError("detector confidence must be finite")
        if not 0.0 <= float(candidate.confidence) <= 1.0:
            raise ValueError("detector confidence must be in [0, 1]")
        coordinates = (
            candidate.bounding_box.x_min,
            candidate.bounding_box.y_min,
            candidate.bounding_box.x_max,
            candidate.bounding_box.y_max,
        )
        if any(isinstance(value, bool) or not isinstance(value, int) for value in coordinates):
            raise ValueError("detector bounding box coordinates must be integer pixels")
        if candidate.bounding_box.x_min < 0 or candidate.bounding_box.y_min < 0:
            raise ValueError("detector bounding box origin must be non-negative")
        if (
            candidate.bounding_box.x_max <= candidate.bounding_box.x_min
            or candidate.bounding_box.y_max <= candidate.bounding_box.y_min
        ):
            raise ValueError("detector bounding box must have positive area")
        if not candidate.bounding_box.is_within(width=width, height=height):
            raise ValueError("detector bounding box is outside the image")

    def _make_observation(
        self,
        *,
        detection: PersonDetection,
        frame_shape: tuple[int, ...],
        captured_at_utc: datetime,
        cycle_id: str,
        candidate_count: int,
    ) -> Observation:
        height, width = frame_shape[:2]
        payload: dict[str, object] = {
            "reference_frame": AR0234_OPTICAL_FRAME,
            "unit": "px",
            "image_size_px": [width, height],
            "person_count": 1,
            "bounding_box_xyxy_px": detection.bounding_box.to_xyxy(),
            "detector": json.loads(json.dumps(self._artifact_metadata, allow_nan=False)),
        }
        return Observation(
            observation_id=f"observation.person.ar0234.{cycle_id}",
            source_id=AR0234_SOURCE_ID,
            timestamp=captured_at_utc.isoformat(),
            cycle_id=cycle_id,
            observation_type="single_person_bbox",
            payload=payload,
            confidence=float(detection.confidence),
            quality={
                "status": "PASS",
                "single_person_required": True,
                "candidate_count": candidate_count,
                "detector_confidence": float(detection.confidence),
                "required_confidence_threshold": max(
                    self.policy.min_confidence,
                    self._artifact.confidence_threshold,
                ),
            },
            evidence_ids=(f"evidence.ar0234_frame.{cycle_id}",),
        )
