from __future__ import annotations

from datetime import datetime, timedelta, timezone
from unittest.mock import patch

import numpy as np
import pytest

import vision_core.person_localization.detector as detector_module
from vision_core.person_localization.ar0234 import AR0234_BY_ID, AR0234CaptureConfig
from vision_core.person_localization.detector import HOGPersonDetector
from vision_core.person_localization.models import BoundingBox, PersonDetection
from vision_core.person_localization.pipeline import (
    AR0234_OPTICAL_FRAME,
    PersonLocalizationPipeline,
    PersonLocalizationPolicy,
)


class FakeDetector:
    detector_id = "fake_person_detector_v1"

    def __init__(self, candidates: list[PersonDetection]) -> None:
        self.candidates = candidates
        self.calls = 0

    def detect(self, _frame: np.ndarray) -> list[PersonDetection]:
        self.calls += 1
        return self.candidates


NOW = datetime(2026, 8, 31, 12, 0, 0, tzinfo=timezone.utc)


def make_pipeline(candidates: list[PersonDetection], **policy_kwargs: float):
    return PersonLocalizationPipeline(
        FakeDetector(candidates),
        PersonLocalizationPolicy(**policy_kwargs),
        now_utc=lambda: NOW,
    )


def test_ar0234_capture_uses_stable_identity_not_video_number() -> None:
    assert str(AR0234_BY_ID) == (
        "/dev/v4l/by-id/"
        "usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0"
    )
    assert AR0234CaptureConfig().device == AR0234_BY_ID


def test_hog_adapter_preserves_all_person_candidates() -> None:
    class FakeHOG:
        def setSVMDetector(self, _detector: object) -> None:
            pass

        def detectMultiScale(self, _frame: np.ndarray, **_kwargs: object):
            return np.array([[2, 3, 4, 5]]), np.array([0.75])

    with (
        patch.object(detector_module.cv2, "HOGDescriptor", return_value=FakeHOG()),
        patch.object(
            detector_module.cv2,
            "HOGDescriptor_getDefaultPeopleDetector",
            return_value=np.array([1]),
        ),
    ):
        detections = HOGPersonDetector().detect(np.zeros((20, 20, 3), dtype=np.uint8))

    assert detections == [PersonDetection(BoundingBox(2, 3, 6, 8), 0.75)]


def test_single_person_creates_observation_and_internal_bbox_mask() -> None:
    detector = FakeDetector([PersonDetection(BoundingBox(20, 10, 80, 50), 0.8)])
    pipeline = PersonLocalizationPipeline(
        detector,
        PersonLocalizationPolicy(maximum_frame_age_s=0.5),
        now_utc=lambda: NOW,
    )
    frame = np.zeros((100, 200, 3), dtype=np.uint8)

    result = pipeline.process(frame, captured_at_utc=NOW, cycle_id="42")

    assert result.status == "SINGLE_PERSON"
    assert result.candidate_count == 1
    assert result.observation is not None
    assert result.observation.observation_type == "single_person_bbox"
    assert result.observation.payload["reference_frame"] == AR0234_OPTICAL_FRAME
    assert result.observation.payload["bounding_box_xyxy_px"] == [20, 10, 80, 50]
    assert result.observation.payload["mask"] == {
        "kind": "bbox_mask",
        "semantic_segmentation": False,
        "shape_px": [200, 100],
        "area_px": 2400,
    }
    assert result.bbox_mask is not None
    assert result.bbox_mask.shape == (100, 200)
    assert int(result.bbox_mask.sum()) == 2400 * 255
    assert detector.calls == 1


def test_no_person_is_reported_without_observation() -> None:
    result = make_pipeline([]).process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW,
        cycle_id="lost",
    )

    assert result.status == "PERSON_LOST"
    assert result.observation is None
    assert result.bbox_mask is None


def test_multiple_people_are_blocked_without_choosing_a_candidate() -> None:
    candidates = [
        PersonDetection(BoundingBox(1, 1, 4, 4), 0.9),
        PersonDetection(BoundingBox(5, 1, 9, 5), 0.8),
    ]
    result = make_pipeline(candidates).process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW,
        cycle_id="many",
    )

    assert result.status == "MULTIPLE_PERSONS"
    assert result.candidate_count == 2
    assert result.observation is None


def test_stale_frame_is_blocked_before_detector_execution() -> None:
    detector = FakeDetector([PersonDetection(BoundingBox(1, 1, 4, 4), 0.9)])
    pipeline = PersonLocalizationPipeline(
        detector,
        PersonLocalizationPolicy(maximum_frame_age_s=0.5),
        now_utc=lambda: NOW,
    )
    result = pipeline.process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW - timedelta(seconds=0.6),
        cycle_id="stale",
    )

    assert result.status == "STALE_FRAME"
    assert result.observation is None
    assert detector.calls == 0


def test_low_confidence_candidates_do_not_become_person_observations() -> None:
    result = make_pipeline(
        [PersonDetection(BoundingBox(1, 1, 4, 4), 0.2)],
        min_confidence=0.5,
    ).process(
        np.zeros((10, 10, 3), dtype=np.uint8),
        captured_at_utc=NOW,
        cycle_id="low-confidence",
    )

    assert result.status == "PERSON_LOST"
    assert result.observation is None


def test_invalid_frame_is_blocked_before_detector_execution() -> None:
    detector = FakeDetector([])
    pipeline = PersonLocalizationPipeline(detector, now_utc=lambda: NOW)

    result = pipeline.process(
        np.zeros((10, 10), dtype=np.uint8),
        captured_at_utc=NOW,
        cycle_id="invalid",
    )

    assert result.status == "INVALID_FRAME"
    assert result.observation is None
    assert detector.calls == 0


def test_naive_capture_timestamp_is_rejected() -> None:
    with pytest.raises(ValueError, match="timezone-aware"):
        make_pipeline([]).process(
            np.zeros((10, 10, 3), dtype=np.uint8),
            captured_at_utc=datetime(2026, 8, 31, 12, 0, 0),
            cycle_id="naive",
        )
