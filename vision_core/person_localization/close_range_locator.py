"""Close-range face-based image locator for AR0234 alignment observation.

This module is intentionally separate from the full-body person/depth pipeline.
It emits image-frame observations only and has no depth, planning, network, or
motor-control dependency.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np


SCHEMA_VERSION = "sie.ar0234_close_range_locator.v1"
REFERENCE_FRAME = "ar0234_image_frame"
UNITS = "px"
DETECTOR_ID = "opencv_haar_frontalface_default"
CONFIDENCE_SEMANTICS = "opencv_haar_level_weight_sigmoid_uncalibrated"
CASCADE_FILENAME = "haarcascade_frontalface_default.xml"
TRACK_WINDOW_FRAMES = 5
PERSISTENT_TRACK_MIN_OBSERVATIONS = 3
TRACK_MATCH_MIN_IOU = 0.20
TRACK_MATCH_MAX_CENTER_DISTANCE_PX = 120.0


class CloseRangeTargetStatus(str, Enum):
    SINGLE_TARGET = "SINGLE_TARGET"
    NO_TARGET = "NO_TARGET"
    MULTIPLE_TARGETS = "MULTIPLE_TARGETS"


@dataclass(frozen=True)
class CloseRangeDetection:
    """One face candidate in the AR0234 image frame."""

    bbox_xyxy_px: tuple[int, int, int, int]
    confidence: float

    def __post_init__(self) -> None:
        x_min, y_min, x_max, y_max = self.bbox_xyxy_px
        if any(type(value) is not int for value in self.bbox_xyxy_px):
            raise ValueError("bbox coordinates must be integer pixels")
        if x_min < 0 or y_min < 0 or x_max <= x_min or y_max <= y_min:
            raise ValueError("bbox must have positive in-frame area")
        if type(self.confidence) not in (int, float) or not math.isfinite(self.confidence):
            raise ValueError("confidence must be finite")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("confidence must be in [0, 1]")

    @property
    def center_x_px(self) -> float:
        return (self.bbox_xyxy_px[0] + self.bbox_xyxy_px[2]) / 2.0

    @property
    def center_y_px(self) -> float:
        return (self.bbox_xyxy_px[1] + self.bbox_xyxy_px[3]) / 2.0


@dataclass(frozen=True)
class PersistentFaceTrack:
    """One current-frame track supported by several observations in the window."""

    track_id: str
    bbox_xyxy_px: tuple[int, int, int, int]
    confidence: float
    observation_count: int


@dataclass(frozen=True)
class TemporalTrackingResult:
    """Current raw candidates and the fail-closed persistent-track decision."""

    raw_detection_count: int
    persistent_tracks: tuple[PersistentFaceTrack, ...]
    target_status: CloseRangeTargetStatus
    selected_track_id: str | None

    @property
    def persistent_track_count(self) -> int:
        return len(self.persistent_tracks)


@dataclass
class _TrackHistory:
    track_id: str
    observations: list[tuple[int, CloseRangeDetection]]

    @property
    def latest(self) -> tuple[int, CloseRangeDetection]:
        return self.observations[-1]


class TemporalFaceTracker:
    """Associate face candidates over exactly five preview frames.

    A candidate must be observed in at least three of those frames and also be
    present in the current frame before it can become a target. This prevents a
    one-frame Haar false positive from becoming a target record.
    """

    def __init__(self) -> None:
        self._frame_index = 0
        self._next_track = 1
        self._tracks: dict[str, _TrackHistory] = {}

    def update(self, detections: Sequence[CloseRangeDetection]) -> TemporalTrackingResult:
        candidates = list(detections)
        if any(not isinstance(detection, CloseRangeDetection) for detection in candidates):
            raise TypeError("detections must contain CloseRangeDetection values")
        self._frame_index += 1
        assigned_tracks: set[str] = set()
        assigned_detections: set[int] = set()
        matches: list[tuple[float, float, str, int]] = []
        for track_id, track in self._tracks.items():
            _last_frame, previous = track.latest
            for detection_index, detection in enumerate(candidates):
                iou = _bbox_iou(previous.bbox_xyxy_px, detection.bbox_xyxy_px)
                center_distance = math.hypot(
                    previous.center_x_px - detection.center_x_px,
                    previous.center_y_px - detection.center_y_px,
                )
                if iou >= TRACK_MATCH_MIN_IOU and center_distance <= TRACK_MATCH_MAX_CENTER_DISTANCE_PX:
                    matches.append((-iou, center_distance, track_id, detection_index))
        for _negative_iou, _distance, track_id, detection_index in sorted(matches):
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            self._tracks[track_id].observations.append(
                (self._frame_index, candidates[detection_index])
            )
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)
        for detection_index, detection in enumerate(candidates):
            if detection_index in assigned_detections:
                continue
            track_id = f"face-track-{self._next_track:06d}"
            self._next_track += 1
            self._tracks[track_id] = _TrackHistory(
                track_id=track_id,
                observations=[(self._frame_index, detection)],
            )

        first_frame = self._frame_index - TRACK_WINDOW_FRAMES + 1
        for track_id, track in list(self._tracks.items()):
            track.observations[:] = [
                observation for observation in track.observations if observation[0] >= first_frame
            ]
            if not track.observations:
                del self._tracks[track_id]

        persistent: list[PersistentFaceTrack] = []
        for track_id in sorted(self._tracks):
            track = self._tracks[track_id]
            latest_frame, latest = track.latest
            if (
                latest_frame == self._frame_index
                and len(track.observations) >= PERSISTENT_TRACK_MIN_OBSERVATIONS
            ):
                persistent.append(
                    PersistentFaceTrack(
                        track_id=track_id,
                        bbox_xyxy_px=latest.bbox_xyxy_px,
                        confidence=latest.confidence,
                        observation_count=len(track.observations),
                    )
                )
        if len(persistent) == 1:
            status = CloseRangeTargetStatus.SINGLE_TARGET
            selected_track_id: str | None = persistent[0].track_id
        elif len(persistent) >= 2:
            status = CloseRangeTargetStatus.MULTIPLE_TARGETS
            selected_track_id = None
        else:
            status = CloseRangeTargetStatus.NO_TARGET
            selected_track_id = None
        return TemporalTrackingResult(
            raw_detection_count=len(candidates),
            persistent_tracks=tuple(persistent),
            target_status=status,
            selected_track_id=selected_track_id,
        )


def default_face_cascade_path() -> Path:
    """Return the bundled OpenCV Haar cascade, never downloading a model."""
    roots: list[Path] = []
    data = getattr(cv2, "data", None)
    if data is not None and isinstance(getattr(data, "haarcascades", None), str):
        roots.append(Path(data.haarcascades))
    cv2_path = getattr(cv2, "__file__", None)
    if isinstance(cv2_path, str):
        roots.append(Path(cv2_path).resolve().parent / "data")
    roots.append(Path("/usr/share/opencv4/haarcascades"))
    for root in roots:
        path = root / CASCADE_FILENAME
        if path.is_file():
            return path
    raise RuntimeError("bundled OpenCV face cascade is unavailable")


class OpenCvHaarFaceLocator:
    """Use the local OpenCV Haar frontal-face cascade without DNN inference."""

    def __init__(self, cascade_path: Path | None = None) -> None:
        self.cascade_path = cascade_path or default_face_cascade_path()
        self._cascade = cv2.CascadeClassifier(str(self.cascade_path))
        if self._cascade.empty():
            raise RuntimeError(f"unable to load OpenCV face cascade: {self.cascade_path}")

    def detect(self, frame_bgr: np.ndarray) -> list[CloseRangeDetection]:
        _validate_frame(frame_bgr)
        height, width = frame_bgr.shape[:2]
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        rectangles, _reject_levels, level_weights = self._cascade.detectMultiScale3(
            gray,
            scaleFactor=1.1,
            minNeighbors=3,
            outputRejectLevels=True,
        )
        detections: list[CloseRangeDetection] = []
        for rectangle, level_weight in zip(rectangles, level_weights, strict=True):
            x, y, bbox_width, bbox_height = (int(value) for value in rectangle)
            x_min = max(0, x)
            y_min = max(0, y)
            x_max = min(width, x + bbox_width)
            y_max = min(height, y + bbox_height)
            if x_max <= x_min or y_max <= y_min:
                continue
            detections.append(
                CloseRangeDetection(
                    (x_min, y_min, x_max, y_max),
                    _normalized_level_weight(float(level_weight)),
                )
            )
        return detections


def build_close_range_target_record(
    *,
    tracking: TemporalTrackingResult,
    evidence_id: str,
    captured_at_utc: datetime,
    image_width: int,
    image_height: int,
) -> dict[str, Any]:
    """Build a JSON-safe, fail-closed image-frame target observation."""
    if type(evidence_id) is not str or not evidence_id:
        raise ValueError("evidence_id must be a non-empty string")
    timestamp = _timestamp(captured_at_utc)
    if type(image_width) is not int or type(image_height) is not int:
        raise ValueError("image dimensions must be integers")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    if not isinstance(tracking, TemporalTrackingResult):
        raise TypeError("tracking must be a TemporalTrackingResult")
    if type(tracking.raw_detection_count) is not int or tracking.raw_detection_count < 0:
        raise ValueError("raw_detection_count must be a non-negative integer")
    persistent_ids = [track.track_id for track in tracking.persistent_tracks]
    if tracking.target_status == CloseRangeTargetStatus.SINGLE_TARGET:
        if len(tracking.persistent_tracks) != 1 or tracking.selected_track_id != persistent_ids[0]:
            raise ValueError("SINGLE_TARGET requires exactly one selected persistent track")
    elif tracking.target_status == CloseRangeTargetStatus.MULTIPLE_TARGETS:
        if len(tracking.persistent_tracks) < 2 or tracking.selected_track_id is not None:
            raise ValueError("MULTIPLE_TARGETS requires multiple unselected persistent tracks")
    elif tracking.target_status == CloseRangeTargetStatus.NO_TARGET:
        if tracking.persistent_tracks or tracking.selected_track_id is not None:
            raise ValueError("NO_TARGET cannot expose a persistent track")
    else:
        raise ValueError("unknown close-range target status")
    for track in tracking.persistent_tracks:
        x_min, y_min, x_max, y_max = track.bbox_xyxy_px
        if x_max > image_width or y_max > image_height:
            raise ValueError("persistent track bbox is outside image dimensions")

    center_x_px: float | None = None
    bbox_xyxy_px: list[int] | None = None
    confidence: float | None = None
    selected = (
        tracking.persistent_tracks[0]
        if tracking.target_status == CloseRangeTargetStatus.SINGLE_TARGET
        else None
    )
    if selected is not None:
        center_x_px = (selected.bbox_xyxy_px[0] + selected.bbox_xyxy_px[2]) / 2.0
        bbox_xyxy_px = list(selected.bbox_xyxy_px)
        confidence = float(selected.confidence)

    return _json_safe(
        {
            "schema_version": SCHEMA_VERSION,
            "evidence_id": evidence_id,
            "timestamp": timestamp,
            "reference_frame": REFERENCE_FRAME,
            "units": UNITS,
            "detector_id": DETECTOR_ID,
            "confidence_semantics": CONFIDENCE_SEMANTICS,
            "target_status": tracking.target_status.value,
            "raw_detection_count": tracking.raw_detection_count,
            "persistent_track_count": tracking.persistent_track_count,
            "selected_track_id": tracking.selected_track_id,
            "center_x_px": center_x_px,
            "bbox_xyxy_px": bbox_xyxy_px,
            "confidence": confidence,
        }
    )


def _bbox_iou(
    first: tuple[int, int, int, int],
    second: tuple[int, int, int, int],
) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return 0.0 if union <= 0 else intersection / union


def _normalized_level_weight(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("OpenCV Haar level weight must be finite")
    bounded = max(-50.0, min(50.0, value))
    return 1.0 / (1.0 + math.exp(-bounded))


def _timestamp(value: datetime) -> str:
    if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("captured_at_utc must be timezone-aware")
    return value.astimezone(timezone.utc).isoformat()


def _validate_frame(frame_bgr: object) -> None:
    if not (
        isinstance(frame_bgr, np.ndarray)
        and frame_bgr.ndim == 3
        and frame_bgr.shape[0] > 0
        and frame_bgr.shape[1] > 0
        and frame_bgr.shape[2] == 3
        and frame_bgr.dtype == np.uint8
    ):
        raise ValueError("frame must be a non-empty BGR uint8 image")


def _json_safe(value: object) -> dict[str, Any]:
    try:
        output = json.loads(json.dumps(value, allow_nan=False, sort_keys=True))
    except (TypeError, ValueError) as error:
        raise ValueError("close-range locator record must be JSON-safe") from error
    if type(output) is not dict:
        raise ValueError("close-range locator record must be an object")
    return output
