#!/usr/bin/env python3
"""Live AR0234 close-range face locator preview; it saves no frames or records."""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_localization import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig  # noqa: E402
from vision_core.person_localization.close_range_locator import (  # noqa: E402
    CloseRangeDetection,
    CloseRangeTargetStatus,
    OpenCvHaarFaceLocator,
    PersistentFaceTrack,
    TemporalFaceTracker,
    build_close_range_target_record,
)
from vision_core.tools.run_ar0234_alignment_preview import PreviewWindow  # noqa: E402


AR_WIDTH = 1920
AR_HEIGHT = 1200
WINDOW_NAME = "AR0234 Close-range Locator"


def draw_overlay(
    frame: np.ndarray,
    *,
    target_status: str,
    raw_detections: list[CloseRangeDetection],
    persistent_tracks: tuple[PersistentFaceTrack, ...],
    selected_track_id: str | None,
    center_x_px: float | None,
    confidence: float | None,
) -> np.ndarray:
    """Annotate a copy only; the captured frame remains untouched."""
    view = frame.copy()
    color = {
        CloseRangeTargetStatus.SINGLE_TARGET.value: (0, 220, 0),
        CloseRangeTargetStatus.NO_TARGET.value: (0, 0, 220),
        CloseRangeTargetStatus.MULTIPLE_TARGETS.value: (0, 180, 255),
    }[target_status]
    for detection in raw_detections:
        x_min, y_min, x_max, y_max = detection.bbox_xyxy_px
        cv2.rectangle(view, (x_min, y_min), (x_max, y_max), (125, 125, 125), 1)
    for track in persistent_tracks:
        x_min, y_min, x_max, y_max = track.bbox_xyxy_px
        thickness = 4 if track.track_id == selected_track_id else 2
        cv2.rectangle(view, (x_min, y_min), (x_max, y_max), (0, 220, 0), thickness)
        cv2.putText(
            view,
            track.track_id,
            (x_min, max(20, y_min - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 0),
            1,
            cv2.LINE_AA,
        )
    text = (
        f"{target_status} raw={len(raw_detections)} "
        f"persistent={len(persistent_tracks)}"
    )
    cv2.putText(view, text, (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
    if target_status == CloseRangeTargetStatus.SINGLE_TARGET.value:
        assert center_x_px is not None and confidence is not None
        cv2.putText(
            view,
            f"face center_x={center_x_px:.1f}px confidence={confidence:.3f}",
            (24, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        center_x = int(round(center_x_px))
        cv2.line(view, (center_x, 0), (center_x, view.shape[0] - 1), color, 1)
    return view


def _preview_size(frame: np.ndarray, width: int) -> np.ndarray:
    if type(width) is not int or width <= 0:
        raise ValueError("--preview-width must be a positive integer")
    if width >= frame.shape[1]:
        return frame
    height = int(round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--max-frames", type=int, default=0, help="0 means until Q or Esc")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.max_frames < 0:
        raise ValueError("--max-frames must be 0 or positive")
    locator = OpenCvHaarFaceLocator()
    tracker = TemporalFaceTracker()
    capture = AR0234Capture(
        AR0234CaptureConfig(
            device=AR0234_BY_ID,
            width=AR_WIDTH,
            height=AR_HEIGHT,
            fps=30.0,
            fourcc="MJPG",
            buffer_size=1,
        )
    )
    preview: PreviewWindow | None = None
    try:
        capture.open()
        for _ in range(60):
            capture.read()
        preview = PreviewWindow(WINDOW_NAME)
        count = 0
        while args.max_frames == 0 or count < args.max_frames:
            frame = capture.read()
            count += 1
            detections = locator.detect(frame)
            tracking = tracker.update(detections)
            record = build_close_range_target_record(
                tracking=tracking,
                evidence_id=f"close-range-preview-{count:06d}",
                captured_at_utc=datetime.now(timezone.utc),
                image_width=AR_WIDTH,
                image_height=AR_HEIGHT,
            )
            view = draw_overlay(
                frame,
                target_status=record["target_status"],
                raw_detections=detections,
                persistent_tracks=tracking.persistent_tracks,
                selected_track_id=record["selected_track_id"],
                center_x_px=record["center_x_px"],
                confidence=record["confidence"],
            )
            if preview.show(_preview_size(view, args.preview_width)):
                break
        return 0
    finally:
        capture.close()
        if preview is not None:
            preview.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
