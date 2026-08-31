#!/usr/bin/env python3

"""Run the RGB-only AR0234 single-person Observation producer.

This tool never starts stereo processing, creates a Measurement, or sends a
network command to the mobile base.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2

from vision_core.person_localization import (
    AR0234_BY_ID,
    AR0234Capture,
    AR0234CaptureConfig,
    HOGPersonDetector,
    PersonLocalizationPipeline,
    PersonLocalizationPolicy,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=Path, default=AR0234_BY_ID)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--maximum-frame-age-s", type=float, default=0.5)
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_frames < 0:
        raise ValueError("max-frames must be non-negative")
    pipeline = PersonLocalizationPipeline(
        HOGPersonDetector(),
        PersonLocalizationPolicy(
            min_confidence=args.min_confidence,
            maximum_frame_age_s=args.maximum_frame_age_s,
        ),
    )
    processed = 0
    with AR0234Capture(AR0234CaptureConfig(device=args.device)) as capture:
        while args.max_frames == 0 or processed < args.max_frames:
            frame = capture.read()
            result = pipeline.process(
                frame,
                captured_at_utc=datetime.now(timezone.utc),
                cycle_id=str(processed),
            )
            print(json.dumps(result.to_dict(), sort_keys=True), flush=True)
            processed += 1

            if not args.no_preview:
                preview = frame.copy()
                if result.bounding_box is not None:
                    box = result.bounding_box
                    cv2.rectangle(
                        preview,
                        (box.x_min, box.y_min),
                        (box.x_max, box.y_max),
                        (0, 255, 0),
                        2,
                    )
                cv2.putText(
                    preview,
                    result.status,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("SIE AR0234 person localization", preview)
                if cv2.waitKey(1) in (27, ord("q")):
                    break
    if not args.no_preview:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
