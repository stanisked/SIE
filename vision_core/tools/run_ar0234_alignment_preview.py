#!/usr/bin/env python3
"""Live AR0234 image-frame alignment preview for supervised yaw-mapping observation."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.person_localization import AR0234_BY_ID, AR0234Capture, AR0234CaptureConfig  # noqa: E402
from vision_core.person_localization.mp_persondet import MPPersonDetOpenCV  # noqa: E402
from vision_core.person_localization.pipeline import PersonLocalizationPipeline  # noqa: E402


AR_WIDTH = 1920
AR_HEIGHT = 1200
WINDOW_NAME = "AR0234 Alignment Preview"


@dataclass(frozen=True)
class ARIntrinsic:
    cx_px: float
    cy_px: float


class PreviewWindow:
    """Display OpenCV overlays through HighGUI or a tkinter fallback."""

    def __init__(self, name: str) -> None:
        self._name = name
        self._highgui = False
        self._root: Any = None
        self._label: Any = None
        self._tk: Any = None
        self._closed = False
        try:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            self._highgui = True
            return
        except cv2.error:
            pass
        try:
            import tkinter as tk

            self._tk = tk
            self._root = tk.Tk()
            self._root.title(name)
            self._label = tk.Label(self._root)
            self._label.pack()
            self._root.protocol("WM_DELETE_WINDOW", self._mark_closed)
            self._root.bind("<KeyPress>", self._on_key)
        except Exception as error:
            raise RuntimeError("no GUI preview backend is available: OpenCV HighGUI is unavailable and tkinter failed to start") from error

    def _mark_closed(self) -> None:
        self._closed = True

    def _on_key(self, event: Any) -> None:
        if event.keysym in {"Escape", "q", "Q"}:
            self._mark_closed()

    def show(self, image_bgr: np.ndarray) -> bool:
        """Present one overlay frame and report whether the user requested exit."""
        if self._highgui:
            cv2.imshow(self._name, image_bgr)
            return (cv2.waitKey(1) & 0xff) in (ord("q"), ord("Q"), 27)
        if self._closed:
            return True
        rgb = np.ascontiguousarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        height, width = rgb.shape[:2]
        ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()
        photo = self._tk.PhotoImage(
            data=ppm,
            format="PPM",
        )
        self._label.configure(image=photo)
        self._label.image = photo
        self._root.update_idletasks()
        self._root.update()
        return self._closed

    def close(self) -> None:
        if self._highgui:
            cv2.destroyWindow(self._name)
        elif self._root is not None:
            try:
                self._root.destroy()
            except self._tk.TclError:
                pass


def positive_finite(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return float(value)


def load_ar_intrinsic(path: Path) -> ARIntrinsic:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        image = document["image"]
        matrix = np.asarray(document["camera_matrix"], dtype=np.float64)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid AR intrinsic JSON: {error}") from error
    if type(document) is not dict or type(image) is not dict or (image.get("width"), image.get("height")) != (AR_WIDTH, AR_HEIGHT):
        raise ValueError("AR intrinsic calibration resolution must be 1920x1200")
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("AR intrinsic camera_matrix must be finite 3x3")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], (0.0, 0.0, 1.0)):
        raise ValueError("AR intrinsic camera_matrix is invalid")
    cx_px, cy_px = float(matrix[0, 2]), float(matrix[1, 2])
    if not 0 <= cx_px < AR_WIDTH or not 0 <= cy_px < AR_HEIGHT:
        raise ValueError("AR intrinsic principal point is outside 1920x1200")
    return ARIntrinsic(cx_px, cy_px)


def classify_image_position(center_x_px: float, cx_px: float, tolerance_px: float) -> str:
    if not all(math.isfinite(value) for value in (center_x_px, cx_px, tolerance_px)) or tolerance_px <= 0:
        raise ValueError("center/cx/tolerance must be finite and tolerance positive")
    if abs(center_x_px - cx_px) <= tolerance_px:
        return "CENTER_BAND"
    return "IMAGE_LEFT" if center_x_px < cx_px else "IMAGE_RIGHT"


class RollingCenters:
    def __init__(self, window_size: int) -> None:
        if type(window_size) is not int or isinstance(window_size, bool) or window_size <= 0:
            raise ValueError("window_size must be a positive integer")
        self._values: deque[float] = deque(maxlen=window_size)

    def update(self, status: str, bbox_xyxy_px: list[int] | None) -> tuple[float | None, float | None]:
        if status != "SINGLE_PERSON" or bbox_xyxy_px is None:
            self._values.clear()
            return None, None
        if len(bbox_xyxy_px) != 4:
            self._values.clear()
            return None, None
        center = (float(bbox_xyxy_px[0]) + float(bbox_xyxy_px[2])) / 2.0
        if not math.isfinite(center):
            self._values.clear()
            return None, None
        self._values.append(center)
        return center, float(np.median(np.asarray(self._values, dtype=np.float64)))


def draw_overlay(
    frame: np.ndarray,
    *,
    intrinsic: ARIntrinsic,
    tolerance_px: float,
    status: str,
    bbox_xyxy_px: list[int] | None,
    center_x_px: float | None,
    median_center_x_px: float | None,
) -> np.ndarray:
    """Return an annotated copy; the captured frame is never modified."""
    view = frame.copy()
    cx = int(round(intrinsic.cx_px))
    low, high = int(round(intrinsic.cx_px - tolerance_px)), int(round(intrinsic.cx_px + tolerance_px))
    cv2.line(view, (cx, 0), (cx, view.shape[0] - 1), (0, 255, 255), 2)
    cv2.line(view, (low, 0), (low, view.shape[0] - 1), (0, 180, 0), 1)
    cv2.line(view, (high, 0), (high, view.shape[0] - 1), (0, 180, 0), 1)
    label = status if status in {"PERSON_LOST", "MULTIPLE_PERSONS"} else "PERSON_LOST"
    if status == "SINGLE_PERSON" and bbox_xyxy_px is not None and center_x_px is not None:
        x1, y1, x2, y2 = bbox_xyxy_px
        cv2.rectangle(view, (x1, y1), (x2, y2), (255, 160, 0), 2)
        label = classify_image_position(center_x_px, intrinsic.cx_px, tolerance_px)
        offset = center_x_px - intrinsic.cx_px
        cv2.putText(view, f"bbox center_x={center_x_px:.1f} offset={offset:+.1f}px", (24, 74), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
        if median_center_x_px is not None:
            cv2.putText(view, f"rolling median center_x={median_center_x_px:.1f}", (24, 106), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(view, f"{label} | optical axis cx={intrinsic.cx_px:.2f} | band +-{tolerance_px:.1f}px", (24, 38), cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 255, 0), 2, cv2.LINE_AA)
    return view


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--ar-intrinsic", type=Path, required=True)
    parser.add_argument("--center-tolerance-px", type=float, required=True)
    parser.add_argument("--device", type=Path, default=AR0234_BY_ID)
    parser.add_argument("--window-size", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.device != AR0234_BY_ID:
        raise ValueError("--device must be the exact approved AR0234 stable by-id path")
    if not args.model.is_absolute() or not args.reference.is_absolute() or not args.ar_intrinsic.is_absolute():
        raise ValueError("--model, --reference, and --ar-intrinsic must be absolute paths")
    if args.max_frames < 0:
        raise ValueError("--max-frames must be 0 or positive")
    tolerance = positive_finite(args.center_tolerance_px, "--center-tolerance-px")
    intrinsic = load_ar_intrinsic(args.ar_intrinsic)
    rolling = RollingCenters(args.window_size)
    detector = MPPersonDetOpenCV(args.model, args.reference)
    pipeline = PersonLocalizationPipeline(detector)
    capture = AR0234Capture(AR0234CaptureConfig(device=AR0234_BY_ID, width=AR_WIDTH, height=AR_HEIGHT, fps=30.0, fourcc="MJPG", buffer_size=1))
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
            result = pipeline.process(frame, captured_at_utc=datetime.now(timezone.utc), cycle_id=f"alignment-preview-{count:06d}")
            bbox = None if result.bounding_box is None else result.bounding_box.to_xyxy()
            center, median = rolling.update(result.status.value, bbox)
            view = draw_overlay(frame, intrinsic=intrinsic, tolerance_px=tolerance, status=result.status.value, bbox_xyxy_px=bbox, center_x_px=center, median_center_x_px=median)
            if preview.show(view):
                break
        return 0
    finally:
        capture.close()
        if preview is not None:
            preview.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
