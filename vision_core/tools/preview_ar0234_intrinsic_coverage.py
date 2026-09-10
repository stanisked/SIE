#!/usr/bin/env python3
"""Live-only AR0234 checkerboard coverage positioning preview; it saves nothing."""
from __future__ import annotations

import argparse
import base64
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_core.ar0234_intrinsic_coverage_preview import draw_preview_overlay, validate_target
from vision_core.ar0234_intrinsic_independent_validation import (
    AR_DEVICE,
    AR_FPS,
    AR_HEIGHT,
    AR_WIDTH,
    CHECKERBOARD_SIZE,
    DirectAr0234Camera,
    IntrinsicValidationError,
    SQUARE_SIZE_MM,
    _checkerboard,
)


WINDOW_NAME = "AR0234 Intrinsic Coverage Preview"


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
            raise RuntimeError("no GUI preview backend is available") from error

    def _mark_closed(self) -> None:
        self._closed = True

    def _on_key(self, event: Any) -> None:
        if event.keysym in {"Escape", "q", "Q"}:
            self._mark_closed()

    def show(self, image_bgr: np.ndarray) -> bool:
        if self._highgui:
            cv2.imshow(self._name, image_bgr)
            return (cv2.waitKey(1) & 0xff) in (ord("q"), ord("Q"), 27)
        if self._closed:
            return True
        rgb = np.ascontiguousarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        height, width = rgb.shape[:2]
        ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()
        try:
            photo = self._tk.PhotoImage(data=base64.b64encode(ppm).decode("ascii"), format="PPM")
            self._label.configure(image=photo)
            self._label.image = photo
            self._root.update_idletasks()
            self._root.update()
        except self._tk.TclError as error:
            raise RuntimeError("tkinter could not present the preview image") from error
        return self._closed

    def close(self) -> None:
        if self._highgui:
            cv2.destroyWindow(self._name)
        elif self._root is not None:
            try:
                self._root.destroy()
            except self._tk.TclError:
                pass


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-row", type=int, required=True)
    parser.add_argument("--target-column", type=int, required=True)
    parser.add_argument("--device", type=Path, default=AR_DEVICE)
    parser.add_argument("--width", type=int, default=AR_WIDTH)
    parser.add_argument("--height", type=int, default=AR_HEIGHT)
    parser.add_argument("--fps", type=int, default=AR_FPS)
    parser.add_argument("--pixel-format", default="MJPG")
    parser.add_argument("--preview-width", type=int, default=960)
    parser.add_argument("--board-cols", type=int, default=CHECKERBOARD_SIZE[0])
    parser.add_argument("--board-rows", type=int, default=CHECKERBOARD_SIZE[1])
    parser.add_argument("--square-size-mm", type=float, default=SQUARE_SIZE_MM)
    return parser.parse_args(argv)


def _validate_arguments(arguments: argparse.Namespace) -> None:
    validate_target(row=arguments.target_row, column=arguments.target_column)
    if arguments.device != AR_DEVICE or (arguments.width, arguments.height, arguments.fps, arguments.pixel_format) != (AR_WIDTH, AR_HEIGHT, AR_FPS, "MJPG"):
        raise ValueError("preview requires /dev/video4 MJPG 1920x1200 @30 to match independent capture")
    if arguments.preview_width < 320:
        raise ValueError("--preview-width must be at least 320")
    if (arguments.board_cols, arguments.board_rows) != CHECKERBOARD_SIZE:
        raise ValueError("preview requires a 9x6 inner-corner board")
    if arguments.square_size_mm != SQUARE_SIZE_MM:
        raise ValueError("--square-size-mm is display provenance and must be 24.5")


def main(argv: list[str] | None = None) -> int:
    arguments = parse_args(argv)
    _validate_arguments(arguments)
    camera = DirectAr0234Camera(AR_DEVICE)
    preview: PreviewWindow | None = None
    try:
        camera.open()
        for _ in range(30):
            camera.read()
        preview = PreviewWindow(WINDOW_NAME)
        while True:
            frame = camera.read().image
            view, _ = draw_preview_overlay(frame, corners=_checkerboard(frame), target_row=arguments.target_row, target_column=arguments.target_column, preview_width=arguments.preview_width)
            cv2.putText(view, f"board={arguments.board_cols}x{arguments.board_rows} | square={arguments.square_size_mm:g} mm", (24, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(view, f"board={arguments.board_cols}x{arguments.board_rows} | square={arguments.square_size_mm:g} mm", (24, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
            if preview.show(view):
                break
        return 0
    finally:
        camera.close()
        if preview is not None:
            preview.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, IntrinsicValidationError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
