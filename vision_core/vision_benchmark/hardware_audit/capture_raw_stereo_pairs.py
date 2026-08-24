#!/usr/bin/env python3

"""
Fast RAW Stereo Pair Capture
SIE Vision Core

The script performs only:
    capture
    split
    preview
    save

No checkerboard detection, rectification, filtering or resizing
is applied to saved images.

Controls:
    SPACE / S  save current stereo pair
    Q / ESC    quit
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path


import cv2
import numpy as np
import os
import subprocess

def save_metadata(
    path: Path,
    metadata: dict,
) -> None:
    path.write_text(
        json.dumps(
            metadata,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fast RAW stereo calibration capture"
    )

    parser.add_argument(
        "--device",
        default="/dev/video2",
    )

    parser.add_argument(
        "--auto_exposure",
        type=int,
        choices=(1, 3),
        default=3,
    )

    parser.add_argument(
        "--output_dir",
        default="vision_core/vision_benchmark/hardware_audit/stereo_calibration_v5/raw_pairs",
    )

    parser.add_argument(
        "--combined_width",
        type=int,
        default=2560,
    )

    parser.add_argument(
        "--combined_height",
        type=int,
        default=800,
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=60,
    )

    parser.add_argument(
        "--preview_width",
        type=int,
        default=960,
    )

    parser.add_argument(
        "--warmup_frames",
        type=int,
        default=60,
    )

    args = parser.parse_args()

    if args.combined_width % 2 != 0:
        raise ValueError(
            "combined_width must be divisible by 2"
        )

    single_width = args.combined_width // 2
    single_height = args.combined_height

    output_dir = Path(args.output_dir)
    left_dir = output_dir / "left"
    right_dir = output_dir / "right"

    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    metadata_path = output_dir / "capture_metadata.json"

    exposure_command = [
        "v4l2-ctl",
        "-d",
        args.device,
        f"--set-ctrl=auto_exposure={args.auto_exposure}",
    ]

    subprocess.run(
        exposure_command,
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )

    exposure_check = subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            args.device,
            "--get-ctrl=auto_exposure",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )

    print(
        "Camera control:",
        exposure_check.stdout.strip(),
    )

    capture = cv2.VideoCapture(
        args.device,
        cv2.CAP_V4L2,
    )

    if not capture.isOpened():
        raise RuntimeError(
            f"Cannot open camera: {args.device}"
        )

    capture.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG"),
    )

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        args.combined_width,
    )

    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        args.combined_height,
    )

    capture.set(
        cv2.CAP_PROP_FPS,
        args.fps,
    )

    capture.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    actual_width = int(
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )

    actual_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    actual_fps = float(
        capture.get(cv2.CAP_PROP_FPS)
    )

    actual_fourcc_int = int(
        capture.get(cv2.CAP_PROP_FOURCC)
    )

    actual_fourcc = "".join(
        chr(
            (actual_fourcc_int >> 8 * index)
            & 0xFF
        )
        for index in range(4)
    )

    print()
    print("Fast RAW Stereo Capture")
    print("-----------------------")
    print("Device:", args.device)
    print(
        "Requested:",
        f"{args.combined_width}x{args.combined_height}",
        f"@ {args.fps} FPS",
    )
    print(
        "Actual:",
        f"{actual_width}x{actual_height}",
        f"@ {actual_fps:.2f} FPS",
        actual_fourcc,
    )
    print()
    print("SPACE / S: save pair")
    print("Q / ESC: quit")
    print()

    if (
        actual_width != args.combined_width
        or actual_height != args.combined_height
    ):
        capture.release()

        raise RuntimeError(
            "Camera did not accept requested resolution. "
            f"Actual: {actual_width}x{actual_height}"
        )

    for _ in range(args.warmup_frames):
        capture.read()

    records: list[dict] = []

    existing_left = sorted(
        left_dir.glob("pair_*.png")
    )

    pair_index = len(existing_left)

    last_time = time.perf_counter()
    displayed_frames = 0
    display_fps = 0.0

    while True:
        ok, combined = capture.read()

        if not ok or combined is None:
            print("Frame read failed")
            continue

        height, width = combined.shape[:2]

        if (
            width != args.combined_width
            or height != args.combined_height
        ):
            print(
                "Unexpected frame size:",
                f"{width}x{height}",
            )
            continue

        right = combined[
            :single_height,
            :single_width,
        ]

        left = combined[
            :single_height,
            single_width:single_width * 2,
        ]

        displayed_frames += 1

        current_time = time.perf_counter()
        elapsed = current_time - last_time

        if elapsed >= 1.0:
            display_fps = (
                displayed_frames / elapsed
            )

            displayed_frames = 0
            last_time = current_time

        preview_height = int(
            args.preview_width
            * single_height
            / args.combined_width
        )

        preview = cv2.resize(
            combined,
            (
                args.preview_width,
                preview_height,
            ),
            interpolation=cv2.INTER_AREA,
        )

        cv2.putText(
            preview,
            (
                f"saved={pair_index} "
                f"display_fps={display_fps:.1f}"
            ),
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            preview,
            "SPACE/S: save    Q/ESC: quit",
            (20, preview.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        center_x = preview.shape[1] // 2

        cv2.line(
            preview,
            (center_x, 0),
            (center_x, preview.shape[0]),
            (0, 255, 255),
            1,
        )

        cv2.imshow(
            "Fast RAW Stereo Capture",
            preview,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (
            27,
            ord("q"),
            ord("Q"),
        ):
            break

        if key in (
            32,
            ord("s"),
            ord("S"),
        ):
            filename = (
                f"pair_{pair_index:03d}.png"
            )

            left_path = left_dir / filename
            right_path = right_dir / filename

            left_saved = cv2.imwrite(
                str(left_path),
                left,
                [
                    cv2.IMWRITE_PNG_COMPRESSION,
                    1,
                ],
            )

            right_saved = cv2.imwrite(
                str(right_path),
                right,
                [
                    cv2.IMWRITE_PNG_COMPRESSION,
                    1,
                ],
            )

            if not left_saved or not right_saved:
                raise RuntimeError(
                    f"Cannot save pair {pair_index}"
                )

            record = {
                "pair_index": pair_index,
                "filename": filename,
                "left": str(left_path),
                "right": str(right_path),
                "captured_at": datetime.now(
                    timezone.utc
                ).isoformat(),
            }

            records.append(record)

            print(
                f"Saved pair {pair_index:03d}"
            )

            pair_index += 1

    capture.release()
    cv2.destroyAllWindows()

    previous_records = []

    if metadata_path.exists():
        try:
            previous_metadata = json.loads(
                metadata_path.read_text(
                    encoding="utf-8"
                )
            )

            previous_records = list(
                previous_metadata.get(
                    "pairs",
                    [],
                )
            )
        except Exception:
            previous_records = []

    metadata = {
        "calibration_id":
            "stereo_calibration_v5",

        "experiment":
            "H0.5b_Persistent_Geometry_Recalibration",

        "runtime_camera_order": {
            "combined_left_half":
                "physical_right",
            "combined_right_half":
                "physical_left",
        },

        "saved_image_semantics": {
            "left":
                "physical_left",
            "right":
                "physical_right",
        },

        "target": {
            "type":
                "checkerboard",
            "inner_corners": [
                9,
                6,
            ],
            "square_size_mm":
                24.0,
            "flat":
                True,
        },

        "optics": {
            "focus_adjusted":
                True,
            "focus_locked":
                True,
            "lenses_changed":
                False,
        },

        "physical_baseline_mm":
            65.10,

        "previous_calibration":
            "vision_core/vision_benchmark/hardware_audit/"
            "stereo_calibration_v4/solution_joint_refine_all_pairs/"
            "stereo_params_v4.npz",

        "previous_calibration_sha256":
            "2bc6682f5ebb2eb5362cf1e61cc77f241ffef83896c427b9eac87898577580da",

        "previous_calibration_status":
            "rejected_runtime_rectification",

        "recalibration_reason":
            "rectified_vertical_residual_12_65_px_persistent_geometry_change",

        "mechanical_stability_check": {
            "status": "PASS",
            "maximum_dy_change_px": 0.010620,
            "acceptance_limit_px": 0.25,
        },
        "thermal_stabilization": {
            "status":
                "PASS",
            "mode":
                "thermally_stabilized_operating_condition",
            "reference_window": {
                "started_at":
                    "2026-08-05T14:04:21+03:00",
                "finished_at":
                    "2026-08-05T14:29:51+03:00",
                "duration_seconds":
                    1530,
                "ambient_change_c":
                    0.0,
                "camera_left_change_c":
                    0.125,
                "camera_right_change_c":
                    0.250,
                "inter_camera_differential_change_c":
                    0.125,
            },
            "reference_temperatures_c": {
                "ambient":
                    26.875,
                "camera_left":
                    33.875,
                "camera_right":
                    33.8125,
            },
            "acceptance_limits_c": {
                "maximum_camera_change":
                    0.250,
                "maximum_differential_change":
                    0.125,
            },
            "temperature_channels": {
                "camera_left": {
                    "sensor_id":
                        "S06",
                    "rom":
                        "2824734EEC367B81",
                },
                "camera_right": {
                    "sensor_id":
                        "S02",
                    "rom":
                        "28FE6BA3299C9AC4",
                },
                "ambient": {
                    "sensor_id":
                        "S07",
                    "rom":
                        "28690845A8D094EF",
                    "distance_from_camera_cm":
                        10,
                },
            },
            "pre_capture_snapshot_required":
                True,
            "post_capture_snapshot_required":
                True,
        },

        "dataset_name":
            "stereo_calibration_v5_raw",

        "created_at":
            datetime.now(
                timezone.utc
            ).isoformat(),

        "device":
            args.device,

        "camera_controls": {
            "auto_exposure":
                args.auto_exposure,
        },

        "requested_capture": {
            "combined_resolution": [
                args.combined_width,
                args.combined_height,
            ],
            "single_resolution": [
                single_width,
                single_height,
            ],
            "fps":
                args.fps,
            "pixel_format":
                "MJPG",
        },

        "actual_capture": {
            "combined_resolution": [
                actual_width,
                actual_height,
            ],
            "single_resolution": [
                actual_width // 2,
                actual_height,
            ],
            "fps":
                actual_fps,
            "pixel_format":
                actual_fourcc,
        },

        "processing": {
            "rectification_applied":
                False,
            "resize_applied_to_saved_images":
                False,
            "crop_applied":
                False,
            "filtering_applied":
                False,
        },

        "num_pairs_total":
            len(
                previous_records
                + records
            ),

        "pairs":
            previous_records
            + records,
    }

    save_metadata(
        metadata_path,
        metadata,
    )

    print()
    print("Capture completed")
    print(
        "New pairs:",
        len(records),
    )
    print(
        "Total pairs:",
        metadata[
            "num_pairs_total"
        ],
    )
    print(
        "Metadata:",
        metadata_path,
    )


if __name__ == "__main__":
    main()
