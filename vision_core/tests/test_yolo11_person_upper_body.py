from __future__ import annotations

import json

import numpy as np

from vision_core.person_localization.yolo11_person_upper_body import (
    LetterboxTransform,
    PersonUpperBodyDetection,
    bbox_truncation_flags,
    build_preview_record,
    decode_yolo11_one_class_output,
    letterbox_bgr,
    nms_detections,
)


def test_letterbox_1920x1200_preserves_aspect_and_normalizes_bgr_to_rgb() -> None:
    frame = np.full((1200, 1920, 3), (10, 20, 30), dtype=np.uint8)
    tensor, transform = letterbox_bgr(frame)
    assert tensor.shape == (1, 3, 640, 640)
    assert transform.scale == 1 / 3
    assert (transform.pad_x, transform.pad_y) == (0, 120)
    np.testing.assert_allclose(
        tensor[0, :, 120, 0], [30 / 255, 20 / 255, 10 / 255], rtol=0.0, atol=1e-7
    )


def test_decoder_unletterboxes_one_class_yolo11_tensor() -> None:
    transform = LetterboxTransform(0.5, 0, 40, 640, 640, 1280, 1120)
    output = np.asarray([[[320.0], [320.0], [100.0], [200.0], [0.91]]], dtype=np.float32)
    detections = decode_yolo11_one_class_output(
        output, transform=transform, confidence_threshold=0.40
    )
    assert len(detections) == 1
    assert detections[0].bbox_xyxy_px == (540.0, 360.0, 740.0, 760.0)
    assert detections[0].center_x_px == 640.0
    assert detections[0].confidence == np.float32(0.91)


def test_nms_and_json_safe_visual_only_record() -> None:
    detections = nms_detections(
        [
            PersonUpperBodyDetection((100.0, 100.0, 300.0, 300.0), 0.9),
            PersonUpperBodyDetection((110.0, 110.0, 310.0, 310.0), 0.8),
            PersonUpperBodyDetection((700.0, 100.0, 800.0, 300.0), 0.7),
        ],
        iou_threshold=0.45,
    )
    assert [item.confidence for item in detections] == [0.9, 0.7]
    record = build_preview_record(
        model_sha256="a" * 64,
        frame_width=1920,
        frame_height=1200,
        confidence_threshold=0.40,
        detections=detections,
    )
    assert record["reference_frame"] == "ar0234_image_frame"
    assert record["detection_count"] == 2
    assert record["detections"][0]["truncated_left"] is False
    assert record["detections"][0]["truncated_right"] is False
    assert record["detections"][0]["truncated_top"] is False
    assert record["detections"][0]["truncated_bottom"] is False
    assert "pixels" not in record
    json.dumps(record, allow_nan=False)


def test_bbox_truncation_flags_describe_each_image_boundary() -> None:
    assert bbox_truncation_flags(
        (0.0, 12.0, 700.0, 1200.0), frame_width=1920, frame_height=1200
    ) == {
        "truncated_left": True,
        "truncated_right": False,
        "truncated_top": False,
        "truncated_bottom": True,
    }
    assert bbox_truncation_flags(
        (200.0, 0.0, 1920.0, 800.0), frame_width=1920, frame_height=1200
    ) == {
        "truncated_left": False,
        "truncated_right": True,
        "truncated_top": True,
        "truncated_bottom": False,
    }
