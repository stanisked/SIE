from __future__ import annotations

import importlib.util
import json
from pathlib import Path


def _audit_module():
    tool = Path(__file__).parents[1] / "tools" / "audit_ar0234_close_range_labelme_annotations.py"
    spec = importlib.util.spec_from_file_location("labelme_audit", tool)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_relative_image_path_and_float_zero_boundary_are_valid(tmp_path: Path) -> None:
    module = _audit_module()
    annotation_dir = tmp_path / "annotations_labelme"
    images_dir = tmp_path / "images"
    annotation_dir.mkdir()
    images_dir.mkdir()
    image = images_dir / "frame.png"
    image.touch()
    annotation = annotation_dir / "frame.json"
    annotation.write_text(
        json.dumps(
            {
                "imagePath": "../images/frame.png",
                "imageWidth": 1920,
                "imageHeight": 1200,
                "shapes": [
                    {
                        "label": "person_upper_body",
                        "shape_type": "rectangle",
                        "points": [[1000.0, -2.842170943040401e-14], [500.0, 1200.0]],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    record = module._parse_annotation(annotation, image)
    assert record["issues"] == []
    assert record["bbox_xyxy_px"] == [500.0, 0.0, 1000.0, 1200.0]
    assert record["x_point_order_reversed"] is True
    assert record["touches_top"] is True
    assert record["touches_bottom"] is True
