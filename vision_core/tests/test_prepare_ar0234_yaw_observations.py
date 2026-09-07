from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


SCRIPT = Path("vision_core/tools/prepare_ar0234_yaw_observations.py")
spec = importlib.util.spec_from_file_location("prepare_ar0234_yaw_observations", SCRIPT)
adapter = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = adapter
spec.loader.exec_module(adapter)


def raw_record(*, status: str = "SINGLE_PERSON", bbox: object = [1100., 300., 1300., 700.]) -> dict:
    return {"status": status, "timestamp": "2026-09-07T12:00:00+00:00", "bbox": bbox}


def live_cycle(*, status: str = "SUCCESS", person_status: str = "SINGLE_PERSON", bbox: object = [1100., 300., 1300., 700.]) -> dict:
    return {
        "schema_version": "sie.person_depth_live_cycle.v1", "status": status,
        "captured_at_utc": "2026-09-07T12:00:00+00:00",
        "person": {"status": person_status, "bbox_xyxy_px": bbox},
        "measurement": {"status": "SUCCESS", "person_evidence_id": "person-evidence-live-001"},
    }


def test_valid_live_cycle_uses_captured_timestamp_and_intrinsic_cx(tmp_path: Path):
    intrinsic = tmp_path / "ar_intrinsic.json"
    intrinsic.write_text(json.dumps({"K": [[2000., 0., 997.365537], [0., 2000., 600.], [0., 0., 1.]]}))
    result = adapter.prepare_person_depth_live_cycle_observation(
        live_cycle(), line_number=1, optical_axis_cx_px=adapter.load_optical_axis_cx(intrinsic), center_tolerance_px=40.,
    )
    assert result["person_status"] == "SINGLE_PERSON"
    assert result["image_offset_px"] == 1200. - 997.365537
    assert result["reference_frame"] == "ar0234_image_frame" and result["units"] == "px"
    assert result["timestamp"] == "2026-09-07T12:00:00+00:00"
    assert result["evidence_id"] == "person-evidence-live-001"
    json.dumps(result, allow_nan=False)


def test_lost_multiple_and_invalid_live_cycle_remain_offset_free():
    for raw in (live_cycle(status="PERSON_LOST", person_status="PERSON_LOST"), live_cycle(person_status="MULTIPLE_PERSONS"), live_cycle(bbox=[1., 2., 1., 5.])):
        result = adapter.prepare_person_depth_live_cycle_observation(raw, line_number=1, optical_axis_cx_px=997., center_tolerance_px=40.)
        assert "image_offset_px" not in result


def test_raw_localization_source_remains_compatible():
    result = adapter.prepare_observation(raw_record(), line_number=1, optical_axis_cx_px=997.365537, center_tolerance_px=40.)
    assert result["person_status"] == "SINGLE_PERSON"
    assert result["image_offset_px"] == 1200. - 997.365537
