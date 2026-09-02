from __future__ import annotations

import datetime as dt
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision_core.person_dataset.capture import (
    AR0234_BY_ID, DATASET_ID, FRAME_SHAPE, RECORD_SCHEMA, CameraMode,
    DatasetCamera, DatasetCaptureError, _paths, _records, default_plan,
    finalize_dataset, frame_filename, load_plan, parse_plan_payload,
    normalize_key, save_accepted_frame, set_auto_exposure, write_default_plan,
)

ROOT = Path(__file__).resolve().parents[2]
NOW = dt.datetime(2026, 9, 2, 12, tzinfo=dt.timezone.utc)
WORKTREES = lambda _: (ROOT, Path("/home/stanislav/dev_ws/sie_person_localization_mvp"), Path("/home/stanislav/dev_ws/sie_person_detector_benchmark"), Path("/home/stanislav/dev_ws/sie_mp_persondet_audit"), Path("/home/stanislav/dev_ws/sie_ar0234_person_dataset"))

def plan_payload():
    p = default_plan(); return {"schema_version": p.schema_version, "dataset_id": p.dataset_id, "scenarios": [x.__dict__ for x in p.scenarios]}
def prepare(root: Path):
    root.mkdir(); write_default_plan(root / "dataset_plan.json", worktree_provider=WORKTREES); return load_plan(root / "dataset_plan.json"), _paths(root, worktree_provider=WORKTREES)
def png(shape=FRAME_SHAPE, dtype=np.uint8):
    image = np.zeros(shape, dtype=dtype); ok, encoded = cv2.imencode(".png", image); assert ok; return encoded.tobytes(), image
def metadata(): return {"camera_stable_by_id": str(AR0234_BY_ID), "resolved_character_device": "/dev/video99", "requested_mode": {}, "actual_mode": {}, "camera_controls": {"requested": 3, "actual": 3}}
def save(paths, scenario, sequence, *, source="NO_PERSON", age=0.0):
    _, image = png(); return save_accepted_frame(paths, scenario, sequence, image, source_frame_at=NOW, source_frame_monotonic=10.0, accepted_at=NOW, accepted_monotonic=10.0 + age, metadata=metadata(), subject_source=source, git_commit="x")

def test_default_plan_counts_and_strict_schema():
    p = default_plan(); assert (sum(x.phase == "A" for x in p.scenarios), sum(x.phase == "B" for x in p.scenarios)) == (16, 4)
    assert [x.scenario_id for x in p.scenarios[3:9]] == ["upper_center_1_5m", "upper_left_1_5m", "upper_right_1_5m", "full_center_3_5m", "full_left_3_5m", "full_right_3_5m"]
    assert parse_plan_payload(plan_payload()) == p
    bad = plan_payload(); bad["scenarios"][0]["expected_person_count"] = True
    with pytest.raises(DatasetCaptureError): parse_plan_payload(bad)

@pytest.mark.parametrize("root", [ROOT / "x", Path("/home/stanislav/dev_ws/sie_person_localization_mvp/x"), Path("/home/stanislav/dev_ws/sie_person_detector_benchmark/x"), Path("/home/stanislav/dev_ws/sie_mp_persondet_audit/x")])
def test_all_registered_worktrees_are_rejected(root):
    with pytest.raises(DatasetCaptureError): _paths(root, worktree_provider=WORKTREES)
    with pytest.raises(DatasetCaptureError): write_default_plan(root / "plan.json", worktree_provider=WORKTREES)

def test_resume_requires_a_real_png_and_exact_record(tmp_path):
    plan, paths = prepare(tmp_path / "d"); record = save(paths, plan.scenarios[0], 1); assert _records(paths, plan)
    target = paths["frames"] / record["image_filename"]; target.write_bytes(b"not-a-png")
    record["image_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest(); record["png_bytes"] = target.stat().st_size
    paths["records"].write_text(json.dumps(record) + "\n")
    with pytest.raises(DatasetCaptureError): _records(paths, plan)

@pytest.mark.parametrize("mutator", [
    lambda r: r.update(sequence=2), lambda r: r.update(image_filename="wrong.png"),
    lambda r: r.pop("phase"), lambda r: r.update(extra=1),
    lambda r: r.update(framing="multiple"), lambda r: r.update(expected_person_count=True),
])
def test_record_schema_sequence_filename_and_plan_fields_fail_closed(tmp_path, mutator):
    plan, paths = prepare(tmp_path / "d"); record = save(paths, plan.scenarios[0], 1); mutator(record)
    paths["records"].write_text(json.dumps(record) + "\n")
    with pytest.raises(DatasetCaptureError): _records(paths, plan)

def test_duplicate_json_key_rejected(tmp_path):
    plan, paths = prepare(tmp_path / "d"); record = save(paths, plan.scenarios[0], 1)
    paths["records"].write_text('{"scenario_id":"x","scenario_id":"%s"}\n' % record["scenario_id"])
    with pytest.raises(DatasetCaptureError, match="duplicate JSON key"): _records(paths, plan)

def test_png_dimension_failure_is_rejected(tmp_path):
    plan, paths = prepare(tmp_path / "d"); record = save(paths, plan.scenarios[0], 1); data, _ = png((4, 4, 3)); target = paths["frames"] / record["image_filename"]; target.write_bytes(data)
    record.update(image_sha256=hashlib.sha256(data).hexdigest(), png_bytes=len(data)); paths["records"].write_text(json.dumps(record) + "\n")
    with pytest.raises(DatasetCaptureError): _records(paths, plan)

def test_stale_monotonic_candidate_creates_no_png_or_record(tmp_path):
    plan, paths = prepare(tmp_path / "d")
    with pytest.raises(DatasetCaptureError): save(paths, plan.scenarios[0], 1, age=30.1)
    assert not paths["frames"].exists() and not paths["records"].exists()

@pytest.mark.parametrize("age", [1.5, 30.0])
def test_practical_review_age_is_accepted(tmp_path, age):
    plan, paths = prepare(tmp_path / "d")
    record = save(paths, plan.scenarios[0], 1, age=age)
    assert record["source_frame_age_s"] == age

def test_review_rejection_is_reported_and_returns_to_live():
    source = (ROOT / "vision_core/person_dataset/capture.py").read_text()
    assert 'print(f"REJECTED {sequence} {scenario.scenario_id} {error}"' in source
    assert 'candidate = None' in source
    assert 'continue' in source[source.index('print(f"REJECTED'):source.index('print(f"REJECTED') + 220]

def test_interactive_key_normalisation_keeps_space_and_escape():
    assert [normalize_key(x) for x in (ord("a"), ord("A"), ord("r"), ord("R"), ord("s"), ord("S"), ord("q"), ord("Q"), ord(" "), 27)] == [ord("a"), ord("a"), ord("r"), ord("r"), ord("s"), ord("s"), ord("q"), ord("q"), ord(" "), 27]

def test_subject_source_is_checked_at_save_resume_and_finalization(tmp_path):
    plan, paths = prepare(tmp_path / "d"); human = plan.scenarios[3]
    with pytest.raises(DatasetCaptureError): save(paths, human, 4, source="NO_PERSON")
    # A syntactically complete but policy-invalid human record also blocks resume.
    good = save(paths, plan.scenarios[0], 1); good["expected_person_count"] = 1; good["subject_source"] = "NO_PERSON"; paths["records"].write_text(json.dumps(good) + "\n")
    with pytest.raises(DatasetCaptureError): _records(paths, plan)
    with pytest.raises(DatasetCaptureError): finalize_dataset(paths["root"], worktree_provider=WORKTREES)

class FakeCap:
    def __init__(self): self.releases = 0
    def isOpened(self): return True
    def set(self, *_): return True
    def get(self, prop): return {cv2.CAP_PROP_FRAME_WIDTH: 1920., cv2.CAP_PROP_FRAME_HEIGHT: 1200., cv2.CAP_PROP_FPS: 30., cv2.CAP_PROP_FOURCC: float(cv2.VideoWriter_fourcc(*"MJPG"))}[prop]
    def read(self): return True, np.zeros(FRAME_SHAPE, np.uint8)
    def release(self): self.releases += 1
def test_second_camera_open_fails_without_replacing_first():
    cap = FakeCap(); camera = DatasetCamera(capture_factory=lambda *_: cap, device_resolver=lambda _: Path("/dev/video99"))
    camera.open()
    with pytest.raises(DatasetCaptureError): camera.open()
    assert camera._cap is cap; camera.close(); assert cap.releases == 1

@pytest.mark.parametrize("stdout", ["", "garbage\n", "auto_exposure: 2\n", "auto_exposure: 3\nextra\n"])
def test_v4l2_parser_rejects_bad_output(stdout):
    def runner(argv, timeout): return subprocess.CompletedProcess(argv, 0, stdout)
    with pytest.raises(DatasetCaptureError): set_auto_exposure(AR0234_BY_ID, runner)
def test_v4l2_uses_timeout_and_accepts_real_descriptor():
    seen=[]
    def runner(argv, timeout):
        seen.append((argv, timeout))
        output = "" if "set-ctrl" in " ".join(argv) else "auto_exposure: 3 (Aperture Priority Mode)\n"
        return subprocess.CompletedProcess(argv, 0, output)
    assert set_auto_exposure(AR0234_BY_ID, runner) == {"before": 3, "requested": 3, "actual": 3} and all(x[1] == 5.0 for x in seen)
def test_v4l2_before_one_empty_set_after_three():
    values = iter(["auto_exposure: 1 (Manual Mode)\n", "", "auto_exposure: 3\n"])
    def runner(argv, timeout): return subprocess.CompletedProcess(argv, 0, next(values))
    assert set_auto_exposure(AR0234_BY_ID, runner)["before"] == 1
def test_v4l2_after_one_fails():
    values = iter(["auto_exposure: 1\n", "", "auto_exposure: 1\n"])
    def runner(argv, timeout): return subprocess.CompletedProcess(argv, 0, next(values))
    with pytest.raises(DatasetCaptureError): set_auto_exposure(AR0234_BY_ID, runner)
def test_v4l2_nonzero_set_fails():
    values = iter(["auto_exposure: 1\n", "failed\n"])
    def runner(argv, timeout): return subprocess.CompletedProcess(argv, 1 if "set-ctrl" in " ".join(argv) else 0, next(values))
    with pytest.raises(DatasetCaptureError): set_auto_exposure(AR0234_BY_ID, runner)
def test_v4l2_timeout_is_fail_closed():
    def runner(argv, timeout): raise subprocess.TimeoutExpired(argv, timeout)
    with pytest.raises(DatasetCaptureError): set_auto_exposure(AR0234_BY_ID, runner)

def test_help_does_not_do_runtime_work():
    result = subprocess.run([sys.executable, "vision_core/tools/capture_ar0234_person_dataset.py", "--help"], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0 and "--subject-source" in result.stdout

def test_interactive_preview_contract_is_explicit_and_safe():
    source = (ROOT / "vision_core/person_dataset/capture.py").read_text()
    assert 'cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)' in source
    assert 'cv2.resizeWindow(window_name, 960, 600)' in source
    assert 'cv2.imshow(window_name, preview)' in source
    assert source.index('cv2.imshow(window_name, preview)') < source.index('cv2.waitKey(1)')
    assert 'preview = current.pixels.copy()' in source
    assert 'state = "LIVE" if candidate is None else "REVIEW"' in source
    assert 'print(f"SCENARIO' in source and 'print(f"SAVED' in source
    assert 'print(f"SKIPPED' in source and 'print("CAPTURE_STOPPED"' in source
    assert 'cv2.destroyAllWindows()' in source

def test_static_boundary_no_forbidden_integrations():
    text = (ROOT / "vision_core/person_dataset/capture.py").read_text().lower()
    for forbidden in ("onnx", "torch", "requests", "socket", "stereo", "esp32", "motor", "person_detector_benchmark"):
        assert forbidden not in text
