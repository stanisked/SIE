"""Fail-closed local AR0234 dataset capture.  No detector is involved."""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
import platform
import re
import subprocess
import tempfile
import time
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol

import cv2
import numpy as np

from vision_core.person_localization.ar0234 import AR0234_BY_ID, VideoCaptureLike, resolve_ar0234_device

DEFAULT_DATASET_ROOT = Path("/home/stanislav/dev_ws/datasets/ar0234_person_semantic_v1")
PLAN_SCHEMA = "sie_ar0234_person_dataset_plan_v1"
RECORD_SCHEMA = "sie_ar0234_person_dataset_capture_record_v1"
MANIFEST_SCHEMA = "sie_ar0234_person_dataset_manifest_v1"
DATASET_ID = "ar0234_person_semantic_v1"
FRAME_SHAPE = (1200, 1920, 3)
STALE_FRAME_THRESHOLD_S = 1.0
MAX_REVIEW_AGE_S = 30.0
V4L2_TIMEOUT_S = 5.0
SCENARIO_ID = re.compile(r"[a-z][a-z0-9_]{0,79}\Z")
AUTO_EXPOSURE = re.compile(r"\Aauto_exposure: ([0-9]+)(?: \(([^()]*)\))?\Z")


class DatasetCaptureError(RuntimeError):
    pass


class ControlRunner(Protocol):
    def __call__(self, argv: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]: ...


class WorktreeProvider(Protocol):
    def __call__(self, repository: Path) -> tuple[Path, ...]: ...


@dataclass(frozen=True)
class Scenario:
    scenario_id: str; phase: str; required: bool; expected_person_count: int
    framing: str; nominal_distance_m: float | None; frame_position: str; orientation: str; instructions: str


@dataclass(frozen=True)
class DatasetPlan:
    schema_version: str; dataset_id: str; scenarios: tuple[Scenario, ...]


@dataclass(frozen=True)
class CameraMode:
    width: int = 1920; height: int = 1200; fps: float = 30.0; fourcc: str = "MJPG"; warmup_valid_frames: int = 60


@dataclass(frozen=True)
class CapturedFrame:
    pixels: np.ndarray; source_frame_at_utc: dt.datetime; source_frame_monotonic: float


def _pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    answer: dict[str, Any] = {}
    for key, value in pairs:
        if key in answer: raise DatasetCaptureError(f"duplicate JSON key: {key}")
        answer[key] = value
    return answer


def _bad_constant(value: str) -> None: raise DatasetCaptureError(f"non-finite JSON constant: {value}")
def _strict_load_bytes(payload: bytes, label: str) -> Any:
    try: return json.loads(payload.decode("utf-8"), object_pairs_hook=_pairs, parse_constant=_bad_constant)
    except (UnicodeDecodeError, json.JSONDecodeError) as error: raise DatasetCaptureError(f"cannot strictly read {label}: {error}") from error
def strict_json_load(path: Path) -> Any:
    try: return _strict_load_bytes(path.read_bytes(), str(path))
    except OSError as error: raise DatasetCaptureError(f"cannot read {path}: {error}") from error


_SCENARIO_KEYS = {"scenario_id", "phase", "required", "expected_person_count", "framing", "nominal_distance_m", "frame_position", "orientation", "instructions"}
def _obj(value: object, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys: raise DatasetCaptureError(f"{label} schema mismatch")
    return value
def _text(value: object, label: str) -> str:
    if type(value) is not str or not value: raise DatasetCaptureError(f"{label} must be non-empty string")
    return value
def _number(value: object, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value): raise DatasetCaptureError(f"{label} must be finite number")
    return float(value)
def _scenario(value: object) -> Scenario:
    item = _obj(value, _SCENARIO_KEYS, "scenario")
    identifier = _text(item["scenario_id"], "scenario_id")
    if not SCENARIO_ID.fullmatch(identifier): raise DatasetCaptureError("scenario_id is not sanitized")
    if item["phase"] not in {"A", "B"} or type(item["required"]) is not bool or (item["phase"] == "A") != item["required"]: raise DatasetCaptureError("invalid phase flags")
    count = item["expected_person_count"]
    if type(count) is not int or count not in {0, 1, 2}: raise DatasetCaptureError("expected_person_count must be 0, 1, or 2")
    distance = item["nominal_distance_m"]
    if distance is not None:
        distance = _number(distance, "nominal_distance_m")
        if distance <= 0: raise DatasetCaptureError("nominal distance must be positive")
    return Scenario(identifier, item["phase"], item["required"], count, _text(item["framing"], "framing"), distance, _text(item["frame_position"], "frame_position"), _text(item["orientation"], "orientation"), _text(item["instructions"], "instructions"))
def parse_plan_payload(value: object) -> DatasetPlan:
    item = _obj(value, {"schema_version", "dataset_id", "scenarios"}, "plan")
    if item["schema_version"] != PLAN_SCHEMA or item["dataset_id"] != DATASET_ID or type(item["scenarios"]) is not list: raise DatasetCaptureError("unsupported plan")
    scenarios = tuple(_scenario(one) for one in item["scenarios"])
    if not scenarios or not any(x.phase == "A" for x in scenarios) or len({x.scenario_id for x in scenarios}) != len(scenarios): raise DatasetCaptureError("invalid plan scenarios")
    return DatasetPlan(PLAN_SCHEMA, DATASET_ID, scenarios)
def load_plan(path: Path) -> DatasetPlan: return parse_plan_payload(strict_json_load(path))


def default_plan() -> DatasetPlan:
    def s(i: str, p: str, n: int, f: str, d: float | None, pos: str, o: str, text: str) -> Scenario: return Scenario(i, p, p == "A", n, f, d, pos, o, text)
    rows = [s("empty_center_01", "A", 0, "empty", None, "center", "none", "Keep the scene empty and centered."), s("empty_center_02", "A", 0, "empty", None, "center", "none", "Keep the scene empty and centered."), s("empty_lighting_variation_01", "A", 0, "empty", None, "center", "none", "Keep the scene empty with a lighting variation.")]
    rows += [s(f"full_{p}_1_5m", "A", 1, "full_body", 1.5, p, "front", "One person, full body visible.") for p in ("center", "left", "right")]
    rows += [s(f"full_{p}_2_5m", "A", 1, "full_body", 2.5, p, "front", "One person, full body visible.") for p in ("center", "left", "right")]
    rows += [s("partial_center_0_7m", "A", 1, "partial", .7, "center", "front", "One person, intentionally partial."), s("partial_center_1_0m", "A", 1, "partial", 1., "center", "front", "One person, intentionally partial."), s("partial_left_edge_1_0m", "A", 1, "partial", 1., "left_edge", "front", "One person at left edge."), s("partial_right_edge_1_0m", "A", 1, "partial", 1., "right_edge", "front", "One person at right edge."), s("orientation_front_1_5m", "A", 1, "full_body", 1.5, "center", "front", "One person facing camera."), s("orientation_side_1_5m", "A", 1, "full_body", 1.5, "center", "side", "One person side-on."), s("orientation_back_1_5m", "A", 1, "full_body", 1.5, "center", "back", "One person facing away."), s("multi_separated", "B", 2, "multiple", None, "separated", "mixed", "Two people separated."), s("multi_close", "B", 2, "multiple", None, "close", "mixed", "Two people close."), s("multi_partial_overlap", "B", 2, "multiple", None, "overlap", "mixed", "Two people partially overlap."), s("multi_different_depths", "B", 2, "multiple", None, "depth", "mixed", "Two people at different depths.")]
    return DatasetPlan(PLAN_SCHEMA, DATASET_ID, tuple(rows))


def _json(value: object) -> bytes: return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
def _exclusive(path: Path, payload: bytes, mode: int = 0o600) -> None:
    if path.exists() or path.is_symlink(): raise DatasetCaptureError(f"refusing overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True); fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent); temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as handle: handle.write(payload); handle.flush(); os.fsync(handle.fileno())
        os.chmod(temporary, mode); os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try: os.fsync(directory)
        finally: os.close(directory)
    except FileExistsError as error: raise DatasetCaptureError(f"refusing overwrite: {path}") from error
    finally:
        try: temporary.unlink()
        except FileNotFoundError: pass


def default_worktrees(repository: Path) -> tuple[Path, ...]:
    try: result = subprocess.run(["git", "-C", str(repository), "worktree", "list", "--porcelain"], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, timeout=5)
    except (OSError, subprocess.SubprocessError) as error: raise DatasetCaptureError(f"cannot discover Git worktrees: {error}") from error
    roots = tuple(Path(line[9:]).resolve() for line in result.stdout.splitlines() if line.startswith("worktree "))
    if not roots: raise DatasetCaptureError("Git worktree discovery returned no roots")
    return roots
def _repository_root() -> Path: return Path(__file__).resolve().parents[2]
def _outside_worktrees(path: Path, provider: WorktreeProvider = default_worktrees) -> None:
    if not path.is_absolute() or path.is_symlink() or ".git" in path.parts: raise DatasetCaptureError("path must be an absolute non-symlink external path")
    target = path.resolve()
    for root in provider(_repository_root()):
        try: target.relative_to(root.resolve())
        except ValueError: continue
        raise DatasetCaptureError(f"path must be outside registered worktree: {root}")
def write_default_plan(path: Path, *, worktree_provider: WorktreeProvider = default_worktrees) -> None:
    _outside_worktrees(path, worktree_provider); plan = default_plan()
    _exclusive(path, _json({"schema_version": plan.schema_version, "dataset_id": plan.dataset_id, "scenarios": [asdict(x) for x in plan.scenarios]}))
def _paths(root: Path, *, worktree_provider: WorktreeProvider = default_worktrees) -> dict[str, Path]:
    _outside_worktrees(root, worktree_provider)
    return {"root": root, "frames": root / "frames", "plan": root / "dataset_plan.json", "records": root / "capture_records.jsonl", "manifest": root / "dataset_manifest.json", "sums": root / "SHA256SUMS"}


def _frame(value: object) -> np.ndarray:
    if type(value) is not np.ndarray or value.dtype != np.uint8 or value.shape != FRAME_SHAPE or not value.size: raise DatasetCaptureError("frame must be non-empty uint8 1200x1920x3")
    return value
def _fourcc(value: float) -> str: return "".join(chr((int(value) >> (8 * i)) & 0xff) for i in range(4))
class DatasetCamera:
    def __init__(self, mode: CameraMode = CameraMode(), *, capture_factory: Callable[[str, int], VideoCaptureLike] = cv2.VideoCapture, device_resolver: Callable[[Path], Path] = resolve_ar0234_device, now_utc: Callable[[], dt.datetime] = lambda: dt.datetime.now(dt.timezone.utc), monotonic: Callable[[], float] = time.monotonic) -> None:
        self.mode, self._factory, self._resolver, self._utc, self._mono, self._cap, self.target = mode, capture_factory, device_resolver, now_utc, monotonic, None, None
    def open(self) -> dict[str, object]:
        if self._cap is not None: raise DatasetCaptureError("camera is already open")
        target = self._resolver(AR0234_BY_ID); cap = self._factory(str(target), cv2.CAP_V4L2)
        try:
            if not cap.isOpened(): raise DatasetCaptureError("AR0234 did not open")
            for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.mode.fourcc)), (cv2.CAP_PROP_FRAME_WIDTH, self.mode.width), (cv2.CAP_PROP_FRAME_HEIGHT, self.mode.height), (cv2.CAP_PROP_FPS, self.mode.fps), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                if not cap.set(prop, value): raise DatasetCaptureError(f"camera rejected property {prop}")
            actual = {"width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), "fps": float(cap.get(cv2.CAP_PROP_FPS)), "fourcc": _fourcc(cap.get(cv2.CAP_PROP_FOURCC))}
            if (actual["width"], actual["height"], actual["fourcc"]) != (1920, 1200, "MJPG") or abs(actual["fps"] - 30.0) > .5: raise DatasetCaptureError(f"camera mode mismatch: {actual}")
        except BaseException: cap.release(); raise
        self._cap, self.target = cap, target; return actual
    def read_timed(self) -> CapturedFrame:
        if self._cap is None: raise DatasetCaptureError("camera is not open")
        try:
            ok, value = self._cap.read()
            if not ok: raise DatasetCaptureError("camera read returned false")
            utc, mono = self._utc(), self._mono(); _frame(value)
            if utc.tzinfo is None or utc.utcoffset() != dt.timedelta(0) or not math.isfinite(mono): raise DatasetCaptureError("invalid source frame clock")
            return CapturedFrame(value, utc, mono)
        except BaseException: self.close(); raise
    def read(self) -> np.ndarray: return self.read_timed().pixels
    def close(self) -> None:
        cap, self._cap = self._cap, None
        if cap is not None: cap.release()


def default_control_runner(argv: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]: return subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False, timeout=timeout_s)
def _control_output(text: object) -> int:
    if type(text) is not str: raise DatasetCaptureError("malformed v4l2 output")
    rows = [x for x in text.splitlines() if x.strip()]
    if len(rows) != 1: raise DatasetCaptureError("cannot confirm auto_exposure")
    match = AUTO_EXPOSURE.fullmatch(rows[0].strip())
    if match is None or (match.group(2) is not None and not match.group(2)): raise DatasetCaptureError("cannot confirm auto_exposure")
    return int(match.group(1))
def set_auto_exposure(device: Path, runner: ControlRunner = default_control_runner) -> dict[str, int]:
    if device != AR0234_BY_ID: raise DatasetCaptureError("control requires exact by-id device")
    def call(argv: list[str], *, parse: bool = True) -> str | int:
        try: result = runner(argv, V4L2_TIMEOUT_S)
        except subprocess.TimeoutExpired as error: raise DatasetCaptureError("v4l2 control timed out") from error
        if result.returncode != 0: raise DatasetCaptureError(f"v4l2 control failed: {result.stdout}")
        return _control_output(result.stdout) if parse else result.stdout
    before = call(["v4l2-ctl", "--device", str(device), "--get-ctrl=auto_exposure"])
    if before not in (1, 3): raise DatasetCaptureError(f"unsupported auto_exposure before set: {before}")
    call(["v4l2-ctl", "--device", str(device), "--set-ctrl=auto_exposure=3"], parse=False)
    actual = call(["v4l2-ctl", "--device", str(device), "--get-ctrl=auto_exposure"])
    if actual != 3: raise DatasetCaptureError("auto_exposure confirmation failed")
    return {"before": int(before), "requested": 3, "actual": int(actual)}


SUBJECT_SOURCES = {"SELF_CAPTURE", "CONSENTED_VOLUNTEER", "NO_PERSON"}
def _subject_for(scenario: Scenario, source: object) -> str:
    if type(source) is not str or source not in SUBJECT_SOURCES: raise DatasetCaptureError("invalid subject source")
    if scenario.expected_person_count == 0:
        if source != "NO_PERSON": raise DatasetCaptureError("empty scenario requires NO_PERSON")
    elif source not in {"SELF_CAPTURE", "CONSENTED_VOLUNTEER"}: raise DatasetCaptureError("human scenario requires an approved subject source")
    return source
def validate_subject_source(value: object, plan: DatasetPlan) -> str:
    if type(value) is not str or value not in SUBJECT_SOURCES: raise DatasetCaptureError("invalid subject source")
    if any(x.expected_person_count > 0 for x in plan.scenarios):
        if value not in {"SELF_CAPTURE", "CONSENTED_VOLUNTEER"}: raise DatasetCaptureError("human plan requires an approved subject source")
    elif value != "NO_PERSON": raise DatasetCaptureError("empty-only plan requires NO_PERSON")
    return value
def frame_filename(sequence: int, scenario_id: str) -> str:
    if type(sequence) is not int or sequence < 1 or not SCENARIO_ID.fullmatch(scenario_id): raise DatasetCaptureError("invalid deterministic filename fields")
    return f"{sequence:04d}_{scenario_id}.png"


_RECORD_KEYS = {"schema_version", "dataset_id", "sequence", "scenario_id", "phase", "required", "expected_person_count", "framing", "nominal_distance_m", "frame_position", "orientation", "subject_source", "image_filename", "image_sha256", "png_bytes", "source_frame_at_utc", "accepted_at_utc", "source_frame_age_s", "tool_git_commit", "host_platform", "evidence_id", "camera_stable_by_id", "resolved_character_device", "requested_mode", "actual_mode", "camera_controls"}
def _utc(text: object, label: str) -> dt.datetime:
    if type(text) is not str: raise DatasetCaptureError(f"{label} must be UTC string")
    try: value = dt.datetime.fromisoformat(text)
    except ValueError as error: raise DatasetCaptureError(f"invalid {label}") from error
    if value.tzinfo is None or value.utcoffset() != dt.timedelta(0): raise DatasetCaptureError(f"{label} must be UTC")
    return value
def _finite_tree(value: object) -> None:
    """Reject non-finite numeric values anywhere in a declared record."""
    if type(value) is float and not math.isfinite(value): raise DatasetCaptureError("record contains non-finite number")
    if type(value) is dict:
        for child in value.values(): _finite_tree(child)
    elif type(value) is list:
        for child in value: _finite_tree(child)
def normalize_key(key: int) -> int:
    """Normalise only ASCII letter keys; SPACE and ESC retain their values."""
    return ord(chr(key).lower()) if ord("A") <= key <= ord("Z") else key
def _record_for_scenario(record: object, scenario: Scenario, sequence: int, paths: Mapping[str, Path]) -> dict[str, Any]:
    item = _obj(record, _RECORD_KEYS, "record")
    _finite_tree(item)
    if item["schema_version"] != RECORD_SCHEMA or item["dataset_id"] != DATASET_ID: raise DatasetCaptureError("record identity mismatch")
    if type(item["sequence"]) is not int or item["sequence"] != sequence: raise DatasetCaptureError("record sequence mismatch")
    comparisons = {"scenario_id": scenario.scenario_id, "phase": scenario.phase, "required": scenario.required, "expected_person_count": scenario.expected_person_count, "framing": scenario.framing, "nominal_distance_m": scenario.nominal_distance_m, "frame_position": scenario.frame_position, "orientation": scenario.orientation}
    if any(item[key] != value for key, value in comparisons.items()): raise DatasetCaptureError("record scenario fields differ from plan")
    _subject_for(scenario, item["subject_source"])
    filename = frame_filename(sequence, scenario.scenario_id)
    if item["image_filename"] != filename or Path(filename).name != filename: raise DatasetCaptureError("record filename mismatch")
    if type(item["png_bytes"]) is not int or item["png_bytes"] < 1 or type(item["image_sha256"]) is not str or not re.fullmatch(r"[0-9a-f]{64}", item["image_sha256"]): raise DatasetCaptureError("record image metadata invalid")
    age = _number(item["source_frame_age_s"], "source_frame_age_s")
    if age < 0 or age > MAX_REVIEW_AGE_S: raise DatasetCaptureError("record source frame age invalid")
    source, accepted = _utc(item["source_frame_at_utc"], "source_frame_at_utc"), _utc(item["accepted_at_utc"], "accepted_at_utc")
    if accepted < source: raise DatasetCaptureError("record accepted before source")
    for key in ("tool_git_commit", "host_platform", "evidence_id", "camera_stable_by_id", "resolved_character_device"):_text(item[key], key)
    if type(item["requested_mode"]) is not dict or type(item["actual_mode"]) is not dict or type(item["camera_controls"]) is not dict: raise DatasetCaptureError("record metadata invalid")
    target = paths["frames"] / filename
    if target.is_symlink() or not target.is_file() or target.parent != paths["frames"]: raise DatasetCaptureError("recorded frame unsafe or absent")
    payload = target.read_bytes()
    if len(payload) != item["png_bytes"] or hashlib.sha256(payload).hexdigest() != item["image_sha256"]: raise DatasetCaptureError("recorded frame SHA or size mismatch")
    decoded = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR); _frame(decoded)
    return item
def _records(paths: Mapping[str, Path], plan: DatasetPlan) -> dict[str, dict[str, Any]]:
    records_path = paths["records"]
    if not records_path.exists(): return {}
    if records_path.is_symlink() or not records_path.is_file(): raise DatasetCaptureError("unsafe records target")
    rows: dict[str, dict[str, Any]] = {}
    for raw in records_path.read_bytes().splitlines():
        item = _strict_load_bytes(raw, "JSONL record")
        if type(item) is not dict: raise DatasetCaptureError("record is not object")
        identifier = item.get("scenario_id")
        matches = [(i, x) for i, x in enumerate(plan.scenarios, 1) if x.scenario_id == identifier]
        if len(matches) != 1 or identifier in rows: raise DatasetCaptureError("unknown or duplicate record scenario")
        sequence, scenario = matches[0]; rows[identifier] = _record_for_scenario(item, scenario, sequence, paths)
    frames = paths["frames"]
    if frames.exists():
        if frames.is_symlink() or not frames.is_dir(): raise DatasetCaptureError("unsafe frames directory")
        if {x.name for x in frames.iterdir()} != {x["image_filename"] for x in rows.values()}: raise DatasetCaptureError("orphan frame requires manual recovery")
    return rows
def _append(path: Path, record: Mapping[str, object]) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | getattr(os, "O_NOFOLLOW", 0), 0o600)
    with os.fdopen(fd, "a") as handle: handle.write(json.dumps(dict(record), sort_keys=True, allow_nan=False) + "\n"); handle.flush(); os.fsync(handle.fileno())
def save_accepted_frame(paths: Mapping[str, Path], scenario: Scenario, sequence: int, frame: object, *, source_frame_at: dt.datetime, source_frame_monotonic: float, accepted_at: dt.datetime, accepted_monotonic: float, metadata: Mapping[str, object], subject_source: str, git_commit: str) -> dict[str, object]:
    _subject_for(scenario, subject_source)
    if sequence < 1 or source_frame_at.tzinfo is None or accepted_at.tzinfo is None or source_frame_at.utcoffset() != dt.timedelta(0) or accepted_at.utcoffset() != dt.timedelta(0): raise DatasetCaptureError("timestamps must be aware UTC")
    age = accepted_monotonic - source_frame_monotonic
    if not math.isfinite(age) or age < 0 or age > MAX_REVIEW_AGE_S: raise DatasetCaptureError("stale or invalid source frame")
    valid = _frame(frame); ok, encoded = cv2.imencode(".png", valid)
    if not ok: raise DatasetCaptureError("PNG encoding failed")
    payload, filename = encoded.tobytes(), frame_filename(sequence, scenario.scenario_id); target = paths["frames"] / filename; _exclusive(target, payload)
    record: dict[str, object] = {"schema_version": RECORD_SCHEMA, "dataset_id": DATASET_ID, "sequence": sequence, "scenario_id": scenario.scenario_id, "phase": scenario.phase, "required": scenario.required, "expected_person_count": scenario.expected_person_count, "framing": scenario.framing, "nominal_distance_m": scenario.nominal_distance_m, "frame_position": scenario.frame_position, "orientation": scenario.orientation, "subject_source": subject_source, "image_filename": filename, "image_sha256": hashlib.sha256(payload).hexdigest(), "png_bytes": len(payload), "source_frame_at_utc": source_frame_at.isoformat(), "accepted_at_utc": accepted_at.isoformat(), "source_frame_age_s": age, "tool_git_commit": git_commit, "host_platform": platform.platform(), "evidence_id": f"{DATASET_ID}:{sequence}:{scenario.scenario_id}", **dict(metadata)}
    _record_for_scenario(record, scenario, sequence, paths); _append(paths["records"], record); return record
def finalize_dataset(root: Path, *, worktree_provider: WorktreeProvider = default_worktrees) -> dict[str, object]:
    paths = _paths(root, worktree_provider=worktree_provider)
    if paths["manifest"].exists() or paths["sums"].exists(): raise DatasetCaptureError("dataset already finalized")
    plan, records = load_plan(paths["plan"]), _records(paths, load_plan(paths["plan"]))
    missing = [x.scenario_id for x in plan.scenarios if x.required and x.scenario_id not in records]
    if missing: raise DatasetCaptureError(f"Phase A incomplete: {missing}")
    phase_b = [x for x in plan.scenarios if not x.required]; status = "COMPLETE" if all(x.scenario_id in records for x in phase_b) else "PHASE_A_COMPLETE_PHASE_B_PENDING"
    manifest = {"schema_version": MANIFEST_SCHEMA, "dataset_id": DATASET_ID, "status": status, "plan_sha256": hashlib.sha256(paths["plan"].read_bytes()).hexdigest(), "records": [records[x.scenario_id] for x in plan.scenarios if x.scenario_id in records], "phase_b_completed": sum(x.scenario_id in records for x in phase_b), "phase_b_total": len(phase_b)}
    _exclusive(paths["manifest"], _json(manifest), 0o444); entries = [paths["plan"], paths["records"], paths["manifest"], *sorted(paths["frames"].glob("*.png"))]
    _exclusive(paths["sums"], "".join(f"{hashlib.sha256(x.read_bytes()).hexdigest()}  {x.relative_to(root)}\n" for x in entries).encode(), 0o444); return manifest


def capture_runtime(root: Path, *, device: Path, subject_source: str | None, capture_factory: Callable[[str, int], VideoCaptureLike] = cv2.VideoCapture, device_resolver: Callable[[Path], Path] = resolve_ar0234_device, control_runner: ControlRunner = default_control_runner, now_utc: Callable[[], dt.datetime] = lambda: dt.datetime.now(dt.timezone.utc), monotonic: Callable[[], float] = time.monotonic, git_commit: str = "unknown") -> None:
    if device != AR0234_BY_ID: raise DatasetCaptureError("capture requires exact approved by-id device")
    paths = _paths(root); plan = load_plan(paths["plan"]); source = validate_subject_source("NO_PERSON" if subject_source is None and not any(x.expected_person_count for x in plan.scenarios) else subject_source, plan); records = _records(paths, plan)
    camera = DatasetCamera(capture_factory=capture_factory, device_resolver=device_resolver, now_utc=now_utc, monotonic=monotonic)
    window_name = "AR0234 Person Dataset"
    try:
        actual = camera.open(); controls = set_auto_exposure(AR0234_BY_ID, control_runner)
        for _ in range(camera.mode.warmup_valid_frames): camera.read_timed()
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 960, 600)
        except cv2.error as error:
            raise DatasetCaptureError(f"cannot create capture preview window: {error}") from error
        for sequence, scenario in enumerate(plan.scenarios, 1):
            if scenario.scenario_id in records: continue
            print(f"SCENARIO {sequence}/{len(plan.scenarios)} {scenario.scenario_id}", flush=True)
            candidate: CapturedFrame | None = None
            while True:
                current = camera.read_timed() if candidate is None else candidate
                preview = current.pixels.copy()
                state = "LIVE" if candidate is None else "REVIEW"
                cv2.putText(preview, f"{state}  SCENARIO {sequence}/{len(plan.scenarios)}  {scenario.scenario_id}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
                cv2.putText(preview, f"persons={scenario.expected_person_count}  phase={scenario.phase}  [SPACE] freeze [A] save [R] live [S] skip [Q] stop", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
                try:
                    cv2.imshow(window_name, preview)
                    key = normalize_key(cv2.waitKey(1) & 0xff)
                except cv2.error as error:
                    raise DatasetCaptureError(f"capture preview UI failed: {error}") from error
                if key in (ord("q"), ord("Q"), 27):
                    print("CAPTURE_STOPPED", flush=True)
                    return
                if key == ord(" ") and candidate is None: candidate = CapturedFrame(current.pixels.copy(), current.source_frame_at_utc, current.source_frame_monotonic); continue
                if key in (ord("r"), ord("R")): candidate = None; continue
                if key in (ord("s"), ord("S")):
                    if scenario.required: raise DatasetCaptureError("required scenario cannot be skipped")
                    print(f"SKIPPED {sequence} {scenario.scenario_id}", flush=True)
                    break
                if key in (ord("a"), ord("A")) and candidate is not None:
                    try: save_accepted_frame(paths, scenario, sequence, candidate.pixels, source_frame_at=candidate.source_frame_at_utc, source_frame_monotonic=candidate.source_frame_monotonic, accepted_at=now_utc(), accepted_monotonic=monotonic(), metadata={"camera_stable_by_id": str(AR0234_BY_ID), "resolved_character_device": str(camera.target), "requested_mode": asdict(camera.mode), "actual_mode": actual, "camera_controls": controls}, subject_source="NO_PERSON" if scenario.expected_person_count == 0 else source, git_commit=git_commit)
                    except DatasetCaptureError as error:
                        print(f"REJECTED {sequence} {scenario.scenario_id} {error}", flush=True)
                        candidate = None
                        continue
                    print(f"SAVED {sequence} {scenario.scenario_id} {frame_filename(sequence, scenario.scenario_id)}", flush=True)
                    break
    except DatasetCaptureError:
        raise
    finally:
        camera.close()
        cv2.destroyAllWindows()
