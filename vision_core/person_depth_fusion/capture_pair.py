"""Capture one fresh AR0234 and OV9281 combined frame pair.

This is a raw-input utility only.  It deliberately performs no detector,
rectification, depth processing, ROS, network, or motor operation.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import os
import stat
import tempfile
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from vision_core.rgb_stereo_extrinsic.capture import (
    AR0234_BY_ID,
    AR_MODE,
    STEREO_BY_ID,
    STEREO_MODE,
    CheckedCamera,
    ControlRunner,
    ExtrinsicCaptureError,
    set_control,
)

AR_SHAPE = (1200, 1920, 3)
STEREO_SHAPE = (800, 2560, 3)
AR_INTRINSIC_DEFAULT = Path("/home/stanislav/sie_rgb_stereo_fusion/ar0234_intrinsic/dataset_v3_daylight/ar0234_intrinsic_fullres_v3/calibration_fullres.json")
STEREO_CALIBRATION_DEFAULT = Path("/home/stanislav/sie_rgb_stereo_fusion/stereo_calibration_v6/solution_joint_refine_corner_order_filtered_freeze_v2_run07/stereo_params_v6.npz")
DEFAULT_OUTPUT_ROOT = Path("/home/stanislav/dev_ws/datasets/person_depth_pair_2_0m_v1")
MAX_PAIR_SKEW_S = 0.050
WARMUP_READS = 60


class PersonDepthPairCaptureError(RuntimeError):
    """Fail-closed capture error with a stable machine-readable status."""

    def __init__(self, status: str, detail: str):
        self.status = status
        self.detail = detail
        super().__init__(f"{status}: {detail}")


def _principal_points(ar_path: Path, stereo_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    try:
        ar_data = json.loads(ar_path.read_text())
        ar_k = np.asarray(ar_data["camera_matrix"], dtype=np.float64)
        ar_size = ar_data["image"]
        ar_id = ar_data.get("calibration_id", ar_data.get("schema", "unknown"))
        stereo = np.load(stereo_path, allow_pickle=False)
        k1 = np.asarray(stereo["K1"], dtype=np.float64)
        size = np.asarray(stereo["size"]).reshape(-1).tolist()
        stereo_id_raw = stereo["calibration_id"]
        stereo_id = str(stereo_id_raw.item() if hasattr(stereo_id_raw, "item") else stereo_id_raw)
    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as error:
        raise PersonDepthPairCaptureError("CALIBRATION_INVALID", str(error)) from error
    if ar_k.shape != (3, 3) or k1.shape != (3, 3) or tuple(size) != (1280, 800):
        raise PersonDepthPairCaptureError("CALIBRATION_INVALID", "expected AR 1920x1200 and stereo 1280x800 3x3 intrinsics")
    if (ar_size.get("width"), ar_size.get("height")) != (1920, 1200):
        raise PersonDepthPairCaptureError("CALIBRATION_INVALID", "AR calibration resolution is not 1920x1200")
    points = ((float(ar_k[0, 2]), float(ar_k[1, 2])), (float(k1[0, 2]), float(k1[1, 2])))
    if not all(np.isfinite(v) for point in points for v in point):
        raise PersonDepthPairCaptureError("CALIBRATION_INVALID", "principal point is not finite")
    if not (0 <= points[0][0] < 1920 and 0 <= points[0][1] < 1200 and 0 <= points[1][0] < 1280 and 0 <= points[1][1] < 800):
        raise PersonDepthPairCaptureError("CALIBRATION_INVALID", "principal point is outside image")
    return ({"id": str(ar_id), "path": str(ar_path), "sha256": _sha256_bytes(ar_path.read_bytes()), "cx": points[0][0], "cy": points[0][1]},
            {"id": stereo_id, "path": str(stereo_path), "sha256": _sha256_bytes(stereo_path.read_bytes()), "cx": points[1][0], "cy": points[1][1]})


def _axis_preview(frame: np.ndarray, point: dict[str, Any], label: str) -> np.ndarray:
    """Return an annotated copy; the captured frame is never touched."""
    out = frame.copy()
    cv2.line(out, (int(round(point["cx"])), 0), (int(round(point["cx"])), out.shape[0] - 1), (0, 255, 255), 2)
    cv2.line(out, (0, int(round(point["cy"]))), (out.shape[1] - 1, int(round(point["cy"]))), (0, 255, 255), 2)
    cv2.drawMarker(out, (int(round(point["cx"])), int(round(point["cy"]))), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(out, f"{label} cx={point['cx']:.2f} cy={point['cy']:.2f}", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2, cv2.LINE_AA)
    return out


def _show_positioning(ar_frame: np.ndarray, combined: np.ndarray, ar_cal: dict[str, Any], stereo_cal: dict[str, Any], *, state: str) -> None:
    try:
        left = combined[:, 1280:]
        ar_view = _axis_preview(ar_frame, ar_cal, "AR0234")
        left_view = _axis_preview(left, stereo_cal, "OV9281 physical-left")
        def fit(image: np.ndarray) -> np.ndarray:
            scale = min(640.0 / image.shape[1], 600.0 / image.shape[0])
            resized = cv2.resize(image, (max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale))), interpolation=cv2.INTER_AREA)
            return resized
        ar_view, left_view = fit(ar_view), fit(left_view)
        panel_height = max(ar_view.shape[0], left_view.shape[0])
        def pad(image: np.ndarray) -> np.ndarray:
            top = (panel_height - image.shape[0]) // 2
            return cv2.copyMakeBorder(image, top, panel_height - image.shape[0] - top, 0, 0, cv2.BORDER_CONSTANT)
        ar_view, left_view = pad(ar_view), pad(left_view)
        panel = np.hstack((ar_view, left_view))
        cv2.putText(panel, state, (18, panel_height - 18), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Person Depth Pair", panel)
    except PersonDepthPairCaptureError:
        raise
    except Exception as error:
        raise PersonDepthPairCaptureError("PREVIEW_UI_ERROR", f"preview update failed: {error}") from error


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _validate_root(root: Path) -> Path:
    if not root.is_absolute():
        raise PersonDepthPairCaptureError("INVALID_OUTPUT_ROOT", "output root must be absolute")
    root = root.resolve()
    if root.exists() and not root.is_dir():
        raise PersonDepthPairCaptureError("INVALID_OUTPUT_ROOT", "output root is not a directory")
    if root.exists() and any(root.iterdir()):
        raise PersonDepthPairCaptureError("OUTPUT_NOT_EMPTY", f"refusing non-empty output root: {root}")
    # A dataset path must not be nested in a Git checkout/worktree.
    worktrees: list[Path] = []
    git_dir = Path(__file__).resolve().parents[2] / ".git"
    try:
        lines = __import__("subprocess").run(
            ["git", "worktree", "list", "--porcelain"], cwd=git_dir.parent,
            text=True, stdout=__import__("subprocess").PIPE,
            stderr=__import__("subprocess").DEVNULL, check=False,
        ).stdout.splitlines()
        worktrees = [Path(line.split(" ", 1)[1]).resolve() for line in lines if line.startswith("worktree ")]
    except OSError:
        pass
    if any(root == worktree or worktree in root.parents for worktree in worktrees):
        raise PersonDepthPairCaptureError("INVALID_OUTPUT_ROOT", "output root is inside a Git worktree")
    root.mkdir(parents=True, exist_ok=True)
    return root


def _frame(frame: object, expected: tuple[int, int, int], label: str) -> np.ndarray:
    if type(frame) is not np.ndarray or frame.dtype != np.uint8 or frame.shape != expected:
        raise PersonDepthPairCaptureError("INVALID_FRAME", f"{label} must be {expected}/uint8")
    return frame


def _png(frame: np.ndarray, label: str) -> bytes:
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise PersonDepthPairCaptureError("PNG_ENCODE_FAILED", label)
    return encoded.tobytes()


def _exclusive(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        raise PersonDepthPairCaptureError("OUTPUT_EXISTS", f"refusing overwrite: {path}")
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except FileExistsError as error:
        raise PersonDepthPairCaptureError("OUTPUT_EXISTS", f"refusing overwrite: {path}") from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _file_metadata(path: Path, payload: bytes, shape: tuple[int, int, int]) -> dict[str, Any]:
    decoded = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if type(decoded) is not np.ndarray or decoded.dtype != np.uint8 or decoded.shape != shape:
        raise PersonDepthPairCaptureError("POST_SAVE_VALIDATION_FAILED", f"{path}")
    return {"filename": path.name, "sha256": _sha256_bytes(payload), "byte_size": len(payload), "shape": list(shape), "dtype": "uint8"}


def _controls(ar_device: Path, stereo_device: Path, runner: ControlRunner) -> dict[str, dict[str, int]]:
    return {
        "ar0234_auto_exposure": set_control(ar_device, "auto_exposure", 3, runner),
        "ar0234_white_balance_automatic": set_control(ar_device, "white_balance_automatic", 1, runner),
        "stereo_auto_exposure": set_control(stereo_device, "auto_exposure", 3, runner),
    }


def capture_pair(
    *,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    countdown_s: float = 10.0,
    ar_device: Path = AR0234_BY_ID,
    stereo_device: Path = STEREO_BY_ID,
    ar_camera: Any | None = None,
    stereo_camera: Any | None = None,
    control_runner: ControlRunner | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    utc_now: Callable[[], str] | None = None,
    preview: bool = False,
    ar_intrinsic: Path = AR_INTRINSIC_DEFAULT,
    stereo_calibration: Path = STEREO_CALIBRATION_DEFAULT,
) -> dict[str, Any]:
    if not np.isfinite(countdown_s) or countdown_s < 0.0:
        raise PersonDepthPairCaptureError("INVALID_COUNTDOWN", "countdown-s must be finite and non-negative")
    root = _validate_root(output_root)
    calibrations = _principal_points(ar_intrinsic, stereo_calibration) if preview else (None, None)
    ar = ar_camera or CheckedCamera(ar_device, AR_MODE)
    stereo = stereo_camera or CheckedCamera(stereo_device, STEREO_MODE)
    runner = control_runner
    if runner is None:
        from vision_core.rgb_stereo_extrinsic.capture import default_control_runner
        runner = default_control_runner
    try:
        ar_actual = ar.open(AR0234_BY_ID)
        stereo_actual = stereo.open(STEREO_BY_ID)
        controls = _controls(ar_device, stereo_device, runner)
        for _ in range(WARMUP_READS):
            _frame(ar.read(AR_SHAPE), AR_SHAPE, "AR0234 warm-up frame")
            _frame(stereo.read(STEREO_SHAPE), STEREO_SHAPE, "stereo warm-up frame")
        if preview:
            try:
                cv2.namedWindow("Person Depth Pair", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Person Depth Pair", 1280, 600)
            except Exception as error:
                raise PersonDepthPairCaptureError("PREVIEW_UI_ERROR", f"namedWindow failed: {error}") from error
            waiting = True
            deadline = 0.0
            while waiting:
                ar_live = _frame(ar.read(AR_SHAPE), AR_SHAPE, "AR0234 preview frame")
                stereo_live = _frame(stereo.read(STEREO_SHAPE), STEREO_SHAPE, "stereo preview frame")
                if deadline == 0.0:
                    _show_positioning(ar_live, stereo_live, calibrations[0], calibrations[1], state="LIVE | SPACE countdown | Q/Esc cancel")
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        raise PersonDepthPairCaptureError("CAPTURE_CANCELLED", "preview cancelled")
                    if key == 32:
                        deadline = monotonic() + countdown_s
                elif monotonic() < deadline:
                    _show_positioning(ar_live, stereo_live, calibrations[0], calibrations[1], state="COUNTDOWN | Q/Esc cancel")
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        raise PersonDepthPairCaptureError("CAPTURE_CANCELLED", "preview cancelled")
                else:
                    ar_frame, stereo_frame = ar_live.copy(), stereo_live.copy()
                    ar_stamp, stereo_stamp = float(monotonic()), float(monotonic())
                    waiting = False
        else:
            deadline = monotonic() + countdown_s
            while monotonic() < deadline:
                _frame(ar.read(AR_SHAPE), AR_SHAPE, "AR0234 countdown frame")
                _frame(stereo.read(STEREO_SHAPE), STEREO_SHAPE, "stereo countdown frame")
            ar_frame = _frame(ar.read(AR_SHAPE), AR_SHAPE, "AR0234 capture frame").copy()
            ar_stamp = float(monotonic())
            stereo_frame = _frame(stereo.read(STEREO_SHAPE), STEREO_SHAPE, "stereo capture frame").copy()
            stereo_stamp = float(monotonic())
        if not np.isfinite(ar_stamp) or not np.isfinite(stereo_stamp):
            raise PersonDepthPairCaptureError("INVALID_TIMESTAMP", "host receive timestamp is not finite")
        skew = abs(ar_stamp - stereo_stamp)
        if skew > MAX_PAIR_SKEW_S:
            raise PersonDepthPairCaptureError("PAIR_SKEW_TOO_HIGH", f"receive skew {skew:.6f}s exceeds {MAX_PAIR_SKEW_S:.3f}s")
        ar_payload, stereo_payload = _png(ar_frame, "ar0234"), _png(stereo_frame, "stereo_combined")
        ar_path, stereo_path = root / "ar0234.png", root / "stereo_combined.png"
        _exclusive(ar_path, ar_payload)
        _exclusive(stereo_path, stereo_payload)
        created = utc_now() if utc_now is not None else dt.datetime.now(dt.timezone.utc).isoformat()
        record = {
            "schema": "sie.person_depth_pair_capture.v1",
            "status": "PAIR_SAVED",
            "created_at_utc": created,
            "ar0234": _file_metadata(ar_path, ar_payload, AR_SHAPE),
            "stereo_combined": _file_metadata(stereo_path, stereo_payload, STEREO_SHAPE),
            "devices": {"ar0234_stable_by_id": str(ar_device), "ar0234_resolved": str(getattr(ar, "target", ar_device)), "stereo_stable_by_id": str(stereo_device), "stereo_resolved": str(getattr(stereo, "target", stereo_device))},
            "actual_modes": {"ar0234": ar_actual, "stereo_combined": stereo_actual},
            "controls": controls,
            "host_receive_timestamps": {"ar0234_monotonic_s": ar_stamp, "stereo_monotonic_s": stereo_stamp},
            "receive_skew_s": skew,
            "combined_frame_mapping": {"combined_left_half": "physical RIGHT", "combined_right_half": "physical LEFT"},
            "calibration": {"ar0234_intrinsic": calibrations[0], "stereo_raw_physical_left": calibrations[1]} if preview else None,
            "preview_overlay_saved_in_png": False,
        }
        _exclusive(root / "capture_pair.json", (json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())
        if preview:
            _show_positioning(ar_frame, stereo_frame, calibrations[0], calibrations[1], state="PAIR SAVED | Q/Esc close")
            print(f"PAIR_SAVED {root}")
            print(f"AR_OPTICAL_AXIS cx={calibrations[0]['cx']:.6f} cy={calibrations[0]['cy']:.6f}")
            print(f"STEREO_LEFT_RAW_OPTICAL_AXIS cx={calibrations[1]['cx']:.6f} cy={calibrations[1]['cy']:.6f}")
            cv2.waitKey(1)
        return record
    except ExtrinsicCaptureError as error:
        raise PersonDepthPairCaptureError("CAMERA_ERROR", str(error)) from error
    finally:
        ar.close()
        stereo.close()
        if preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
