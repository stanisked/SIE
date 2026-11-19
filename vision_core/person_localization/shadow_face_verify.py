"""Local, opt-in face verification for a selected Shadow RGB instance.

Face embedding is biometric data. It is NOT a motor admission, a persistent
RGB Track ID or proof of identity. This module never talks to ROS/UART.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import hashlib
import math

import cv2
import numpy as np


FACE_SCORE_MIN = 0.85
FACE_WIDTH_MIN_PX = 44
FACE_DETECT_MAX_SIDE = 960
SFACE_FEATURE_LENGTH = 128


def model_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selected_face(detections: np.ndarray | None, person_bbox,
                  *, minimum_score: float = FACE_SCORE_MIN,
                  minimum_face_px: float = FACE_WIDTH_MIN_PX
                  ) -> np.ndarray | None:
    """Choose an unambiguous face inside the upper part of selected person.

    A face elsewhere in frame prevents selection: this first experiment is
    restricted to a single visible face. Bounding-box overlap is *not*
    independent identity evidence.
    """
    if not isinstance(person_bbox, (tuple, list)) or len(person_bbox) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(x) for x in person_bbox]
    except (TypeError, ValueError):
        return None
    if (not all(math.isfinite(v) for v in (x1,y1,x2,y2))
            or x2 <= x1 or y2 <= y1):
        return None
    if detections is None:
        return None
    faces = np.asarray(detections)
    if faces.ndim != 2 or faces.shape[1] < 15 or len(faces) != 1:
        return None
    face = faces[0]
    if not np.isfinite(face).all():
        return None
    fx, fy, fw, fh = [float(v) for v in face[:4]]
    score = float(face[14])
    if (score < minimum_score or min(fw,fh) < minimum_face_px
            or fw <= 0 or fh <= 0):
        return None
    cx, cy = fx + fw / 2, fy + fh / 2
    if not (x1 <= cx <= x2 and y1 <= cy <= y1 + .60 * (y2 - y1)):
        return None
    return np.asarray(face[:15], dtype=np.float32)


def normalized_feature(raw) -> np.ndarray:
    array = np.asarray(raw, dtype=np.float32).reshape(-1)
    if array.size != SFACE_FEATURE_LENGTH or not np.isfinite(array).all():
        raise ValueError("INVALID_SFACE_FEATURE")
    length = float(np.linalg.norm(array))
    if length <= 1e-8:
        raise ValueError("EMPTY_SFACE_FEATURE")
    return array / length


def face_cosine(first, second) -> float:
    a = normalized_feature(first)
    b = normalized_feature(second)
    return float(np.clip(np.dot(a, b), -1., 1.))


@dataclass(frozen=True)
class FaceObservation:
    feature: np.ndarray
    face_box_xywh: tuple[float, float, float, float]
    score: float


class LocalFaceEncoder:
    def __init__(self, yunet_path: Path, sface_path: Path):
        yunet_path, sface_path = Path(yunet_path), Path(sface_path)
        if not yunet_path.is_file() or not sface_path.is_file():
            raise FileNotFoundError("FACE_MODELS_NOT_AVAILABLE")
        detector_factory = getattr(cv2, "FaceDetectorYN_create", None)
        if detector_factory is None and hasattr(cv2, "FaceDetectorYN"):
            detector_factory = cv2.FaceDetectorYN.create
        recognizer_factory = getattr(cv2, "FaceRecognizerSF_create", None)
        if recognizer_factory is None and hasattr(cv2, "FaceRecognizerSF"):
            recognizer_factory = cv2.FaceRecognizerSF.create
        if detector_factory is None or recognizer_factory is None:
            raise RuntimeError("OPENCV_FACE_DNN_APIS_UNAVAILABLE")
        self.detector = detector_factory(str(yunet_path), "", (320,320),
                                         FACE_SCORE_MIN, .3, 5000)
        self.recognizer = recognizer_factory(str(sface_path), "")
        self.yunet_sha256 = model_sha256(yunet_path)
        self.sface_sha256 = model_sha256(sface_path)

    def observe(self, bgr: np.ndarray, person_bbox) -> tuple[FaceObservation | None, str]:
        if (not isinstance(bgr, np.ndarray) or bgr.ndim != 3
                or bgr.shape[2] != 3 or bgr.dtype != np.uint8):
            return None, "INVALID_BGR_FRAME"
        height, width = bgr.shape[:2]
        if min(height,width) < 64:
            return None, "FRAME_TOO_SMALL"
        scale = min(1., FACE_DETECT_MAX_SIDE / max(width,height))
        reduced = (cv2.resize(bgr, (round(width*scale), round(height*scale)),
                              interpolation=cv2.INTER_AREA)
                   if scale < 1 else bgr)
        self.detector.setInputSize((reduced.shape[1], reduced.shape[0]))
        _, detected = self.detector.detect(reduced)
        if detected is None or len(detected) == 0:
            return None, "NO_FACE_VISIBLE"
        if len(detected) != 1:
            return None, "MULTIPLE_VISIBLE_FACES"
        full = np.asarray(detected, dtype=np.float32).copy()
        full[:, :14] /= scale
        face = selected_face(full, person_bbox)
        if face is None:
            return None, "FACE_NOT_UNIQUE_OR_OUTSIDE_SELECTED_PERSON"
        aligned = self.recognizer.alignCrop(bgr, face)
        feature = normalized_feature(self.recognizer.feature(aligned))
        return FaceObservation(feature, tuple(float(v) for v in face[:4]),
                               float(face[14])), "CURRENT_SELECTED_FACE_EMBEDDED"
