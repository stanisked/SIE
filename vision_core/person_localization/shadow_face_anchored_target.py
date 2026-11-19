"""Face-anchored identity state for SIEE, independent of camera Track ID.

Pure identity evidence coordinator. It does not read images, verify faces,
measure depth or issue motor commands. The producer must independently verify
face-to-mask geometry, embedding provenance and source frame binding.

Body-only re-identification is disabled by default: it requires a separate,
validated body model and externally calibrated match threshold. No decision
returned here is movement authorization.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal


FACE_NONE = "NO_FACE"
FACE_MATCH = "MATCHED_ENROLLED_FACE"
FACE_MISMATCH = "DIFFERENT_FACE"
FACE_UNKNOWN = "UNVERIFIED_FACE"
MAX_CONTINUOUS_RGB_GAP_S = 0.75
MAX_CURRENT_AGE_S = 0.5
MAX_APPEARANCE_GALLERY = 8


@dataclass(frozen=True)
class FaceBodyEvidence:
    cycle_id: str
    captured_s: float
    camera_track_id: str
    mask_sha256: str
    unique_body_binding: bool
    face_result: Literal[
        "NO_FACE", "MATCHED_ENROLLED_FACE", "DIFFERENT_FACE", "UNVERIFIED_FACE"
    ] = FACE_NONE
    face_profile_id: str | None = None
    face_cycle_id: str | None = None
    face_mask_sha256: str | None = None
    face_verifier_calibrated: bool = False
    body_model_sha256: str | None = None
    body_embedding: tuple[float, ...] | None = None


@dataclass(frozen=True)
class TargetIdentityDecision:
    entity_id: str | None
    camera_track_id: str | None
    status: str
    reason: str
    face_visible: bool
    identity_supported: bool
    motor_authorized: bool = False


def _normalized_body(feature):
    if feature is None:
        return None
    try:
        vector = tuple(float(x) for x in feature)
    except (ValueError, TypeError):
        return None
    if not 16 <= len(vector) <= 2048 or not all(math.isfinite(x) for x in vector):
        return None
    magnitude = math.sqrt(sum(x*x for x in vector))
    if magnitude <= 1e-8:
        return None
    return tuple(x/magnitude for x in vector)


def _cosine(a, b):
    return sum(x*y for x,y in zip(a,b))


class FaceAnchoredTarget:
    """Persistent target entity, tied initially to a verified selected face."""

    def __init__(self, *, entity_id: str, face_profile_id: str,
                 body_model_sha256: str | None = None,
                 calibrated_body_min_cosine: float | None = None):
        if not entity_id or not face_profile_id:
            raise ValueError("TARGET_ENTITY_AND_LOCAL_FACE_PROFILE_REQUIRED")
        if calibrated_body_min_cosine is not None and (
                not body_model_sha256 or
                not 0 < calibrated_body_min_cosine < 1):
            raise ValueError("CALIBRATED_BODY_MODEL_AND_THRESHOLD_REQUIRED")
        self.entity_id = entity_id
        self.face_profile_id = face_profile_id
        self.body_model_sha256 = body_model_sha256
        self.calibrated_body_min_cosine = calibrated_body_min_cosine
        self.camera_track_id = None
        self.last_supported_s = None
        self.gallery = []
        self._candidate_id = None
        self._candidate_s = None

    def _decision(self, status, reason, *, face_visible=False, supported=False):
        return TargetIdentityDecision(self.entity_id if self.camera_track_id else None,
            self.camera_track_id, status, reason, face_visible, supported)

    def _clear_candidate(self):
        self._candidate_id = None
        self._candidate_s = None

    def _add_body(self, evidence):
        if (self.body_model_sha256 is None or
                evidence.body_model_sha256 != self.body_model_sha256):
            return
        feature = _normalized_body(evidence.body_embedding)
        if feature is not None:
            self.gallery.append(feature)
            del self.gallery[:-MAX_APPEARANCE_GALLERY]

    def observe(self, evidence: FaceBodyEvidence | None, *, now_s: float):
        if not math.isfinite(now_s):
            raise ValueError("INVALID_OBSERVATION_TIME")
        if evidence is None:
            self._clear_candidate()
            return self._decision("TARGET_LOST", "NO_CURRENT_SELECTED_BODY")
        if (not evidence.cycle_id or not evidence.camera_track_id
                or len(evidence.mask_sha256) != 64
                or not math.isfinite(evidence.captured_s)
                or not 0 <= now_s - evidence.captured_s <= MAX_CURRENT_AGE_S
                or not evidence.unique_body_binding):
            self._clear_candidate()
            return self._decision("TARGET_UNAVAILABLE",
                                  "STALE_OR_UNBOUND_RGB_MASK")
        same_id = self.camera_track_id == evidence.camera_track_id
        face_visible = evidence.face_result != FACE_NONE
        if evidence.face_result not in (
                FACE_NONE, FACE_MATCH, FACE_MISMATCH, FACE_UNKNOWN):
            raise ValueError("INVALID_FACE_RESULT")
        verified = (evidence.face_result == FACE_MATCH
                    and evidence.face_profile_id == self.face_profile_id
                    and evidence.face_cycle_id == evidence.cycle_id
                    and evidence.face_mask_sha256 == evidence.mask_sha256
                    and evidence.face_verifier_calibrated)
        if evidence.face_result == FACE_MISMATCH and same_id:
            self._clear_candidate()
            return self._decision("TARGET_CONFLICT",
                                  "DIFFERENT_FACE_ON_CURRENT_RGB_TRACK",
                                  face_visible=True)
        if evidence.face_result == FACE_MATCH and not verified:
            self._clear_candidate()
            return self._decision("TARGET_UNAVAILABLE",
                                  "FACE_IDENTITY_OR_MASK_BINDING_UNVERIFIED",
                                  face_visible=True)
        if verified:
            previous = self.camera_track_id
            self.camera_track_id = evidence.camera_track_id
            self.last_supported_s = evidence.captured_s
            self._clear_candidate()
            self._add_body(evidence)
            return self._decision(
                "FACE_ANCHORED" if previous is None else
                "FACE_REVERIFIED" if previous != self.camera_track_id else
                "FACE_CONFIRMED",
                "SAME_CYCLE_ENROLLED_FACE_AND_BODY_BOUND",
                face_visible=True, supported=True)
        if self.camera_track_id is None:
            return self._decision("AWAITING_FACE",
                                  "TARGET_MUST_FIRST_BE_FACE_ANCHORED",
                                  face_visible=face_visible)
        continuous = (same_id and self.last_supported_s is not None
                      and 0 <= evidence.captured_s - self.last_supported_s <=
                      MAX_CONTINUOUS_RGB_GAP_S)
        if continuous:
            self.last_supported_s = evidence.captured_s
            self._clear_candidate()
            self._add_body(evidence)
            return self._decision("BODY_TRACKED",
                                  "FACE_ABSENT_BUT_CAMERA_BODY_CONTINUITY_CONFIRMED",
                                  face_visible=face_visible, supported=True)
        # No single-person shortcut or old depth can establish identity.
        feature = _normalized_body(evidence.body_embedding)
        if (self.calibrated_body_min_cosine is None or
                evidence.body_model_sha256 != self.body_model_sha256
                or feature is None or not self.gallery):
            self._clear_candidate()
            return self._decision("TARGET_LOST",
                                  "BODY_REID_NOT_CALIBRATED_OR_NO_GALLERY",
                                  face_visible=face_visible)
        if len(feature) != len(self.gallery[0]):
            self._clear_candidate()
            return self._decision("TARGET_LOST", "BODY_FEATURE_DIMENSIONS_CHANGED",
                                  face_visible=face_visible)
        best = max(_cosine(feature, known) for known in self.gallery)
        if best < self.calibrated_body_min_cosine:
            self._clear_candidate()
            return self._decision("TARGET_LOST", "BODY_APPEARANCE_NOT_MATCHED",
                                  face_visible=face_visible)
        if self._candidate_id != evidence.camera_track_id:
            self._candidate_id = evidence.camera_track_id
            self._candidate_s = evidence.captured_s
            return self._decision("BODY_REID_PENDING",
                                  "FIRST_MATCHING_BODY_OBSERVATION",
                                  face_visible=face_visible)
        if (evidence.captured_s <= self._candidate_s or
                evidence.captured_s - self._candidate_s > MAX_CONTINUOUS_RGB_GAP_S):
            self._clear_candidate()
            return self._decision("BODY_REID_PENDING",
                                  "SECOND_DISTINCT_FRESH_FRAME_REQUIRED",
                                  face_visible=face_visible)
        previous = self.camera_track_id
        self.camera_track_id = evidence.camera_track_id
        self.last_supported_s = evidence.captured_s
        self._clear_candidate()
        # Do not assimilate the new body into gallery merely because it
        # matched: avoid appearance drift when different people dress alike.
        return self._decision("BODY_REIDENTIFIED",
                              "TWO_MATCHING_FRESH_BODY_EMBEDDINGS_HYPOTHESIS; "
                              "NOT_BIOMETRIC_PROOF; PRIOR_TRACK=" + previous,
                              face_visible=face_visible, supported=True)
