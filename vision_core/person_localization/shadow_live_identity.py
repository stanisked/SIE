"""Asynchronous, latest-wins Face + Body ReID preview for current AR0234 mask.

One producer is the audited ROS control-preview camera; this worker owns
YuNet/SFace and OSNet on an independent Python thread. The selected crop is
transient; neither image nor private biometric vector leaves this worker.
Only compact, source-labeled diagnostics are published and stored.

Identity is NOT verified by cosine on a positive-only face profile or by
cross-track OSNet similarity; the existing FaceAnchoredTarget is used without
a calibrated match threshold, and no motor path reads these outputs.
"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import json
import math
import os
import threading
import time

import numpy as np

from vision_core.person_localization.shadow_face_anchored_target import (
    FACE_NONE, FACE_UNKNOWN, FaceAnchoredTarget, FaceBodyEvidence)
from vision_core.person_localization.shadow_body_osnet import (
    FixedBatchOsnet, body_input_tensor)
from vision_core.person_localization.shadow_face_verify import (
    LocalFaceEncoder, face_cosine)
from vision_core.person_localization.shadow_identity_crop import (
    unpack_selected_identity_source)
from vision_core.tools.shadow_face_offline import load_private_profile


class LiveIdentityProcessor:
    def __init__(self, *, yunet, sface, face_profile, osnet):
        self.face=LocalFaceEncoder(Path(yunet),Path(sface))
        self.profile,self.profile_metadata=load_private_profile(
            Path(face_profile),self.face)
        self.body=FixedBatchOsnet(Path(osnet))
        self.coordinator=FaceAnchoredTarget(
            entity_id="person_001",
            face_profile_id=Path(face_profile).stem,
            body_model_sha256=self.body.model_sha256)
        self.track_gallery={}
        self.last_track=None

    def process(self, event):
        source=event["source"]
        cycle_id=source["cycle_id"]
        track=event["track_id"]
        if not isinstance(track,str) or not track:
            raise ValueError("IDENTITY_CURRENT_RGB_TRACK_REQUIRED")
        frame,mask=unpack_selected_identity_source(source)
        h,w=frame.shape[:2]
        image_box=(0,0,w,h)
        obs,face_reason=self.face.observe(frame,image_box)
        score=None
        if obs is not None:
            score=max(face_cosine(f,obs.feature) for f in self.profile)
        tensor=body_input_tensor(
            frame,mask,image_box,variant="mask_neutral_background")
        body_feature=self.body.encode(tensor[None])[0]
        new_track=track not in self.track_gallery
        prev=self.last_track if self.last_track!=track else None
        cross=None
        if prev is not None and prev in self.track_gallery:
            old=np.stack(self.track_gallery[prev])
            sims=old@body_feature
            cross={
                "previous_rgb_track_id":prev,
                "comparison_count":len(sims),
                "min":round(float(np.min(sims)),5),
                "median":round(float(np.median(sims)),5),
                "max":round(float(np.max(sims)),5),
                "interpretation":"SAME_PERSON_HYPOTHESIS_ONLY",
            }
        self.track_gallery.setdefault(track,deque(maxlen=8)).append(
            body_feature.copy())
        # Bound historical memory; never allow an unbounded gallery.
        while len(self.track_gallery)>2:
            oldest=next(iter(self.track_gallery))
            if oldest==track:
                break
            self.track_gallery.pop(oldest)
        self.last_track=track
        at=time.monotonic()
        elapsed=at-event["submitted_mono_s"]
        evidence=FaceBodyEvidence(
            cycle_id=cycle_id,captured_s=event["submitted_mono_s"],
            camera_track_id=track,
            mask_sha256=source["full_instance_mask_sha256"],
            unique_body_binding=True,
            sole_visible_person=event["sole_visible_person"],
            face_result=FACE_UNKNOWN if obs is not None else FACE_NONE,
            face_profile_id=self.coordinator.face_profile_id if obs is not None else None,
            face_cycle_id=cycle_id if obs is not None else None,
            face_mask_sha256=source["full_instance_mask_sha256"]
                if obs is not None else None,
            face_verifier_calibrated=False,
            body_model_sha256=self.body.model_sha256,
            body_embedding=tuple(float(v) for v in body_feature),
        )
        # Late processing cannot be called a fresh identity observation.
        decision=self.coordinator.observe(evidence,now_s=at)
        if decision.identity_supported or decision.motor_authorized:
            raise AssertionError("UNCALIBRATED_IDENTITY_ADMITTED")
        return {
            "scope":"LIVE_IDENTITY_DIAGNOSTIC_ONLY_NO_MOTOR_AUTHORIZATION",
            "status":"PROCESSED" if elapsed<=0.5 else "LATE_DIAGNOSTIC",
            "cycle_id":cycle_id,
            "captured_at_utc_declared":event["captured_at_utc"],
            "rgb_track_id":track,
            "instance_index":source["instance_index"],
            "segmentation_model_sha256":source["segmentation_model_sha256"],
            "rgb_arrays_sha256":source["rgb_arrays_sha256"],
            "full_instance_mask_sha256":source["full_instance_mask_sha256"],
            "crop_jpeg_sha256":source["crop_jpeg_sha256"],
            "crop_mask_bits_sha256":source["crop_mask_bits_sha256"],
            "reference_frame":"ar0234_image_frame",
            "face_profile_id":self.coordinator.face_profile_id,
            "face_observation":face_reason,
            "face_cosine_max_positive_only":(
                None if score is None else round(score,5)),
            "face_calibrated":False,
            "osnet_model_sha256":self.body.model_sha256,
            "osnet_embedding_dimension":512,
            "osnet_cross_track_comparison":cross,
            "body_first_sample_on_rgb_track":new_track,
            "body_calibrated":False,
            "coordinator_status":decision.status,
            "coordinator_reason":decision.reason,
            "confirmed_entity_id":None,
            "candidate_entity_id":(
                "person_001" if obs is not None else None),
            "identity_supported":False,
            "motor_authorized":False,
            "worker_submit_to_result_ms":round(elapsed*1000,2),
            "freshness_basis":"SUBMISSION_MONOTONIC_NOT_DRIVER_EXPOSURE",
            "raw_biometric_vectors_emitted":False,
        }


class LatestIdentityPreview:
    """One in-flight inference, one replaceable pending input, bounded results."""

    def __init__(self, processor_factory, *, report_path=None):
        self._factory=processor_factory
        self._report_path=Path(report_path) if report_path else None
        self._lock=threading.Condition()
        self._pending=None
        self._result=None
        self._stop=False
        self._failure=None
        self.accepted=0
        self.replaced=0
        self.processed=0
        self.errors=0
        self._thread=threading.Thread(
            target=self._run,daemon=True,name="sie-live-identity-worker")
        self._thread.start()

    def submit(self, source, *, track_id, captured_at_utc,
               sole_visible_person):
        if not isinstance(source,dict) or not source.get("cycle_id"):
            raise ValueError("IDENTITY_SOURCE_MISSING")
        item={
            "source":source,
            "track_id":track_id,
            "captured_at_utc":captured_at_utc,
            "sole_visible_person":sole_visible_person is True,
            "submitted_mono_s":time.monotonic(),
        }
        with self._lock:
            if self._stop:
                return False
            if self._pending is not None:
                self.replaced+=1
            self._pending=item
            self.accepted+=1
            self._lock.notify()
            return True

    def drain(self):
        with self._lock:
            result,self._result=self._result,None
            return result

    def _run(self):
        processor=None
        stream=None
        try:
            processor=self._factory()
            if self._report_path is not None:
                os.umask(0o077)
                stream=self._report_path.open("x",encoding="utf-8")
                os.chmod(self._report_path,0o600)
            while True:
                with self._lock:
                    while self._pending is None and not self._stop:
                        self._lock.wait()
                    if self._stop:
                        return
                    event,self._pending=self._pending,None
                try:
                    result=processor.process(event)
                    self.processed+=1
                except Exception as error:
                    self.errors+=1
                    result={
                        "scope":"LIVE_IDENTITY_DIAGNOSTIC_ONLY_NO_MOTOR_AUTHORIZATION",
                        "status":"INFERENCE_REFUSED",
                        "cycle_id":event["source"]["cycle_id"],
                        "error_type":type(error).__name__,
                        "confirmed_entity_id":None,
                        "motor_authorized":False,
                    }
                if stream is not None:
                    stream.write(json.dumps(result,allow_nan=False,separators=(',',':'))+"\n")
                    stream.flush()
                with self._lock:
                    self._result=result
        except Exception as error:
            self._failure=type(error).__name__
            with self._lock:
                self._result={
                    "scope":"LIVE_IDENTITY_DIAGNOSTIC_ONLY_NO_MOTOR_AUTHORIZATION",
                    "status":"IDENTITY_WORKER_UNAVAILABLE",
                    "error_type":type(error).__name__,
                    "confirmed_entity_id":None,
                    "motor_authorized":False}
        finally:
            if stream is not None:
                stream.close()

    def close(self):
        with self._lock:
            self._stop=True
            self._pending=None
            self._lock.notify_all()
        self._thread.join(timeout=3.)
        return not self._thread.is_alive()
