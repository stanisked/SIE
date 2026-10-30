#!/usr/bin/env python3
"""Verify a complete Shadow RGB session and select 5-second annotation views.

One unlabelled image per 5-second window. A large consecutive-frame change
gets priority within the window, otherwise the midpoint is used. No person
presence, box, identity or motion decision is inferred from image differences.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil


FILE_PATTERN = re.compile(r"images/ar0234_[0-9]{6}\.jpg\Z")
MIB = 1024 * 1024


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(MIB), b""):
            h.update(part)
    return h.hexdigest()


def verify_session(root: Path) -> tuple[dict, list[dict]]:
    session_path = root / "session.json"
    journal_path = root / "frames.jsonl"
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if (session.get("schema_version") != "sie.shadow_rgb_presence_capture.v1"
            or session.get("status") != "COMPLETE" or session.get("frames_saved") != 1200
            or session.get("duration_requested_s") != 600 or session.get("sample_hz") != 2
            or session.get("slots_skipped") != 0 or session.get("actuator_access") is not False
            or session.get("frames_jsonl_sha256") != digest(journal_path)):
        raise ValueError("INVALID_OR_INCOMPLETE_CAPTURE_SESSION")
    rows = [json.loads(line) for line in journal_path.read_text(encoding="utf-8").splitlines()]
    if len(rows) != 1200:
        raise ValueError("JOURNAL_LENGTH_MISMATCH")
    total_bytes = 0
    previous_sequence = -1
    for index, row in enumerate(rows):
        name = row.get("file")
        expected_name = f"images/ar0234_{index+1:06d}.jpg"
        sequence = row.get("driver_sequence")
        if (not isinstance(name, str) or not FILE_PATTERN.fullmatch(name)
                or name != expected_name or row.get("slot") != index
                or type(sequence) is not int or sequence <= previous_sequence):
            raise ValueError(f"INVALID_FRAME_ORDER:{index+1}")
        previous_sequence = sequence
        path = root / name
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"MISSING_OR_LINKED_FRAME:{index+1}")
        payload = path.read_bytes()
        if (len(payload) != row.get("bytes") or not payload.startswith(b"\xff\xd8")
                or hashlib.sha256(payload).hexdigest() != row.get("sha256")):
            raise ValueError(f"SOURCE_IMAGE_HASH_MISMATCH:{index+1}")
        total_bytes += len(payload)
    if total_bytes != session.get("bytes_saved"):
        raise ValueError("SESSION_BYTE_COUNT_MISMATCH")
    return session, rows


def change_scores(root: Path, rows: list[dict]) -> list[float]:
    """Decode for offline ranking only; never write a re-encoded source frame."""
    import cv2
    import numpy as np

    previous = None
    scores = []
    for index, row in enumerate(rows):
        packet = (root / row["file"]).read_bytes()
        thumbnail = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8),
                                  cv2.IMREAD_REDUCED_GRAYSCALE_8)
        if thumbnail is None or thumbnail.size == 0:
            raise ValueError(f"INVALID_JPEG_DECODE:{index+1}")
        if previous is not None and thumbnail.shape != previous.shape:
            raise ValueError("CAMERA_IMAGE_DIMENSIONS_CHANGED")
        score = (0.0 if previous is None else
                 float(np.mean(cv2.absdiff(thumbnail, previous))))
        if not math.isfinite(score):
            raise ValueError("INVALID_CHANGE_SCORE")
        scores.append(score)
        previous = thumbnail
    return scores


def choose_frames(rows: list[dict], scores: list[float], *, hz: int = 2,
                  window_s: int = 5) -> list[dict]:
    if len(rows) != len(scores) or len(rows) != 1200 or hz != 2 or window_s != 5:
        raise ValueError("INVALID_SELECTION_TIMELINE")
    # A change event is a frame-to-frame image difference above the global
    # 80th percentile. This is scene novelty, not a human classification.
    ordered = sorted(scores)
    cutoff = ordered[int(.8 * (len(ordered) - 1))]
    selected = []
    size = hz * window_s
    for begin in range(0, len(rows), size):
        end = begin + size
        midpoint = begin + (size - 1) // 2
        winner = max(range(begin, end), key=lambda i: (scores[i], -abs(i-midpoint)))
        index = winner if scores[winner] > cutoff else midpoint
        selected.append({"file": rows[index]["file"], "sha256": rows[index]["sha256"],
                         "source_slot": rows[index]["slot"],
                         "window_start_s": begin / hz,
                         "scene_change_score": scores[index],
                         "selection_reason": "HIGH_FRAME_CHANGE" if index == winner and
                         scores[winner] > cutoff else "WINDOW_MIDPOINT",
                         "person_visible": None, "person_boxes_xyxy_px": None})
    return selected


def prepare(root: Path, output: Path) -> dict:
    if output.exists():
        raise FileExistsError(f"refusing existing output: {output}")
    session, rows = verify_session(root)
    selected = choose_frames(rows, change_scores(root, rows))
    required = sum((root / row["file"]).stat().st_size for row in selected)
    if shutil.disk_usage(output.parent).free < required + 512 * MIB:
        raise RuntimeError("INSUFFICIENT_FREE_SPACE_FOR_SUBSET")
    os.umask(0o077)
    output.mkdir(parents=True, mode=0o700, exist_ok=False)
    (output / "images").mkdir(mode=0o700)
    for row in selected:
        with (root / row["file"]).open("rb") as source, \
                (output / row["file"]).open("xb") as destination:
            shutil.copyfileobj(source, destination, MIB)
        if digest(output / row["file"]) != row["sha256"]:
            raise RuntimeError(f"SUBSET_COPY_HASH_MISMATCH:{row['file']}")
    result = {"schema_version": "sie.shadow_rgb_annotation_subset.v1",
              "status": "UNLABELLED_VERIFIED_SUBSET", "source_root": str(root.resolve()),
              "source_session_sha256": digest(root / "session.json"),
              "source_frames_jsonl_sha256": session["frames_jsonl_sha256"],
              "source_frames_verified": len(rows), "selection_method": "FIVE_SECOND_WINDOWS_CHANGE80_OR_MIDPOINT",
              "selected_count": len(selected), "bytes_copied": required,
              "projector_state_operator_reported": session["projector_state_operator_reported"],
              "metric_measurement_authorized": False, "navigation_eligible": False,
              "actuator_access": False, "frames": selected,
              "limitations": ["Scene change is not person presence, identity, or a validated box.",
                              "All labels are null pending manual review of each image.",
                              "One image per five seconds may miss brief appearances."]}
    (output / "selection_manifest.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.session_root.resolve()
    output = args.output or root / "annotation_subset_5s_v1"
    result = prepare(root, output)
    print("SUMMARY", json.dumps({"status": result["status"],
                                 "source_frames_verified": result["source_frames_verified"],
                                 "selected_count": result["selected_count"],
                                 "bytes_copied": result["bytes_copied"]}))
    print("Saved:", output / "selection_manifest.json")


if __name__ == "__main__":
    main()
