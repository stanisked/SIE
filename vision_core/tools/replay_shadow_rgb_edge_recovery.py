"""Passive replay of saved SHADOW RGB edge recovery evidence; no UART or motor I/O.

Usage:
  python -m vision_core.tools.replay_shadow_rgb_edge_recovery \
    /home/elwis/.local/state/sie/evidence/shadow_fast_20261117_193238
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from vision_core.tools.shadow_fast_adapter import from_control_preview, front_profile_sha256
from vision_core.tools.shadow_live_bridge import observation_from_edge_guidance
from vision_core.tools.run_shadow_uart import select_edge_turn
from vision_core.tools.shadow_range_gate import select_target_range
from vision_core.tools.shadow_live_bridge import AR_INTRINSIC_SHA256


def replay(evidence_dir: Path, *, optical_offset_m: float = .0125) -> dict:
    paths = sorted(evidence_dir.glob("control-preview-*/*.json"))
    if not paths:
        raise ValueError("CONTROL_PREVIEW_EVIDENCE_NOT_FOUND")
    records = []
    profile_sha = front_profile_sha256()
    for path in paths:
        audit = json.loads(path.read_text())
        stamp = float(audit["frame_freshness"]["frames"]["ar0234"]["driver_timestamp_s"])
        # Structural replay only: use a uniform 0.1 s receipt delay, not
        # a claim about timing of the original physical session.
        preview, reason = from_control_preview(
            audit, now_s=stamp + .1,
            rim_forward_of_optical_center_m=optical_offset_m,
            expected_front_profile_sha256=profile_sha,
            expected_segmentation_model_sha256=(
                audit.get("segmentation_observation") or {}).get("model_sha256", ""),
            semantic_source="segmentation_mask", local_preview=True)
        edge = observation_from_edge_guidance(audit, now_s=stamp + .1)
        rng = None
        if preview.target is not None and preview.ranges:
            decision = select_target_range(
                preview.target, list(preview.ranges), stamp + .1,
                expected_rgb_calibration_id=AR_INTRINSIC_SHA256,
                minimum_confidence=.4)
            if decision.selected is not None and decision.action in (
                    "FOLLOW_ELIGIBLE", "HOLD"):
                rng = (preview.target.track_id,
                       decision.selected.observed_s,
                       decision.selected.distance_m)
        records.append({
            "cycle": audit.get("cycle_id"),
            "stamp": stamp,
            "rgb_track": (preview.target.track_id if preview.target else None),
            "range": rng, "range_refusal": reason,
            "edge_track": (edge.target.track_id if edge.target else None),
            "edge_bearing_deg": edge.edge_recovery_bearing_deg,
            "edge_target": edge.target,
        })
    edges = [r for r in records if r["edge_target"] is not None]
    status_path = evidence_dir / "bridge.jsonl"
    if not status_path.is_file():
        raise ValueError("BRIDGE_JOURNAL_NOT_FOUND")
    status_rows = [json.loads(line) for line in status_path.read_text().splitlines()
                   if line.strip()]
    # Analyze a real observed PAUSED state, never fabricate a stopped motor.
    paused = [r for r in status_rows if r.get("method") == "GET"
              and r.get("endpoint") == "/status"
              and (r.get("body") or {}).get("state") == "PAUSED"
              and (r.get("body") or {}).get("right_pwm") == 0
              and (r.get("body") or {}).get("left_pwm") == 0
              and (r.get("body") or {}).get("active_command_id")]
    opportunities = []
    seen_transitions = set()
    for state in paused:
        at_s = float(state["started_s"])
        seq = (state["body"].get("active_command_id"),
               state["body"].get("shadow_transition_seq"))
        if seq in seen_transitions:
            continue
        edge = next((r for r in reversed(edges)
                     if 0 <= at_s - r["stamp"] <= .5), None)
        if edge is None:
            continue
        prior = next((r["range"] for r in reversed(records)
                      if r["range"] is not None
                      and r["stamp"] <= edge["stamp"]
                      and r["range"][0] == edge["edge_track"]), None)
        turn = select_edge_turn(edge["edge_target"], edge["edge_bearing_deg"],
                                prior, at_s, minimum_confidence=.4)
        seen_transitions.add(seq)
        opportunities.append({
            "pause_seq": seq[1],
            "at_s": round(at_s, 3),
            "edge_track": edge["edge_track"],
            "bearing_deg": round(edge["edge_bearing_deg"], 2),
            "prior_range_m": round(prior[2], 3) if prior else None,
            "prior_age_s": round(at_s - prior[1], 3) if prior else None,
            "bounded_turn_candidate": (list(turn) if turn else None),
            "note": "offline_estimate_not_motor_authorization",
        })
    edge_ids = Counter(r["edge_track"] for r in edges)
    return {
        "mode": "PASSIVE_OFFLINE_REPLAY_NO_MOTOR_IO",
        "cycle_count": len(records),
        "valid_edge_cycles": len(edges),
        "valid_edge_track_ids": dict(edge_ids),
        "valid_range_cycles": sum(r["range"] is not None for r in records),
        "first_valid_edge_bearing_deg": (
            round(edges[0]["edge_bearing_deg"], 2) if edges else None),
        "last_valid_edge_bearing_deg": (
            round(edges[-1]["edge_bearing_deg"], 2) if edges else None),
        "paused_opportunities": opportunities,
        "limitations": [
            "Uses 0.1 s synthetic receipt offset; not actual camera-to-control latency",
            "Potential TURN is a software gate result only; no firmware or physical pivot tested",
            "No forward movement without a post-turn same-ID current RGB/Depth pair",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.evidence_dir), indent=2,
                     ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
