from __future__ import annotations

from vision_core.close_range_person_alignment_dataset.splits import SessionSummary, select_session_assignment


def test_session_assignment_is_deterministic_and_keeps_required_evidence_in_each_split() -> None:
    sessions = [
        SessionSummary(f"day-positive-{index}", 21 + index, True, "daylight") for index in range(3)
    ] + [
        SessionSummary(f"artificial-positive-{index}", 21 + index, True, "artificial") for index in range(3)
    ] + [
        SessionSummary(f"negative-{index}", 20 + index, False, "daylight" if index % 2 == 0 else "artificial") for index in range(4)
    ]
    first = select_session_assignment(sessions, seed=20260915)
    assert first == select_session_assignment(sessions, seed=20260915)
    for split in ("train", "val", "test"):
        members = [session for session in sessions if first[session.session_id] == split]
        assert any(session.contains_person for session in members)
        assert any(not session.contains_person for session in members)
        assert {session.lighting for session in members} == {"daylight", "artificial"}
