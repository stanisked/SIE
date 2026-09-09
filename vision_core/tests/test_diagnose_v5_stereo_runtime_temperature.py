from __future__ import annotations

from pathlib import Path

import pytest

from vision_core.tools.diagnose_v5_stereo_runtime import (
    build_temperature_monitoring,
    observe_temperature,
)


def test_required_temperature_mode_preserves_snapshot_failure():
    def unavailable(*_args):
        raise FileNotFoundError("state file not found")

    with pytest.raises(FileNotFoundError, match="state file not found"):
        observe_temperature(
            mode="required",
            phase="before_frame",
            bridge_tool=Path("bridge.py"),
            state_file=Path("state.json"),
            maximum_age_s=5.0,
            snapshot_reader=unavailable,
        )


def test_optional_temperature_mode_records_missing_snapshot_without_delta():
    def unavailable(*_args):
        raise FileNotFoundError("state file not found")

    before = observe_temperature(
        mode="optional",
        phase="before_frame",
        bridge_tool=Path("bridge.py"),
        state_file=Path("state.json"),
        maximum_age_s=5.0,
        snapshot_reader=unavailable,
    )
    after = observe_temperature(
        mode="optional",
        phase="after_frame",
        bridge_tool=Path("bridge.py"),
        state_file=Path("state.json"),
        maximum_age_s=5.0,
        snapshot_reader=unavailable,
    )

    monitoring = build_temperature_monitoring(
        mode="optional", before_frame=before, after_frame=after
    )
    assert monitoring["status"] == "NOT_OBSERVED"
    assert "FileNotFoundError: state file not found" in monitoring["reason"]
    assert monitoring["temperature_change"] is None


def test_disabled_temperature_mode_never_reads_snapshot():
    def must_not_run(*_args):
        raise AssertionError("snapshot reader must not be called")

    before = observe_temperature(
        mode="disabled",
        phase="before_frame",
        bridge_tool=Path("bridge.py"),
        state_file=Path("state.json"),
        maximum_age_s=5.0,
        snapshot_reader=must_not_run,
    )
    after = observe_temperature(
        mode="disabled",
        phase="after_frame",
        bridge_tool=Path("bridge.py"),
        state_file=Path("state.json"),
        maximum_age_s=5.0,
        snapshot_reader=must_not_run,
    )

    monitoring = build_temperature_monitoring(
        mode="disabled", before_frame=before, after_frame=after
    )
    assert monitoring["status"] == "DISABLED"
    assert monitoring["reason"] == "temperature_monitoring_disabled_for_this_run"
    assert monitoring["temperature_change"] is None
