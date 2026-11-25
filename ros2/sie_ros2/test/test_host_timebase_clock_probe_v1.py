"""Pure offline tests for the opt-in read-only clock sampler."""
import pytest

from sie_ros2.host_timebase_clock_probe_v1 import collect_host_clock_brackets


def test_sampled_clock_order_and_requested_spacing(monkeypatch):
    from sie_ros2 import host_timebase_clock_probe_v1 as probe

    readings = iter([
        100, 10_000_000_155, 200,
        10_000_100, 10_010_000_155, 10_000_200,
        20_000_100, 10_020_000_155, 20_000_200,
    ])
    observed = []
    def clock_gettime_ns(clock_id):
        observed.append(clock_id)
        return next(readings)

    slept = []
    monkeypatch.setattr(probe.time, "clock_gettime_ns", clock_gettime_ns)
    monkeypatch.setattr(probe.time, "sleep", lambda dt: slept.append(dt))

    rows = collect_host_clock_brackets(count=3, interval_ms=10)
    assert rows == [
        {"monotonic_before_ns": 100, "realtime_ns": 10_000_000_155, "monotonic_after_ns": 200},
        {"monotonic_before_ns": 10_000_100, "realtime_ns": 10_010_000_155, "monotonic_after_ns": 10_000_200},
        {"monotonic_before_ns": 20_000_100, "realtime_ns": 10_020_000_155, "monotonic_after_ns": 20_000_200},
    ]
    assert observed == [
        probe.time.CLOCK_MONOTONIC, probe.time.CLOCK_REALTIME, probe.time.CLOCK_MONOTONIC
    ] * 3
    assert slept == [0.01, 0.01]


@pytest.mark.parametrize("count,interval", [
    (2, 10), (1001, 10), (True, 10), (3, -1), (3, 1001), (3, float("nan")),
])
def test_invalid_capture_configuration_refused(count, interval):
    with pytest.raises(ValueError):
        collect_host_clock_brackets(count=count, interval_ms=interval)
