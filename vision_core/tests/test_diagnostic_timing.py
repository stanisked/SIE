"""Deterministic accounting tests without sleeping or requiring ROS."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2/sie_ros2/sie_ros2"))
from diagnostic_timing import StageTimings, timed_stage


class TimingTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.timer = StageTimings(lambda: self.now)

    def advance(self, seconds):
        self.now += seconds
        return "unchanged result"

    def test_nested_exclusive_durations_do_not_double_count(self):
        def outer():
            self.advance(1)
            self.timer.call("matcher", self.advance, 2)
            self.advance(3)
        self.timer.call("association", outer)
        self.advance(.5)
        snapshot = self.timer.snapshot()
        self.assertEqual(snapshot["stages"]["association"]["inclusive_s"], 6.)
        self.assertEqual(snapshot["stages"]["association"]["exclusive_s"], 4.)
        self.assertEqual(snapshot["stages"]["matcher"]["exclusive_s"], 2.)
        self.assertEqual(snapshot["unattributed_s"], .5)

    def test_exception_keeps_timing_and_original_error(self):
        error = ValueError("original refusal")
        def fail():
            self.advance(2)
            raise error
        with self.assertRaises(ValueError) as caught:
            self.timer.call("capture", fail)
        self.assertIs(caught.exception, error)
        self.assertEqual(self.timer.snapshot()["stages"]["capture"]["exclusive_s"], 2.)

    def test_repeated_calls_accumulate_and_snapshots_are_independent(self):
        for _ in range(3):
            self.assertEqual(self.timer.call("write", self.advance, .5), "unchanged result")
        snapshot = self.timer.snapshot()
        self.assertEqual(snapshot["stages"]["write"]["calls"], 3)
        self.assertEqual(snapshot["stages"]["write"]["exclusive_s"], 1.5)
        snapshot["stages"]["write"]["exclusive_s"] = -1
        self.assertEqual(self.timer.snapshot()["stages"]["write"]["exclusive_s"], 1.5)
        self.assertEqual(StageTimings(lambda: self.now).snapshot()["stages"], {})

    def test_decorator_preserves_disabled_behavior_and_return_value(self):
        class Example:
            @timed_stage("example")
            def method(self, value):
                return value
        example = Example()
        token = object()
        self.assertIs(example.method(token), token)
        example._stage_timings = self.timer
        self.assertIs(example.method(token), token)
        self.assertEqual(self.timer.snapshot()["stages"]["example"]["calls"], 1)


if __name__ == "__main__":
    unittest.main()
