"""Keep command-ID checks inside the bounded API that receives command_id."""

import unittest
from pathlib import Path


SKETCH = (Path(__file__).resolve().parents[1] /
          "traction_control_v4_2_bounded_motion_api" /
          "traction_control_v4_2_bounded_motion_api.ino")


class ReplayScopeTests(unittest.TestCase):
    def test_replay_gate_belongs_to_bounded_request(self):
        source = SKETCH.read_text()
        legacy = source.split("void queueLegacyMotionRequest(", 1)[1].split(
            "void handleStart()", 1)[0]
        bounded = source.split("void queueBoundedMotionRequest(", 1)[1].split(
            "void handleMoveForward()", 1)[0]
        self.assertNotIn("commandId", legacy)
        self.assertNotIn("acceptedCommandIds", legacy)
        self.assertEqual(bounded.count("acceptedCommandIds.contains(commandId.c_str()"), 1)
        self.assertLess(bounded.index("acceptedCommandIds.contains"),
                        bounded.index("const int8_t newIndex = reserveCommandRecord()"))


if __name__ == "__main__":
    unittest.main()
