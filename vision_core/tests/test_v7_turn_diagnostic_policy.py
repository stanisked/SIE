"""Offline checks for the one-turn camera diagnostic; no ROS or motor access."""

import ast
import math
import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / 'tools/run_v7_supervised_approach.py'


def load_steering():
    tree = ast.parse(SOURCE.read_text())
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'steering_request')
    namespace = {'math': math, 'TURN_DEADBAND_DEG': 1.5,
                 'MAX_TURN_PER_STEP_DEG': 3.0}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'),
         namespace)
    return namespace['steering_request'], tree


class TurnDiagnosticPolicyTests(unittest.TestCase):
    def test_one_turn_diagnostic_requests_3_degrees_in_target_direction(self):
        steer, _ = load_steering()
        self.assertEqual(steer(2.555, diagnostic_turn_only=True),
                         {'endpoint': '/turn-right', 'angle_deg': 3.0})
        self.assertEqual(steer(-2.555, diagnostic_turn_only=True),
                         {'endpoint': '/turn-left', 'angle_deg': 3.0})
        self.assertIsNone(steer(1.4, diagnostic_turn_only=True))
        self.assertAlmostEqual(steer(2.555)['angle_deg'], 1.2775)

    def test_turn_only_exits_before_forward_dispatch(self):
        _, tree = load_steering()
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        loops = [node for node in ast.walk(main) if isinstance(node, ast.For)]
        diagnostic_break = [node for loop in loops for node in loop.body
                            if isinstance(node, ast.If) and
                            ast.unparse(node.test) == 'args.diagnostic_turn_only' and
                            any(isinstance(child, ast.Break) for child in node.body)]
        self.assertEqual(len(diagnostic_break), 1)
        forward_dispatch = [node for node in ast.walk(main) if isinstance(node, ast.Call)
                            and isinstance(node.func, ast.Name)
                            and node.func.id == 'execute_observed_step']
        self.assertEqual(len(forward_dispatch), 1)
        self.assertLess(diagnostic_break[0].lineno, forward_dispatch[0].lineno)


if __name__ == '__main__':
    unittest.main()
