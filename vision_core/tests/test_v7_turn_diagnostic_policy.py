"""Offline checks for the one-turn camera diagnostic; no ROS or motor access."""

import ast
import json
import math
from tempfile import TemporaryDirectory
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


def load_steering_schedule():
    tree = ast.parse(SOURCE.read_text())
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == 'steering_due_before_step')
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'),
         namespace)
    return namespace['steering_due_before_step']


def load_progress_policy():
    tree = ast.parse(SOURCE.read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in {'operator_budget_valid', 'encoder_step_progress'}]
    namespace = {'math': math,
                 'RIGHT_M_PER_COUNT': math.pi * 0.0646 / 205.0,
                 'LEFT_M_PER_COUNT': math.pi * 0.0652 / 206.0,
                 'MIN_ENCODER_STEP_M': 0.01,
                 'MAX_ENCODER_STEP_M': 0.15}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), 'exec'),
         namespace)
    return namespace['operator_budget_valid'], namespace['encoder_step_progress'], tree


class TurnDiagnosticPolicyTests(unittest.TestCase):
    def test_first_approach_step_starts_straight_then_enables_correction(self):
        steering_due = load_steering_schedule()
        self.assertFalse(steering_due(0))
        self.assertTrue(steering_due(1))
        self.assertTrue(steering_due(0, diagnostic_turn_only=True))

    def test_rgb_box_remains_available_without_metric_depth(self):
        tree = ast.parse(SOURCE.read_text())
        names = {'depth_and_box', 'diagnostic_depth_delta'}
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in names]
        namespace = {'math': math}
        exec(compile(ast.Module(body=functions, type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
        audit = {'ar0234_dynamic_target': {'candidate':
                 {'bbox_xyxy_px': [300., 100., 700., 900.]}},
                 'shape_diagnostic': {'mask_depth':
                 {'status': 'DEPTH_MAD_EXCEEDED'}}}
        self.assertEqual(namespace['depth_and_box'](audit),
                         (None, [300., 100., 700., 900.]))
        self.assertIsNone(namespace['diagnostic_depth_delta'](None, 3.0))
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        self.assertIn('if previous_box is not None', ast.unparse(main))

    def test_encoder_progress_replaces_optical_delta_gate(self):
        valid_budget, progress, tree = load_progress_policy()
        self.assertTrue(valid_budget(5.0, 11))
        self.assertFalse(valid_budget(2.5, 14))
        self.assertFalse(valid_budget(2.5, 0))
        self.assertAlmostEqual(progress({'right_count': 44, 'left_count': 38})
                               ['mean_distance_m'], 0.0406719229)
        self.assertAlmostEqual(progress({'right_count': 63, 'left_count': 58})
                               ['mean_distance_m'], 0.0600200719)
        for status in ({'right_count': 0, 'left_count': 38},
                       {'right_count': 200, 'left_count': 200},
                       {'right_count': True, 'left_count': 38}):
            with self.subTest(status=status), self.assertRaises(RuntimeError):
                progress(status)
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        self.assertTrue(any(isinstance(node, ast.Call) and
                            isinstance(node.func, ast.Name) and
                            node.func.id == 'encoder_step_progress'
                            for node in ast.walk(main)))
        self.assertNotIn('DEPTH_PROGRESS_UNCONFIRMED', ast.unparse(main))

    def test_one_turn_diagnostic_requests_3_degrees_in_target_direction(self):
        steer, _ = load_steering()
        self.assertEqual(steer(2.555, diagnostic_turn_only=True),
                         {'endpoint': '/turn-right', 'angle_deg': 3.0})
        self.assertEqual(steer(-2.555, diagnostic_turn_only=True),
                         {'endpoint': '/turn-left', 'angle_deg': 3.0})
        self.assertIsNone(steer(1.4, diagnostic_turn_only=True))
        self.assertAlmostEqual(steer(2.555, reference_bearing_deg=0.0)['angle_deg'], 1.2775)

    def test_approach_turn_corrects_displacement_from_initial_target_anywhere(self):
        steer, _ = load_steering()
        self.assertIsNone(steer(2.0, reference_bearing_deg=2.0))
        self.assertIsNone(steer(2.0, reference_bearing_deg=3.0))
        # An initially right-hand target that shifts left requests a left turn,
        # even if it has moved closer to the optical axis.
        self.assertEqual(steer(0.5, reference_bearing_deg=2.5)['endpoint'],
                         '/turn-left')
        self.assertEqual(steer(-3.0, reference_bearing_deg=-1.0)['endpoint'],
                         '/turn-left')
        self.assertEqual(steer(3.2, reference_bearing_deg=1.6)['endpoint'],
                         '/turn-right')
        self.assertEqual(steer(2.0, reference_bearing_deg=2.0,
                               diagnostic_turn_only=True)['endpoint'],
                         '/turn-right')
        with self.assertRaisesRegex(ValueError, 'INVALID_REFERENCE_BEARING'):
            steer(2.0, reference_bearing_deg=math.nan)
        with self.assertRaisesRegex(ValueError, 'INVALID_REFERENCE_BEARING'):
            steer(2.0)

    def test_turn_feedback_improves_initial_coordinate_even_when_off_axis(self):
        tree = ast.parse(SOURCE.read_text())
        names = {'overlap', 'target_center_x', 'target_bearing_deg',
                 'verify_turn_feedback'}
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in names]
        namespace = {'math': math, 'RGB_CX_PX': 924.649158,
                     'RGB_FX_PX': 2021.546012}
        exec(compile(ast.Module(body=functions, type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
        before = [725., 0., 1125., 800.]
        after = [745., 0., 1145., 800.]
        result = namespace['verify_turn_feedback'](
            None, before, None, after, reference_bearing_deg=2.0)
        self.assertGreater(result, namespace['target_bearing_deg'](before))
        with self.assertRaisesRegex(RuntimeError,'TURN_CAMERA_ALIGNMENT_UNCONFIRMED'):
            namespace['verify_turn_feedback'](
                None, before, None, after, reference_bearing_deg=-2.0)

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

    def test_execute_flag_replaces_interactive_token(self):
        _, tree = load_steering()
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        self.assertFalse(any(isinstance(node, ast.Call) and
                             isinstance(node.func, ast.Name) and node.func.id == 'input'
                             for node in ast.walk(main)))
        execute_gate = next(node for node in main.body if isinstance(node, ast.If)
                            and ast.unparse(node.test) == 'not args.execute')
        ros_start = next(node for node in ast.walk(main) if isinstance(node, ast.Call)
                         and isinstance(node.func, ast.Attribute) and
                         ast.unparse(node.func) == 'rclpy.init')
        self.assertLess(execute_gate.lineno, ros_start.lineno)

    def test_read_only_preflight_can_finish_before_fresh_frame(self):
        _, tree = load_steering()
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and
                        node.name == 'preflight_controller_before_fresh_frame')
        now = [0.0]
        calls = []
        def request(method, url, timeout):
            calls.append((method, url, timeout))
            now[0] += .57  # Longer than the former 500 ms status deadline.
            return 200, {'boot_session_id': '0123456789ABCDEF', 'state': 'READY'}
        namespace = {'Http': None, 'preflight': lambda status: None,
                     'time': __import__('time'), 'json': json}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'turn_preflight.json'
            status, completed = namespace[function.name](
                base_url='http://controller', output=output, request=request,
                clock=lambda: now[0])
            self.assertEqual(status['state'], 'READY')
            self.assertEqual(completed, .57)
            self.assertEqual(calls, [('GET', 'http://controller/status', 1.0)])
            self.assertEqual(json.loads(output.read_text())['http_code'], 200)

    def test_status_timeout_retries_read_only_then_uses_new_frame_boundary(self):
        _, tree = load_steering()
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and
                        node.name == 'preflight_controller_before_fresh_frame')
        now = [0.0]
        calls = []
        def request(method, url, timeout):
            calls.append((method, timeout))
            now[0] += 1.12 if len(calls) == 1 else .15
            if len(calls) == 1:
                raise TimeoutError('timed out')
            return 200, {'state': 'READY'}
        namespace = {'Http': None, 'preflight': lambda status: None,
                     'time': __import__('time'), 'json': json}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'turn_preflight.json'
            status, completed = namespace[function.name](
                base_url='http://controller', output=output, request=request,
                clock=lambda: now[0])
            self.assertEqual(status['state'], 'READY')
            self.assertEqual(completed, 1.27)
            self.assertEqual(calls, [('GET', 1.0), ('GET', 2.0)])
            attempts = json.loads(output.read_text())['attempts']
            self.assertEqual(attempts[0]['error'], 'TimeoutError: timed out')
            self.assertEqual(attempts[1]['http_code'], 200)

    def test_two_status_timeouts_exit_without_motion(self):
        _, tree = load_steering()
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and
                        node.name == 'preflight_controller_before_fresh_frame')
        calls = []
        def request(method, url, timeout):
            calls.append(method)
            raise TimeoutError('timed out')
        namespace = {'Http': None, 'preflight': lambda status: None,
                     'time': __import__('time'), 'json': json}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     str(SOURCE), 'exec'), namespace)
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'turn_preflight.json'
            with self.assertRaises(TimeoutError):
                namespace[function.name](base_url='http://controller',
                                         output=output, request=request)
            self.assertEqual(calls, ['GET', 'GET'])
            self.assertEqual(len(json.loads(output.read_text())['attempts']), 2)

    def test_forward_status_precedes_fresh_frame_and_no_second_get(self):
        _, tree = load_steering()
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        loop = next(node for node in ast.walk(main)
                    if isinstance(node, ast.For) and ast.unparse(node.iter) == 'range(args.max_steps)')
        calls = [(node.lineno, node.func.id, node) for node in ast.walk(loop)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                 and node.func.id in {'preflight_controller_before_fresh_frame',
                                      'next_observation', 'execute_observed_step'}]
        preflight = max(line for line, name, _ in calls
                        if name == 'preflight_controller_before_fresh_frame')
        forward = next((line, node) for line, name, node in calls
                       if name == 'execute_observed_step')
        self.assertTrue(any(preflight < line < forward[0] and name == 'next_observation'
                            for line, name, _ in calls))
        self.assertIn('ready_status=ready_status', ast.unparse(forward[1]))
        self.assertIn('ready_status_at=ready_at', ast.unparse(forward[1]))

    def test_fresh_camera_frame_follows_preflight_before_turn_post(self):
        _, tree = load_steering()
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        align = next(node for node in ast.walk(main)
                     if isinstance(node, ast.FunctionDef) and node.name == 'align_once')
        calls = {}
        for node in ast.walk(align):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id in {'preflight_controller_before_fresh_frame',
                                         'next_observation', 'execute_bounded_turn'}):
                calls[node.func.id] = min(node.lineno,
                                          calls.get(node.func.id, node.lineno))
        self.assertLess(calls['preflight_controller_before_fresh_frame'],
                        calls['next_observation'])
        self.assertLess(calls['next_observation'], calls['execute_bounded_turn'])


if __name__ == '__main__':
    unittest.main()
