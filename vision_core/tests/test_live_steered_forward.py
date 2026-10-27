"""Dry-run contract checks; no ROS, controller, camera or motors required."""
import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest
from urllib.parse import parse_qs, urlsplit


def module(name):
    obj = types.ModuleType(name)
    sys.modules[name] = obj
    return obj


names = ('firmware', 'firmware.esp32', 'firmware.esp32.tools',
         'vision_core', 'vision_core.tools',
         'firmware.esp32.tools.run_remote_stop_trial',
         'vision_core.tools.run_v7_supervised_one_step',
         'vision_core.tools.run_v7_supervised_approach')
saved_modules = {name: sys.modules.get(name) for name in names}
for name in names[:5]:
    module(name)
trial = module('firmware.esp32.tools.run_remote_stop_trial')
trial.Http = object
def preflight(status, *, accepted_builds):
    assert status['firmware_build_id'] in accepted_builds
    assert status['state'] == 'READY'
trial.preflight = preflight
one = module('vision_core.tools.run_v7_supervised_one_step')
one.TOPIC = '/test'
def check_observation(audit, armed, now, *, supervised_rgb_only):
    assert supervised_rgb_only
    assert audit['stamp'] >= armed and now - audit['stamp'] < .5
    return {'oldest_driver_stamp_s': audit['stamp'], 'source_cycle_id': audit['cycle_id']}
one.check_observation = check_observation
approach = module('vision_core.tools.run_v7_supervised_approach')
approach.depth_and_box = lambda a: (None, a['box'])
approach.overlap = lambda a, b: max(0, min(a[2], b[2]) - max(a[0], b[0])) / max(a[2]-a[0], b[2]-b[0])
approach.target_bearing_deg = lambda box: (box[0]+box[2])/2 - 50

spec = importlib.util.spec_from_file_location(
    'live', Path(__file__).resolve().parents[1] / 'tools' /
    'run_v7_live_steered_forward.py')
live = importlib.util.module_from_spec(spec)
spec.loader.exec_module(live)
for name, original in saved_modules.items():
    if original is None:
        sys.modules.pop(name, None)
    else:
        sys.modules[name] = original


class FakeController:
    def __init__(self, terminal_after=2, starting_reads=0):
        self.calls = []
        self.active = False
        self.status_reads_during_active = 0
        self.terminal_after = terminal_after
        self.starting_reads = starting_reads

    def __call__(self, method, url, timeout):
        endpoint = urlsplit(url).path
        query = {k: v[0] for k, v in parse_qs(urlsplit(url).query).items()}
        self.calls.append((method, endpoint, query))
        base = {'firmware_build_id': live.BUILD,
                'steering_api_version': 'v1_active_forward_pwm_bias',
                'boot_session_id': 'A1B2C3D4E5F60708',
                'state': 'READY', 'right_pwm': 0, 'left_pwm': 0,
                'remote_stop_pending': False,
                'bounded_fault_latched': False,
                'active_command_id': None,
                'last_command_id': None, 'last_command_state': None}
        if endpoint == '/status':
            if self.active:
                self.status_reads_during_active += 1
                if self.status_reads_during_active >= self.terminal_after:
                    self.active = False
                    base.update(last_command_id=self.command_id,
                                last_command_state='SUCCESS')
                else:
                    state = ('STARTING' if self.status_reads_during_active <= self.starting_reads
                             else 'DRIVING')
                    base.update(state=state, active_command_id=self.command_id,
                                right_pwm=170, left_pwm=135)
            return 200, base
        if endpoint == '/move-forward-steered':
            self.command_id = query['command_id']
            self.active = True
            return 202, {'accepted': True, 'command_id': self.command_id}
        if endpoint == '/steer-update':
            assert self.active
            return 200, {'accepted': True, 'seq': int(query['seq'])}
        if endpoint == '/stop':
            self.active = False
            return 200, {'accepted': True, 'stopped': True}
        raise AssertionError(endpoint)


class LiveSteeringTest(unittest.TestCase):
    def test_bias_and_correspondence(self):
        self.assertEqual(live.steering_bias(0), 0)
        self.assertEqual(live.steering_bias(3), 12)
        self.assertEqual(live.steering_bias(-10), -20)
        self.assertTrue(live.same_target([0, 0, 100, 200], [25, 0, 125, 200]))
        self.assertFalse(live.same_target([0, 0, 100, 200], [200, 0, 300, 200]))

    def test_camera_preview_journals_straight_then_bias_without_network(self):
        import tempfile
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'second', 'stamp': time.monotonic(),
                   'box': [43, 0, 63, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.preview_session(output=root + '/preview', camera_factory=camera)
        self.assertEqual(report['motor_posts'], 0)
        self.assertEqual(report['network_requests'], 0)
        self.assertEqual(report['rows'][0]['intended_bias_pwm'], 0)
        self.assertGreater(report['rows'][1]['intended_bias_pwm'], 0)

    def test_preview_records_loss_and_reacquires_without_motor_posts(self):
        import tempfile
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'missing', 'ar0234_dynamic_target': {
                   'current_eligible_count': 0}, 'segmentation_observation': {
                   'instances': []}}
            yield {'cycle_id': 'new', 'stamp': time.monotonic(),
                   'box': [30, 0, 50, 100]}
            yield {'cycle_id': 'moved', 'stamp': time.monotonic(),
                   'box': [28, 0, 48, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.preview_session(output=root + '/preview', camera_factory=camera)
        self.assertEqual(report['result'], 'PREVIEW_RECORDED_WITH_LOSS')
        self.assertEqual(report['target_loss_events'], 1)
        self.assertEqual(report['acquisition_segments'], 2)
        self.assertEqual(report['motor_posts'], 0)
        self.assertEqual([row['phase'] for row in report['rows']],
                         ['ACQUIRE_STRAIGHT', 'TARGET_UNAVAILABLE',
                          'ACQUIRE_STRAIGHT', 'LIVE_STEER'])
        self.assertLess(report['rows'][-1]['intended_bias_pwm'], 0)

    def test_first_frame_straight_second_frame_steers_same_command(self):
        import tempfile
        controller = FakeController()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'second', 'stamp': time.monotonic() + .001,
                   'box': [45, 0, 65, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(report['movement_posts'], 1)
        self.assertEqual(report['steer_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        paths = [x[1] for x in controller.calls]
        self.assertLess(paths.index('/move-forward-steered'), paths.index('/steer-update'))
        self.assertEqual(controller.calls[paths.index('/steer-update')][2]['bias_pwm'], '20')

    def test_lost_camera_stream_stops_without_steering(self):
        import tempfile
        controller = FakeController(terminal_after=999)
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'STOPPED_ON_REFUSAL')
        self.assertIn('CAMERA_FEEDBACK_TIMEOUT', report['error'])
        self.assertEqual(report['movement_posts'], 1)
        self.assertEqual(report['steer_posts'], 0)
        self.assertEqual(report['stop_posts'], 1)

    def test_startup_updates_are_zero_until_driving(self):
        import tempfile
        controller = FakeController(terminal_after=3, starting_reads=1)
        def camera(armed):
            for i, center in enumerate((50, 53, 56)):
                yield {'cycle_id': str(i), 'stamp': time.monotonic(),
                       'box': [center-10, 0, center+10, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual([x['bias_pwm'] for x in report['updates']], [0, 20])


if __name__ == '__main__':
    unittest.main()
