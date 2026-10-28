"""Dry-run contract checks; no ROS, controller, camera or motors required."""
import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest
from urllib.error import URLError
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
one.EXTRINSIC_SHA = 'extrinsic'
one.MAX_AGE_S = .5
one.MODEL_SHA = 'model'
one.SOURCE_HASHES = {'ar0234_intrinsic': 'rgb',
                     'ov9281_stereo_calibration': 'stereo'}
def fake_check_observation(audit, armed, now):
    assert audit['stamp'] >= armed and now - audit['stamp'] < .5
    return {'oldest_driver_stamp_s': audit['stamp'],
            'source_cycle_id': audit['cycle_id'],
            'target_box_xyxy_px': audit['box']}
approach = module('vision_core.tools.run_v7_supervised_approach')
approach.depth_and_box = lambda a: (None, a['box'])
approach.overlap = lambda a, b: max(0, min(a[2], b[2]) - max(a[0], b[0])) / max(a[2]-a[0], b[2]-b[0])
approach.target_bearing_deg = lambda box: (box[0]+box[2])/2 - 50

spec = importlib.util.spec_from_file_location(
    'live', Path(__file__).resolve().parents[1] / 'tools' /
    'run_v7_live_steered_forward.py')
live = importlib.util.module_from_spec(spec)
spec.loader.exec_module(live)
real_check_live_rgb_target = live.check_live_rgb_target
live.check_live_rgb_target = fake_check_observation
for name, original in saved_modules.items():
    if original is None:
        sys.modules.pop(name, None)
    else:
        sys.modules[name] = original


class FakeController:
    def __init__(self, terminal_after=1, starting_steer_rejections=0):
        self.calls = []
        self.active = False
        self.status_reads_during_active = 0
        self.terminal_after = terminal_after
        self.starting_steer_rejections = starting_steer_rejections

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
                'steered_safety_latched': False,
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
                    base.update(state='DRIVING', active_command_id=self.command_id,
                                right_pwm=170, left_pwm=135)
            return 200, base
        if endpoint == '/move-forward-steered':
            self.command_id = query['command_id']
            self.active = True
            return 202, {'accepted': True, 'command_id': self.command_id}
        if endpoint == '/steer-update':
            assert self.active
            if query['bias_pwm'] != '0' and self.starting_steer_rejections:
                self.starting_steer_rejections -= 1
                return 409, {'accepted': False,
                             'reason': 'STEER_ONLY_AFTER_DRIVE_STARTED'}
            return 200, {'accepted': True, 'seq': int(query['seq'])}
        if endpoint == '/stop':
            self.active = False
            return 200, {'accepted': True, 'stopped': True}
        raise AssertionError(endpoint)


class LiveSteeringTest(unittest.TestCase):
    def test_transport_timeout_classification(self):
        self.assertTrue(live.is_transport_timeout(TimeoutError('timed out')))
        self.assertTrue(live.is_transport_timeout(URLError(TimeoutError('timed out'))))
        self.assertFalse(live.is_transport_timeout(URLError('connection refused')))

    def test_lost_move_reply_retries_same_id_after_new_camera_frame(self):
        import tempfile
        class LostMoveReply(FakeController):
            def __init__(self):
                super().__init__()
                self.move_attempts = 0
            def __call__(self, method, url, timeout):
                if urlsplit(url).path == '/move-forward-steered':
                    self.move_attempts += 1
                    if self.move_attempts == 1:
                        super().__call__(method, url, timeout)
                        raise TimeoutError('reply lost after acceptance')
                    self.calls.append((method, '/move-forward-steered',
                        {k: v[0] for k,v in parse_qs(urlsplit(url).query).items()}))
                    return 200, {'accepted': False, 'duplicate': True,
                        'command_id': self.command_id, 'command_state': 'STARTING'}
                return super().__call__(method, url, timeout)
        controller = LostMoveReply()
        def camera(armed):
            for i, center in enumerate((50, 52, 54)):
                yield {'cycle_id': str(i), 'stamp': time.monotonic(),
                       'box': [center-10, 0, center+10, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://192.0.2.1',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(report['movement_posts'], 2)
        self.assertTrue(report['movement_first_response_timeout'])
        moves = [c[2] for c in controller.calls if c[1] == '/move-forward-steered']
        self.assertEqual(len({c['command_id'] for c in moves}), 1)

    def test_lost_move_reply_with_missing_target_stops_without_retry(self):
        import tempfile
        class LostMoveReply(FakeController):
            def __call__(self, method, url, timeout):
                result = super().__call__(method, url, timeout)
                if urlsplit(url).path == '/move-forward-steered':
                    raise TimeoutError('reply lost after acceptance')
                return result
        controller = LostMoveReply()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'missing', 'stamp': time.monotonic()}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://192.0.2.1',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'STOPPED_ON_REFUSAL')
        self.assertEqual(report['movement_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        self.assertEqual(len([c for c in controller.calls
                              if c[1] == '/move-forward-steered']), 1)

    def test_non_timeout_move_transport_error_never_retries(self):
        import tempfile
        class RejectedMove(FakeController):
            def __call__(self, method, url, timeout):
                if urlsplit(url).path == '/move-forward-steered':
                    self.calls.append((method, '/move-forward-steered', {}))
                    raise URLError('connection refused')
                return super().__call__(method, url, timeout)
        controller = RejectedMove()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://192.0.2.1',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'STOPPED_ON_REFUSAL')
        self.assertEqual(report['movement_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)

    def test_one_lost_steer_reply_can_be_superseded_by_new_frame(self):
        import tempfile
        class LostSteerReply(FakeController):
            def __init__(self):
                super().__init__()
                self.lost = False
            def __call__(self, method, url, timeout):
                if urlsplit(url).path == '/steer-update' and not self.lost:
                    self.lost = True
                    super().__call__(method, url, timeout)
                    raise TimeoutError('reply lost after acceptance')
                return super().__call__(method, url, timeout)
        controller = LostSteerReply()
        def camera(armed):
            for i, center in enumerate((50, 53, 56)):
                yield {'cycle_id': str(i), 'stamp': time.monotonic(),
                       'box': [center-10, 0, center+10, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://192.0.2.1',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(report['steer_posts'], 2)
        self.assertEqual([x['seq'] for x in report['unconfirmed_steer_updates']], [1])
        self.assertEqual([x['seq'] for x in report['updates']], [2])

    def test_second_lost_steer_reply_stops(self):
        import tempfile
        class LostSteerReplies(FakeController):
            def __call__(self, method, url, timeout):
                result = super().__call__(method, url, timeout)
                if urlsplit(url).path == '/steer-update':
                    raise TimeoutError('reply lost after acceptance')
                return result
        controller = LostSteerReplies()
        def camera(armed):
            for i, center in enumerate((50, 53, 56)):
                yield {'cycle_id': str(i), 'stamp': time.monotonic(),
                       'box': [center-10, 0, center+10, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://192.0.2.1',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'STOPPED_ON_REFUSAL')
        self.assertEqual(report['steer_posts'], 2)
        self.assertEqual(report['stop_posts'], 1)
        self.assertEqual([x['seq'] for x in report['unconfirmed_steer_updates']], [1])

    def test_v3_refuses_controller_without_independent_safety_status(self):
        import tempfile
        def controller(method, url, timeout):
            self.assertEqual(urlsplit(url).path, '/status')
            return 200, {'firmware_build_id': live.BUILD,
                'steering_api_version': 'v1_active_forward_pwm_bias',
                'boot_session_id': 'A1B2C3D4E5F60708', 'state': 'READY',
                'bounded_fault_latched': False, 'active_command_id': None,
                'remote_stop_pending': False, 'right_pwm': 0, 'left_pwm': 0}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=lambda armed: iter(()))
        self.assertEqual(report['movement_posts'], 0)
        self.assertIn('INDEPENDENT_STEERED_SAFETY_NOT_READY', report['error'])

    def test_real_rgb_validator_accepts_moving_top_clipped_single_person(self):
        now = time.monotonic()
        instance = {'instance_index': 0, 'bbox_xyxy_px': [200, 0, 500, 1100],
                    'confidence': .91, 'truncated_top': True,
                    'truncated_left': False, 'truncated_right': False}
        audit = {'schema_version': 'sie.ar0234_ov9281.control_input_preview.v1',
                 'cycle_id': 'current',
                 'calibration': {'ar0234_intrinsic_sha256': 'rgb',
                                 'ov9281_stereo_sha256': 'stereo',
                                 'ar0234_to_physical_left_extrinsic_sha256': 'extrinsic'},
                 'working_targets': {'freshness_policy_id':
                                     'experimental_freshness_500ms_v1',
                                     'exposure_to_command_age_max_s': .5},
                 'segmentation_observation': {'cycle_id': 'current',
                     'status': 'CURRENT_INSTANCES_DIAGNOSTIC_ONLY',
                     'model_sha256': 'model', 'class_name': 'person',
                     'arrays_sha256': 'arrays', 'instances': [instance]},
                 'ar0234_dynamic_target': {'disposition': 'UNAVAILABLE',
                     'reason': 'NO_CURRENT_NON_EDGE_TARGET',
                     'candidate': None, 'current_eligible_count': 0},
                 'existing_profile_static_gate': {'status': 'FAIL',
                     'reason': 'MOTION_DETECTED'},
                 'frame_freshness': {'capture_backend': 'latest_v4l2',
                     'frames': {'ar0234': {'driver_timestamp_s': now - .1,
                         'host_packet_received_monotonic_s': now - .05,
                         'flags': 0x12000}}}}
        evidence = real_check_live_rgb_target(audit, now - .2, now)
        self.assertEqual(evidence['target_box_xyxy_px'], instance['bbox_xyxy_px'])
        self.assertTrue(evidence['top_truncated'])
        self.assertFalse(evidence['static_depth_gate_used'])
        weak_edge = {'instance_index': 1, 'bbox_xyxy_px': [0, 600, 250, 950],
                     'confidence': .33, 'truncated_top': False,
                     'truncated_left': True, 'truncated_right': False}
        audit['segmentation_observation']['instances'].append(weak_edge)
        audit['ar0234_dynamic_target'] = {'disposition': 'OBSERVED',
            'reason': 'DIRECT_CURRENT_FRAME_SINGLE_TARGET',
            'candidate': instance, 'current_eligible_count': 1}
        evidence = real_check_live_rgb_target(audit, now - .2, now)
        self.assertEqual(evidence['target_box_xyxy_px'], instance['bbox_xyxy_px'])
        weak_edge['confidence'] = .82
        with self.assertRaisesRegex(ValueError, 'AMBIGUOUS'):
            real_check_live_rgb_target(audit, now - .2, now)
        audit['segmentation_observation']['instances'].pop()
        audit['segmentation_observation']['instances'].append(dict(instance))
        with self.assertRaisesRegex(ValueError, 'AMBIGUOUS'):
            real_check_live_rgb_target(audit, now - .2, now)
        audit['segmentation_observation']['instances'].pop()
        instance['bbox_xyxy_px'] = [0, 0, 500, 1100]
        instance['truncated_left'] = True
        with self.assertRaisesRegex(ValueError, 'GEOMETRY'):
            real_check_live_rgb_target(audit, now - .2, now)

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
        self.assertNotIn('/status', paths[paths.index('/move-forward-steered') + 1:
                                          paths.index('/steer-update')])
        self.assertEqual(controller.calls[paths.index('/steer-update')][2]['bias_pwm'], '20')

    def test_confirmed_driving_uses_steer_ack_without_per_frame_status(self):
        import tempfile
        controller = FakeController()
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
        paths = [call[1] for call in controller.calls]
        first_update = paths.index('/steer-update')
        second_update = paths.index('/steer-update', first_update + 1)
        self.assertNotIn('/status', paths[first_update + 1:second_update])
        self.assertEqual(len(report['updates']), 2)

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
        controller = FakeController(starting_steer_rejections=1)
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
        self.assertEqual(report['steer_posts'], 3)
        attempts = [call[2] for call in controller.calls if call[1] == '/steer-update']
        self.assertEqual([x['bias_pwm'] for x in attempts], ['12', '0', '20'])
        self.assertEqual([x['seq'] for x in attempts], ['1', '1', '2'])

    def test_braking_fault_skips_steering_and_confirms_stop_after_http_timeout(self):
        import tempfile
        class BrakingFaultController:
            def __init__(self):
                self.calls = []
                self.moved = False
                self.stop_attempted = False
            def __call__(self, method, url, timeout):
                endpoint = urlsplit(url).path
                self.calls.append(endpoint)
                if endpoint == '/status':
                    state = ('FAULT' if self.stop_attempted else
                             'BRAKING' if self.moved else 'READY')
                    return 200, {'firmware_build_id': live.BUILD,
                        'steering_api_version': 'v1_active_forward_pwm_bias',
                        'boot_session_id': 'A1B2C3D4E5F60708', 'state': state,
                        'bounded_fault_latched': self.moved,
                        'steered_safety_latched': False,
                        'bounded_fault_reason': 'BREAKAWAY_NOT_FOUND' if self.moved else None,
                        'active_command_id': self.command_id if self.moved and not self.stop_attempted else None,
                        'last_command_id': self.command_id if self.stop_attempted else None,
                        'last_command_state': 'FAULT' if self.stop_attempted else None,
                        'remote_stop_pending': False, 'right_pwm': 0, 'left_pwm': 0}
                if endpoint == '/move-forward-steered':
                    self.command_id = parse_qs(urlsplit(url).query)['command_id'][0]
                    self.moved = True
                    return 202, {'accepted': True, 'command_id': self.command_id}
                if endpoint == '/stop':
                    self.stop_attempted = True
                    raise TimeoutError('response lost after stop')
                if endpoint == '/steer-update':
                    return 409, {'accepted': False,
                                 'reason': 'NO_MATCHING_ACTIVE_STEERED_FORWARD'}
                raise AssertionError('unexpected endpoint ' + endpoint)
        controller = BrakingFaultController()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'second', 'stamp': time.monotonic() + .001,
                   'box': [43, 0, 63, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_FAULT_STOPPED')
        self.assertIn('CONTROLLER_FAULT_DURING_MOVE', report['error'])
        self.assertEqual(report['steer_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        self.assertIn('stop_error', report)
        self.assertEqual(report['confirmed_stop_status']['state'], 'FAULT')
        self.assertEqual(report['updates'], [])

    def test_normal_braking_waits_for_terminal_without_steer_updates(self):
        import tempfile
        class BrakingController(FakeController):
            def __init__(self):
                super().__init__()
                self.braking_reads = 0
            def __call__(self, method, url, timeout):
                endpoint = urlsplit(url).path
                if endpoint == '/steer-update':
                    self.calls.append((method, endpoint, {}))
                    return 409, {'accepted': False,
                                 'reason': 'NO_MATCHING_ACTIVE_STEERED_FORWARD'}
                if endpoint == '/status' and self.active:
                    self.calls.append((method, endpoint, {}))
                    self.braking_reads += 1
                    base = {'firmware_build_id': live.BUILD,
                        'steering_api_version': 'v1_active_forward_pwm_bias',
                        'boot_session_id': 'A1B2C3D4E5F60708',
                        'state': 'BRAKING', 'active_command_id': self.command_id,
                        'last_command_id': None, 'last_command_state': None,
                        'bounded_fault_latched': False,
                        'steered_safety_latched': False,
                        'remote_stop_pending': False, 'right_pwm': 0, 'left_pwm': 0}
                    if self.braking_reads >= 3:
                        self.active = False
                        base.update(state='READY', active_command_id=None,
                                    last_command_id=self.command_id,
                                    last_command_state='SUCCESS')
                    return 200, base
                return super().__call__(method, url, timeout)
        controller = BrakingController()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            while True:
                time.sleep(.04)
                yield {'cycle_id': 'moving', 'stamp': time.monotonic(),
                       'box': [43, 0, 63, 100]}
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(report['steer_posts'], 1)
        self.assertEqual(report['terminal_status']['last_command_state'], 'SUCCESS')

    def test_steer_rejected_after_bounded_braking_waits_for_terminal(self):
        import tempfile
        class BrakingRaceController(FakeController):
            def __init__(self):
                super().__init__(terminal_after=999)
                self.braking = False
                self.braking_reads = 0
            def __call__(self, method, url, timeout):
                endpoint = urlsplit(url).path
                if endpoint == '/steer-update':
                    self.calls.append((method, endpoint, {}))
                    self.braking = True
                    return 409, {'accepted': False,
                                 'reason': 'NO_MATCHING_ACTIVE_STEERED_FORWARD'}
                if endpoint == '/status' and self.braking and self.active:
                    self.calls.append((method, endpoint, {}))
                    self.braking_reads += 1
                    if self.braking_reads == 1:
                        return 200, {'boot_session_id': 'A1B2C3D4E5F60708',
                                     'state': 'BRAKING',
                                     'active_command_id': self.command_id,
                                     'last_command_id': None,
                                     'last_command_state': None}
                    self.active = False
                    return 200, {'boot_session_id': 'A1B2C3D4E5F60708',
                                 'state': 'READY', 'active_command_id': None,
                                 'last_command_id': self.command_id,
                                 'last_command_state': 'SUCCESS'}
                return super().__call__(method, url, timeout)
        controller = BrakingRaceController()
        def camera(armed):
            yield {'cycle_id': 'first', 'stamp': time.monotonic(),
                   'box': [40, 0, 60, 100]}
            yield {'cycle_id': 'second', 'stamp': time.monotonic(),
                   'box': [43, 0, 63, 100]}
            while True:
                time.sleep(.04)
                yield None
        with tempfile.TemporaryDirectory() as root:
            report = live.run_session(base_url='http://10.13.32.229',
                front_clearance_m=2.5, output=root + '/run',
                request=controller, camera_factory=camera)
        self.assertEqual(report['result'], 'TERMINAL_OBSERVED_DIAGNOSTIC_ONLY')
        self.assertEqual(report['steer_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        self.assertEqual(report['terminal_status']['last_command_state'], 'SUCCESS')


if __name__ == '__main__':
    unittest.main()

