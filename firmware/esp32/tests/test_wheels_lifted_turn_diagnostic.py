"""Offline transport tests for one wheels-lifted turn; no controller access."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from urllib.parse import parse_qs, urlsplit

from firmware.esp32.tools.run_wheels_lifted_turn_diagnostic import run


def status():
    return {'firmware_build_id': 'v4_2_micro_turn_loaded_start_v4',
            'motion_api_version': 'v2_bounded_motion_api',
            'boot_session_id': 'B05C5663A9707458',
            'bounded_fault_latched': True,
            'bounded_fault_reason': 'BREAKAWAY_NOT_FOUND',
            'fault': 'BREAKAWAY_NOT_FOUND',
            'state': 'FAULT', 'active_command_id': None,
            'remote_stop_pending': False, 'square_state': 'IDLE',
            'right_pwm': 0, 'left_pwm': 0}


class Transport:
    def __init__(self, *, ack=True, turn_timeout=False, initially_ready=False):
        self.data = status()
        if initially_ready:
            self.data.update(bounded_fault_latched=False,
                             bounded_fault_reason=None, fault='', state='READY')
        self.calls = []
        self.ack = ack
        self.turn_timeout = turn_timeout
        self.time = 1.0
        self.command_id = None

    def clock(self):
        return self.time

    def sleep(self, amount):
        self.time += amount

    def request(self, method, url, timeout):
        parsed = urlsplit(url)
        route = parsed.path
        self.calls.append((method, route))
        self.time += 0.01
        if route == '/status':
            return 200, deepcopy(self.data)
        if route == '/ack-fault':
            if not self.ack:
                return 409, {'acknowledged': False}
            self.data.update(bounded_fault_latched=False,
                             bounded_fault_reason=None, fault='', state='READY')
            return 200, {'acknowledged': True,
                         'previous_fault_reason': 'BREAKAWAY_NOT_FOUND'}
        if route == '/turn-left':
            self.command_id = parse_qs(parsed.query)['command_id'][0]
            if self.turn_timeout:
                raise TimeoutError('ambiguous turn transport')
            self.data.update(bounded_fault_latched=True,
                             bounded_fault_reason='BREAKAWAY_NOT_FOUND',
                             fault='BREAKAWAY_NOT_FOUND', state='FAULT',
                             last_command_id=self.command_id,
                             last_command_endpoint='/turn-left',
                             last_command_state='FAULT')
            return 202, {'accepted': True, 'command_id': self.command_id}
        if route == '/stop':
            return 200, {'accepted': True, 'stopped': True}
        raise AssertionError((method, route))


class WheelsLiftedTurnTests(unittest.TestCase):
    def trial(self, transport):
        with TemporaryDirectory() as directory:
            return run(base_url='http://127.0.0.1', output=Path(directory)/'trial',
                       execute=True, wheels_lifted=True,
                       request=transport.request, clock=transport.clock,
                       sleep=transport.sleep)

    def test_latched_fault_ack_then_one_turn_and_one_stop(self):
        t = Transport()
        report = self.trial(t)
        self.assertEqual(report['result'], 'TURN_FAULT_OBSERVED')
        self.assertEqual((report['fault_ack_posts'], report['turn_posts'],
                          report['stop_posts']), (1, 1, 1))
        self.assertEqual(t.calls,
            [('GET', '/status'), ('POST', '/ack-fault'), ('GET', '/status'),
             ('POST', '/turn-left'), ('GET', '/status'), ('POST', '/stop'),
             ('GET', '/status')])

    def test_ready_after_reboot_needs_no_fault_ack(self):
        t = Transport(initially_ready=True)
        report = self.trial(t)
        self.assertEqual(report['fault_ack_posts'], 0)
        self.assertEqual(report['turn_posts'], 1)
        self.assertNotIn(('POST', '/ack-fault'), t.calls)
        self.assertEqual(t.calls.count(('POST', '/stop')), 1)

    def test_ack_refusal_never_sends_turn(self):
        t = Transport(ack=False)
        report = self.trial(t)
        self.assertEqual(report['turn_posts'], 0)
        self.assertEqual(report['stop_posts'], 0)
        self.assertEqual(t.calls[-1], ('POST', '/ack-fault'))

    def test_ambiguous_turn_still_sends_stop_once(self):
        t = Transport(turn_timeout=True)
        report = self.trial(t)
        self.assertEqual(report['result'], 'TRIAL_REFUSED_OR_UNKNOWN')
        self.assertEqual(report['turn_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        self.assertEqual(t.calls.count(('POST', '/turn-left')), 1)
        self.assertEqual(t.calls.count(('POST', '/stop')), 1)

    def test_old_firmware_refused_before_motor_command(self):
        t = Transport(initially_ready=True)
        t.data['firmware_build_id'] = 'v4_2_micro_turn_watchdog_v3'
        report = self.trial(t)
        self.assertEqual(report['result'], 'TRIAL_REFUSED_OR_UNKNOWN')
        self.assertEqual(report['expected_firmware_build_id'],
                         'v4_2_micro_turn_loaded_start_v4')
        self.assertIn('FIRMWARE_BUILD_ID_MISMATCH', report['error'])
        self.assertEqual((report['turn_posts'], report['stop_posts']), (0, 0))
        self.assertEqual(t.calls, [('GET', '/status')])

    def test_default_is_no_execution(self):
        t = Transport()
        report = run(base_url='http://127.0.0.1', output='unused',
                     request=t.request)
        self.assertEqual(report['result'], 'NO_EXECUTION')
        self.assertEqual(t.calls, [])


if __name__ == '__main__':
    unittest.main()
