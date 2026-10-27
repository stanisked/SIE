"""Offline checks for the one-shot floor turn; no controller access."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from firmware.esp32.tests.test_wheels_lifted_turn_diagnostic import Transport
from firmware.esp32.tools.run_floor_turn_diagnostic import run


class FloorTurnTests(unittest.TestCase):
    def trial(self, transport):
        with TemporaryDirectory() as directory:
            return run(base_url='http://127.0.0.1',
                       output=Path(directory) / 'trial', execute=True,
                       turn_space_clear=True, people_clear=True,
                       request=transport.request, clock=transport.clock,
                       sleep=transport.sleep)

    def test_no_execution_default(self):
        transport = Transport(initially_ready=True)
        report = run(base_url='http://127.0.0.1', output='unused',
                     request=transport.request)
        self.assertEqual(report['result'], 'NO_EXECUTION')
        self.assertEqual(transport.calls, [])

    def test_both_operator_assertions_required(self):
        transport = Transport(initially_ready=True)
        with self.assertRaisesRegex(ValueError, 'CONFIRMATION_REQUIRED'):
            run(base_url='http://127.0.0.1', output='unused', execute=True,
                turn_space_clear=True, request=transport.request)
        self.assertEqual(transport.calls, [])

    def test_one_turn_and_confirmed_stop(self):
        transport = Transport(initially_ready=True)
        report = self.trial(transport)
        self.assertEqual(report['turn_posts'], 1)
        self.assertEqual(report['stop_posts'], 1)
        self.assertEqual(report['fault_ack_posts'], 0)
        self.assertEqual(transport.calls.count(('POST', '/turn-left')), 1)
        self.assertEqual(transport.calls.count(('POST', '/stop')), 1)
        self.assertNotIn(('POST', '/ack-fault'), transport.calls)

    def test_latched_fault_refuses_before_turn(self):
        transport = Transport()
        report = self.trial(transport)
        self.assertEqual(report['result'], 'TRIAL_REFUSED_OR_UNKNOWN')
        self.assertEqual((report['turn_posts'], report['stop_posts']), (0, 0))
        self.assertEqual(transport.calls, [('GET', '/status')])

    def test_old_build_refuses_before_turn(self):
        transport = Transport(initially_ready=True)
        transport.data['firmware_build_id'] = 'v4_2_micro_turn_watchdog_v3'
        report = self.trial(transport)
        self.assertIn('FIRMWARE_BUILD_ID_MISMATCH', report['error'])
        self.assertEqual((report['turn_posts'], report['stop_posts']), (0, 0))
        self.assertEqual(transport.calls, [('GET', '/status')])


if __name__ == '__main__':
    unittest.main()
