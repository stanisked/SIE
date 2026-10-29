#!/usr/bin/env python3
"""Read-only UART check and rejected stale-boot command; motors must be unpowered."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlencode

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from vision_core.tools.uart_http_transport import UartHttp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-port', required=True)
    parser.add_argument('--count', type=int, default=100)
    args = parser.parse_args()
    if not 1 <= args.count <= 1000:
        parser.error('--count must be 1..1000')
    latencies = []
    boots = set()
    with UartHttp(args.serial_port) as request:
        for _ in range(args.count):
            started = time.monotonic()
            code, status = request('GET', 'http://uart.local/status', .3)
            latencies.append(time.monotonic() - started)
            if code != 200 or status.get('firmware_build_id') != 'v4_2_uart_motion_v1c':
                raise RuntimeError('UNEXPECTED_FIRMWARE_OR_STATUS')
            if (status.get('state') != 'READY' or status.get('right_pwm') != 0
                    or status.get('left_pwm') != 0
                    or status.get('active_command_id') is not None):
                raise RuntimeError('CONTROLLER_NOT_STATIONARY')
            boots.add(status.get('boot_session_id'))
            time.sleep(.1)
        if len(boots) != 1:
            raise RuntimeError('BOOT_SESSION_CHANGED')
        boot = next(iter(boots))
        wrong_boot = '0000000000000000' if boot != '0000000000000000' else 'FFFFFFFFFFFFFFFF'
        query = urlencode({'boot_session_id': wrong_boot,
                           'command_id': 'uart-probe-rejected-00000001',
                           'distance_m': '0.100'})
        code, body = request('POST',
            'http://uart.local/move-forward-steered?' + query, .3)
        if code != 409 or body.get('reason') != 'BOOT_SESSION_MISMATCH':
            raise RuntimeError('STALE_BOOT_COMMAND_NOT_REJECTED')
        code, final = request('GET', 'http://uart.local/status', .3)
        if (code != 200 or final.get('boot_session_id') != boot
                or final.get('state') != 'READY'
                or final.get('active_command_id') is not None
                or final.get('right_pwm') != 0 or final.get('left_pwm') != 0):
            raise RuntimeError('POST_REJECTION_STATE_CHANGED')
    print(json.dumps({'status_success': len(latencies), 'boot_session_id': boot,
                      'median_ms': round(sorted(latencies)[len(latencies)//2] * 1000, 2),
                      'max_ms': round(max(latencies) * 1000, 2),
                      'wrong_boot_move': 'REJECTED', 'final_state': 'READY',
                      'motor_power_required': False}))


if __name__ == '__main__':
    main()
