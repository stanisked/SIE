"""Compile the isolated micro-turn startup-power ESP32 candidate; never upload it."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

SKETCH = Path(__file__).resolve().parents[1] / 'traction_control_v4_2_micro_turn_power_v5'
SOURCES = (
    'traction_control_v4_2_micro_turn_power_v5.ino',
    'remote_stop_observer.h',
    'command_replay_guard.h',
)
BUILD_ID = 'v4_2_micro_turn_power_v5'

def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build(*, cli: str, fqbn: str, credentials: Path, output: Path) -> dict:
    if not fqbn.startswith('esp32:esp32:') or len(fqbn.split(':')) < 3:
        raise ValueError('Provide the installed ESP32 board FQBN explicitly')
    binary = shutil.which(cli)
    if binary is None:
        raise ValueError('arduino-cli not found on this computer')
    if output.exists():
        raise ValueError('Output must be a new directory')
    for name in SOURCES:
        if not (SKETCH / name).is_file():
            raise ValueError('Missing candidate source: ' + name)
    if BUILD_ID not in (SKETCH / SOURCES[0]).read_text():
        raise ValueError('Candidate build ID mismatch')
    if not credentials.is_file():
        raise ValueError('Local wifi_credentials.h is missing')
    if 'CHANGE_ME' in credentials.read_text():
        raise ValueError('Replace local credential placeholders before compiling')
    source_sha = {name: digest(SKETCH / name) for name in SOURCES}
    metadata = {}
    for name, args in (
        ('cli_version', ['version', '--format', 'json']),
        ('installed_cores', ['core', 'list', '--format', 'json']),
    ):
        result = subprocess.run([binary, *args], check=True, capture_output=True, text=True)
        metadata[name] = json.loads(result.stdout)
    output.mkdir(parents=True)
    with tempfile.TemporaryDirectory(prefix='sie_loaded_turn_build_') as temp:
        stage = Path(temp) / SKETCH.name
        stage.mkdir()
        for name in SOURCES:
            shutil.copyfile(SKETCH / name, stage / name)
        shutil.copyfile(credentials, stage / 'wifi_credentials.h')
        command = [
            binary, 'compile', '--fqbn', fqbn, '--warnings', 'all',
            '--build-path', str(Path(temp) / 'build'),
            '--output-dir', str(output.resolve()), str(stage),
        ]
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        log = output / 'compile.local.log'
        log.touch(mode=0o600)
        log.write_text(result.stdout + result.stderr)
        report = {
            'status': 'COMPILED_NOT_DEPLOYED' if result.returncode == 0 else 'COMPILE_FAILED',
            'build_id': BUILD_ID, 'fqbn': fqbn, 'toolchain': metadata,
            'source_sha256': source_sha, 'compile_returncode': result.returncode,
            'credentials_copied_to_output': False, 'upload_performed': False,
            'physical_validation': False,
            'binaries': {
                p.name: digest(p) for p in output.iterdir()
                if p.is_file() and p.suffix in {'.bin', '.elf', '.hex'}
            },
        }
        (output / 'build_report.json').write_text(json.dumps(report, indent=2) + '\n')
        if result.returncode:
            raise ValueError('Compilation failed; inspect local log: ' + str(log))
        return report

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cli', default='arduino-cli')
    parser.add_argument('--fqbn', required=True)
    parser.add_argument('--credentials', type=Path,
                        default=SKETCH / 'wifi_credentials.h')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        report = build(cli=args.cli, fqbn=args.fqbn,
                       credentials=args.credentials, output=args.output)
    except (ValueError, OSError, subprocess.CalledProcessError,
            json.JSONDecodeError) as error:
        parser.exit(1, str(error) + '\n')
    print(report['status'], 'report:', args.output / 'build_report.json')

if __name__ == '__main__':
    main()
