"""Build wrapper tests with a fake compiler; these do not compile ESP32 firmware."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from firmware.esp32.tools.build_remote_stop_candidate import build


class BuildWrapperTests(unittest.TestCase):
    def test_compile_only_stages_header_and_records_artifacts(self):
        calls=[]
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);credentials=root/'private.h'
            credentials.write_text('constexpr char WIFI_SSID[]="test";\nconstexpr char WIFI_PASSWORD[]="private";\n')
            output=root/'output'
            def fake_run(args, **kwargs):
                calls.append(args)
                if args[1]=='version':return subprocess.CompletedProcess(args,0,'{"VersionString":"test"}','')
                if args[1]=='core':return subprocess.CompletedProcess(args,0,'{"platforms":[]}','')
                self.assertEqual(args[1],'compile')
                stage=Path(args[-1]);self.assertTrue((stage/'remote_stop_observer.h').is_file())
                self.assertEqual((stage/'wifi_credentials.h').read_bytes(),credentials.read_bytes())
                (output/'candidate.bin').write_bytes(b'fake-compiler-output')
                return subprocess.CompletedProcess(args,0,'compile-only test','')
            with patch('shutil.which',return_value='/fake/arduino-cli'),patch('subprocess.run',side_effect=fake_run):
                report=build(cli='arduino-cli',fqbn='esp32:esp32:esp32',credentials=credentials,output=output)
            self.assertEqual(report['status'],'COMPILED_NOT_DEPLOYED')
            self.assertFalse(report['upload_performed'])
            self.assertIn('candidate.bin',report['binaries'])
            self.assertFalse((output/'wifi_credentials.h').exists())
            self.assertTrue(all('upload' not in c for c in calls))
            self.assertFalse(Path(calls[-1][-1]).exists())
            self.assertEqual((output/'compile.local.log').stat().st_mode & 0o777,0o600)
            self.assertEqual(json.loads((output/'build_report.json').read_text())['fqbn'],'esp32:esp32:esp32')

    def test_missing_cli_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as tmp,patch('shutil.which',return_value=None):
            out=Path(tmp)/'output'
            with self.assertRaisesRegex(ValueError,'arduino-cli not found'):
                build(cli='missing',fqbn='esp32:esp32:esp32',credentials=Path(tmp)/'private.h',output=out)
            self.assertFalse(out.exists())

if __name__=='__main__':unittest.main()
