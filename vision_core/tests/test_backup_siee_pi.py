import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from vision_core.tools import backup_siee_pi as backup


class PiBackupTests(unittest.TestCase):
    def test_preflight_refuses_drive_remote_outside_pinned_folder(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            config = home / ".config/rclone/rclone.conf"
            config.parent.mkdir(parents=True)
            config.write_text("[siee-drive]\ntype = drive\nroot_folder_id = other\n")
            password = home / ".config/sie/backup-restic-password"
            password.parent.mkdir(parents=True)
            password.write_text("private test password\n")
            os.chmod(password, 0o600)
            with patch.object(backup.shutil, "which", return_value="/usr/bin/fake"):
                with self.assertRaisesRegex(RuntimeError,
                                            "DRIVE_REMOTE_NOT_PINNED"):
                    backup.preflight(home, home)

    def test_failed_restic_check_does_not_publish_pass_report(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            repo = home / "repo"
            repo.mkdir()
            calls = []

            def fake_run(*args, **_kwargs):
                calls.append(args)
                if args[:2] == ("restic", "check"):
                    raise RuntimeError("restic check failed")
                return ""

            with patch.object(backup, "preflight", return_value=([repo], {})), \
                    patch.object(backup, "latest_snapshot", return_value={"id": "snap",
                        "time": "2026-11-11T19:00:00+03:00"}), \
                    patch.object(backup, "run", side_effect=fake_run):
                with self.assertRaisesRegex(RuntimeError, "restic check failed"):
                    backup.backup(home, repo)
            self.assertFalse(any(args[:2] == ("rclone", "copyto") for args in calls))
            self.assertEqual(list((home / ".local/state/sie/backup").iterdir()), [])

    def test_preflight_requires_custom_oauth_client(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            config = home / ".config/rclone/rclone.conf"
            config.parent.mkdir(parents=True)
            config.write_text("[siee-drive]\ntype = drive\nroot_folder_id = "
                              + backup.DRIVE_FOLDER_ID + "\n")
            password = home / ".config/sie/backup-restic-password"
            password.parent.mkdir(parents=True)
            password.write_text("private test password\n")
            os.chmod(password, 0o600)
            with patch.object(backup.shutil, "which", return_value="/usr/bin/fake"):
                with self.assertRaisesRegex(RuntimeError,
                                            "DRIVE_CUSTOM_OAUTH_CLIENT_MISSING"):
                    backup.preflight(home, home)

    def test_success_report_is_read_back_from_drive(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            repo = home / "repo"
            repo.mkdir()
            uploaded = []

            def fake_run(*args, **_kwargs):
                if args[:2] == ("rclone", "copyto"):
                    uploaded.append(Path(args[2]).read_bytes())
                if args[:4] == ("git", "-C", str(repo), "rev-parse"):
                    return "abc123\n"
                return ""

            def fake_subprocess(*_args, **_kwargs):
                return subprocess.CompletedProcess([], 0, stdout=uploaded[-1])

            with patch.object(backup, "preflight", return_value=([repo], {})), \
                    patch.object(backup, "latest_snapshot", return_value={"id": "snap",
                        "time": "2026-11-11T19:00:00+03:00"}), \
                    patch.object(backup, "run", side_effect=fake_run), \
                    patch.object(backup.subprocess, "run", side_effect=fake_subprocess):
                report = backup.backup(home, repo)
            self.assertEqual(report["result"], "PASS")
            self.assertEqual(report["restic_snapshot_id"], "snap")
            self.assertEqual(report["restore_test"], "NOT_RUN")
            self.assertEqual(len(uploaded), 1)


if __name__ == "__main__":
    unittest.main()
