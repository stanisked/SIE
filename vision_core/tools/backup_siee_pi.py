#!/usr/bin/env python3
"""Incremental encrypted SIEE Pi backup to the pinned private Drive folder.

Requires a preconfigured rclone Drive remote and an initialized restic repo.
Never initializes a new remote implicitly or deletes old snapshots.
"""
from __future__ import annotations

import argparse
import configparser
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
from zoneinfo import ZoneInfo


DRIVE_FOLDER_ID = "1roTCxsE8S8SehwLz2Q6-tnfxxB1J7yrK"
REMOTE_NAME = "siee-drive"
REPOSITORY = f"rclone:{REMOTE_NAME}:restic"
TAG = "siee-pi5-daily"
TIMEZONE = ZoneInfo("Europe/Moscow")
EXCLUDES = ("**/__pycache__", "**/.cache", "**/build", "**/install",
            "**/log", "**/.pytest_cache", "**/node_modules")


def run(*args: str, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(args, env=env, text=True, capture_output=True,
                            check=False)
    if result.returncode:
        # Tool diagnostics may include OAuth details. Leave them out of the
        # scheduler's journal and expose only the failing operation.
        raise RuntimeError(f"{args[0]} {args[1]} failed (exit {result.returncode})")
    return result.stdout


def sources(home: Path, repo: Path) -> list[Path]:
    return [repo,
            home / "dev_ws/sie_v6_temperature_disabled_mvp",
            home / "dev_ws/runtime_artifacts",
            home / "dev_ws/model_artifacts",
            home / "dev_ws/runtime_envs",
            home / "best.onnx",
            home / ".local/state/sie/evidence",
            Path("/boot/firmware/config.txt")]


def preflight(home: Path, repo: Path) -> tuple[list[Path], dict[str, str]]:
    for binary in ("restic", "rclone", "git"):
        if shutil.which(binary) is None:
            raise RuntimeError(f"MISSING_TOOL:{binary}")
    config_path = home / ".config/rclone/rclone.conf"
    config = configparser.ConfigParser(interpolation=None)
    if not config_path.is_file() or not config.read(config_path):
        raise RuntimeError("RCLONE_CONFIG_MISSING")
    if (not config.has_section(REMOTE_NAME) or
            config.get(REMOTE_NAME, "type", fallback="") != "drive" or
            config.get(REMOTE_NAME, "root_folder_id", fallback="") != DRIVE_FOLDER_ID):
        raise RuntimeError("DRIVE_REMOTE_NOT_PINNED_TO_SIEE_BACKUP_FOLDER")
    if (not config.get(REMOTE_NAME, "client_id", fallback="").strip() or
            not config.get(REMOTE_NAME, "client_secret", fallback="").strip()):
        raise RuntimeError("DRIVE_CUSTOM_OAUTH_CLIENT_MISSING")
    password_file = home / ".config/sie/backup-restic-password"
    if (not password_file.is_file() or not password_file.read_bytes() or
            stat.S_IMODE(password_file.stat().st_mode) & 0o077):
        raise RuntimeError("RESTIC_PASSWORD_FILE_MISSING_OR_INSECURE")
    paths = sources(home, repo)
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        raise RuntimeError("BACKUP_SOURCE_MISSING:" + ",".join(missing))
    run("git", "-C", str(repo), "rev-parse", "HEAD")
    env = os.environ.copy()
    env.update(RESTIC_REPOSITORY=REPOSITORY,
               RESTIC_PASSWORD_FILE=str(password_file))
    run("rclone", "lsf", f"{REMOTE_NAME}:", env=env)
    try:
        run("restic", "cat", "config", env=env)
    except RuntimeError as error:
        raise RuntimeError("RESTIC_REPOSITORY_NOT_INITIALIZED_OR_UNREADABLE") from error
    return paths, env


def latest_snapshot(env: dict[str, str], started: datetime) -> dict:
    records = json.loads(run("restic", "snapshots", "--json", "--tag", TAG,
                             "--latest", "1", env=env))
    if len(records) != 1 or not records[0].get("id"):
        raise RuntimeError("BACKUP_SNAPSHOT_NOT_FOUND")
    snap = records[0]
    created = datetime.fromisoformat(snap["time"].replace("Z", "+00:00"))
    if created < started:
        raise RuntimeError("BACKUP_SNAPSHOT_IS_OLD")
    return snap


def backup(home: Path, repo: Path) -> dict:
    paths, env = preflight(home, repo)
    started = datetime.now(TIMEZONE)
    output_dir = home / ".local/state/sie/backup"
    output_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(output_dir, 0o700)
    command = ["restic", "backup", "--quiet", "--tag", TAG]
    for pattern in EXCLUDES:
        command.extend(("--exclude", pattern))
    command.extend(str(path) for path in paths)
    run(*command, env=env)
    snapshot = latest_snapshot(env, started)
    run("restic", "check", env=env)
    report = {
        "schema_version": "siee.pi5_daily_backup.v1",
        "result": "PASS",
        "started_at": started.isoformat(),
        "completed_at": datetime.now(TIMEZONE).isoformat(),
        "drive_folder_id": DRIVE_FOLDER_ID,
        "restic_snapshot_id": snapshot["id"],
        "restic_snapshot_time": snapshot["time"],
        "sources": [str(path) for path in paths],
        "excluded_generated_directories": list(EXCLUDES),
        "git_head": run("git", "-C", str(repo), "rev-parse", "HEAD").strip(),
        "git_worktree_clean": not bool(run("git", "-C", str(repo),
                                           "status", "--porcelain").strip()),
        "check": "RESTIC_METADATA_CHECK_PASS",
        "restore_test": "NOT_RUN",
    }
    # The report contains no password or rclone token. Publish it only after
    # the backup and remote restic metadata check have succeeded.
    payload = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    local_report = output_dir / f"SIEE_Pi5_backup_{started.date()}.json"
    local_report.write_bytes(payload)
    os.chmod(local_report, 0o600)
    remote_report = f"{REMOTE_NAME}:{local_report.name}"
    run("rclone", "copyto", str(local_report), remote_report, env=env)
    remote_payload = subprocess.run(("rclone", "cat", remote_report), env=env,
                                    capture_output=True, check=True).stdout
    if hashlib.sha256(remote_payload).digest() != hashlib.sha256(payload).digest():
        raise RuntimeError("DRIVE_REPORT_READBACK_MISMATCH")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--project-root", required=True, type=Path,
                        help="The active Pi checkout to back up, not the tools checkout")
    args = parser.parse_args()
    home = Path.home()
    repo = args.project_root.resolve()
    try:
        if args.preflight:
            paths, _ = preflight(home, repo)
            print(json.dumps({"result": "PREFLIGHT_OK",
                              "sources": [str(path) for path in paths]}))
        else:
            print(json.dumps(backup(home, repo), ensure_ascii=False))
        return 0
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"SIEE_BACKUP_FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
