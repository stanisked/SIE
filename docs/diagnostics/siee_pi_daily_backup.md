# SIEE Pi daily backup

Status: prepared on 2026-11-11, **not installed or run on the Pi**. The Drive
destination exists at [SIEE Pi5 Daily Backups](https://drive.google.com/drive/folders/1roTCxsE8S8SehwLz2Q6-tnfxxB1J7yrK)
inside the private SIE Active Projector folder. No backup snapshot is present
until the Pi-side setup and first restore check succeed.

## Rule

At 19:00 Europe/Moscow each Monday through Friday, the Pi user timer runs an
incremental, encrypted restic backup over a pinned rclone Google Drive remote.
`Persistent=true` runs a missed scheduled job after the user manager starts.
The script fails without a report if the Drive folder ID, sources, password,
tools or remote repository are missing. It does not silently initialize a new
repository, delete snapshots, or alter calibration/evidence sources.

Sources: the current Shadow checkout (including `.git` and uncommitted files),
the historical V6 checkout, `runtime_artifacts`, `model_artifacts`,
`runtime_envs`, `/home/elwis/best.onnx`, SIE evidence, and Pi boot
`config.txt`. Generated `build`, `install`, `log`, caches and `node_modules`
are omitted and must be rebuilt after restoration. This is a project/data
backup, not a bootable Pi disk image. The backup encryption password and
rclone OAuth configuration are deliberately outside the backed-up paths; keep
recoverable copies in a separate password manager. Do not put them in Git or
the chat.

After each snapshot, `restic check` verifies repository metadata. A dated
`SIEE_Pi5_backup_YYYY-MM-DD.json` is uploaded to the Drive folder and read
back byte-for-byte only after that check. This does not prove that all file
contents can be restored: perform a separate sample restore before calling
the backup operational. The script never prunes snapshots; monitor Drive
capacity and decide retention separately.

## One-time Pi setup

Keep the active detached-HEAD checkout and its local `build/`, `install/`, and
`log/` untouched. Clone the published backup tools into a separate directory:

```bash
git clone --branch codex/shadow-rgb-detector-20261030 --single-branch \
  https://github.com/stanisked/SIE.git /home/elwis/dev_ws/sie_backup_tools
cd /home/elwis/dev_ws/sie_backup_tools
```

Confirm the source checkout and paths listed in
`vision_core/tools/backup_siee_pi.py`. Check installed tools before installing:

```bash
command -v restic; command -v rclone; command -v git
```

If missing, install `restic` and `rclone` from Ubuntu packages. Configure a
remote named `siee-drive` with `rclone config`: type `drive`,
`root_folder_id=1roTCxsE8S8SehwLz2Q6-tnfxxB1J7yrK`. On the headless Pi,
follow [rclone's remote setup](https://rclone.org/remote_setup/) using a
browser on the laptop. Keep the rclone configuration readable only by
`elwis`. The script checks the remote type and exact folder ID before upload.

Create a unique strong restic password in a password manager, write it to
`/home/elwis/.config/sie/backup-restic-password` with mode `0600`, and keep
an independent copy. Initialize the repository **once** under that folder:

```bash
export RESTIC_REPOSITORY='rclone:siee-drive:restic'
export RESTIC_PASSWORD_FILE=/home/elwis/.config/sie/backup-restic-password
restic init
python3 vision_core/tools/backup_siee_pi.py \
  --project-root /home/elwis/dev_ws/sie_shadow_blind_v3_20261110 --preflight
```

Do not paste the password or OAuth token into a command, report, or chat.
Run the first backup manually, inspect the result and the dated Drive report,
then install the user timer:

```bash
python3 vision_core/tools/backup_siee_pi.py \
  --project-root /home/elwis/dev_ws/sie_shadow_blind_v3_20261110
mkdir -p ~/.config/systemd/user
cp vision_core/tools/systemd/siee-pi-backup.{service,timer} ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now siee-pi-backup.timer
systemctl --user list-timers siee-pi-backup.timer
```

For runs while `elwis` is logged out, enable user lingering once with
`sudo loginctl enable-linger elwis`. Check the unit paths if the checkout is
moved. On the first day, verify the Drive report from another machine and
restore a few source, model, calibration and evidence files into a temporary
directory using `restic restore <snapshot_id> --target <empty_dir>`; compare
their SHA-256 to the Pi originals. Do not restore over live runtime paths.

Operational checks:

```bash
systemctl --user status siee-pi-backup.timer
journalctl --user -u siee-pi-backup.service -n 80 --no-pager
RESTIC_REPOSITORY='rclone:siee-drive:restic' RESTIC_PASSWORD_FILE=/home/elwis/.config/sie/backup-restic-password restic snapshots
```

References: [restic rclone backend](https://restic.readthedocs.io/en/stable/030_preparing_a_new_repo.html#other-services-via-rclone),
[rclone Drive](https://rclone.org/drive/),
[systemd timers](https://www.freedesktop.org/software/systemd/man/latest/systemd.timer.html).
