# Ten-minute AR0234 presence/absence image session

Purpose: collect both person-visible and person-absent **raw AR0234 MJPEG
frames** for later human annotation and RGB detector comparison. Camera only;
no ROS node, segmentation inference, stereo, motor controller, or telemetry
transport. This run neither changes the detector threshold nor grants
Measurement or motion authority.

## Before starting

- Park the cart, switch off motor power, and close other AR0234 camera users.
- Keep the camera setup and exposure mode unchanged; the script sets MJPG
  1920×1200 at requested 30 fps, `auto_exposure=3`, `gain=0` and verifies
  the two controls. The camera-native stream is drained continuously.
- Leave the existing 650 nm dot projector **on** if comparing this collection
  with the 2026-10-30 moving-person run; report its actual state. Avoid
  additional people appearing in the background.
- Arrange several passes into and out of frame. Include empty-room periods,
  front, profile and back views, central and near-edge positions, and natural
  lateral motion. Do not label frames by schedule: a human will inspect each
  selected image later. The subject should know the images remain local.

## Start on the Raspberry Pi 5

Use a separate checkout so any local Pi changes remain intact:

```bash
git -C /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp fetch origin codex/sie-ros2-integration-v1
git -C /home/elwis/dev_ws/sie_v6_temperature_disabled_mvp worktree add --detach \
  /home/elwis/dev_ws/sie_shadow_rgb_presence_capture FETCH_HEAD
cd /home/elwis/dev_ws/sie_shadow_rgb_presence_capture
python3 vision_core/tools/capture_shadow_rgb_presence.py \
  --projector-state on \
  --scenario 'one person entering and leaving view repeatedly, front/profile/back; empty scene between passes'
```

The script starts its ten-minute clock after the first valid camera packet.
At default 2 samples/s it writes at most 1200 numbered JPEGs under a new
private `~/.local/state/sie/evidence/shadow_rgb_presence_*` directory.
These are camera-supplied compressed bytes; they are not decoded and
re-encoded. Frame-specific SHA-256, driver sequence and timestamp, host
arrival time and sample slot go into `frames.jsonl`. The terminal prints
`CAPTURE_STARTED`, periodic counts, `SUMMARY`, and the path to `session.json`.
The output starts only if there is at least 2 GiB free; a 1 GiB free-space
reserve and 1 GiB session byte limit protect the disk. If the camera stops,
space runs out, or the user interrupts the capture, `session.json` marks the
run incomplete and retains already saved evidence. A completed run with any
missed sample slots reports `COMPLETE_WITH_GAPS`.

The V4L2 timestamps are driver-provided metadata, not independent proof of
physical exposure time. The images establish neither person identity nor
stereo depth. The manifest's projector state is operator reported.

## After capture

Send the `SUMMARY` line and the `session.json` path first. Check the saved
frame count, skipped slots and disk usage before preparing a small
hash-verified annotation subset that includes both person-present and
person-absent frames. Keep original frames local and outside Git. Annotate
presence and actual upper-body boxes; use the frozen images to compare the
current YOLO11n against a second candidate model with the same test set.
