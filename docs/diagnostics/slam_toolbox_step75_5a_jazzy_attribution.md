# Step 75.5a — Jazzy TF publisher attribution retry

**FACT — original Pi evidence, operator-reported, 2026-11-28.**
Preserve the positive `FAIL` at `step75_5a_5t0dgdzt` and the negative
`PASS` at `step75_5a_niqj29yf` under `~/.local/state/sie/evidence/`.
The positive case published three 120 × 80 maps at 0.05 m resolution,
two confirmed scan-stamped poses and zero TF-filter drop log events.
Input TF resolved at 64/64 and the full map chain at 62/64 scan timestamps.
The sole refusal was
`MAP_TF_PUBLISHER_GID_REQUIRED_FOR_MULTIPLE_ENDPOINTS`. The negative case
had 32/32 missing input transforms, no maps and no mapper poses.
Unattended SSH authentication failed, so the private Pi journals were
unavailable for direct review. The retry verifies the existing manifests
and reports the original scan stamps in a separate read-back artifact.

## Root cause and correction

**FACT.** The Jazzy `rclpy` native take binding gives the two-argument
callback a dictionary containing source and receipt timestamps and the
two sequence numbers when available. It omits the GID even though
`rcl_take` supplies it. The original Python verifier expects the
`publisher_gid.data` dictionary used by newer `rclpy`. Consequently
its observed GID sets stayed empty on Jazzy.
[Jazzy rclpy take binding](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/subscription.cpp#L122-L174)

**DECISION — diagnostic compatibility.** Run a subscriber-only C++
receipt recorder on Jazzy. It captures the original RMW publisher GID,
edge and stamp from each received `/tf` and `/tf_static` transform.
It starts before the diagnostic publishers and shuts down after them.
The Python verifier drains the closed journal, matches GIDs to discovered
publisher identities, and rejects missing, unknown or multiple writer
GIDs for an edge, including identically stamped duplicates from the same
node. An unused publisher endpoint can exist without owning any edge.
The verifier requires native receipts covering the TF stamps it observed.
The existing publisher inventory and frame checks still apply.
Lyrical can continue using its Python callback metadata.
[Jazzy rclcpp MessageInfo](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/message_info.hpp#L22-L45)

## Full-chain coverage and startup

**FACT — verified original Lyrical journal.** Both misses are the first
two scan stamps. The first scan also has an exact-stamp mapper pose.
All three rows below have input TF at the original stamp.

| Scan | Header stamp, ns | Relative to first scan, ms | Map chain resolved |
| --- | ---: | ---: | --- |
| 1 | 1795868861387017527 | 0.000000 | No |
| 2 | 1795868861551203058 | 164.185531 | No |
| 3 | 1795868861711153570 | 324.136043 | Yes |

**HYPOTHESIS — historical startup explanation.** At
`restamp_tf=false`, SLAM Toolbox waits for a valid scan header and stamps
`map → odom` with that header plus `transform_timeout`. The original
Lyrical profile used 200 ms. If scan 1 supplied the first published map
transform, its stamp was 1795868861587017527. Both misses precede that
boundary and the third lookup follows it. The initial scan was processed,
so its full-chain miss is consistent with the map transform's initial
time coverage. The original monitor did not journal the actual first
broadcast stamp. An aggregate 62/64 from the Pi alone does not identify
the missed scan indices or establish its first TF stamp.
[Jazzy mapper timestamp formula](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L450-L475)

**FACT — instrumented local replay.** Using SLAM Toolbox 2.10.0 on
Lyrical with the native Jazzy-compatible receipt path, the first actual
`map → odom` stamp was 1795876825982369380 ns. It was exactly 200 ms
after scan 1. Scan 2 preceded it by 31.412557 ms and scan 3 followed it
by 128.580041 ms. These were the only two full-chain misses, and input
TF resolved for 64/64 scans. The run recorded one attributed writer
per TF edge, two scan-stamped mapper poses, three grids and no filter
drop log events. The missing-TF run passed with 32/32 input TF misses
and no map or poses. Both shut down cleanly. The respective evidence
directories are `step75_5a_pkkuki81` and `step75_5a_954f6dke`.
Ten targeted regressions passed, including native receipt attribution
with two legitimate publishers and rejection of a duplicate writer from
the same mapper node publishing the identical edge and stamp.

The updated `qualification.json` records the earliest attributed map TF
stamp and classifies misses that precede it. The retry also reports which
original Jazzy rows missed and their offsets relative to the predicted
boundary. It preserves every original scan header and the full-chain
coverage metric. Physical LD19 acquisition time remains unknown.

## Operator retry

Use the separate laptop launcher delivered with the new archive and SHA-256
file. It transfers one archive, verifies its hash on the Pi, and makes a
private staging directory. It verifies the existing checkout
`/home/elwis/dev_ws/sie_step75_5a_baseline` at
`2a00bd80348e8d10d4577e886f354d30e24057c1`, records Jazzy package versions,
and builds the isolated helper with CMake and the installed ROS headers.
The positive and missing-TF synthetic runs write new evidence directories.
The retry uses interactive SSH/SCP authentication. It touches neither
UART nor the historical evidence. Navigation admission and powered
motion remain disabled. The v13 braking defect remains open.

For a separately authorized live run, build the same helper as part of
the Jazzy preflight and pass its path to the existing live command with
`--tf-audit-executable`. [Original mapping runbook](slam_toolbox_step75_5a.md)

## Pi synthetic qualification closure

**DECISION — 2026-11-28.** Close as
`SLAM_TOOLBOX_SYNTHETIC_INTEGRATION_QUALIFIED` for the isolated Pi 5
ROS 2 Jazzy graph running actual SLAM Toolbox 2.8.5 on synthetic input.
The operator supplied the results below. The private Pi journals were
unavailable for independent inspection. The verified local archive
matches the reported diagnostic bundle SHA-256
`491d71e04c4dbaa573e23f707874151af8620d694f0cf97fe8d7704808269e89`.

| Pi evidence directory under `/home/elwis/.local/state/sie/evidence/` | Original result | Reported observations |
| --- | --- | --- |
| `step75_5a_5t0dgdzt` | FAIL | Map output was present. Sole refusal was missing message-level publisher GIDs. |
| `step75_5a_niqj29yf` | PASS | 32 missing input TFs, zero maps and poses. |
| `step75_5a_88wx3g0s` | PASS | 64/64 input TFs, 49/64 full map chains, three occupancy grids, two verified poses and one TF-filter queue-full log event. Each of the three TF edges had one GID from `RCLCPP_MESSAGE_INFO`. Lifecycle and process shutdown were clean. |
| `step75_5a_me45dwr6` | PASS | 32/32 missing input TFs, zero maps and poses, two queue-full log events and clean shutdown. |

**FACT — startup applicability.** All 15 positive full-chain misses were
classified as `BEFORE_FIRST_ATTRIBUTED_MAP_TF`. Input TF was available
for every scan at its original stamp in that trial. Queue-full log events
are a separate count from scan-time TF lookup misses and from scans
without a verified mapper pose. Preserve earlier local evidence and
both Step 75.4 `BLOCKED` trials.
[Step 75.4 retained verdicts](robot_geometry_footprint_v1.md#pi-qualification-evidence)

**OPEN.** Physical mapping, live scan-time TF continuity, live UART
release and mapping accuracy remain unverified. Physical LD19 acquisition
time remains unknown. Navigation admission and powered motion remain
disabled. The v13 braking defect remains open.
[Step 75.5b stationary motor-off runbook](slam_toolbox_step75_5a.md#step-755b-stationary-live-motor-off-runbook)
