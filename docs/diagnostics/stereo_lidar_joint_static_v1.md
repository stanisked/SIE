# SIEE Step 66 | Stereo–LiDAR Joint Spatial Validation v1

## Step 65 horizontal mechanical closure

**DECISION** The project owner closed the standalone horizontal stage as
`MECHANICAL_HORIZONTAL_GEOMETRY_DIAGNOSTIC_COMPLETE`. The MVP retains the
Step 60 mechanical profile. There is no millimetre-scale fit or angular
correction from these tests.

**FACT / operator report** Each of the five static acquisitions had 100 unique
LaserScan headers. T02 retrospective is a subset analysis of T02. The results
below were supplied by the operator. The original scan files and point
membership were not independently verified in Step 66.

| Test | Distance from rim (mm) | Plane angle | Median signed error (mm) | P95 absolute error (mm) |
| --- | ---: | ---: | ---: | ---: |
| T01 | 1600 | 0° | 1.158 | 9.880 |
| T02 original | 2015 | 0° | 4.414 | 2481.688 |
| T02 retrospective ±3° | 2015 | 0° | 3.000 | 8.796 |
| T03 | 3010 | 0° | 2.776 | 14.978 |
| T04 | 1466 | +30° | −1.005 | 25.985 |
| T05 | 1463 | −29.587° | 10.280 | 30.329 |

**FACT / operator report** The original T02 result retains 104 anomalous
points. The T04 and T05 angular regression slopes were +3.8436 and
−2.8541 mm/degree, respectively. **HYPOTHESIS** Background contamination
explains the T02 tail. The retrospective angular restriction supports that
hypothesis but does not establish which points struck the target.
**OPEN** The horizontal tests do not establish a complete optical
Camera–LiDAR extrinsic.

## Optical datum and comparison rules

**DECISION** The independent
[optical datum profile](../../vision_core/config/diagnostics/camera_lidar_optical_datum_v1.json)
takes the [historical](shadow_uart_candidate_20261029.md) approximate
0.0125 m optical-center-to-rim robot-forward component and assumes the offset
lies along the physical-left optical axis. At the current 20° pitch the axial
estimate is `0.0125 / cos(20°) = 0.01330222 m`. Adding this positive amount
to optical Z after the [mechanical rim transform](camera_lidar_mechanical_geometry_v1.md)
puts LD19 points in `estimated_physical_left_optical_frame`. The comparison
uses that estimated frame as coincident with the actual camera optical frame
for this diagnostic. Uncertainty and confidence are unknown. The historical
[mount notes](robot_front_geometry_operator_v1.md) retain the incomplete
datum, and the [pose-aware stereo validator](../../vision_core/tools/validate_ov9281_pi_depth_v7_pose_aware.py)
requires independent uncertainty for its separate optical-Z reference. The
mechanical v1 API still refuses optical-center transforms. No validation
data are used to fit the Camera–LiDAR rotation, translation or optical datum.

**FACT** The standalone tool checks the active OV9281 v7 file SHA-256
`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
Packed MJPEG is physical right first, physical left second. Offline analysis
reuses the repository's full-resolution `sgbm7` and 1.5 px left/right
consistency policy, including OpenCV's disparity division by 16. It remaps
each stream with the v7 calibration, uses `Q` to reconstruct rectified-left
XYZ in millimetres, divides by 1000, and applies `R1ᵀ` to convert to the
physical-left optical frame. `P1` projects points back onto the full
1280×800 rectified-left grid.

**DECISION** The operator declares a fixed convex target polygon on the
rectified-left reference image and a physical LD19 angular sector before
examining any residuals. The target and rig stay stationary. Stereo fits
only LR-consistent positive-depth pixels in the eroded polygon. The trial
screen requires at least 48 points and 15% interior coverage, and checks
the stereo-only plane P95 against 40 mm. The fit uses up to 512 uniformly
indexed points, and its residuals are checked over all valid stereo support.
The normal points towards positive camera optical Z. The signed residual
is positive on the farther side of a front-facing plane.

**DECISION** Infinite-plane metrics include every valid LD19 return in
the declared sector. Finite-surface candidates require both the LD19 hit
and the same beam's intersection with the stereo plane to project inside
the polygon and within two pixels of LR-supported stereo pixels. The beam
intersection defines where a return was expected even if none was received.
The analyzer preserves all beam indices, including returns outside the
target, missing ranges and ambiguous projections. A nearer or farther
return can project through the target footprint, so the candidate test
alone does not establish physical target membership. No LD19 residual
cutoff trims either result set.

**DECISION** Capture reuses the Host Timebase Bridge and trial policy. For
each camera packet it selects only LD19 callbacks already delivered by
camera receipt, comparing a bridged camera stamp with LD19 conversion
headers. It applies the existing 50 ms link and 250 ms age bounds to this
diagnostic stereo anchor. Use a current, same-boot, same-publisher clock
evidence digest only. The tool checks the sole discovered publisher GID at
each callback and refuses mismatches from the first observed GID. Discovery
cannot itself prove callback publisher identity or validate the supplied
clock evidence. Omitting the digest archives payloads with clock provenance
unverified and produces no admitted comparisons. Physical LD19 acquisition
time remains `UNKNOWN`. The numeric thresholds are trial screening limits.

## First Pi test with the 320 mm target

**DECISION** Perform this procedure only as a separately initiated
sensor-only test with traction motor power physically disconnected. Leave
the existing LD19 publisher running without starting a duplicate. Stop other
camera owners before opening OV9281. There is no ESP32, TF, navigation,
World State or Measurement route in this tool. Keep the rig stationary and
start around 1.6 m from the lens rim, unless visibility calls for a different
distance within the active stereo range. Place the same textured, matte
320 mm face so it is visible in both stereo images and the horizontal laser
plane cuts its interior. Avoid a visible background through gaps or along
the selected sector. With an upward-pitched camera and a raised LD19, verify
vertical overlap explicitly. If the panel cannot be seen and hit in both
sensors, reposition it before collecting the test.

**DECISION** Run from the checked-out Spatial Association branch on the Pi.
Use an existing Step 57-style empirical evidence JSON only if its boot and
observed publisher still apply. Set `LD19_CLOCK_FILE` to that file. If its
applicability is unknown, leave the variable unset for an archive-only check
until fresh clock evidence is available.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD:$PWD/ros2/sie_ros2:$PWD/vision_core/tools${PYTHONPATH:+:$PYTHONPATH}"
runtime=/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python
calibration=/home/elwis/dev_ws/runtime_artifacts/ov9281_pi_stereo_v7/ov9281_pi_stereo_v7.npz
tool=vision_core/tools/stereo_lidar_joint_static_v1.py
ros2 topic info /sie/lidar/scan_raw -v
v4l2-ctl -d /dev/video2 --set-fmt-video=width=2560,height=800,pixelformat=MJPG --set-parm=60 -c auto_exposure=3
v4l2-ctl -d /dev/video2 --get-fmt-video --get-parm -C auto_exposure

native_dir=$(mktemp -d)
cc -std=c11 -O2 -Wall -Wextra -Werror vision_core/tools/stream_v4l2_mjpeg.c -o "$native_dir/capture"
umask 077
base="$HOME/.local/state/sie/evidence/step66_$(date -u +%Y%m%dT%H%M%SZ)_$$"
mkdir -p "$(dirname "$base")"
clock_args=()
if [[ -n "${LD19_CLOCK_FILE:-}" ]]; then
  clock_hash=$(sha256sum -- "$LD19_CLOCK_FILE" | cut -d ' ' -f 1)
  clock_args=(--ld19-clock-evidence "sha256:$clock_hash")
fi
SIE_V4L2_BUFFER_COUNT=1 "$runtime" "$tool" capture \
  --native-capture "$native_dir/capture" --device /dev/video2 \
  --calibration "$calibration" --frames 30 --interval-s .2 \
  "${clock_args[@]}" --output "${base}_capture"
```

**DECISION** Open `"${base}_capture/target-reference-left.png"` in an image
viewer. Read the four rectified-left pixel vertices around the observed
panel, in perimeter order, and choose the sector from the panel's physical
location relative to LD19. At 1.6 m a 320 mm panel centered on a ray spans
roughly 11° in total. Use the actual placement and sight lines to choose
bounds. The following reads your values instead of imposing an assumed
alignment. Record the target pose and distance in the label.

```bash
read -r -p 'Rectified-left polygon x1 y1 x2 y2 x3 y3 x4 y4: ' -a polygon
read -r -p 'Declared LD19 sector start and end in degrees: ' -a sector
read -r -p 'Target dimensions, position, and pose: ' target_label
"$runtime" "$tool" analyze --capture "${base}_capture" --output "${base}_analysis" \
  --polygon "${polygon[@]}" --sector-deg "${sector[@]}" --target-label "$target_label"
cat "${base}_analysis/summary.json"
```

## Evidence and limits

| File | Recorded evidence |
| --- | --- |
| `session.json`, `mechanical.json`, `optical-datum.json`, `ov9281-v7.npz` | Input artifact hashes, boot ID, capture helper hash, timing basis, active calibration snapshot, geometry provenance and admission flags. |
| `frames.jsonl`, `frame-*.mjpg` | Unmodified packed MJPEG, hashes, driver sequence and timestamp, dequeue/receipt timing, frame IDs, bridge status, and snapshot refusals. |
| `scans.jsonl`, `scan-*.cdr` | Serialized ROS LaserScan payloads with hashes, decoded raw ranges/intensities (including non-finite tokens), original headers, callback receipt clocks, publisher observations, and rejected header or payload reasons. |
| `capture-summary.json`, `target-reference-left.png` | Sample totals and any scan-capacity omissions, plus the rectified-left reference image for operator marking. |
| `analysis-policy.json`, `comparisons.jsonl`, `stereo-*.npz` | Journal hashes, declared region and gates, causal matches/refusals, full disparity and LR arrays, stereo plane fit, projections, residuals and per-beam decisions. |
| `summary.json` | Separate infinite-plane and finite-candidate median, MAD and P95 absolute residual, frame/beam status counts and distinct scan count. A scan can be reused across frames, so pooled counts are per beam per stereo frame. |

**FACT** Both stages require a new output directory. Capture exit 1 signals
incomplete acquisition. Analysis exit 2 signals zero finite candidates
with a written summary. Inspect refusal counts in either case. Keep the
raw evidence private. The tool leaves active calibration and runtime files
untouched and does not admit an extrinsic, Measurement, World State update
or motor command.

**OPEN** The first physical run has not been executed. The tool and target
selection were software-checked with synthetic evidence, including a real
stereo reconstruction. Extrinsic accuracy, operating limits and actual
finite-target membership remain open.

**FACT** Local verification passed 114 targeted tests, including 19 Step 66
cases. Four Python files compiled, the Pi procedure passed Bash syntax
validation, and the existing V4L2 helper compiled on the development host.
