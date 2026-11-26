# Camera–LiDAR Mechanical Extrinsic v1 | Step 60

**FACT / provenance** On 2026-11-26 the project operator reported that the
LD19 scan origin was 158 mm forward, 30 mm left and 195 mm up from the
center of the physical-left OV9281 lens front rim. The operator measured
the camera block at 20° upward and checked the LD19 scan plane as horizontal.
For this geometry the 20° measurement supersedes the earlier 10° and 15°
values. The [historical robot front mount profile](../../vision_core/config/diagnostics/robot_front_mount_operator_v1.json)
retains its original values.

**DECISION** The [mechanical geometry profile](../../vision_core/config/diagnostics/camera_lidar_mechanical_geometry_v1.json)
uses SI metres. Its source frame is `ld19_sensor`, with axes oriented forward,
left and up under the stated alignment assumptions. Its target is
`physical_left_lens_front_rim_frame`, whose origin is the center of the
physical-left OV9281 lens front rim and whose axes point right, down and
along the forward camera optical axis. The prior `base_link` → `ld19_sensor`
ROS TF is provisional and provides no validation for this transform.

**OPEN** LD19 yaw is provisionally 0°. Camera yaw and roll each use an
unverified 0° assumption. Measurement uncertainties and confidence are
unknown. The rim-to-optical-center offset is unknown. The profile and its
local provenance label carry no canonical Evidence Graph ID or independently
validated extrinsic calibration.

## Coordinate transformation

**FACT / mathematical result** Let `d = [0.158, 0.030, 0.195]ᵀ m` be the
LD19 origin expressed from the rim in robot-aligned axes. For column vectors,
`p_rim = R @ p_ld19 + t`, where `t = R @ d`. The inverse is
`p_ld19 = Rᵀ @ p_rim − Rᵀ @ t`. The three leftmost columns below give
the rotation and the final column gives translation in metres. These
float64 coefficients describe the mathematical transform. They do not
quantify physical accuracy.

```text
LD19 → lens rim
[ 0                  -1  0                  -0.0300000000000000 ]
[ 0.3420201433256687  0  -0.9396926207859084  -0.1292008784077965 ]
[ 0.9396926207859084  0   0.3420201433256687   0.2151653620326789 ]

Lens rim → LD19
[ 0                   0.3420201433256687  0.9396926207859084  -0.1580000000000000 ]
[-1                   0                   0                  -0.0300000000000000 ]
[ 0                  -0.9396926207859084  0.3420201433256687  -0.1950000000000000 ]
```

**DECISION** The [pure Python API](../../sie_core/geometry/camera_lidar_mechanical_geometry_v1.py)
requires the source and destination frame identifiers in both directions.
`mechanical_transform` returns `(R, t)`. `transform_points` accepts a
point or an `N × 3` point array in metres. The code validates
orthogonality and determinant +1 with a float64 tolerance of `1e-12`,
rejects invalid geometry and refuses optical-center transforms. Using
the optical axes for orientation does not authorize moving the origin
from the rim to the optical center. The implementation adds no runtime
integration, TF publication, Measurement, World State update or motor
admission.

## Proposed independent static validation

**HYPOTHESIS** With the robot stationary and defined surfaces visible to
both sensors, transforming LD19 hits into the lens-rim frame will place
them on the independently surveyed surfaces within a separately established
uncertainty budget. This is a proposal only. No hardware was operated.

**DECISION** For a separately approved diagnostic, use broad planar targets
with sufficient camera coverage and a laser intersection inside the target
edges. Include a front-facing panel, oblique panels with different normals,
and a tilted surface at multiple distances and lateral positions. Survey
target planes relative to the physical rim datum independently of the
Step 60 dimensions. Record survey instrument and datum uncertainty. If
stereo depth surfaces are compared in the rim frame, first independently
establish the rim-to-optical-center offset and rectified-to-physical camera
transform for the active calibration. The v1 profile supplies neither.

**DECISION** Retain time-linked scan and stereo payloads, camera calibration
hashes, survey records, static pose identifiers, target boundaries and all
refusals. Apply v1 as specified, without adjusting its parameters on the
validation scenes. For known co-visible hits, compute signed point-to-plane
residuals, median and upper-tail absolute residuals by plane and pose.
Report sample support, occlusions, missing returns and ambiguities. Check
the sensitivity to pitch, yaw, roll and translation against the actual
survey uncertainties. If a later fitted transform is proposed, use held-out
poses for independent validation.

**OPEN** Individual sensor yaw and roll and parameters weakly constrained by
a planar LiDAR scan may require independent mount measurements. A single
plane cannot establish a full extrinsic. Set evaluation criteria from the
survey uncertainty and the intended application before gathering data.
