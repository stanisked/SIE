# One-point robot-front board distance comparison

Status: diagnostic only. Operator setup confirmed 2026-10-16: 9x6 inner corners, 25 mm square, stationary vertical board plane 1500 mm horizontally forward from the robot front. Reference standard uncertainty is not supplied and stays null unless explicitly declared. No optical-centre submillimetre survey is required for this comparison.

## Capture and computation

`vision_core/tools/check_robot_front_board_distance.py` reads the physical stereo device only. It verifies the existing v7 calibration SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52` and physical-left camera semantics. The right half of the 2560x800 combined frame remains physical LEFT. Existing MJPG 60 fps / auto_exposure=3 acquisition settings are requested. Sixty initial read frames flush capture; this is not a thermal qualification or a 30-minute warmup.

The target is three accepted frames within eight capture attempts. The physical-left view must detect the checkerboard with sb_exhaustive to define the dense board polygon. Right-view corner detection and pairing are a separate supplementary calculation. Only original/reverse-180 corner order candidates with sufficient positive disparity and median vertical residual <=1.5 original pixels are retained for that supplementary estimate; unresolved/bad corner correspondence remains explicitly refused. Such a refusal does not skip dense matching, which retains its existing bilateral LR check and minimum support. The old lateral-offset and board-normal-to-optical-axis gates are not used. The physical board faces travel direction while the camera points 15 degrees up, so those former pose gates would be inappropriate here.

Primary estimate: current runtime matcher configurations are read from their source definitions, converted with the existing explicit half-resolution helper, matched bilaterally and LR checked. This uses full 640x400 frames, not the RGB frustum crop. The eroded convex hull of inner corners selects only the board interior. Sample density is one per 4x4 original-image grid and minimum dense support is 100. Valid samples retain the declared diagnostic 0.5..4.5 m interval. Each sample is projected with the existing partial-front geometry function, undoing R1 before applying reported block pitch and the 0.210 m rim-to-front dimension.

Secondary estimate: paired rectified checkerboard corners supply an independent correspondence method on the same images, with the same calibration/mount geometry. This comparison can reveal a dense-matching discrepancy but is not independent physical ground truth. The known square dimension also supplies PnP reprojection and board-normal-to-robot-forward diagnostics. PnP depth is not substituted for stereo distance and no new pose acceptance limit is introduced.

## Output meaning

`known_term_minus_physical_reference_m` compares median known longitudinal term K against the declared physical front-to-plane distance. It combines unknown optical datum, mount errors, stereo error and reference error; these terms cannot be separated from one point. No offset or scale correction is estimated or applied. The numeric 100 mm screen is the user's working target, not an accuracy qualification. No camera-to-front offset, runtime admission, calibration or active range is updated.

Every valid captured attempt saves decoded combined pixels losslessly as PNG before corner screening. Attempts with a detected left board also save both raw disparity maps, LR mask, board mask, selected pixel/depth samples and available corner sets in NPZ, even if dense support is insufficient. Reports retain per-attempt status and supplementary corner residuals for both candidate orders. Capture report includes file/calibration/mount/helper/source hashes, matcher settings, reprojection, board normal and all rejections. Files are new per run; source data are not overwritten. Results are preserved even on incomplete capture. This is not a control-loop timing benchmark.

## Command on Pi

Stop any preview using the stereo camera, keep motors disabled and the board stationary. From the updated repository:

```bash
/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/check_robot_front_board_distance.py \
  --reference-front-distance-mm 1500
```

Inspect the three FRAME lines and saved report. If it cannot collect three frames, inspect its recorded reason before another attempt. No automatic retries beyond the eight capture attempts and no series across distances is requested.

## Verification and preceding evidence

Seven offline tests cover stereo corner ordering, ambiguous/bad correspondences, absent supplementary right-board detection, independent dense-versus-corner refusal reporting, missing/invalid values and non-promoting comparison semantics. Together with the nine front-geometry tests, sixteen tests pass. Both new Python files pass py_compile. Actual OpenCV capture/matching on Pi is pending; this editing environment has no OpenCV/camera.

Preceding camera-only run 20261016_184327 successfully produced partial front geometry for 96/96 target cycles, K median 2.4427 m, processing median 1.03 ms. Steady target depth age by driver stamps was within 250 ms for 95/95 cycles (RGB maximum 245.48 ms); these are driver-to-result diagnostics, not exposure-to-command qualification. This establishes that the new diagnostic code executes on Pi; it does not resolve the physical offset or accuracy bound. No repetition of that person run is requested by the board comparison.

## First board capture refusal and correction

The first capture completed eight attempts, all with CORNER_EPIPOLAR_ERROR_EXCEEDED and no dense frame result. This establishes a corner-screen failure, not a dense SGBM support failure. The initial runner incorrectly let the supplementary corner check gate both estimates and did not save rejected images or numerical residuals. No exact residual magnitude or calibration-failure diagnosis can be inferred from that report.

The corrected runner separates the estimates without relaxing the corner threshold, LR rule, dense support or active policies. Supplementary corner refusal remains visible in every frame and in `comparison.corner_geometry_warning`; a numerical dense/reference comparison with that warning is not evidence that calibration geometry is valid. Rejected images and arrays are now retained to enable offline inspection. One corrected capture at the same physical setup is required because the prior rejected source images were not saved. Do not move the board or start a distance series.
