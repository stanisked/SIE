# OV9281 v7 original/current epipolar comparison, 2026-10-16

Status: OFFLINE_DIAGNOSTIC_ONLY. No calibration, threshold, runtime activation, Measurement or actuator changes.

## Physical comparison and datum limitation

The operator placed a vertical 9x6-inner-corner board with 25 mm squares 1500 mm horizontally from the robot front. Reported mount geometry remains a lens rim 210 mm behind the front and a block pitch of 15 degrees upward. Three dense-board frames supplied 403, 621 and 509 LR-consistent samples. Their nominal front-distance terms were 1.512272, 1.508318 and 1.513809 m; median 1.5122719975027035 m, frame-median span 5.4913 mm. The median exceeds the declared physical reference by 12.2720 mm.

This is a single-point comparison, not a physical error bound. The operator subsequently estimated the presumed optical centre as approximately 12.3–12.5 mm behind the front lens rim using calipers and recollection of the disassembled optics. Treat this as approximate operator evidence, not a surveyed optical datum or a fitted correction. If measured along the tilted lens axis, its longitudinal component requires projection through the mount orientation. No submillimetre accuracy follows from numerical agreement.

All three corner comparisons refused with CORNER_EPIPOLAR_ERROR_EXCEEDED. Original-order median absolute rectified vertical residuals were 2.1414795, 2.1128540 and 2.0968018 full-resolution pixels. Reverse-180 order was clearly worse. Dense-board results did not waive this warning.

## Offline checks actually performed

Archive extraction used safe data-only extraction; source data were not overwritten. Calibration SHA-256 and source image/array hashes matched the current report. With OpenCV 5.0.0, repeated exhaustive SB detection reproduced median vertical residuals about 2.10–2.14 px. Classic chessboard detection followed by cornerSubPix reproduced approximately 2.08–2.13 px.

A separate NumPy implementation inverted the five-coefficient distortion model iteratively, rotated normalized rays by R1/R2, and projected with P1/P2. Stored rectified coordinates agreed within 0.00004 px. The residual therefore is not explained by stored-coordinate rounding or one corner detector. Across the current board it is approximately a constant signed vertical offset with smaller spatial variation; this does not identify a physical cause.

The original candidate report contains 62 pairs, split 47 train / 15 internal holdout. Its calibration file is byte-identical to the active v7:
39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52.

Both original capture metadata hashes and all 30 supplied holdout image hashes matched. Physical LEFT is stored in stereo_right; physical RIGHT in stereo_left. No camera semantic reversal was applied.

All 15 original holdout pairs were redetected using exhaustive SB and rectified with that same active calibration. Median of per-pair median absolute vertical residuals: 0.1027984619140625 px; worst pair median: 0.2151336669921875 px. Original report values were 0.09844970703125 and 0.2166595458984375 px respectively. The small redetection differences do not approach the current approximately 2.11 px residual.

Spatial-overlap check: 86 original physical-left corners lay within 20 px of at least one current first-frame physical-left corner. Their median absolute vertical residual was 0.0907440185546875 px. Median distance from each current corner to its nearest old corner was 6.6671 px. A single old board hull contained up to 30/54 current corners. This is partial coverage, not identical binocular rays, depth, pose or full-field validation.

## Findings and exclusions

The same calibration describes the original images substantially better than the current images, including partly overlapping image regions. A different board location alone is insufficient to dismiss this evidence. The current residual is approximately one pixel in the 640x400 processing mode; same-row stereo correspondence can be degraded, but its causal contribution to person refusals was not isolated.

Operator clarification at 2026-10-16 22:54 Moscow: OV9281 lenses were unscrewed/adjusted BEFORE the original 2026-09-28 capture. That adjustment is not evidence of a post-calibration optical change. Do not claim later lens adjustment, mechanical movement, thermal drift or a specific calibration defect as an established cause.

A rigid-body pitch of the complete block does not itself change internal stereo extrinsics. Mechanical flex during handling would be a separate unverified hypothesis.

## Next bounded step

Use the already saved three current pairs to test whether one small relative-camera rotation can explain their vertical residual consistently. Evaluate on held-out corners/frames and report remaining spatial structure and changes to horizontal disparity. A good fit would establish compatibility with a rotation hypothesis, not prove physical movement. No live calibration replacement, image shift, threshold relaxation, depth correction or new distance sweep is authorized by this comparison.

## Source identifiers

| Artifact | SHA-256 |
| --- | --- |
| front_board_offline_review.tar.gz | 1e21bac65689279820faaf3c8ea2c3b20708cbc80d65a667a269d196b5268537 |
| v7_original_holdout_review.tar.gz | d7626969995456c8d2f759372fd0387584b4f29ff19364c91b443f6b2b8c6f8c |
| stereo_candidate_report.json | 9c67c701359181151edf41c2bf435b141c2b78b697753e81e74c27bfe664bcb1 |
| Current board report.json | 5f4ea35683b4170f230784491cc57be41f97c0161e10ac59efba34de43d2bda4 |

Operator source paths:
- /home/elwis/dev_ws/ov9281_pi_stereo_candidate_v7/stereo_candidate_report.json
- /home/elwis/dev_ws/ar0234_ov9281_extrinsic_raw_v1
- /home/elwis/dev_ws/ar0234_ov9281_extrinsic_supplement_right_v1

Source archives remain external evidence; hashes identify the reviewed copies. No current capture directory path was inferred from its uploaded archive.
