# Shadow: two-person RGB and stereo diagnostic, 2026-10-31

This is a camera-only, offline review. The operator reported the 650 nm projector
off and two people swapping positions between the two sessions. The frozen
archive SHA-256 is
`b8d0437995ff81b9b6ddcd24029ecfb0857571729614566fa97c70796b2b1cbb`;
the v2 RGB ONNX SHA-256 is
`5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b`.
The included stereo calibration is OV9281 Pi v7
(`39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`).

The source audit recorded 12 cycles in run `223127` and 15 in run `223230`.
Only 12 and 7 cycles, respectively, saved a disparity map: the old audit
computed stereo after selecting a single RGB candidate. The segmentation
worker counted two people in every cycle with saved disparity. The old RGB
candidate was the farther, right person in 18 of those 19 cycles.

Using the frozen LR-consistent disparity and calibrated stereo-to-RGB
projection, manually chosen central torso regions gave these **diagnostic**
medians, in meters:

| Run | Paired disparity cycles | Left | Right | Median right-minus-left |
| --- | ---: | ---: | ---: | ---: |
| 223127 | 12/12 | 1.6774 | 2.2930 | 0.6174 |
| 223230 | 7/15 | 1.6926 | 2.2886 | 0.6008 |

The left person was closer in all 19 comparable cycles. The people swapped
identities between the runs, so this is evidence for relative depth ranking
in this scene, not for identity continuity.

The supplied v2 model was also run on all 27 saved source JPEGs with the
repository's 640x640 letterbox and one-class ONNX decoder. Sampling the
central 40% of each detected box in both image axes gave:

| RGB confidence threshold | Both detected / 27 | Both detected with saved disparity / 19 | Nearer left in paired cases |
| --- | ---: | ---: | ---: |
| 0.70 | 9 | 5 | 5 |
| 0.40 | 21 | 14 | 14 |

At 0.40 the other six cycles detected only the farther person at right.
Five of them (first run cycles 4, 6, 7, 8, 9) had stereo maps; the sixth
(second run cycle 11) did not. The six nearer people were visible but fell
below 0.40; low-threshold model proposals ranged from 0.135 to 0.379.
The separate review package contains six byte-identical source JPEGs and
unreviewed LabelMe proposals for **both** people. It is not approved training
data until the boxes are manually checked. Adding these evaluated frames to
training also retires this series as an independent holdout.

The opt-in raw-pair capture now writes rectified OV9281 images, raw disparity,
LR-consistent disparity, and LR mask before RGB target selection. Its original
association gates are unchanged, and a successful target reuses the same
computed map. The capture wrapper pins the supplied v2 model SHA and the
0.40 **diagnostic detection** threshold. The summary checks that every
declared saved map belongs to its cycle. A new Pi capture is needed to verify
hardware timing and that maps are saved on actual RGB refusals.

These are camera optical depths from asynchronously captured frames, with
manually chosen or model-box interior regions. They are not surveyed
robot-front clearance, approved Measurements, a validated 0.8 m stop,
or permission to move motors. Until every plausible person has associated
fresh range and target ownership is clear, the Shadow command remains PAUSED.
