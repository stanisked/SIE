# Fast Shadow source snapshot run, 2026-11-12 19:34:49

## Source and scope

- Operator evidence: `shadow_fast_20261112_193449_review.tar.gz`, SHA-256
  `2072ba9230f9ed79a0863ba203eba89519a8d719e03b807eeadf612c2d65d43c`.
- Pi evidence directory:
  `/home/elwis/.local/state/sie/evidence/shadow_fast_20261112_193449`.
- The archive contains `bridge.jsonl` and nine original-JPEG, disparity,
  LR-consistency, instance-mask and calibration-array bundles with manifests.
  The source verifier found 9/9 requested bundles, zero missing and zero
  errors. The Pi writer reported accepted/completed 9/9, skipped/errors/
  refusals 0/0/0. No images or camera data are committed to Git.
- This was a camera-only, motor-disabled `segmentation_mask` preview. Its
  diagnostic depth range was 0.5–4.5 m, separate from the activated 0.5–2.0 m
  Measurement range. No Measurement, approved range or motor action resulted.

## Per-cycle findings

The 75-second Bridge journal contains 1,565 rows: 1,269 polls with
`NO_NEW_CAMERA_CYCLE` and 296 distinct camera cycles. The first camera result
arrived 13.382 s after the Bridge began; camera results span 61.607 s.
The segmentation reported exactly one instance on cycles 1–48 and zero
instances on cycles 49–296. Of the first 48, 44 had a non-edge eligible
target, 43 reached mask-depth evaluation, and one was refused as
`BBOX_NEAR_LEFT_IMAGE_EDGE`. No second person or appearance after cycle 48
was observed in this journal. The nine saved RGB and stereo frames show one
person moving within the room and approaching the camera during cycles 4–44.
No empty-room RGB snapshots were retained, so the final 248 no-person cycles
are a segmentation-journal observation, not an independent image audit.

Of the 43 evaluated masks, 13 had `COHERENT_MASKED_DEPTH_DIAGNOSTIC_ONLY` and
30 had `DEPTH_MAD_EXCEEDED` (MAD 0.050234–0.139686 m, unchanged 0.05 m
threshold). The 13 coherent cycles form intervals 4, 16–20, 31–33, 38–40,
and 44. They carry five different track IDs despite the visible single
person. The ID is reset after a refused depth/mask cycle, so the five
`FIRST_COHERENT_AFTER_TRACK_LOSS` snapshot reasons do **not** demonstrate
reappearance after the person left the frame. Every one of the 1,565 Bridge
decisions was PAUSE; the 13 coherent cycles were refused by
`FRONT_LAYER_BINDING_UNVERIFIED`, and the 30 MAD failures had no confirmed
track. The no-target cycles remained PAUSE.

Driver-timestamp-to-Bridge-decision ages on all 296 camera cycles (seconds):

| Source | Median | Maximum | At or below 0.5 s |
| --- | ---: | ---: | ---: |
| AR0234 | 0.176763 | 0.359337 | 296/296 |
| OV9281 | 0.175512 | 0.355097 | 296/296 |

For the 48 cycles with one segmented person, medians were 0.284523 s
(AR0234) and 0.282028 s (OV9281); maxima stayed 0.359337/0.355097 s.
The all-cycle host-processing median/max were 0.195700/0.379070 s. These
ages use driver-declared timestamps at the Bridge decision, not independent
exposure timing or a motor-command endpoint. This snapshot-enabled run meets
the experimental 0.5 s driver-age target for all *observed camera cycles*;
it does not replace the earlier fast-path benchmarks.

## Source replay and front surface

Using the saved LR-gated half-resolution disparity, exact mask and saved
calibration arrays, `prepare_stereo_points` and `evaluate_prepared` reproduced
the Pi mask-depth dictionaries, front point counts and p05 values for all nine
bundles. The source verifier also checked original JPEG bytes, decoded RGB
pixels against the segmentation payload SHA, mask SHA, bundle SHA, track ID,
instance index, model SHA, cycle ID and both driver timestamps against the
Bridge journal.

In this run, all-point and numerical candidate-layer front p05 differed by
at most 0.029194 m across all 43 evaluated cycles; none crossed the 0.15 m
disagreement snapshot trigger. This does **not** undo the earlier archived
two-layer mismatch. The new scene mostly produced one supported numerical
layer, including all 13 coherent cycles. For example, cycle 44 had a
1.875412 m median *optical depth* and a 1.466537 m front known-term p05;
these are different reference quantities and percentiles. The known term
also depends on the operator-reported 15-degree block pitch and 0.210 m
front-rim offset; the optical-to-rim datum remains unknown. Projected near
p05 samples lie inside the selected mask and torso-like ROI in the saved
frames. An additional offline distance-transform check found the near-p05
sample median 82–178 RGB pixels inside the mask across the nine bundles,
with 3–97 pixel minimum distances. Thus these samples are not uniformly a
thin mask-edge fringe; cycle 20 and 44 still contain near samples close to
the boundary. This geometric test alone cannot establish physical person
ownership, near-obstacle clearance or distance accuracy. The adapter's
front-layer refusal remains correct.

## Next independent work

The next depth question is whether these projected points represent the
physical person's visible surface with an independently checked error bound.
Compare saved stereo/body geometry with an appropriate measured datum before
choosing a front-range statistic; do not change the MAD threshold, front
point-set contract or motor boundary from this journal. The archive cannot
test post-exit reacquisition or simultaneous two-person selection; keep both
cases explicitly untested. Do not ask for another fast latency proof or a
straight-wheel motion probe.
