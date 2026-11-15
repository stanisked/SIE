# Fast Shadow: moving-person track and torso depth, 2026-11-15

## Primary evidence and code

- Operator Pi evidence: `/home/elwis/.local/state/sie/evidence/shadow_fast_20261115_134056`.
- Complete review archive: `shadow_fast_20261115_134056_review.tar.gz`, SHA-256
  `4d85b02bf2efe3f9e26545aee62c2a8b58e7fa10afd6839ea223419e876db5f1`.
  The operator-uploaded ChatGPT Library file is
  `libfile_d5cfa7b3a2748191a90a7ab457bef1f9`.
  Its `bridge.jsonl`, exact per-cycle audit JSON, and 15 source bundles were
  reviewed. The source verifier reported 15/15 validated, no missing bundles
  or errors; the upstream journal reported dropped=0, errors=0.
- Branch `codex/shadow-turtlesim-follow-20261111`, commit
  `d00d4e0949c291ae86b45f80c5c7c0a49ce1b472`: top-truncated RGB
  selection from commit `a140f71c09e32d8fc1e0f71955f3aaa312be4541`,
  followed by two-current-mask diagnostic ID bootstrap. No calibration,
  activation profile, or motor execution route was changed in these commits.
- This was the `segmentation_mask` Fast Shadow **motor-off** preview. The
  following depth and front-distance values are diagnostics, not admitted
  SIE `Measurement` records or motor authorization.

## What the moving-person run established

| Cycles | RGB target and track | Depth and Bridge result |
| --- | --- | --- |
| 1–41 | One moving person, track `:1` throughout; 10 eligible boxes touched the image top. | All 41 `FOLLOW_ELIGIBLE`, including whole-mask MAD refusals using the current local torso surface. |
| 42–111 | Target moved to the right image boundary; 61 cycles had no segmentation instance and nine had a raw instance that was not eligible or failed right-edge geometry. | 70 `PAUSE`; no range or track was carried through the long absence. No RGB source bundle was saved inside the long absence, so physical absence relies on segmentation and adjacent frames. |
| 112–113 | One current instance on each cycle; `shadow_rgb_bootstrap` counted 1 then 2 matching masks. The diagnostic ID was seeded on cycle 113 for the next cycle. | Whole-mask MAD exceeded, but the stable torso-core set had 1,005 and 1,075 points. Both cycles stayed `PAUSE` without publishing distance. |
| 114–215 | Reacquired track `:2` persisted for all 102 cycles; 22 eligible boxes touched the image top. Saved RGB frames include partial top cropping and visible torso. | All 102 `FOLLOW_ELIGIBLE`. Cycle 114 already used the current torso surface while whole-mask MAD was still exceeded. |

Across the 215 camera cycles, 143 were `FOLLOW_ELIGIBLE` and 72 `PAUSE`.
Exactly 145 cycles had a current selected mask and stereo diagnostic: 121
whole-mask coherent, 24 with `DEPTH_MAD_EXCEEDED`. Of the 24 MAD cycles, 22
retained current RGB identity and an eligible local torso surface; the other
two were the deliberate bootstrap pauses. In 32 eligible cycles the RGB box
touched the top boundary. There were no two eligible people, so this run does
not test simultaneous-person disambiguation.

The selected torso-front diagnostic range was 1.633–2.352 m, median 2.121 m.
AR0234 driver-declared SOE-to-Bridge-decision age had median/max 286.10/326.60
ms; OV9281 had 284.39/328.72 ms. All 215 were under the experimental 500 ms
budget. The timestamps are driver declarations, not an independently measured
physical exposure-to-command latency.

## Precise conclusion and boundary

**FACT:** While the moving person remained selected, the AR0234 current-frame
instance mask maintained a stable diagnostic track ID even when the top of
the person was cropped. The diagnostic projection selected current OV9281
stereo points within the RGB instance and torso region; 143/145 selected cycles
produced a current torso-front range for the motor-off Shadow preview. After
a long departure, two current RGB/mask/torso cycles re-established diagnostic
identity; the next cycle produced an eligible current range while whole-mask
MAD was still high. This is the demonstrated RGB-target-to-stereo-depth
capability for the moving-person MVP preview.

**NOT VERIFIED:** A physical error bound for distance to the person, independent
exposure synchronization, identity ownership with two people in view, complete
swept-path obstacle clearance, canonical `Measurement` admission, and a
motor-on follow session. Local fit residuals quantify internal consistency,
not physical depth accuracy. Do not relabel `FOLLOW_ELIGIBLE` as permission to
move or the diagnostic torso value as an admitted `Measurement`.

## Earlier comparison, with scene caveat

The previous motor-off archive `shadow_fast_20261115_131118_review.tar.gz`
(SHA-256 `6924eef4dc1243ba6fb9dc54d1e9c2636236ea802ba4525304b353b0a5a195cc`)
had 216 camera cycles, 136 eligible, 80 pauses and ten invalid RGB/mask/track
bindings. Nine of those occurred on the return before a coherent mask could
seed a new ID. The new run had two such binding pauses, both explicitly
labelled bootstrap cycles. Different person trajectories and lighting mean
that the change in total eligible count is not a controlled accuracy gain.
