# Shadow RGB ten-minute annotation subset

The 2026-10-30 camera-only session reported `COMPLETE`, 1200/1200 saved slots,
zero skipped, and 573261798 JPEG bytes. Evidence root on Pi:

```text
/home/elwis/.local/state/sie/evidence/shadow_rgb_presence_20261030_175847_3dc7a8
```

This status is operator-reported. The selector first checks `session.json`,
its journal SHA, all 1200 image hashes, bytes and ordered driver sequences.
It does not rewrite the original capture. It writes a separate private
`annotation_subset_5s_v1` directory with **120 byte-identical copies**,
one for every five seconds of the 600-second run. It ranks consecutive-frame
changes on small decoded grayscale thumbnails. Within a five-second window,
a strong scene-change frame has priority; otherwise the midpoint is chosen.
These changes can be person motion, exposure or background motion. They are
not person labels. Every entry in the new manifest has `person_visible: null`
and `person_boxes_xyxy_px: null` pending manual annotation.

Run on Pi with the `sie_mvp` environment, where OpenCV is already available:

```bash
cd /home/elwis/dev_ws/sie_shadow_rgb_annotation_subset
root=/home/elwis/.local/state/sie/evidence/shadow_rgb_presence_20261030_175847_3dc7a8
/home/elwis/dev_ws/runtime_envs/sie_mvp/bin/python \
  vision_core/tools/prepare_shadow_rgb_annotation_subset.py \
  --session-root "$root"
```

The directory named in `Saved:` contains JPEGs and `selection_manifest.json`.
Do not add the photos to Git. Review each selected image and annotate whether
a person is actually present, then mark the upper-body box when present;
empty frames must remain explicit negatives. A positive or negative balance
is not claimed until the human labels have been counted. For missed brief
entrances, review the original adjacent frames and add them in a separate,
hash-bound annotation round. Split future validation by capture session, not
random nearby frames, to avoid temporal leakage.
