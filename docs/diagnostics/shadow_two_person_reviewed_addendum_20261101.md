# Shadow: reviewed two-person training addendum

Date: 2026-11-01. This is RGB training data preparation, with motors unused.

The operator corrected the six LabelMe proposals in
`shadow_two_people_reviewed_20261101.tar.gz` (SHA-256
`8b79e947ef381825440bfe2a826b463ac4568d42182b62917e1cb606b792438f`).
The six JPEGs, draft manifest, previews, and README match the prior draft byte
for byte. Five corrections tighten the far-right person's left edge; the sixth
adjusts both boxes. Visual review confirms two plausible upper-body rectangles
on each full 1920×1200 JPEG. Original LabelMe `sie_ignore_for_training` and
`sie_review_required` flags remain intact; the operator's subsequent review is
recorded in the **derived** dataset manifest.

`shadow_two_people_training_addendum_20261101.tar.gz` has SHA-256
`8e5038c5eb2c98150a5ab5c1110fb5ce69d111a0a7b939c76ec2175bf8445612`
and 4,369,588 bytes. It contains 6 unchanged JPEGs, 6 new YOLO labels of
class 0 (`person_upper_body`), and `manifest.json`/`README.md`. All 12 boxes
lie inside their source image. Every JPEG and derived label hash is recorded
in the manifest. Baseline training pool SHA-256 is
`9cec0505c3390efc44c59513204469d3f2d84785fff3598092fc5d306dadd6ef`:
230 training images, 208 positive boxes, 22 empty-label negatives, and 10
excluded records. The new JPEG hashes do not duplicate the 230 training JPEGs.

Rebuild with the exact three archives:

```bash
python3 vision_core/tools/build_shadow_two_person_training_addendum.py \
  --reviewed /path/to/shadow_two_people_reviewed_20261101.tar.gz \
  --baseline-train /path/to/shadow_rgb_upper_body_train_2sessions_20261031.tar.gz \
  --output /path/to/shadow_two_people_training_addendum_20261101.tar.gz
```

To expand the training pool, unpack the baseline training archive and then
this addendum into the **same new dataset directory**. The addendum's
`images/train/two_*` and `labels/train/two_*` paths add six pairs without
overwriting the `s1_*`/`s2_*` pairs. Preserve the baseline `manifest.json`
before extraction because the addendum contains its own `manifest.json` at
the root; put the addendum in a staging directory and copy only its
`images/train` and `labels/train` folders into the dataset. The result is
236 training images. Reuse the separate development validation split, and
train a newly named checkpoint. Do not call the six selected failure frames
or their source stereo runs an independent holdout after using them to train.

The current v2 model SHA-256 is
`5583d645afebd3d52706b76655662cb4ce16110b2a0b224af2cc806d0292d07b`.
These six frames were chosen because the current detector missed the nearer
person. No model accuracy improvement has been measured yet. Before robot
use, evaluate the retrained model on a fresh two-person blind capture and
separately validate depth, target ownership, front clearance, and stopping.

## Colab v3 preparation

The new notebook `SIEE_Shadow_RGB_YOLO11n_v3_Two_People_Colab.ipynb` has
SHA-256 `f7972bb7896eed13697b9ed9276c3179c471fe962c6e952fcc3ef43e737916a6`.
It is stored beside the three original v2 dataset parts and the addendum in
the [Shadow v2 Drive folder](https://drive.google.com/drive/folders/1d9OYCDuvjT4cKxyN86foeagra9lVpd_p).
The starting v2 `best.pt` has the recorded SHA-256
`575c480b03b1722222211f474fae2dc8c7c6fe5e02c9078433d5d9e16667b395`.
The notebook refuses a checkpoint, archive, image, or label mismatch, merges
six pairs only into training, and writes a distinct 236-image dataset YAML.
It trains with `resume=False` from the v2 weights, compares the v2 and v3
checkpoints on the previously inspected 120-image development split, and
exports a static 640×640 ONNX. It also checks both boxes on the six training
frames and labels that result as training-set diagnostics. Results are saved
under a new `ar0234_shadow_person_upper_body_yolo11n_v3_two_people` folder.
This notebook is prepared and syntax checked; training has **not** been run.
