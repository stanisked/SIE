# Shadow RGB person detector: reviewed data and Colab v2 handoff

Date: 2026-10-31. This is a camera-only model-training preparation record.
The Colab v2 training has **not** run. No motor command, admitted Measurement,
identity tracking, or safety envelope is authorized by these RGB labels.

## Evidence and split

| Set | Source | Images | Positive upper-body boxes | Clean negative | Excluded |
| --- | --- | ---: | ---: | ---: | ---: |
| Train s1 | 10-minute AR0234 session, projector on | 120 reviewed | 108 | 10 | 2 |
| Train s2 | 10-minute AR0234 session, projector off, two people appearing in turn | 120 reviewed | 100 | 12 | 8 |
| Development validation | Different 10-minute AR0234 session, projector on | 120 reviewed | 89 | 31 | 0 |

Each original session saved 1200/1200 scheduled frames. The reviewed subsets
are 120 frames per session. All 350 included images have a paired YOLO text
label; empty label files represent clean negatives. The two training sessions
use `s1_` and `s2_` filename prefixes to prevent collisions. SHA-256 of each
source JPEG was checked against the selection manifest. No train/validation
image has a matching SHA-256. The v2 training pool contains 230 images with
208 positive boxes and 22 negatives. Its complete archive SHA-256 is
`9cec0505c3390efc44c59513204469d3f2d84785fff3598092fc5d306dadd6ef`.

The operator-reviewed s2 LabelMe archive SHA-256 is
`4ee79b52c1c421fdeb91be9abedf938c6bf85afc777f97819a3a8cfdd41d97af`.
Two s2 frames (`000830`, `000983`) contain a partial limb without a usable
upper-body box and are excluded, rather than incorrectly used as negatives.
Five nearly black frames and one further partial frame were already marked
ignore in the reviewed annotations. The source JSON flags `review_required`
were left unchanged; the operator explicitly confirmed review of all frames.

The development validation was previously inspected and diagnosed. It is
suitable for comparative development and early stopping, **not** for an
independent final generalization claim. The original ONNX model (SHA-256
`dde42238b5742f9c0b79c29863c44bc97b678aa75b86a4db5bb602a2d72c259c`)
localized 71/89 positive frames at confidence 0.40 and IoU >= 0.50, with
zero detected negatives out of 31. The frozen offline report records the
preprocessing and postprocessing used for that baseline. A Colab PyTorch
rerun can differ from this ONNX result; comparisons in the new notebook use
the same Colab path for both v1 and v2 before a separate Pi ONNX check.

## Colab provenance and files

The [v1 Drive run](https://drive.google.com/drive/folders/1W4LH5pjws_t8GTmdKTLCddKT9Y4nspqg)
contains `train_run/weights/best.pt`, `last.pt`, `best.onnx`, the training
summary and the original notebook. It used `yolo11n.pt`, Ultralytics 8.4.152,
Tesla T4, `imgsz=640`, batch 16, up to 100 epochs, patience 20, and selected
epoch 27 after stopping at epoch 47.

The [Shadow v2 Drive folder](https://drive.google.com/drive/folders/1d9OYCDuvjT4cKxyN86foeagra9lVpd_p)
contains the notebook and three archives. Their SHA-256 values are:

| File | SHA-256 | Bytes |
| --- | --- | ---: |
| `shadow_rgb_colab_v2_train_s1_20261031.tar.gz` | `9a6a65a800c0847bdac646cd05c631de92aeacc976226160f9ec720ea03e4ebb` | 56,519,059 |
| `shadow_rgb_colab_v2_train_s2_20261031.tar.gz` | `76550684e0826f210d93989a62f233443ae12af179066365e6bd53f38c191feb` | 50,483,630 |
| `shadow_rgb_colab_v2_val_20261031.tar.gz` | `80b74757e35c0a76a7d13f21c43bc0f9ef3813c5af0f4156053c9a26ed706c6c` | 53,610,610 |
| `SIEE_Shadow_RGB_YOLO11n_v2_Colab.ipynb` | `7b9e2722082a3e92277b15aa4307b1434e76f4cd795887e447eb3aafd45ef851` | 11,213 |

The combined 160,612,646-byte archive was split to stay below the Drive
upload limit. The pieces contain 238, 224 and 240 tar members respectively,
702 distinct members in all: 350 JPEG/YOLO label pairs, `dataset.yaml` and
`bundle_manifest.json`. The notebook checks each archive SHA-256, every JPEG
against the manifest, the split counts, and safe tar members. It loads the
preserved v1 `best.pt`, trains with `resume=False`, evaluates both checkpoints
on the same development validation, and exports static 640x640 ONNX opset 17.
It writes new `best.pt`, `best.onnx`, `args.yaml`, and `comparison.json` in a
new Drive run folder. These outputs do not exist until the notebook finishes.

## Next verification

Run the notebook in Colab with a T4 GPU. Inspect frame-level differences and
the saved `comparison.json`. Then collect a new blind projector-off session
with two people appearing in turn, including empty-room frames. Evaluate
person localization, false-positive images, behavior at frame edges, and
end-to-end inference latency on Pi. The active OV9281 Pi v7 calibration stays
SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
Fresh target ownership, robot-front distance and physical stopping checks are
separate prerequisites for any Shadow motor runner.
