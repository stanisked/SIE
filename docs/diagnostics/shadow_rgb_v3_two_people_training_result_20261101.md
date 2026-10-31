# Shadow RGB v3: two-person training result

Date: 2026-11-01. Development diagnostics only, no motor use.

The operator ran `SIEE_Shadow_RGB_YOLO11n_v3_Two_People_Colab.ipynb` on a
Tesla T4 with Ultralytics 8.4.152 and torch 2.11.0+cu130. The v3 checkpoint
SHA-256 is `e5d7e67f7ab2b53f6a8548e3eec13fdf52e84d390cf0e58088ba40dca79989db`.
The exported ONNX SHA-256 is
`fccd45fbcf490134eb786ab3b38cf115dd03466edd85801ad44c8353f38549f5`.
Its attached bytes match the recorded digest. The notebook ran
`onnx.checker.check_model` before writing the report; independent Raspberry Pi
inference is still pending. The three v2 dataset archive hashes and the
reviewed addendum hash match the inputs recorded in the earlier handoff.

| Same inspected 120-image development set | v2 weights | v3 weights |
| --- | ---: | ---: |
| Positive images with an IoU ≥ 0.50 box, conf 0.40 | 88/89 | 89/89 |
| Empty images with a detection, conf 0.40 | 1/31 | 2/31 |
| mAP50 | 0.99456 | 0.99500 |
| mAP50–95 | 0.78947 | 0.81349 |
| Six selected *training* frames with both boxes, IoU ≥ 0.50 | 1/6 | 6/6 |

The Colab notebook's original `frames_conf_0p40_iou_0p50` output mistakenly
used dictionary keys `v1` and `v2` for `old_model=V2_BEST` and
`new_model=NEW_BEST` (v3). The numeric values are unchanged. The original
[`comparison.json`](https://drive.google.com/file/d/1UiXGi6Z0xDdXcJ5Jzowc0jeh34XdZr9r/view?usp=drivesdk)
is preserved with SHA-256
`3595740d77535415a60ddc08657efd2ea0749ca0b968e49e08550fb867c40ee1`.
The [label-corrected derivative](https://drive.google.com/file/d/1KrxcwhtNdqBKoS0RFC3CyQJUbBWwWDHM/view?usp=drivesdk)
has SHA-256 `10a1a14c280bf45ba929e10058b8eab2265974c5eedf8b81ae8b3c6d4399086a`;
it includes the original SHA and explicit `v1→v2`, `v2→v3` mapping.
The notebook source now uses the correct keys and refuses to export a report
from stale `frames` values until its frame-metrics cell is rerun.

The six selected failure frames were included in training, and the 120-image
development set was previously inspected. Neither measures fresh two-person
generalization. The extra false-positive empty image also needs visual review.
The existing offline ONNX evaluator now accepts the pinned v3 hash for its
single-person reviewed subsets. It rejects more than one labeled person per
frame, so it cannot score a new two-person session without a separate
multi-person annotation and matching evaluator.
Collect a new, projector-off blind session with two people jointly visible,
one-person and empty intervals, occlusion and edge cases. Freeze labels before
comparing v2 and v3 ONNX on Pi with the same preprocessing and threshold.
Test freshness, timing, target ownership, range association and stopping
separately before considering Shadow motor use. Active OV9281 Pi v7 calibration
remains SHA-256 `39e1520efd2c589009426312c6968c79cc9c3896e6c3a2c7e044686106eb7d52`.
