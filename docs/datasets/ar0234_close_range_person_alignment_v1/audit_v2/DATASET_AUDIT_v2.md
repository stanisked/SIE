# DATASET_AUDIT_v2

Dataset: `/home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1` (read-only audit).

Records: **394**, sessions: **18**, images: **394**, labels: **0**.

## Integrity

All records/images valid: **True**. Missing=0, extra=0, SHA mismatches=0, decode failures=0, wrong dimensions=0.

## Distribution

- Sessions: `{"hard-negative-room-objects-artificial-01": 26, "hard-negative-room-objects-daylight-01": 21, "hard-negative-teddy-artificial-01": 21, "hard-negative-teddy-daylight-01": 21, "sitting-center-artificial-01": 22, "sitting-center-daylight-01": 21, "standing-center-artificial-01": 22, "standing-center-daylight-01": 21, "standing-center-far-artificial-01": 22, "standing-center-near-daylight-01": 22, "standing-half-profile-left-artificial-01": 21, "standing-half-profile-left-daylight-01": 21, "standing-half-profile-right-artificial-01": 24, "standing-half-profile-right-daylight-01": 21, "standing-left-artificial-01": 25, "standing-left-daylight-01": 21, "standing-right-artificial-01": 21, "standing-right-daylight-01": 21}`
- Pose: `{"not_applicable": 89, "sitting": 43, "standing_front": 175, "standing_half_profile_left": 42, "standing_half_profile_right": 45}`
- Lateral position: `{"center": 217, "left": 46, "right": 42, "varied": 89}`
- Distance band: `{"0.8-1.2m": 22, "1.5-2.0m": 261, "2.5-3.5m": 22, "not_applicable": 89}`
- Lighting: `{"artificial": 204, "daylight": 190}`
- Contains person: `{"false": 89, "true": 305}`

## Duplicates

Exact duplicate groups: **0**. Near-duplicate pairs: **45**. Near criterion: mean absolute grayscale difference on 64×40 thumbnail ≤2.0; diagnostic only.

## Image quality

Brightness mean/median: `129.736` / `133.120`; clipping mean/max: `0.01193%` / `0.04527%`; sharpness mean/median: `156.582` / `142.751`.

## Status

Кадры ещё **НЕ размечены**: `labels/` пустой. Модель **НЕ обучалась**. Этот аудит не является acceptance policy и не даёт qualification для yaw или движения.

Contact sheet: `docs/datasets/ar0234_close_range_person_alignment_v1/audit_v2/contact_sheet_3_per_session_v2.jpg`.
