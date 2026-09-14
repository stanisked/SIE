# DATASET_AUDIT_v1

Dataset: `/home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1` (read-only audit).

Records: **190**, sessions: **9**, images: **190**, labels: **0**.

## Integrity

All records/images valid: **True**. Missing=0, extra=0, SHA mismatches=0, decode failures=0, wrong dimensions=0.

## Distribution

- Sessions: `{"hard-negative-room-objects-daylight-01": 21, "hard-negative-teddy-daylight-01": 21, "sitting-center-daylight-01": 21, "standing-center-daylight-01": 21, "standing-center-near-daylight-01": 22, "standing-half-profile-left-daylight-01": 21, "standing-half-profile-right-daylight-01": 21, "standing-left-daylight-01": 21, "standing-right-daylight-01": 21}`
- Pose: `{"not_applicable": 42, "sitting": 21, "standing_front": 85, "standing_half_profile_left": 21, "standing_half_profile_right": 21}`
- Lateral position: `{"center": 106, "left": 21, "right": 21, "varied": 42}`
- Distance band: `{"0.8-1.2m": 22, "1.5-2.0m": 126, "not_applicable": 42}`
- Lighting: `{"daylight": 190}`
- Contains person: `{"false": 42, "true": 148}`

## Duplicates

Exact duplicate groups: **0**. Near-duplicate pairs: **25**. Near criterion: mean absolute grayscale difference on 64×40 thumbnail ≤2.0; diagnostic only.

## Image quality

Brightness mean/median: `126.990` / `132.583`; clipping mean/max: `0.00834%` / `0.02561%`; sharpness mean/median: `164.381` / `140.387`.

## Status

Кадры ещё **НЕ размечены**: `labels/` пустой. Модель **НЕ обучалась**. Этот аудит не является acceptance policy и не даёт qualification для yaw или движения.

Contact sheet: `docs/datasets/ar0234_close_range_person_alignment_v1/audit_v1/contact_sheet_3_per_session.jpg`.
