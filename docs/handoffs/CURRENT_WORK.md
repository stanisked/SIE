# Current work

## AR0234 ↔ OV9281 static extrinsic v1

Ветка `feature/ar0234-ov9281-static-extrinsic-v1` добавляет отдельные
инструменты capture и solver для static target с шахматной мишенью `9×6` и
стороной `24.5 mm`. Pairing использует только kernel `CLOCK_MONOTONIC` и
принимает skew не более `5 ms`; combined OV9281 делится по подтверждённой
семантике, где physical-left находится в правой половине кадра.

Кандидат трансформации имеет статус `PROVISIONAL_DIAGNOSTIC_ONLY`. Он не
активирует calibration, не попадает в runtime и не разрешает движение.
AR0234 intrinsic остаётся candidate, пока для него не появится отдельное
evidence. Hardware sync не доказан: static AR↔OV extrinsic разрешён только
для неподвижной сцены; dynamic pairing по host timestamps по-прежнему
запрещён.
