# Frozen LabelMe to YOLO conversion v1

## Scope

Controlled conversion of the frozen LabelMe package
`ar0234_close_range_person_alignment_v1` into derived YOLO labels. The only
dataset files created by this operation are `labels/*.txt`.

## Provenance

- source snapshot ID:
  `ar0234-close-range-person-alignment-v1-7ca5104ddcd29e22`;
- converter commit:
  `f8db449958ddee58f71a044b57ebe98a264bd3aa`;
- converter schema:
  `sie.ar0234_close_range_person_alignment_annotation.v1`;
- inventory timestamp: `2026-09-15T12:35:12.154453+00:00`.

## Result

- apply result: `CONVERTED`;
- positive YOLO labels: `305`, each with one class `0` bbox;
- negative images without `.txt`: `89`;
- normalized LabelMe point-order records: `43`;
- apply failures: `0`;
- immediate validator result: `VALID`;
- `positive_pending_annotation=0`;
- `split_created=false`.

No overwrite was requested or used. Raw PNG, `images/`, manifests, LabelMe
JSON, frozen inventory, splits and model artifacts were not modified. Training
and model qualification were not performed.

`DERIVED_LABEL_INVENTORY_v1.json` contains SHA-256 for every created `.txt`,
the source snapshot, converter commit and source hash. Its own SHA-256 is
`2af0bd4cfdc814e631f8673c02b8f786d79369127181b6c694b8abf7687cae5b`.
