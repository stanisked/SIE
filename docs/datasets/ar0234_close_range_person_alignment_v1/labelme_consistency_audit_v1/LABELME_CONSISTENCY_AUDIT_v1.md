# LabelMe consistency audit v1

## Scope

Read-only audit of `/home/stanislav/sie_rgb_stereo_fusion/datasets/ar0234_close_range_person_alignment_v1`. No source image, LabelMe JSON, manifest, frozen inventory, YOLO label, split or model artifact was changed.

## Facts

- LabelMe JSON: `305`; annotated images: `305`; total shapes: `305`.
- Exact `person_upper_body`: `305`; rectangles: `305`.
- Proven impossible geometry: `0`; wrong labels: `0`; multiple/non-single shapes: `0`; image mismatches: `0`.
- Edge-touch diagnostics: top `23`, bottom `103`, left `0`, right `0`.
- Point-order diagnostic: `43` rectangles have reversed X point order. This is not automatically a visual bbox error, but the current converter's strict `x1 < x2` rule will reject them.

## Bbox distribution

| Metric | min | P05 | median | P95 | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| aspect ratio | 0.2819 | 0.3628 | 0.4451 | 0.9878 | 1.3126 |
| relative area | 0.1084 | 0.1239 | 0.2604 | 0.3736 | 0.4274 |

Aspect-ratio extremes are concentrated in sitting sessions; this is a pose correlation, not proof of incorrect annotation. The contact sheet is the required visual review aid.

## Workflow findings

Current LabelMe config declaration is `labels: [person]`. It does not match the committed single-class contract unless it declares only `person_upper_body`. Existing annotations remain `person_upper_body`; do not relabel or mass-edit them from this finding alone. `imagePath` values resolve correctly to the source PNG, but are relative (`../images/...`); current converter requires only a basename and therefore is incompatible with these otherwise consistent LabelMe files.

## Manual review candidates

No semantic bbox inconsistency is proven by geometry alone. Manual review is limited to the listed structural candidates plus top/bottom edge-touch subsets, where the protocol requires confirming that an image boundary really truncates the torso.

- structural candidates: `43`
- top-edge candidates: `23`
- bottom-edge candidates: `103`

## Decision

**A is the work-preserving path:** retain `person_upper_body`. B would silently change training semantics and is unsupported. C is appropriate only for individually reviewed bbox candidates, not a bulk redraw. Before any YOLO conversion, make a separate narrow converter/config compatibility change for resolved relative `imagePath` and normalized rectangle point ordering; that is a workflow fix, not a label rewrite.

The audit does not train a model, create a split, qualify yaw alignment, or change motion behavior.
