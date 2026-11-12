# Fast Shadow front range: selected depth layer versus all mask points

## Source and scope

- Pi session: `shadow_fast_20261112_133000`.
- Archived control-preview audits: `control_preview_audits.tar.gz`, SHA-256
  `f15bc25d42ccfe3abf91e8c8ce8add5c27a3ceaba24dbe12dcfe8a26c065877b`.
  The archive contains 124 JSON records, cycles 1–124, with no source images
  or disparity arrays.
- Bridge journal: `shadow_fast_20261112_133000_bridge.jsonl`, SHA-256
  `565515a2fd0cc85519e5dd7667be45dbc4547ec5b4b24353fefc9c6d16cf37d9`.
  It contains cycles 1–122. Cycle 123 is an additional upstream diagnostic;
  cycle 124 ended in `CANCELLED_SHUTDOWN`.
- The bridge reported 21 `FOLLOW_ELIGIBLE` *diagnostic range-selection*
  results. All 124 upstream control previews remained STOP, with
  `metric_measurement_authorized=false` and `actuator_access=false`.
  The fast-mask launcher and bridge did not execute motors.

## Exact mechanism in the current implementation

`depth_from_owned_samples` in
`vision_core/person_localization/instance_mask_stereo_fusion.py` sorts the
mask-owned stereo depths and splits them at a depth gap. Groups with fewer
than 100 samples are omitted from the reported `depth_layers`. Of the
remaining groups, the largest group and the 25% support rule determine
which layers are plausible. The candidate MAD is computed within the one
plausible layer, not across all mask-owned depths.

`evaluate_prepared` in `vision_core/tools/preview_mask_surface.py` instead
calls `describe_front(u[selected], v[selected], z[selected], ...)` with
**all** mask-owned stereo points. The result's `known_term_m.p05` therefore
need not describe the depth layer that passed MAD. The fast Shadow adapter
checks the cycle, mask and front-profile hashes, but it does not establish
that `front_geometry.point_count` belongs to the selected depth layer. It
subtracts the operator's 0.0125 m optical-to-rim term from that all-point
p05. This is a mismatch of point sets, not a timestamp or hash failure.

| Cycle | Selected / candidate samples | Supported depth layers (samples, optical median m) | Candidate MAD m | All-point front p05 m | Bridge range m |
| --- | ---: | --- | ---: | ---: | ---: |
| 18 | 2850 / 2726 | 2726 at 2.542 | 0.043 | 2.054 | 2.041 |
| 19 | 2851 / 2529 | 321 at 1.282; 2529 at 2.383 | 0.038 | 1.045 | 1.032 |
| 96 | 1841 / 1841 | 1841 at 2.677 | 0.036 | 1.584 | 1.571 |
| 97 | 2368 / 2367 | 2367 at 2.713 | 0.037 | 2.302 | 2.289 |
| 105 | 1588 / 1431 | 157 at 1.777; 1431 at 2.866 | 0.041 | 1.583 | 1.571 |
| 109 | 1536 / 1296 | 240 at 1.814; 1296 at 2.880 | 0.028 | 1.523 | 1.510 |

In cycle 19 the nearer 321-point layer fails the 25% support fraction
relative to the 2529-point layer, so the far layer passes MAD while the
all-point front p05 lands near the nearer layer. Between cycles 18 and 19,
the bridge range changes by 1.009 m in 0.272 s. Between cycles 96 and 97,
it changes by 0.718 m in 0.267 s. Cycle 96 shows that even one *reported*
supported group and equal selected/candidate counts do not guarantee a
stable front p05: the front median is 2.457 m while p05 is 1.584 m. The
underlying array distribution and physical target ownership cannot be
reconstructed from these JSON records alone.

Across the 21 bridge-eligible cycles, ten contain two reported supported
depth layers, and 14 have more selected mask points than candidate-layer
points. During cycles 105–109 the candidate depth stays around 2.866–2.880 m,
while all-point front p05 ranges from 1.425 to 1.583 m. Nine of the 21 bridge
ranges exceed the separate 2.0 m activated Measurement range. No change to
the active calibration, MAD threshold or Measurement policy follows.

## Decision boundary

The same-cycle RGB, mask, stereo, driver timestamps and fast latency were
observed. The front range consumed by the experimental Shadow bridge is
**not proven to be derived from the depth layer that passed quality checks**.
The missing images/arrays also prevent identifying which physical surface
made each layer. Keep fast-mask motor execution disabled. Before proposing
an activation path, record layer-specific front geometry and all-point
near-surface evidence separately, define and challenge an ownership and
ambiguity rule with source frames and physical reference, then verify the
end-to-end decision again. Do not simply discard the nearer layer or relax
the MAD gate.

## Fail-closed integration fix

The front diagnostic producer now marks its point set explicitly as
`ALL_CURRENT_MASK_OWNED_STEREO_POINTS`. The fast Shadow adapter requires a
separately demonstrated `SELECTED_PLAUSIBLE_DEPTH_LAYER` point set with the
exact candidate-layer count before it can form a range. Current producer
output therefore returns `FRONT_LAYER_BINDING_UNVERIFIED` and PAUSE. This
does not claim that filtering to the largest layer would establish person
ownership; no producer currently emits the accepted point-set marker.

An offline replay of the 122 archived bridge cycles first reproduced the
original 21 `FOLLOW_ELIGIBLE`, 101 PAUSE and all 122 adapter reasons exactly.
With the new guard, the same records give 21
`FRONT_LAYER_BINDING_UNVERIFIED`, 43 earlier RGB/mask/track refusals, 58
target-unavailable refusals, and 122 PAUSE. This verifies the refusal on
recorded inputs, not live Pi behavior. No new camera or motor run is needed
to prove the original point-set mismatch.

## Local candidate-layer diagnostic after the archived replay

The current working candidate adds a second front diagnostic derived from the
**exact numerical sample indices** of the one plausible depth group. The
existing `front_geometry` remains the all-mask-owned-point result and retains
its `ALL_CURRENT_MASK_OWNED_STEREO_POINTS` marker. The new
`candidate_layer_front_geometry` has its own point count, front p05/median,
same-cycle mask/calibration provenance, and the explicit marker
`NUMERIC_CANDIDATE_DEPTH_LAYER_DIAGNOSTIC_ONLY`. It is absent when there is no
unique plausible group. The bridge journal records both point sets, their
front summaries, the candidate MAD and the supported depth groups.

The public depth report is unchanged for existing callers. The adapter does
not consume the new candidate front as a range, so the current producer still
leads to `FRONT_LAYER_BINDING_UNVERIFIED` and PAUSE. Selecting the numerical
depth group does **not** establish physical person ownership or remove nearer
obstacle evidence. The 2026-11-12 archived JSON files lack the source arrays,
so this implementation cannot be retroactively applied to those frames.

Local synthetic two-layer checks and the affected preview, ownership, front,
adapter, bridge and depth tests passed (41 tests, 15 subtests). No Pi latency,
camera behavior or physical distance was measured for this change. On the next
camera-only Pi run, retain same-cycle source RGB and stereo/disparity arrays
for selected disagreement/refusal cycles and compare both front values with
the actual image. Do not repeat the old run to rediscover its proven mismatch.

## Opt-in source snapshots for the next camera-only run

`SHADOW_CAPTURE_SNAPSHOTS=1` on the Fast Shadow launcher enables bounded
source snapshots. The worker returns the exact selected instance mask only
for a front-point-set p05 difference of at least 0.15 m, the first coherent
candidate after track loss, or every tenth selected cycle. The node queues a
bundle **after** publishing its primary diagnostic: original AR0234 and
OV9281 JPEG packets, current half-resolution disparity and LR-consistency,
the exact calibration arrays used for projection,
the full-resolution instance mask, cycle metadata and both driver timestamps.
The background writer keeps at most one pending bundle and accepts at most
24 per session. A corresponding JSON manifest records source hashes; the
post-run summarizer checks the bundle against the Bridge journal and reports
requested cycles with missing files. An absent or failed bundle is not
evidence of physical ownership.

This capture mode is **opt-in diagnostic instrumentation**. Packing the mask
into the worker response and copying arrays into a background queue may add
time to selected cycles; the next Pi run must report age with capture enabled
separately from the earlier ~250 ms median without snapshots. Source JPEG
decoding is checked against the segmentation worker's RGB payload SHA, but
the physical surface still needs image review. The launcher remains motor-off;
the adapter still refuses the all-point front as a control range. No Pi run
had not been performed at the time of implementation.

The first opt-in Pi run has since been completed. Its nine verified source
bundles, 296 camera cycles, decision ages and front-point comparison are in
[the 19:34:49 run report](shadow_fast_source_snapshot_run_20261112.md).
