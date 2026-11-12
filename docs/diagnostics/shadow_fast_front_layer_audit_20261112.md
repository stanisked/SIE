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
