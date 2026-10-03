# Range-Dependent Stereo Modes — External Reference Note

**Status:** reference only, not an approved SIEE architecture rule  
**Added:** 2026-10-03  
**Topic:** future range-dependent stereo operating modes, especially near-field depth

## Reference

RealSense AI, **"Extending Stereo Depth to 2 cm with Min-Z"**

Source:
https://www.realsenseai.com/news-insights/extending-stereo-depth-to-2-cm-with-min-z/

## Why this reference matters to SIEE

The article is useful because it treats stereo depth as a range-dependent engineering problem rather than assuming one stereo configuration is optimal across the full operating envelope.

The main design lesson for SIEE is not to copy the RealSense Min-Z implementation. The useful hypothesis is that future stereo operation may require **separately validated range modes** with different matcher settings, disparity search ranges, quality gates, uncertainty models, and applicability envelopes.

A possible future structure is:

```text
near-field stereo profile
mid-field stereo profile
far-field stereo profile
```

Each profile would need its own evidence, calibration applicability checks, metrics, and physical validation before it could be used for engineering Measurement or Decision.

## Relevant engineering ideas

### 1. Minimum usable stereo range is not only a calibration question

At close range, disparity becomes large and practical limitations appear:

- disparity search range limits;
- stronger left/right occlusion;
- incomplete common field of view;
- increased correspondence ambiguity;
- edge and foreground/background artifacts;
- greater sensitivity to surface texture and reflectance.

Therefore, a geometrically valid calibration does not by itself guarantee usable near-field depth.

### 2. Range-specific processing may be justified

A configuration optimized for close manipulation may not be optimal for mid- or far-range perception.

For SIEE, a future range mode must never be selected only because an algorithm can produce depth. It must have:

- a declared applicability range;
- explicit quality metrics;
- independent physical validation;
- uncertainty characterization;
- versioned configuration;
- fail-closed behavior outside the validated envelope.

### 3. Surface observability remains independent from geometric accuracy

Improved near-field processing does not remove the stereo requirement for observable correspondence.

Low-texture, reflective, transparent, strongly absorbing, or repetitive surfaces can still produce weak or ambiguous stereo support.

This reinforces the current SIEE distinction between:

```text
geometric accuracy
        ×
target-surface observability
        =
useful stereo depth
```

### 4. Relevance to the future manipulator stage

This reference becomes particularly relevant when SIEE moves from person-range perception to close-range manipulation.

The current active AR0234 + OV9281 work is focused on human-target association and stereo-surface support at substantially longer distances. Min-Z is therefore **not a repair strategy for the present 1.5 m person-surface problem**.

Future near-field work should start with a dedicated physical benchmark rather than reusing mid-range thresholds.

## Non-goals

This reference does **not** currently authorize:

- a new near-field runtime profile;
- changing the active 0.5–4.5 m measurement envelope;
- changing StereoSGBM parameters in the current person tests;
- copying RealSense-specific Min-Z behavior;
- treating vendor claims as SIEE validation evidence;
- bypassing the current 30-second stationary-person vision tests.

## Future validation questions

Before adopting range-dependent stereo modes, SIEE should answer experimentally:

1. At what distance does the current OV9281 pipeline begin to lose reliable common stereo support?
2. Is the limiting factor disparity range, occlusion, optical geometry, surface observability, or matcher configuration?
3. Can a dedicated near-field profile improve valid support without increasing false correspondence?
4. What range transition logic is required between profiles?
5. How should profile identity and applicability be represented in Measurement provenance?
6. Does close-range work require active texture or an additional depth modality?

## Relationship to current priorities

Current priority remains:

1. stable AR0234 semantic target selection;
2. correct AR0234-to-OV9281 projection;
3. trustworthy connected stereo support on the actual human surface;
4. temporal consistency;
5. only later, range-specific stereo optimization.

This document is a future reference and must not distract from the active vision-validation sequence.
