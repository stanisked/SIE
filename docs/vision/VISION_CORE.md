# Vision Core

Vision Core turns stereo images into disparity, depth, and confidence signals.

Current responsibilities:

- stereo disparity
- depth computation
- temporal stabilization
- ROI-based depth estimation
- confidence estimation
- safe handling of invalid depth

## SIE Boundary

Vision Core does not publish raw OpenCV or NumPy implementation details as its
main API.

At the SIE boundary it publishes:

- Observation
- Measurement
- QualityReport
- VisionFrameResult

Depth maps and disparity maps are internal evidence or debug data. Engineering
consumers should use measurements with unit, reference frame, confidence, and
quality.

See `REFERENCE_BOUNDARY.md` for the rule that separates laboratory prototypes
from SIE contract code.
