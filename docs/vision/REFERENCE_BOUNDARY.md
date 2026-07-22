# Vision Reference Boundary

Vision Core is not a dump of stereo/depth scripts.

The root `dev_ws` scripts and `src/sie/reference` files are laboratory
prototypes. They are useful evidence, but they do not define SIE contracts.

## Allowed Inside Prototypes

- OpenCV-specific calls
- NumPy arrays as primary data
- direct camera paths such as `/dev/video2`
- calibration files loaded by path
- ad hoc dictionaries
- debug visualizations

## Required At The SIE Boundary

Vision Core must publish canonical engineering objects:

- `Observation`
- `Measurement`
- `QualityReport`
- `VisionFrameResult`

Every measurement must include:

- value
- unit
- reference frame
- confidence
- quality
- source observations

Raw depth, disparity, masks, and intermediate arrays may exist only inside the
implementation or debug payloads. They must not become the architectural API.

## Promotion Checklist

Before moving prototype logic into SIE:

- the algorithm is isolated from the contract object
- output is represented as Observation and Measurement
- spatial values declare unit and reference frame
- quality is explicit, not implied
- debug data is optional and traceable
