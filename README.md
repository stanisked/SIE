# SIE — Spatial Intelligence Engine

SIE is a modular Spatial Intelligence system for robotics.

## Philosophy

- Geometry First
- World Model over Images
- AI is Replaceable
- Decision over Detection
- Measured Reality over Predictions

## Current stage

Geometry Engine v0.3

## Goal

Turn stereo vision into reliable spatial measurements for robotics.

## Vision Core v0.2

- confidence map for per-pixel depth confidence
- Confidence Engine v2: valid ratio, disparity noise, temporal stability, local consistency
- ROI-based stable depth measurement
- center ROI depth estimator for stable target distance
- temporal median filtering for depth stability
- spatial median filtering for local noise reduction
- batch pixel-to-3D conversion
- safe geometry guard against NaN/inf 3D points
- ruler benchmark metrics: MAE, RMSE, bias, max absolute error

## Geometry Engine v0.3

- point cloud generation from depth maps
- real object dimensions through 3D box fitting
- object measurement engine
- RANSAC plane fitting
- SVD plane detection
- wall deviation in millimeters
- 3D bounding boxes
- depth and point uncertainty propagation
