from __future__ import annotations

import cv2


def create_sgbm_matcher(
    *,
    block_size: int = 7,
    uniqueness_ratio: int = 6,
    min_disparity: int = 0,
    num_disparities: int = 160,
    disp12_max_diff: int = 1,
    speckle_window_size: int = 80,
    speckle_range: int = 4,
    pre_filter_cap: int = 31,
) -> cv2.StereoSGBM:
    """
    Create the classical StereoSGBM matcher used by Vision Core benchmarks.

    All parameters must be recorded by the experiment that uses this matcher.
    """

    if block_size < 1 or block_size % 2 == 0:
        raise ValueError(
            f"block_size must be a positive odd number, got {block_size}"
        )

    if num_disparities <= 0 or num_disparities % 16 != 0:
        raise ValueError(
            "num_disparities must be positive and divisible by 16, "
            f"got {num_disparities}"
        )

    if uniqueness_ratio < 0:
        raise ValueError(
            f"uniqueness_ratio must be >= 0, got {uniqueness_ratio}"
        )

    return cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * block_size * block_size,
        P2=32 * block_size * block_size,
        disp12MaxDiff=disp12_max_diff,
        uniquenessRatio=uniqueness_ratio,
        speckleWindowSize=speckle_window_size,
        speckleRange=speckle_range,
        preFilterCap=pre_filter_cap,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
