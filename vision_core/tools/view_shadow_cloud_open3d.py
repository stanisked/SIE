#!/usr/bin/env python3
"""Desktop-only viewer for original diagnostic PLY points; no robot access."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cloud", type=Path)
    args = parser.parse_args()
    if not args.cloud.is_file():
        parser.error("PLY cloud does not exist")
    try:
        import open3d as o3d
    except ImportError as error:
        raise SystemExit("Install open3d in an isolated desktop environment; the Pi analysis does not need it.") from error
    cloud = o3d.io.read_point_cloud(str(args.cloud))
    if cloud.is_empty():
        raise SystemExit("PLY cloud is empty")
    print("Original rectified_left_optical_frame points; unit=m; diagnostic only")
    print("Points:", len(cloud.points))
    o3d.visualization.draw_geometries([cloud], window_name="SIEE original stereo diagnostic",
                                     width=1000, height=800)


if __name__ == "__main__":
    main()
