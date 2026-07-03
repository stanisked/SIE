import numpy as np


class PointCloudEngine:
    def __init__(self, fx, fy, cx, cy, stride=2):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.stride = stride

    def depth_to_points(self, depth_map):
        depth = np.asarray(depth_map, dtype=np.float64)
        h, w = depth.shape
        points = []

        for v in range(0, h, self.stride):
            for u in range(0, w, self.stride):
                z = depth[v, u]

                if not np.isfinite(z) or z <= 0:
                    continue

                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                points.append((x, y, z))

        return np.asarray(points, dtype=np.float64)
