import numpy as np

from sie_core.geometry.bounds import BoundingBox3D


class MeasurementEngine:
    def __init__(self):
        self.bbox = BoundingBox3D()

    def bounding_box_3d(self, points):
        return self.bbox.fit(points)

    def object_size(self, points):
        bbox = self.bounding_box_3d(points)
        if bbox is None:
            return None
        return bbox["size"]

    def diagonal(self, points):
        bbox = self.bounding_box_3d(points)
        if bbox is None:
            return None
        return float(np.linalg.norm(bbox["size"]))
