class VisionAPI:
    """
    Public API of Spatial Intelligence Engine (SIE)
    """

    def __init__(
        self,
        stereo_engine=None,
        depth_engine=None,
        geometry_engine=None,
        quality_engine=None,
    ):
        self.stereo_engine = stereo_engine
        self.depth_engine = depth_engine
        self.geometry_engine = geometry_engine
        self.quality_engine = quality_engine

    def get_depth(self, left_frame, right_frame):
        disparity = self.stereo_engine.compute(left_frame, right_frame)
        depth = self.depth_engine.compute(disparity)
        return depth

    def pixel_to_3d(self, u, v, depth_value):
        return self.geometry_engine.pixel_to_3d(u, v, depth_value)

    def measure_distance(self, p1, p2):
        return self.geometry_engine.distance(p1, p2)

    def get_confidence(self, depth_map):
        return self.quality_engine.evaluate(depth_map)
