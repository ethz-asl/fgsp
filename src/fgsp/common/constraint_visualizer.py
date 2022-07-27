#! /usr/bin/env python3

import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from src.fgsp.common.comms import Comms
from src.fgsp.common.visualizer import Visualizer


class ConstraintVisualizer(Visualizer):
    def __init__(self, config=None):
        super().__init__(config)
        self.line_marker = self.create_line_marker()

    def create_small_scale_constraint(self, point_a, point_b):
        pass

    def create_mid_scale_constraint(self, point_a, point_b):
        pass

    def create_large_scale_constraint(self, point_a, point_b):
        pass
