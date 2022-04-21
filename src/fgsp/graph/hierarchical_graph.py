#! /usr/bin/env python2
import rospy
import time
import sys

import numpy as np
from liegroups import SE3
from pygsp import graphs, filters, reduction, utils
from geometry_msgs.msg import Point
from maplab_msgs.msg import Graph
from scipy import spatial
from scipy.spatial.transform import Rotation

from fgsp.graph.base_graph import BaseGraph

class HierarchicalGraph(BaseGraph):
    def __init__(self, config):
        self.config = config
        self.adj = None
        self.coords = np.array([])
        self.G = None
        self.is_built = False
        self.reduced_ind = []
        self.skip_ind = []
        self.submap_ind = []
        self.graph_seq = -1
        self.latest_graph_msg = None
        rospy.loginfo("[HierarchicalGraph] Initialized.")

    def build(self, graph_msg):
        pass

    def get_graph(self):
        pass

    def build_from_poses(self, poses):
        pass

    def write_graph_to_disk(self, coords_file, adj_file):
        pass

if __name__ == '__main__':
    rospy.init_node('foo', log_level=rospy.DEBUG)

    g = BaseGraph(None)
    g.build(None)
