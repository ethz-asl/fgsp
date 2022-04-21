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

from fgsp.common.visualizer import Visualizer
from fgsp.common.utils import Utils

class BaseGraph(object):
    def __init__(self, config):
        self.config = config
        self.is_built = False
        self.graph_seq = -1
        self.latest_graph_msg = None
        rospy.loginfo("[HierarchicalGraph] Initialized.")

    def build(self, graph_msg):
        rospy.logfatal("Called method in HierarchicalGraph")

    def get_graph(self):
        rospy.logfatal("Called method in HierarchicalGraph")

    def build_from_poses(self, poses):
        rospy.logfatal("Called method in HierarchicalGraph")

    def write_graph_to_disk(self, coords_file, adj_file):
        rospy.logfatal("Called method in HierarchicalGraph")
