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
        BaseGraph.__init__(self, config)
        self.G = [None]
        self.adj = [None]
        self.coords = [None]
        self.idx = 0
        self.node_threshold = 200
        rospy.loginfo("[HierarchicalGraph] Initialized with a threshold of {th}".format(th=self.node_threshold))

    def build(self, graph_msg):
        pass

    def build_graph(self, graph_msg):
        if len(self.adj[self.idx].tolist()) == 0:
            rospy.loginfo("[HierarchicalGraph] Path adjacency matrix is empty. Aborting graph building.")
            return False
        self.G[self.idx] = graphs.Graph(self.adj[self.idx])

        n_nodes = self.G[self.idx].N

        if n_nodes != self.coords[self.idx].shape[0]:
            rospy.logerr("[HierarchicalGraph] Path graph size is {coords} but coords are {coords}".format(n=n_nodes, coords=self.coords[self.idx].shape))
            return False
        if n_nodes <= 1:
            rospy.logdebug("[HierarchicalGraph] Path graph vertex count is less than 2.")
            return False

        self.G[self.idx].set_coordinates(self.coords[self.idx][:,[0,1]])
        self.G[self.idx].compute_fourier_basis()

        return self.build_hierarchy()

    def build_from_poses(self, poses):
        self.idx = 0
        self.coords[self.idx] = poses
        self.adj[self.idx] = self.create_adjacency_from_poses(self.coords[self.idx])
        self.build_graph()

    def build_hierarchy(self):
        current_n = self.G[self.idx].N
        if current_n < self.node_threshold:
            return True
        self.coords.append(None)
        self.adj.append(None)
        self.G.append(None)

        indices = self.reduce_every_other(self.coords[self.idx])
        G_next = reduction.kron_reduction(self.G[self.idx], indices)

        self.idx = self.idx + 1
        self.G[self.idx] = G_next
        self.adj[self.idx] = G_next.W.toarray()
        self.coords[self.idx] = self.coords[indices]

        return True

    def get_graph(self):
        return self.G[self.idx]

    def write_graph_to_disk(self, coords_file, adj_file):
        np.save(coords_file, self.coords[0])
        np.save(adj_file, self.adj[0])

if __name__ == '__main__':
    rospy.init_node('graph_test', log_level=rospy.DEBUG)

    g = HierarchicalGraph(None)
    g.build(None)
    print(g.is_built)
