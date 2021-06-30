#! /usr/bin/env python3

import rospy
import numpy as np

class FeatureNode(object):
    def __init__(self, id, robot_name, opt_nodes, features, node_indices):
        self.id = id
        self.robot_name = robot_name
        self.features = features
        self.node_indices = node_indices
        self.nodes, self.initialized = self._retrieve_nodes_in_submap(opt_nodes, node_indices)
        self.label = None


    def _retrieve_nodes_in_submap(self, opt_nodes, node_indices):
        nodes = []
        initialized = False
        opt_indices = np.arange(0, len(opt_nodes), 1)
        for id in self.node_indices:
            if id in opt_indices:
                nodes.append(opt_nodes[id])
                initialized = True
        return nodes, initialized
