#! /usr/bin/env python3

import rospy
import numpy as np

class FeatureNode(object):
    def __init__(self, id, robot_name, opt_nodes, features, node_ids):
        self.id = id
        self.robot_name = robot_name
        self.features = features
        self.node_ids = node_ids
        self.nodes, self.initialized = self._retrieve_nodes_in_submap(opt_nodes, node_ids)
        self.label = None


    def _retrieve_nodes_in_submap(self, opt_nodes, node_ids):
        nodes = []
        initialized = True
        for id in self.node_ids:
            if id in opt_nodes:
                nodes.append(opt_nodes[id])
            else:
                return None, False
        return nodes, True
