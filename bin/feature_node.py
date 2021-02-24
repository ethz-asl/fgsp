#! /usr/bin/env python3

import rospy
import numpy as np
from pygsp import graphs, filters, reduction

class FeatureNode(object):
    def __init__(self, opt_nodes, features, submap_ids):
        self.features = features
        self.submap_ids = submap_ids
        self.nodes = self.retrieve_nodes_in_submap(opt_nodes, submap_ids)
        self.label = None

    def retrieve_nodes_in_submap(self, opt_nodes, submap_ids):
        return [opt_nodes[id] for id in submap_ids]
