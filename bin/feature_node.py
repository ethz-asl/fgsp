#! /usr/bin/env python3

import rospy
import numpy as np

class FeatureNode(object):
    def __init__(self, robot_name, opt_nodes, features, labels):
        self.robot_name = robot_name
        self.opt_nodes = opt_nodes
        self.features = features
        self.labels = labels

    def construct_constraint_at(self, idx):
        label = self.labels[idx]
        if label == 0:
            return None
        if label == 1:
            return self.construct_large_area_constraint(idx)
        if label == 2:
            return self.construct_mid_area_constraint(idx)
        if label == 3:
            return self.construct_small_area_constraint(idx)

    def construct_large_area_constraint(self, idx):
        return None
