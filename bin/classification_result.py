#! /usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ClassificationResult(object):
    def __init__(self, robot_name, opt_nodes, features, labels):
        self.robot_name = robot_name
        self.opt_nodes = opt_nodes
        self.features = features
        self.labels = labels

    def size(self):
        return len(self.opt_nodes)

    def check_and_construct_constraint_at(self, idx):
        label = self.labels[idx]
        if label == 0:
            return None
        if label == 1:
            return self.construct_large_area_constraint(idx)
        if label == 2:
            return self.construct_mid_area_constraint(idx)
        if label == 3:
            return self.construct_small_area_constraint(idx)
        rospy.logerror(f'[ClassificationResult] Unknown label ({label})')
        return None

    def construct_large_area_constraint(self, idx):
        relative_constraint = self.construct_mid_area_constraint(idx)
        if idx - 3 >= 0:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx - 3])
            relative_constraint.poses.append(pose_msg)
        if idx - 4 >= 0:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx - 4])
            relative_constraint.poses.append(pose_msg)
        if idx + 3 < n_nodes:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx + 3])
            relative_constraint.poses.append(pose_msg)
        if idx + 4 < n_nodes:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx + 4])
            relative_constraint.poses.append(pose_msg)
        return relative_constraint

    def construct_mid_area_constraint(self, idx):
        relative_constraint = self.construct_small_area_constraint(idx)
        if idx - 2 >= 0:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx - 2])
            relative_constraint.poses.append(pose_msg)
        if idx + 2 < n_nodes:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx + 2])
            relative_constraint.poses.append(pose_msg)
        return relative_constraint

    def construct_small_area_constraint(self, idx):
        n_nodes = len(self.opt_nodes)
        if n_nodes <= 1:
            return None
        cur_opt = self.opt_nodes[idx]
        relative_constraint = Path()
        relative_constraint.header.stamp = cur_opt.ts
        if idx - 1 >= 0:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx - 1])
            relative_constraint.poses.append(pose_msg)
        if idx + 1 < n_nodes:
            pose_msg = self.create_pose_msg_from_node(self.opt_nodes[idx + 1])
            relative_constraint.poses.append(pose_msg)
        return relative_constraint

    def create_pose_msg_from_node(self, cur_opt):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = cur_opt.ts
        pose_msg.pose.position.x = cur_opt.position[0]
        pose_msg.pose.position.y = cur_opt.position[1]
        pose_msg.pose.position.z = cur_opt.position[2]
        pose_msg.pose.orientation.w = cur_opt.orientation[0]
        pose_msg.pose.orientation.x = cur_opt.orientation[1]
        pose_msg.pose.orientation.y = cur_opt.orientation[2]
        pose_msg.pose.orientation.z = cur_opt.orientation[3]
        return pose_msg
