#! /usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from utils import Utils

class ClassificationResult(object):
    def __init__(self, robot_name, opt_nodes, features, labels):
        self.robot_name = robot_name
        self.opt_nodes = opt_nodes
        self.n_nodes = len(opt_nodes)
        self.features = features
        self.labels = labels

    def size(self):
        return len(self.opt_nodes)

    def check_and_construct_constraint_at(self, idx):
        label = self.labels[idx]
        if label == 0:
            return None
        if label == 1:
            return self.construct_small_area_constraint(idx)
        if label == 2:
            return self.construct_mid_area_constraint(idx)
        if label == 3:
            return self.construct_large_area_constraint(idx)
        rospy.logerror(f'[ClassificationResult] Unknown label ({label})')
        return None

    def construct_large_area_constraint(self, idx):
        cur_opt = self.opt_nodes[idx]
        relative_constraint = Path()
        relative_constraint.header.stamp = cur_opt.ts
        # if idx - 13 >= 0:
        #     pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 13])
        #     relative_constraint.poses.append(pose_msg)
        if idx - 14 >= 0:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 14])
            relative_constraint.poses.append(pose_msg)
        if idx - 15 >= 0:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 15])
            relative_constraint.poses.append(pose_msg)

        # if idx + 13 < self.n_nodes:
        #     pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 13])
        #     relative_constraint.poses.append(pose_msg)
        if idx + 14 < self.n_nodes:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 14])
            relative_constraint.poses.append(pose_msg)
        if idx + 15 < self.n_nodes:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 15])
            relative_constraint.poses.append(pose_msg)
        return relative_constraint

    def construct_mid_area_constraint(self, idx):
        cur_opt = self.opt_nodes[idx]
        relative_constraint = Path()
        relative_constraint.header.stamp = cur_opt.ts
        if idx - 5 >= 0:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 5])
            relative_constraint.poses.append(pose_msg)
        if idx + 5 < self.n_nodes:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 5])
            relative_constraint.poses.append(pose_msg)
        return relative_constraint

    def construct_small_area_constraint(self, idx):
        if self.n_nodes <= 1:
            return None
        cur_opt = self.opt_nodes[idx]
        relative_constraint = Path()
        relative_constraint.header.stamp = cur_opt.ts
        if idx - 1 >= 0:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 1])
            relative_constraint.poses.append(pose_msg)
        if idx + 1 < self.n_nodes:
            pose_msg = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 1])
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

    def compute_relative_distance(self, opt_node_from, opt_node_to):
        T_G_B_a = self.create_transformation_from_node(opt_node_from)
        T_G_B_b = self.create_transformation_from_node(opt_node_to)
        T_a_b = np.matmul(np.linalg.inv(T_G_B_a), T_G_B_b)
        t_a_b, q_a_b = self.convert_transform(T_a_b)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = opt_node_to.ts
        pose_msg.pose.position.x = t_a_b[0]
        pose_msg.pose.position.y = t_a_b[1]
        pose_msg.pose.position.z = t_a_b[2]
        pose_msg.pose.orientation.x = q_a_b[0]
        pose_msg.pose.orientation.y = q_a_b[1]
        pose_msg.pose.orientation.z = q_a_b[2]
        pose_msg.pose.orientation.w = q_a_b[3]
        return pose_msg

    def convert_transform(self, T_a_b):
        pos =  T_a_b[0:3, 3]
        R = T_a_b[0:3,0:3]
        return pos, Rotation.from_matrix(R).as_quat() # x, y, z, w

    def create_transformation_from_node(self, node):
        pose_msg = self.create_pose_msg_from_node(node)
        pos, orien = Utils.convert_pose_stamped_msg_to_array(pose_msg)
        return Utils.convert_pos_quat_to_transformation(pos, orien)
