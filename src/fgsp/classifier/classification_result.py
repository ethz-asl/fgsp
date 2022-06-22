#! /usr/bin/env python3

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from src.fgsp.common.utils import Utils
from src.fgsp.common.logger import Logger
from src.fgsp.common.transform_history import TransformHistory


class ClassificationResult(object):
    def __init__(self, robot_name, opt_nodes, features, labels):
        self.robot_name = robot_name
        self.opt_nodes = opt_nodes
        self.n_nodes = len(opt_nodes)
        self.features = features
        self.labels = self.check_and_fix_labels(labels)
        self.history = {}
        self.partitions = self.partition_nodes(method='nth')
        self.ts_partitions = self.get_ts_from_nodes(self.partitions)

    def check_and_fix_labels(self, labels):
        n_nodes = len(labels)
        for i in range(0, n_nodes):
            if labels[i] is None:
                labels[i] = []
        return labels

    def partition_nodes(self, method='nth'):
        if method == 'nth':
            return self.take_every_nth_node()
        else:
            Logger.LogWarn(f'Unknown partiion method {method} specified')
            return []

    def get_ts_from_nodes(self, nodes):
        ts = []
        for node_idx in nodes:
            node = self.opt_nodes[node_idx]
            ts.append(Utils.ros_time_msg_to_ns(node.ts))
        return np.array(ts)

    def take_every_nth_node(self):
        n_steps = 10
        prev = 0
        partitioned_nodes = []
        partitions = np.arange(prev, self.n_nodes, n_steps)
        for cur in partitions:
            pivot = (cur + prev) // 2
            partitioned_nodes.append(pivot)
            prev = cur
        return partitioned_nodes

    def size(self):
        return len(self.opt_nodes)

    def check_and_construct_constraint_at(self, idx, transform_history):
        local_labels = self.labels[idx]
        if local_labels is None or len(local_labels) == 0:
            return None, 0, 0, 0

        cur_opt = self.opt_nodes[idx]
        relative_constraint = Path()
        relative_constraint.header.stamp = cur_opt.ts

        small_relative_counter = 0
        mid_relative_counter = 0
        large_relative_counter = 0
        if transform_history == None:
            transform_history = TransformHistory()

        if 1 in local_labels:
            relative_constraint, transform_history, n_added = self.construct_small_area_constraint(
                idx, relative_constraint, transform_history)
            small_relative_counter = n_added
        if 2 in local_labels:
            relative_constraint, transform_history, n_added = self.construct_mid_area_constraint(
                idx, relative_constraint, transform_history)
            mid_relative_counter = n_added
        if 3 in local_labels:
            relative_constraint, transform_history, n_added = self.construct_large_area_constraint(
                idx, relative_constraint, transform_history)
            large_relative_counter = n_added

        if len(relative_constraint.poses) == 0:
            return None, 0, 0, 0

        self.history[idx] = transform_history
        return relative_constraint, small_relative_counter, mid_relative_counter, large_relative_counter

    def construct_large_area_constraint(self, idx, relative_constraint, history):
        cur_opt = self.opt_nodes[idx]
        cur_submap_idx = self.lookup_closest_submap(cur_opt)
        counter = 0

        target_idx = cur_submap_idx - 1
        if target_idx >= 0:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b)
                counter = counter + 1
        target_idx = cur_submap_idx - 2
        if target_idx >= 0:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b)
                counter = counter + 1

        target_idx = cur_submap_idx + 1
        if target_idx < self.n_nodes:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b)
                counter = counter + 1
        target_idx = cur_submap_idx + 2
        if target_idx < self.n_nodes:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b)
                counter = counter + 1

        if idx != 0:
            target_idx = 1
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b)
                counter = counter + 1

        return relative_constraint, history, counter

    def lookup_closest_submap(self, cur_opt):
        ts_diff = np.absolute(self.ts_partitions -
                              Utils.ros_time_msg_to_ns(cur_opt.ts))
        ts_min = np.amin(ts_diff)

        # Retrieve the index in opt_nodes.
        partition_idx = np.where(ts_diff == ts_min)[0][0]
        return self.partitions[partition_idx]

    def construct_mid_area_constraint(self, idx, relative_constraint, history):
        cur_opt = self.opt_nodes[idx]
        counter = 0
        if idx - 5 >= 0:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[idx - 5])
            if history.has_different_transform(idx - 5, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[idx - 5], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(idx - 5, T_a_b)
                counter = counter + 1
        if idx + 5 < self.n_nodes:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[idx + 5])
            if history.has_different_transform(idx + 5, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[idx + 5], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(idx + 5, T_a_b)
                counter = counter + 1
        return relative_constraint, history, counter

    def construct_small_area_constraint(self, idx, relative_constraint, history):
        if self.n_nodes <= 1:
            return relative_constraint, history, 0
        cur_opt = self.opt_nodes[idx]
        counter = 0
        if idx - 1 >= 0:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[idx - 1])
            if history.has_different_transform(idx - 1, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[idx - 1], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(idx - 1, T_a_b)
                counter = counter + 1
        # if idx - 2 >= 0:
        #     T_a_b = self.compute_relative_distance(cur_opt, self.opt_nodes[idx - 2])
        #     if history.has_different_transform(idx - 2, T_a_b):
        #         pose_msg = self.create_pose_msg(self.opt_nodes[idx - 2], T_a_b)
        #         relative_constraint.poses.append(pose_msg)
        #         history.add_record(idx - 2, T_a_b)
        #         counter = counter + 1
        if idx + 1 < self.n_nodes:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[idx + 1])
            if history.has_different_transform(idx + 1, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[idx + 1], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(idx + 1, T_a_b)
                counter = counter + 1
        # if idx + 2 < self.n_nodes:
        #     T_a_b = self.compute_relative_distance(cur_opt, self.opt_nodes[idx + 2])
        #     if history.has_different_transform(idx + 2, T_a_b):
        #         pose_msg = self.create_pose_msg(self.opt_nodes[idx + 2], T_a_b)
        #         relative_constraint.poses.append(pose_msg)
        #         history.add_record(idx + 2, T_a_b)
                # counter = counter + 1
        return relative_constraint, history, counter

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
        return T_a_b

    def create_pose_msg(self, opt_node_to, T_a_b):
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
        pos = T_a_b[0:3, 3]
        R = T_a_b[0:3, 0:3]
        return pos, Rotation.from_matrix(R).as_quat()  # x, y, z, w

    def create_transformation_from_node(self, node):
        pose_msg = self.create_pose_msg_from_node(node)
        pos, orien = Utils.convert_pose_stamped_msg_to_array(pose_msg)
        return Utils.convert_pos_quat_to_transformation(pos, orien)
