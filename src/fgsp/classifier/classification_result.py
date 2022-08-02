#! /usr/bin/env python3

from more_itertools import partition
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from scipy import spatial

from src.fgsp.common.utils import Utils
from src.fgsp.common.logger import Logger
from src.fgsp.common.transform_history import TransformHistory, ConstraintType


class ClassificationResult(object):
    def __init__(self, config, robot_name, opt_nodes, features, labels):
        self.config = config
        self.robot_name = robot_name
        self.opt_nodes = opt_nodes
        self.n_nodes = len(opt_nodes)
        self.features = features
        self.labels = self.check_and_fix_labels(labels)
        self.history = {}
        self.partitions = self.partition_nodes(
            self.config.large_scale_partition_method)
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
        if method == 'id':
            return self.take_nodes_by_id()
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
        n_steps = 15
        prev = 0
        partitioned_nodes = []
        partitions = np.arange(prev, self.n_nodes, n_steps)
        for cur in partitions:
            pivot = (cur + prev) // 2
            partitioned_nodes.append(pivot)
            prev = cur
        return np.array(partitioned_nodes)

    def take_nodes_by_id(self):
        partitioned_nodes = []
        prev = 0
        prev_id = 0
        for i in range(0, self.n_nodes):
            cur_id = self.opt_nodes[i].id
            if cur_id != prev_id:
                pivot = (i + prev) // 2
                partitioned_nodes.append(pivot)
                prev_id = cur_id
                prev = i

        if prev_id > 0:
            pivot = (self.n_nodes + prev) // 2
            partitioned_nodes.append(pivot)

        return np.array(partitioned_nodes)

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
        if self.config.large_scale_partition_method == 'nth':
            cur_submap_idx = self.lookup_closest_submap(cur_opt)
        else:
            cur_submap_idx = cur_opt.id
        counter = 0

        submap_partitions = self.lookup_spatially_close_submaps(cur_submap_idx)
        for target_idx in submap_partitions:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b, ConstraintType.LARGE)
                counter = counter + 1

        if idx != 0:
            target_idx = 1
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[target_idx])
            if history.has_different_transform(target_idx, T_a_b):
                pose_msg = self.create_pose_msg(
                    self.opt_nodes[target_idx], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(target_idx, T_a_b, ConstraintType.LARGE)
                counter = counter + 1

        return relative_constraint, history, counter

    def lookup_closest_submap(self, cur_opt):
        ts_diff = np.absolute(self.ts_partitions -
                              Utils.ros_time_msg_to_ns(cur_opt.ts))
        ts_min = np.amin(ts_diff)

        # Retrieve the index in opt_nodes.
        partition_idx = np.where(ts_diff == ts_min)[0][0]
        return self.partitions[partition_idx]

    def lookup_spatially_close_submaps(self, submap_idx):
        submap_positions = [
            self.opt_nodes[i].position for i in self.partitions]
        tree = spatial.KDTree(submap_positions)
        _, nn_indices = self.query_tree(submap_idx, tree)
        return self.partitions[nn_indices]

    def query_tree(self, cur_id, tree, n_neighbors=10, p_norm=2, dist=10):
        cur_position = self.opt_nodes[self.partitions[cur_id]].position
        nn_dists, nn_indices = tree.query(
            cur_position,
            p=p_norm,
            k=n_neighbors,
            distance_upper_bound=dist)

        # Remove self and fix output.
        nn_dists, nn_indices = Utils.fix_nn_output(
            n_neighbors, cur_id, nn_dists, nn_indices)
        mask = nn_dists >= 5
        return nn_dists[mask], nn_indices[mask]

    def construct_mid_area_constraint(self, idx, relative_constraint, history):
        cur_opt = self.opt_nodes[idx]
        counter = 0
        n_hop = 10
        lower = idx - n_hop
        upper = idx + n_hop
        if lower >= 0:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[lower])
            if history.has_different_transform(lower, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[lower], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(lower, T_a_b, ConstraintType.MID)
                counter = counter + 1
        if upper < self.n_nodes:
            T_a_b = self.compute_relative_distance(
                cur_opt, self.opt_nodes[upper])
            if history.has_different_transform(upper, T_a_b):
                pose_msg = self.create_pose_msg(self.opt_nodes[upper], T_a_b)
                relative_constraint.poses.append(pose_msg)
                history.add_record(upper, T_a_b, ConstraintType.MID)
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
                history.add_record(idx - 1, T_a_b, ConstraintType.SMALL)
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
                history.add_record(idx + 1, T_a_b, ConstraintType.SMALL)
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
