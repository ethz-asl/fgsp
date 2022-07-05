#! /usr/bin/env python3

import numpy as np
import time
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from src.fgsp.common.logger import Logger
from src.fgsp.common.comms import Comms


class CommandPost(object):
    def __init__(self, config):
        self.config = config
        self.comms = Comms()

        self.degenerate_path_msg = None
        self.degenerate_indices = []
        self.previous_relatives = {}
        self.small_constraint_counter = 0
        self.mid_constraint_counter = 0
        self.large_constraint_counter = 0
        self.anchor_constraint_counter = 0
        self.history = None

        Logger.LogInfo("CommandPost: Initialized command post center.")

    def reset_msgs(self):
        self.degenerate_path_msg = Path()
        self.small_constraint_counter = 0
        self.mid_constraint_counter = 0
        self.large_constraint_counter = 0
        self.anchor_constraint_counter = 0

    def evaluate_labels_per_node(self, labels):
        # Should always publish for all states as we don't know
        # whether they reached the clients.
        n_nodes = labels.size()
        print(f'WE HAVE {n_nodes} NODES')
        for i in range(0, n_nodes):
            history = None
            if i in self.previous_relatives.keys():
                labels.labels[i] = list(
                    set(labels.labels[i]+self.previous_relatives[i]))
                if i in self.history.keys():
                    history = self.history[i]

            relative_constraint, small_relative_counter, mid_relative_counter, large_relative_counter = labels.check_and_construct_constraint_at(
                i, history)
            if relative_constraint is None:
                continue  # no-op
            self.previous_relatives[i] = labels.labels[i]
            self.comms.publish(relative_constraint, Path,
                               self.config.relative_node_topic)
            self.add_to_constraint_counter(
                small_relative_counter, mid_relative_counter, large_relative_counter)
            self.history = labels.history
            time.sleep(0.001)

    def add_to_constraint_counter(self, n_small_constraints, n_mid_constraints, n_large_constraints):
        self.small_constraint_counter = self.small_constraint_counter + n_small_constraints
        self.mid_constraint_counter = self.mid_constraint_counter + n_mid_constraints
        self.large_constraint_counter = self.large_constraint_counter + n_large_constraints

    def get_total_amount_of_constraints(self):
        return self.small_constraint_counter + self.mid_constraint_counter + self.large_constraint_counter

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

    def send_anchors(self, all_opt_nodes, begin_send, end_send):
        Logger.LogError(
            f'CommandPost: Sending degenerate anchors for {end_send - begin_send} nodes.')
        indices = np.arange(begin_send, end_send, 1)
        self.send_anchors_based_on_indices(all_opt_nodes, indices)

    def send_anchors_based_on_indices(self, opt_nodes, indices):
        n_constraints = len(indices)
        Logger.LogError(
            f'CommandPost: Sending anchors for {n_constraints} nodes.')
        for i in indices:
            pose_msg = self.create_pose_msg_from_node(opt_nodes[i])
            self.degenerate_path_msg.poses.append(pose_msg)

            # Update the degenerate anchor indices
            if not i in self.degenerate_indices:
                self.degenerate_indices.append(i)

        # Publish the anchor nodes.
        self.degenerate_path_msg.header.stamp = self.comms.time_now()
        self.comms.publish(self.degenerate_path_msg, Path,
                           self.config.anchor_node_topic)
        self.anchor_constraint_counter = self.anchor_constraint_counter + n_constraints

    def update_degenerate_anchors(self, all_opt_nodes):
        if len(self.degenerate_indices) == 0:
            return
        Logger.LogError(
            f'CommandPost: Sending degenerate anchor update for {self.degenerate_indices}')
        self.send_degenerate_anchors_based_on_indices(
            all_opt_nodes, self.degenerate_indices)
