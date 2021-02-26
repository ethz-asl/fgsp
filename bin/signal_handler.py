#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction
from maplab_msgs.msg import Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped

from signal_node import SignalNode


class SignalHandler(object):
    def __init__(self):
        self.signals = {}

    def convert_signal(self, signal_msg):
        n_nodes = len(signal_msg.nodes)
        if (n_nodes <= 0):
            return ""

        signals = [SignalNode] * n_nodes
        signals[0] = self.convert_trajectory_node(signal_msg.nodes[0])
        key = signals[0].robot_name


        for i in range(1, n_nodes):
            signals[i] = self.convert_trajectory_node(signal_msg.nodes[i])

        self.signals[key] = signals

        return key

    def get_all_nodes(self, key):
        return self.signals[key]

    def get_number_of_submaps(self, key):
        return self.signals[key][-1].id + 1

    def get_nodes_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        filtered_nodes = []
        for node in nodes:
            if node.id == id:
                filtered_nodes.append(node)
        return filtered_nodes

    def get_mask_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)
        mask = [False] * n_nodes
        for i in range(0, n_nodes):
            if nodes[i].id == id:
                mask[i] = True
        return mask

    def get_indices_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)
        indices = []
        for i in range(0, n_nodes):
            if nodes[i].id == id:
                indices.append(i)
        return indices

    def convert_trajectory_node(self, node_msg):
        id = node_msg.id;
        robot_name = node_msg.robot_name
        pose_msg = node_msg.pose
        ts = pose_msg.header.stamp
        position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        orientation = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        residual = node_msg.signal

        signal = SignalNode()
        signal.init(ts, id, robot_name, position, orientation, residual)
        return signal

    def compute_signal_from_key(self, key):
        nodes = self.signals[key]
        traj = self.compute_trajectory(nodes)
        traj_origin = traj[0,:]

        pos_signal = (traj - traj_origin).squeeze()

        x = np.linalg.norm(pos_signal, ord=2, axis=1)
        return x

    def compute_signal(self, nodes):
        traj = self.compute_trajectory(nodes)
        traj_origin = traj[0,:]

        pos_signal = (traj - traj_origin).squeeze()

        x = np.linalg.norm(pos_signal, ord=2, axis=1)
        return x


    def compute_trajectory(self, nodes):
        n_nodes = len(nodes)
        trajectory = np.zeros((n_nodes, 3))
        for i in range(n_nodes):
            trajectory[i,0:3] = nodes[i].position

        return trajectory

    def to_signal_msg(self, key):
        traj_msg = Trajectory()
        node_msg = TrajectoryNode()
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)

        for i in range(n_nodes):
            node_msg = TrajectoryNode()
            node_msg.id = nodes[i].id;
            node_msg.robot_name = nodes[i].robot_name

            pose_msg = PoseStamped()
            pose_msg.header.stamp = nodes[i].ts
            pose_msg.pose.position.x = nodes[i].position[0]
            pose_msg.pose.position.y = nodes[i].position[1]
            pose_msg.pose.position.z = nodes[i].position[2]
            pose_msg.pose.orientation.w = nodes[i].orientation[0]
            pose_msg.pose.orientation.x = nodes[i].orientation[1]
            pose_msg.pose.orientation.y = nodes[i].orientation[2]
            pose_msg.pose.orientation.z = nodes[i].orientation[3]

            node_msg.pose = pose_msg
            node_msg.signal = nodes[i].residual
            traj_msg.nodes.append(node_msg)

        return traj_msg

if __name__ == '__main__':
    sh = SignalHandler()
    sh.to_signal_msg("foo")
