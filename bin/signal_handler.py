#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction

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
