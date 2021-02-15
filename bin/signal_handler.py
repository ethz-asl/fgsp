#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction

from signal_node import SignalNode


class SignalHandler(object):
    def __init__(self):
        self.processed = 0
        self.signals = {}

    def convert_signal(self, signal_msg):
        n_nodes = len(signal_msg.nodes)
        signals = [SignalNode] * n_nodes
        rospy.loginfo("[SignalHandler] Computing signal for " + str(n_nodes) + " nodes")

        for i in range(0, n_nodes):
            signals[i] = self.convert_trajectory_node(signal_msg.nodes[i])

        key = self.processed
        self.processed = self.processed + 1


        self.signals[key] = signals

        return key


    def convert_trajectory_node(self, node_msg):
        id = node_msg.id;
        robot_name = node_msg.robot_name
        pose_msg = node_msg.pose
        pose = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        residual = node_msg.signal

        signal = SignalNode()
        signal.init(id, robot_name, pose, residual)
        return signal

    def compute_signal(self, key):
        nodes = self.signals[key]
        traj = self.compute_trajectory(nodes)
        return traj


    def compute_trajectory(self, nodes):
        n_nodes = len(nodes)
        trajectory = np.zeros((n_nodes, 3))
        for i in range(n_nodes):
            trajectory[i,0:3] = nodes[i].pose

        return trajectory
