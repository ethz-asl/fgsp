#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction

from signla_node import SignalNode


class SignalHandler(object):
    def __init__(self):
        self.processed = 0
        self.signals = []

    def convert_signal(sefl, signal_msg):
        n_nodes = len(signal_msg.nodes)
        signals = [SignalNode] * n_nodes
        rospy.loginfo("[SignalHandler] Computing signal for " + str(n_nodes) + " nodes")

        for i in range(0, n_nodes):
            self.convert_trajectory_node(signal_msg.nodes[i], signals[i])

        self.signals.append(signals)
        key = self.processed
        self.processed = self.processed + 1


        return key


    def convert_trajectory_node(self, node_msg, signal):
        id = node_msg.id;
        robot_name = node_msg.robot_name
        pose_msg = node_msg.pose
        pose = np.array(pose_msg.pose.point.x, pose_msg.pose.point.y, pose_msg.pose.point.z)
        signal = node_msg.signal

        signal.set_node(id, robot_name, pose, signal)


    def compute_trajectory(self, nodes):
        n_nodes = len(nodes)
        trajectory = np.zeros((n_nodes, 3))
        for i in range(n_nodes):
            trajectory[i,0:2] = nodes[i].pose

        return trajectory
