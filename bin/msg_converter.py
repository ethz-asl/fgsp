#! /usr/bin/env python3

import rospy
import pandas
import numpy as np

from maplab_msgs.msg import Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MessageConverter(object):
    def __init__(self):
        conv_topic = rospy.get_param("~conv_topic")
        rospy.Subscriber(conv_topic, Path, self.path_callback)
        rospy.loginfo("[MessageConverter] Listening for paths from " + conv_topic)

        traj_topic = rospy.get_param("~traj_topic")
        self.pub = rospy.Publisher(traj_topic, Trajectory, queue_size=10)

        self.robot_name = rospy.get_param("~conv_robot")

        rospy.loginfo("[MessageConverter] Initialized")

    def path_callback(self, msg):
        # Convert the nav_msgs::Path to a maplab_msgs::Trajectory.
        traj_msg = self.create_trajectory_message(msg)

        # Republish the converted message.
        self.pub.publish(traj_msg)

    def create_trajectory_message(self, msg):
        traj_msg = Trajectory()

        n_nodes = len(msg.poses)
        ts = 0
        for i in range(0, n_nodes):
            node_msg = TrajectoryNode()
            node_msg.robot_name = self.robot_name
            node_msg.pose = msg.poses[i]

            traj_msg.nodes.append(node_msg)

        traj_msg.header.stamp = msg.header.stamp
        return traj_msg


if __name__ == '__main__':
    rospy.init_node('msg_converter')
    node = MessageConverter()
    rospy.spin()
