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
        rospy.loginfo("[MessageConverter] Initialized")

    def path_callback(self):
        rospy.loginfo("[MessageConverter] Received path")

if __name__ == '__main__':
    rospy.init_node('msg_converter')
    node = MessageConverter()
    rospy.spin()
