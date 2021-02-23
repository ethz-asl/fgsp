#! /usr/bin/env python3

import rospy
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommandPost(object):
    def __init__(self):
        rospy.loginfo("[CommandPost] Initializing command post center.")

    def publish_update_messages(self, predictions):
        # Should always publish for all states as we don't know
        # whether they reached the clients.
        rospy.loginfo("[CommandPost] Publishing evaluation results.")
