#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from maplab_msgs import Graph, Trajectory, TrajectoryNode


class GraphMonitor(object):

    def __init__(self):
        rospy.loginfo("[GraphMonitor] Initializing...")

        graph_topic = rospy.get_param("~graph_topic")
        traj_topic = rospy.get_param("~traj_topic")

        rospy.Subscriber(graph_topic, Header, self.graph_callback)
        rospy.Subscriber(graph_topic, Header, self.traj_callback)

        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")


    def graph_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received graph message.")

    def traj_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received trajectory message.")



if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    rospy.spin()
