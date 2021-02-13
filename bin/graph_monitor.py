#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header


class GraphMonitor(object):

    def __init__(self):
        rospy.loginfo("[GraphMonitor] Initializing...")

        graph_topic = rospy.get_param("~graph_topic")
        rospy.Subscriber(graph_topic, Header, self.graph_callback)

        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")


    def graph_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received graph message.")



if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    rospy.spin()
