#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from graph_visualizer import GraphVisualizer


class GraphMonitor(object):

    def __init__(self):
        rospy.loginfo("[GraphMonitor] Initializing monitor node...")

        graph_topic = rospy.get_param("~graph_topic")
        traj_topic = rospy.get_param("~traj_topic")

        rospy.Subscriber(graph_topic, Graph, self.graph_callback)
        rospy.Subscriber(graph_topic, Trajectory, self.traj_callback)
        rospy.loginfo("[GraphMonitor] Listening for graphs from " + graph_topic)
        rospy.loginfo("[GraphMonitor] Listening for trajectory from " + traj_topic)

        self.graph = GlobalGraph(reduced=True)
        self.signal = SignalHandler()
        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")


    def graph_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received graph message.")
        self.graph.build(msg)
        #GraphVisualizer.visualize_adjacency(self.graph)
        #GraphVisualizer.visualize_graph(self.graph)



    def traj_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received trajectory message.")
        key = self.signal.convert_signal(msg)




if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    rospy.spin()
