#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from graph_visualizer import GraphVisualizer
from signal_synchronizer import SignalSynchronizer


class GraphMonitor(object):

    def __init__(self):
        rospy.loginfo("[GraphMonitor] Initializing monitor node...")
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))

        graph_topic = rospy.get_param("~graph_topic")
        traj_opt_topic = rospy.get_param("~traj_opt_topic")
        traj_topic = rospy.get_param("~traj_topic")
        opt_key = rospy.get_param("~opt_key")

        rospy.Subscriber(graph_topic, Graph, self.graph_callback)
        rospy.Subscriber(traj_opt_topic, Trajectory, self.traj_opt_callback)
        rospy.Subscriber(traj_topic, Trajectory, self.traj_callback)
        rospy.loginfo("[GraphMonitor] Listening for graphs from " + graph_topic)
        rospy.loginfo("[GraphMonitor] Listening for trajectory from " + traj_topic + " and " + traj_opt_topic)

        self.graph = GlobalGraph(reduced=True)
        self.signal = SignalHandler()
        self.optimized_signal = SignalHandler()
        self.synchronizer = SignalSynchronizer()

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.keys = []

        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")

    def graph_callback(self, msg):
        rospy.loginfo("[GraphMonitor] Received graph message.")
        self.graph.build(msg)
        #GraphVisualizer.visualize_adjacency(self.graph)
        #GraphVisualizer.visualize_graph(self.graph)

    def traj_opt_callback(self, msg):
        key = self.optimized_signal.convert_signal(msg)
        rospy.loginfo(f"[GraphMonitor] Received opt trajectory message from {key}.")

        if self.key_in_optimized_keys(key):
            return
        self.optimized_keys.append(key)

    def traj_callback(self, msg):
        key = self.signal.convert_signal(msg)
        rospy.loginfo(f"[GraphMonitor] Received trajectory message from {key}.")

        if self.key_in_keys(key):
            return
        self.keys.append(key)


    def update(self):
        rospy.loginfo(f"[GraphMonitor] Checking graph updates for {len(self.keys)} keys")
        if self.graph.is_built is False:
            rospy.loginfo("[GraphMonitor] Graph is not built yet.")
            return


        # Iterate over all estimated trajectories.
        for key in self.keys:
            # Check whether we have an optimized version of it.
            if not self.key_in_optimized_keys(key):
                rospy.loginfo(f"[GraphMonitor] Found no optimized version of {key}.")
                continue
            rospy.loginfo(f"[GraphMonitor] Comparing trajectories for {key}.")

            est_nodes = self.signal.get_all_nodes(key)
            opt_nodes = self.optimized_signal.get_all_nodes(key)
            est_nodes = self.synchronizer.syncrhonize(opt_nodes, est_nodes)
            assert(len(est_nodes) == len(opt_nodes))

            x = self.signal.compute_signal(key)
            GraphVisualizer.visualize_signal(self.graph, x)


    def key_in_optimized_keys(self, key):
       return any(key in k for k in self.optimized_keys)

    def key_in_keys(self, key):
       return any(key in k for k in self.keys)

if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
