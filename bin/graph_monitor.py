#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode
from multiprocessing import Lock
import pandas

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from graph_visualizer import GraphVisualizer
from signal_synchronizer import SignalSynchronizer
from wavelet_evaluator import WaveletEvaluator
from command_post import CommandPost

class GraphMonitor(object):

    def __init__(self):
        self.is_initialized = False
        self.mutex = Lock()
        self.mutex.acquire()
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

        # Handlers and evaluators.
        self.graph = GlobalGraph(reduced=False)
        self.signal = SignalHandler()
        self.optimized_signal = SignalHandler()
        self.synchronizer = SignalSynchronizer()
        self.eval = WaveletEvaluator()
        self.commander = CommandPost()

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.keys = []

        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")
        self.is_initialized = True
        self.mutex.release()

    def graph_callback(self, msg):
        if self.is_initialized is False:
            return
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.graph.msg_contains_updates(msg) is True:
            self.graph.build(msg)
            self.eval.compute_wavelets(self.graph.G)

        self.mutex.release()

    def traj_opt_callback(self, msg):
        if self.is_initialized is False:
            return

        key = self.optimized_signal.convert_signal(msg)
        rospy.loginfo(f"[GraphMonitor] Received opt trajectory message from {key}.")

        if self.key_in_optimized_keys(key):
            return
        self.optimized_keys.append(key)

    def traj_callback(self, msg):
        if self.is_initialized is False:
            return

        key = self.signal.convert_signal(msg)

        if self.key_in_keys(key):
            return
        self.keys.append(key)


    def update(self):
        self.mutex.acquire()
        if self.graph.is_built is False:
            self.mutex.release()
            return


        # Iterate over all estimated trajectories.
        for key in self.keys:
            # Check whether we have an optimized version of it.
            if not self.key_in_optimized_keys(key):
                rospy.logwarn(f"[GraphMonitor] Found no optimized version of {key}.")
                continue
            rospy.loginfo(f"[GraphMonitor] Comparing trajectories for {key}.")

            all_est_nodes = self.signal.get_all_nodes(key)
            all_opt_nodes = self.optimized_signal.get_all_nodes(key)
            n_submaps = self.optimized_signal.get_number_of_submaps(key)
            psi = self.eval.get_wavelets()


            all_features = pandas.DataFrame({})
            for i in range(0, n_submaps):
                submap_mask = self.optimized_signal.get_mask_for_submap(key, i)
                opt_nodes = [all_opt_nodes[i] for i in range(0,len(all_opt_nodes)) if submap_mask[i]]
                submap_psi = psi[submap_mask, submap_mask, :]

                print(f"size of opt_nodes {len(opt_nodes)} size of psi {submap_psi.shape} ")

                # If the graph is reduced, we need to reduce the optimized nodes too.
                if self.graph.is_reduced:
                    # TODO(lbern): this is bugged with the new submapping logic.
                    opt_nodes = [opt_nodes[i] for i in self.graph.reduced_ind]
                est_nodes = self.synchronizer.syncrhonize(opt_nodes, all_est_nodes)
                n_nodes = len(opt_nodes)
                assert(len(est_nodes) == n_nodes)
                print(f"size of est_nodes {len(opt_nodes)}")

                # Compute the signal using the synchronized estimated nodes.
                x_est = self.signal.compute_signal(est_nodes)
                x_opt = self.signal.compute_signal(opt_nodes)

                # Compute the coeffs and the features.
                W_est = self.eval.compute_wavelet_coeffs_using_wavelet(submap_psi, x_est)
                W_opt = self.eval.compute_wavelet_coeffs_using_wavelet(submap_psi, x_opt)
                features = self.eval.compute_features(W_opt, W_est)
                all_features = all_features.append(features)

            if all_features.empty:
                continue
            # Predict the state of all the submaps.
            labels = self.eval.classify_submap(all_features)

            # Publish the update message.
            all_opt_nodes = self.optimized_signal.get_all_nodes(key)
            self.commander.publish_update_messages(all_est_nodes, all_opt_nodes, labels)

        self.mutex.release()


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
