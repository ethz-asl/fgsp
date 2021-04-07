#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode, SubmapConstraint
from multiprocessing import Lock
import pandas

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from graph_visualizer import GraphVisualizer
from signal_synchronizer import SignalSynchronizer
from wavelet_evaluator import WaveletEvaluator
from command_post import CommandPost
from feature_node import FeatureNode

class GraphClient(object):
    def __init__(self):
        self.is_initialized = False
        self.mutex = Lock()
        self.mutex.acquire()
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))

        rospy.loginfo("[GraphClient] Initializing client node...")
        graph_topic = rospy.get_param("~graph_topic")
        traj_opt_topic = rospy.get_param("~traj_opt_topic")
        traj_topic = rospy.get_param("~traj_topic")
        traj_path_topic = rospy.get_param("~traj_path_topic")
        submap_constraint_topic = rospy.get_param("~submap_constraint_topic")

        rospy.Subscriber(graph_topic, Graph, self.graph_callback)
        rospy.Subscriber(traj_opt_topic, Trajectory, self.traj_opt_callback)
        rospy.Subscriber(traj_topic, Trajectory, self.traj_callback)
        rospy.Subscriber(traj_path_topic, Path, self.traj_path_callback)
        rospy.Subscriber(submap_constraint_topic, SubmapConstraint, self.submap_constraint_callback)
        rospy.loginfo("[GraphClient] Listening for graphs from " + graph_topic)
        rospy.loginfo("[GraphClient] Listening for trajectory from " + traj_topic + " and " + traj_opt_topic)
        rospy.loginfo("[GraphClient] Listening for submap constraints from " + submap_constraint_topic)

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

        self.mutex.release()
        self.is_initialized = True

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
        rospy.loginfo(f"[GraphClient] Received opt trajectory message from {key}.")

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

    def traj_path_callback(self, msg):
        if self.is_initialized is False:
            rospy.loginfo("[GraphClient] Received path message before being initialized.")
            return

        key = self.signal.convert_signal_from_path(msg)
        if not key:
            rospy.logerror("[GraphClient] Unable to convert msg to signal.")
            return

        if self.key_in_keys(key):
            return
        self.keys.append(key)

    def submap_constraint_callback(self, msg):
        if self.is_initialized is False:
            rospy.loginfo("[GraphClient] Received submap constraint message before being initialized.")
            return
        rospy.loginfo("[GraphClient] Received submap constraint message.")

    def update(self):
        self.mutex.acquire()
        if self.graph.is_built is False:
            self.mutex.release()
            return
        # Iterate over all estimated trajectories.
        for key in self.keys:
            # Check whether we have an optimized version of it.
            if not self.key_in_optimized_keys(key):
                rospy.logwarn(f"[GraphClient] Found no optimized version of {key}.")
                continue
            self.compare_stored_signals(key)

        self.mutex.release()

    def compare_stored_signals(self, key):
        # Retrieve the estimated and optimized versions of the trajectory.
        all_est_nodes = self.signal.get_all_nodes(key)
        all_opt_nodes = self.optimized_signal.get_all_nodes(key)
        n_opt_nodes = len(all_opt_nodes)

        # Compute the features and publish the results.
        all_opt_nodes, all_est_nodes = self.reduce_and_synchronize(all_opt_nodes, all_est_nodes)
        all_features = self.compute_all_submap_features(key, all_opt_nodes, all_est_nodes)
        self.evaluate_and_publish_features(all_features)

        return True

    def reduce_and_synchronize(self, all_opt_nodes, all_est_nodes):
        # If the graph is reduced, we need to reduce the optimized nodes too.
        # Synchronize the node lists based on their TS.
        # We always sync to the optimized nodes.
        if self.graph.is_reduced:
            all_opt_nodes = [all_opt_nodes[i] for i in self.graph.reduced_ind]
        (all_opt_nodes, all_est_nodes) = self.synchronizer.synchronize(all_opt_nodes, all_est_nodes)
        assert(len(all_est_nodes) == len(all_opt_nodes))
        return (all_opt_nodes, all_est_nodes)

    def compute_all_submap_features(self, key, all_opt_nodes, all_est_nodes):
        # Compute the signal using the synchronized estimated nodes.
        x_est = self.signal.compute_signal(all_est_nodes)
        x_opt = self.optimized_signal.compute_signal(all_opt_nodes)

        # Compute all the wavelet coefficients.
        # We will filter them later per submap.
        W_est = self.eval.compute_wavelet_coeffs(x_est)
        W_opt = self.eval.compute_wavelet_coeffs(x_opt)

        n_submaps = self.optimized_signal.get_number_of_submaps(key)
        psi = self.eval.get_wavelets()

        all_features = []
        rospy.loginfo(f"[GraphClient] Computing features for {n_submaps} submaps")
        rospy.loginfo(f"[GraphClient] Got {W_est.shape} est wavelets")
        rospy.loginfo(f"[GraphClient] Got {W_opt.shape} opt wavelets")
        for i in range(0, n_submaps):
            # Compute the submap features.
            node_ids = self.optimized_signal.get_indices_for_submap(key, i)
            rospy.loginfo(f"[GraphClient] Checking the submap nr: {i}")
            rospy.loginfo(f"[GraphClient] Got these IDs: {node_ids}")
            features = self.eval.compute_features_for_submap(W_opt, W_est, node_ids)
            if features.empty:
                continue
            all_features.append(FeatureNode(i, key, all_opt_nodes, features, node_ids))
        return all_features

    def evaluate_and_publish_features(self, all_features):
        if len(all_features) == 0:
            return

        self.commander.reset_msgs()
        for submap_features in all_features:
            # Predict the state of all the submaps and publish an update.
            submap_features.label = self.eval.classify_submap(submap_features.features)[0]
            self.commander.accumulate_update_messages(submap_features)
        self.commander.publish_update_messages()

    def key_in_optimized_keys(self, key):
       return any(key in k for k in self.optimized_keys)

    def key_in_keys(self, key):
       return any(key in k for k in self.keys)

if __name__ == '__main__':
    rospy.init_node('graph_client')
    node = GraphClient()
    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
