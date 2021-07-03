#! /usr/bin/env python3
import os
import time

import numpy as np
import pandas
import rospy
from nav_msgs.msg import Path
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode, SubmapConstraint
from multiprocessing import Lock

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from graph_visualizer import GraphVisualizer
from signal_synchronizer import SignalSynchronizer
from wavelet_evaluator import WaveletEvaluator
from command_post import CommandPost
from feature_node import FeatureNode
from constraint_handler import ConstraintHandler
from config import ClientConfig
from plotter import Plotter

class GraphClient(object):
    def __init__(self):
        self.is_initialized = False
        self.config = ClientConfig()
        self.config.init_from_config()
        Plotter.PlotClientBanner()
        Plotter.PrintClientConfig(self.config)
        Plotter.PrintSeparator()

        self.mutex = Lock()
        self.constraint_mutex = Lock()
        self.mutex.acquire()

        # Subscribers.
        rospy.Subscriber(self.config.opt_graph_topic, Graph, self.global_graph_callback)
        rospy.Subscriber(self.config.opt_traj_topic, Trajectory, self.traj_opt_callback)
        rospy.Subscriber(self.config.est_traj_topic, Trajectory, self.traj_callback)
        rospy.Subscriber(self.config.est_traj_path_topic, Path, self.traj_path_callback)
        rospy.Subscriber(self.config.client_update_topic, Graph, self.client_update_callback)
        if self.config.enable_submap_constraints:
            rospy.Subscriber(self.config.submap_constraint_topic, SubmapConstraint, self.submap_constraint_callback)
            self.constraint_handler = ConstraintHandler()

        # Publishers
        self.intra_constraint_pub = rospy.Publisher(self.config.intra_constraint_topic, Path, queue_size=20)
        self.client_update_pub = rospy.Publisher(self.config.client_update_topic, Graph, queue_size=20)

        # Handlers and evaluators.
        self.global_graph = GlobalGraph(reduced=False)
        self.robot_graph = GlobalGraph(reduced=False)
        self.latest_traj_msg = None
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

    def global_graph_callback(self, msg):
        if not (self.is_initialized and self.config.enable_anchor_constraints):
            return
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.global_graph.msg_contains_updates(msg) is True:
            self.global_graph.build(msg)
            self.eval.compute_wavelets(self.global_graph.G)

        self.mutex.release()

    def client_update_callback(self, graph_msg):
        if not (self.is_initialized and self.config.enable_anchor_constraints):
            return
        # Theoretically this does exactly the same as the graph_callback, but
        # lets separate it for now to be a bit more flexible.
        rospy.loginfo(f'[GraphClient] Received client update')
        client_seq = graph_msg.header.seq
        self.mutex.acquire()
        graph_seq = self.global_graph.graph_seq
        if self.global_graph.msg_contains_updates(graph_msg) is True:
            self.global_graph.build(graph_msg)
            self.eval.compute_wavelets(self.global_graph.G)

        self.mutex.release()

    def traj_opt_callback(self, msg):
        if not (self.is_initialized and self.config.enable_anchor_constraints):
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
        if not (self.is_initialized and self.config.enable_anchor_constraints):
            return
        msg.header.frame_id = self.config.robot_name
        self.latest_traj_msg = msg

    def process_latest_robot_data(self):
        if self.latest_traj_msg == None:
            return False
        self.robot_graph.build_from_path(self.latest_traj_msg)
        key = self.signal.convert_signal_from_path(self.latest_traj_msg)
        if not key:
            rospy.logerror("[GraphClient] Unable to convert msg to signal.")
            return False

        if self.key_in_keys(key):
            return True
        self.keys.append(key)
        return True

    def submap_constraint_callback(self, msg):
        if not (self.is_initialized and self.config.enable_submap_constraints):
            rospy.loginfo("[GraphClient] Received submap constraint message before being initialized.")
            return
        rospy.loginfo("[GraphClient] Received submap constraint message.")
        self.constraint_mutex.acquire()
        self.constraint_handler.add_constraints(msg)
        self.constraint_mutex.release()

    def update(self):
        rospy.loginfo("[GraphClient] Updating...")
        self.check_for_submap_constraints()

        if not self.process_latest_robot_data():
            rospy.logwarn('[GrpahClient] Found no robot data to process')
            return
        self.compare_estimations()
        self.publish_client_update()
        rospy.loginfo("[GraphClient] Updating completed")

    def record_all_signals(self, key, x_est, x_opt):
        if not self.config.enable_signal_recording:
            return
        self.record_signal_for_key(key, x_est, 'est')
        self.record_signal_for_key(key, x_opt, 'opt')

    def record_all_trajectories(self, key, traj_est, traj_opt):
        if not self.config.enable_trajectory_recording:
            return
        self.record_traj_for_key(key, traj_est, 'est')
        self.record_traj_for_key(key, traj_opt, 'opt')

    def record_signal_for_key(self, key, x, src):
        signal_file = self.config.dataroot + self.config.signal_export_path.format(key=key, src=src)
        rospy.loginfo(f'Writing signals from {src}.')
        np.save(signal_file, x)
        graph_coords_file = self.config.dataroot + self.config.graph_coords_export_path.format(key=key, src=src)
        graph_adj_file = self.config.dataroot + self.config.graph_adj_export_path.format(key=key, src=src)
        if src == 'opt':
            self.global_graph.write_graph_to_disk(graph_coords_file, graph_adj_file)
        elif src == 'est':
            self.robot_graph.write_graph_to_disk(graph_coords_file, graph_adj_file)


    def record_traj_for_key(self, key, traj, src):
        filename = self.config.dataroot + self.config.trajectory_export_path.format(key=key, src=src)
        rospy.loginfo(f'Writing trajectory from {src}.')
        np.save(filename, traj)

    def compare_estimations(self):
        if not self.config.enable_anchor_constraints:
            return
        self.mutex.acquire()
        if self.global_graph.is_built is False:
            self.mutex.release()
            return
        # Iterate over all estimated trajectories.
        for key in self.keys:
            # Check whether we have an optimized version of it.
            if not self.key_in_optimized_keys(key):
                rospy.logwarn(f"[GraphClient] Found no optimized version of {key} for comparison.")
                continue
            self.compare_stored_signals(key)
        self.mutex.release()

    def check_for_submap_constraints(self):
        if not self.config.enable_submap_constraints:
            return
        self.constraint_mutex.acquire()
        path_msgs = self.constraint_handler.create_msg_for_intra_constraints(self.config.robot_name)
        self.constraint_mutex.release()
        for msg in path_msgs:
            self.intra_constraint_pub.publish(msg)
            time.sleep(0.10)

    def publish_client_update(self):
        if not (self.config.enable_anchor_constraints and self.global_graph.is_built):
            return
        self.mutex.acquire()
        graph_msg = self.global_graph.latest_graph_msg
        if graph_msg is not None:
            self.client_update_pub.publish(graph_msg)
        self.mutex.release()

    def compare_stored_signals(self, key):
        rospy.logwarn(f"[GraphClient] Comparing signals for {key}.")
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
        if self.global_graph.is_reduced:
            all_opt_nodes = [all_opt_nodes[i] for i in self.global_graph.reduced_ind]
        (all_opt_nodes, all_est_nodes) = self.synchronizer.synchronize(all_opt_nodes, all_est_nodes)
        assert(len(all_est_nodes) == len(all_opt_nodes))
        return (all_opt_nodes, all_est_nodes)

    def compute_all_submap_features(self, key, all_opt_nodes, all_est_nodes):
        if not self.eval.is_available:
            return []

        # Compute the signal using the synchronized estimated nodes.
        x_est = self.signal.compute_signal(all_est_nodes)
        x_opt = self.optimized_signal.compute_signal(all_opt_nodes)
        self.record_all_signals(key, x_est, x_opt)
        self.record_all_trajectories(key, self.signal.compute_trajectory(all_est_nodes), self.optimized_signal.compute_trajectory(all_opt_nodes))

        psi = self.eval.get_wavelets()
        n_dim = psi.shape[0]
        if n_dim != x_est.shape[0] or n_dim != x_opt.shape[0]:
            return []

        # Compute all the wavelet coefficients.
        # We will filter them later per submap.
        W_est = self.eval.compute_wavelet_coeffs(x_est)
        W_opt = self.eval.compute_wavelet_coeffs(x_opt)

        n_submaps = self.optimized_signal.get_number_of_submaps(key)

        all_features = []
        rospy.loginfo(f"[GraphClient] Computing features for {n_submaps} submaps")
        for i in range(0, n_submaps):
            # Get all nodes for submap i.
            node_indices = self.optimized_signal.get_indices_for_submap(key, i)
            if len(node_indices) == 0:
                continue
            features = self.eval.compute_features_for_submap(W_opt, W_est, node_indices)
            if features.empty:
                continue
            feature_node = FeatureNode(i, key, all_opt_nodes, features, node_indices)
            if not feature_node.initialized:
                continue

            all_features.append(feature_node)
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
        node.config.rate.sleep()
