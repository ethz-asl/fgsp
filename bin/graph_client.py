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
from classification_result import ClassificationResult
from constraint_handler import ConstraintHandler
from config import ClientConfig
from plotter import Plotter
from utils import Utils

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

        # Subscriber and publisher
        rospy.Subscriber(self.config.opt_graph_topic, Graph, self.global_graph_callback)
        rospy.Subscriber(self.config.opt_traj_topic, Trajectory, self.traj_opt_callback)
        rospy.Subscriber(self.config.est_traj_topic, Trajectory, self.traj_callback)
        rospy.Subscriber(self.config.est_traj_path_topic, Path, self.traj_path_callback)
        if self.config.enable_submap_constraints:
            rospy.Subscriber(self.config.submap_constraint_topic, SubmapConstraint, self.submap_constraint_callback)
            self.constraint_handler = ConstraintHandler()

        self.intra_constraint_pub = rospy.Publisher(self.config.intra_constraint_topic, Path, queue_size=20)

        if self.config.enable_client_update:
            rospy.Subscriber(self.config.client_update_topic, Graph, self.client_update_callback)
            self.client_update_pub = rospy.Publisher(self.config.client_update_topic, Graph, queue_size=20)

        # Handlers and evaluators.
        self.global_graph = GlobalGraph(reduced=False)
        self.robot_graph = GlobalGraph(reduced=False)
        self.latest_traj_msg = None
        self.signal = SignalHandler()
        self.optimized_signal = SignalHandler()
        self.synchronizer = SignalSynchronizer()
        self.eval = WaveletEvaluator()
        self.robot_eval = WaveletEvaluator()
        self.commander = CommandPost()

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.keys = []

        self.create_data_export_folder()
        self.mutex.release()
        self.is_initialized = True


    def create_data_export_folder(self):
        if not self.config.enable_signal_recording and not self.config.enable_trajectory_recording:
            return
        cur_ts = Utils.ros_time_to_ns(rospy.Time.now())
        export_folder = self.config.dataroot + '/data/' + self.config.robot_name + '_%d'%np.float32(cur_ts)
        rospy.logwarn(f'[GraphClient] Setting up dataroot folder to {export_folder}')
        if not os.path.exists(export_folder):
            os.mkdir(export_folder)
            os.mkdir(export_folder + '/data')
        self.config.dataroot = export_folder

    def global_graph_callback(self, msg):
        if not (self.is_initialized and (self.config.enable_anchor_constraints or self.config.enable_relative_constraints)):
            return
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.global_graph.msg_contains_updates(msg) is True:
            self.global_graph.build(msg)
            self.eval.compute_wavelets(self.global_graph.G)

        self.mutex.release()

    def client_update_callback(self, graph_msg):
        if not (self.is_initialized and (self.config.enable_anchor_constraints or self.config.enable_relative_constraints)):
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
        if not (self.is_initialized and (self.config.enable_anchor_constraints or self.config.enable_relative_constraints)):
            return

        keys = self.optimized_signal.convert_signal(msg)
        rospy.loginfo(f"[GraphClient] Received opt trajectory message from {keys}.")

        for key in keys:
            if self.key_in_optimized_keys(key):
                continue
            self.optimized_keys.append(key)

    def traj_callback(self, msg):
        if self.is_initialized is False:
            return

        key = self.signal.convert_signal(msg)

        if self.key_in_keys(key):
            return
        self.keys.append(key)

    def traj_path_callback(self, msg):
        if not (self.is_initialized and (self.config.enable_anchor_constraints or self.config.enable_relative_constraints)):
            return
        self.latest_traj_msg = msg

    def process_latest_robot_data(self):
        if self.latest_traj_msg == None:
            return False

        key = self.signal.convert_signal_from_path(self.latest_traj_msg, self.config.robot_name)
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
        self.commander.reset_msgs()
        self.check_for_submap_constraints()
        self.update_degenerate_anchors()

        if not self.process_latest_robot_data():
            rospy.logwarn('[GraphClient] Unable to process latest robot data.')
            return
        self.compare_estimations()
        self.publish_client_update()

        rospy.loginfo("[GraphClient] Updating completed")

    def record_all_signals(self, x_est, x_opt):
        if not self.config.enable_signal_recording:
            return
        self.record_signal_for_key(x_est, 'est')
        self.record_signal_for_key(x_opt, 'opt')

    def record_raw_est_trajectory(self, traj):
        filename = self.config.dataroot + self.config.trajectory_raw_export_path.format(src='est')
        np.save(filename, traj)

    def record_synchronized_trajectories(self, traj_est, traj_opt):
        if not self.config.enable_trajectory_recording:
            return
        self.record_traj_for_key(traj_est, 'est')
        self.record_traj_for_key(traj_opt, 'opt')

    def record_signal_for_key(self, x, src):
        signal_file = self.config.dataroot + self.config.signal_export_path.format(src=src)
        np.save(signal_file, x)
        graph_coords_file = self.config.dataroot + self.config.graph_coords_export_path.format(src=src)
        graph_adj_file = self.config.dataroot + self.config.graph_adj_export_path.format(src=src)
        if src == 'opt':
            self.global_graph.write_graph_to_disk(graph_coords_file, graph_adj_file)
        elif src == 'est':
            self.robot_graph.write_graph_to_disk(graph_coords_file, graph_adj_file)

    def record_traj_for_key(self, traj, src):
        filename = self.config.dataroot + self.config.trajectory_export_path.format(src=src)
        np.save(filename, traj)

    def compare_estimations(self):
        if not self.config.enable_relative_constraints:
            return
        self.mutex.acquire()
        if self.global_graph.is_built is False:

            self.mutex.release()
            return
        # Check whether we have an optimized version of it.
        if self.key_in_optimized_keys(self.config.robot_name):
            self.compare_stored_signals(self.config.robot_name)
        else:
            rospy.logwarn(f"[GraphClient] Found no optimized version of {key} for comparison.")
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
        if not (self.config.enable_anchor_constraints and self.global_graph.is_built and self.config.enable_client_update):
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
        # This evaluates per node the scale of the difference
        # and creates a relative constraint accordingly.
        self.record_raw_est_trajectory(self.signal.compute_trajectory(all_est_nodes))
        all_opt_nodes, all_est_nodes = self.reduce_and_synchronize(all_opt_nodes, all_est_nodes)
        labels = self.compute_all_features(key, all_opt_nodes, all_est_nodes)
        self.evaluate_and_publish_features(labels)

        # Check if we the robot identified a degeneracy in its state.
        # Publish an anchor node curing the affected areas.
        self.check_for_degeneracy(all_opt_nodes, all_est_nodes)

        return True

    def reduce_and_synchronize(self, all_opt_nodes, all_est_nodes):
        # If the graph is reduced, we need to reduce the optimized nodes too.
        # Synchronize the node lists based on their TS.
        # We always sync to the optimized nodes.
        if self.global_graph.is_reduced:
            all_opt_nodes = [all_opt_nodes[i] for i in self.global_graph.reduced_ind]
        (all_opt_nodes, all_est_nodes, opt_idx, est_idx) = self.synchronizer.synchronize(all_opt_nodes, all_est_nodes)
        assert(len(all_est_nodes) == len(all_opt_nodes))
        assert(len(est_idx) == len(opt_idx))

        # Reduce the robot graph and compute the wavelet basis functions.
        positions = np.array([np.array(x.position) for x in all_est_nodes])
        self.robot_graph.build_from_poses(positions)
        # self.robot_graph.reduce_graph_using_indices(est_idx)
        self.robot_eval.compute_wavelets(self.robot_graph.G)
        return (all_opt_nodes, all_est_nodes)

    def check_for_degeneracy(self, all_opt_nodes, all_est_nodes):
        if not self.config.enable_anchor_constraints:
            return
        rospy.loginfo('[GraphClient] Checking for degeneracy.')
        n_nodes = len(all_opt_nodes)
        assert n_nodes == len(all_est_nodes)
        for i in range(0, n_nodes):
            if not all_est_nodes[i].degenerate:
                continue
            pivot = self.config.degenerate_window // 2
            begin_send = max(i - pivot, 0)
            end_send = min(i + (self.config.degenerate_window - pivot), n_nodes)
            rospy.logerr(f'[GraphClient] Sending degenerate anchros from {begin_send} to {end_send}')
            self.commander.send_degenerate_anchors(all_opt_nodes, begin_send, end_send)

    def update_degenerate_anchors(self):
        all_opt_nodes = self.optimized_signal.get_all_nodes(self.config.robot_name)
        if len(all_opt_nodes) == 0:
            rospy.logerr(f'[GraphClient] Robot {self.config.robot_name} does not have any optimized nodes yet.')
            return
        self.commander.update_degenerate_anchors(all_opt_nodes)

    def compute_all_features(self, key, all_opt_nodes, all_est_nodes):
        if not self.eval.is_available:
            return []

        # Compute the signal using the synchronized estimated nodes.
        x_est = self.signal.compute_signal(all_est_nodes)
        x_opt = self.optimized_signal.compute_signal(all_opt_nodes)
        self.record_all_signals(x_est, x_opt)
        self.record_synchronized_trajectories(self.signal.compute_trajectory(all_est_nodes), self.optimized_signal.compute_trajectory(all_opt_nodes))

        psi = self.eval.get_wavelets()
        robot_psi = self.robot_eval.get_wavelets()
        n_dim = psi.shape[0]
        if n_dim != x_est.shape[0] or n_dim != x_opt.shape[0]:
            return []
        if n_dim != robot_psi.shape[0] or psi.shape[1] != robot_psi.shape[1]:
            rospy.logwarn(f'[GraphClient] Optimized wavelet does not match robot wavelet: {psi.shape} vs. {robot_psi.shape}')
            return []

        # Compute all the wavelet coefficients.
        # We will filter them later per submap.
        W_est = self.robot_eval.compute_wavelet_coeffs(x_est)
        W_opt = self.eval.compute_wavelet_coeffs(x_opt)
        features = self.eval.compute_features(W_opt, W_est)

        labels =  self.eval.classify_simple(features)
        return ClassificationResult(key, all_opt_nodes, features, labels)

    def evaluate_and_publish_features(self, labels):
        if labels.size() == 0:
            return
        self.commander.evaluate_labels_per_node(labels)

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
