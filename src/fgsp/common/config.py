#! /usr/bin/env python3

import numpy as np
from src.fgsp.common.logger import Logger


class BaseConfig(object):
    def __init__(self, node):
        self.node = node

    def try_get_param(self, key, default=None):
        Logger.LogDebug('[BaseConfig] try_get_param: {key} with default {default}'.format(
            key=key, default=default))
        self.node.declare_parameter(key, default)
        return self.node.get_parameter(key).value


class MonitorConfig(BaseConfig):
    def __init__(self, node):
        super().__init__(node)

        # general config
        self.rate = 0.1
        self.enable_graph_building = True
        self.min_node_count = 10
        self.submap_min_count = 3
        self.send_separate_traj_msgs = True

        # Reduction settings.
        self.reduce_global_graph = False
        self.reduction_method = 'positive_ev'
        self.reduce_to_n_percent = 1.0

        # submap constraints
        self.pivot_distance = 20.0
        self.min_pivot_distance = 20.0
        self.n_nearest_neighbors = 50
        self.p_norm = 2
        self.compute_poses_in_LiDAR = False

        # input
        self.in_graph_topic = '/maplab_server/sparse_graph/graph'
        self.in_traj_opt_topic = '/maplab_server/sparse_graph/trajectory'
        self.verification_service_topic = '/grap_monitor/verification'
        self.opt_pc_topic = '/maplab_server/sparse_graph/submap'

        # output
        self.out_graph_topic = '/graph_monitor/sparse_graph/graph'
        self.out_traj_opt_topic = '/graph_monitor/sparse_graph/trajectory'
        self.accumulated_map_topic = '/graph_monitor/map'

    def init_from_config(self):
        # general config
        self.rate = self.try_get_param("update_rate", self.rate)
        self.enable_graph_building = self.try_get_param(
            "enable_graph_building", self.enable_graph_building)
        self.min_node_count = self.try_get_param(
            "min_node_count", self.min_node_count)
        self.send_separate_traj_msgs = self.try_get_param(
            "send_separate_traj_msgs", self.send_separate_traj_msgs)

        # Reduction settings.
        self.reduce_global_graph = self.try_get_param(
            "reduce_global_graph", self.reduce_global_graph)
        self.reduction_method = self.try_get_param(
            "reduction_method", self.reduction_method)
        self.reduce_to_n_percent = self.try_get_param(
            "reduce_to_n_percent", self.reduce_to_n_percent)

        # input
        self.in_graph_topic = self.try_get_param(
            "in_graph_topic", self.in_graph_topic)
        self.in_traj_opt_topic = self.try_get_param(
            "in_traj_opt_topic", self.in_traj_opt_topic)
        self.verification_service_topic = self.try_get_param(
            "verification_service", self.verification_service_topic)
        self.opt_pc_topic = self.try_get_param(
            "opt_pc_topic", self.opt_pc_topic)

        # output
        self.out_graph_topic = self.try_get_param(
            "out_graph_topic", self.out_graph_topic)
        self.out_traj_opt_topic = self.try_get_param(
            "out_traj_opt_topic", self.out_traj_opt_topic)
        self.accumulated_map_topic = self.try_get_param(
            "accumulated_map_topic", self.accumulated_map_topic)


class ClientConfig(BaseConfig):
    def __init__(self, node=None):
        super().__init__(node)

        # general config
        self.rate = 0.5
        self.dataroot = '/home/berlukas/Documents/workspace/fgsp_ws/src/fgsp'
        self.robot_name = 'cerberus'
        self.enable_client_update = True
        self.enable_anchor_constraints = False
        self.enable_relative_constraints = False
        self.enable_signal_recording = False
        self.enable_trajectory_recording = False
        self.signal_export_path = "/data/{key}_{src}_signal.npy"
        self.graph_coords_export_path = "/data/{key}_{src}_graph_coords.npy"
        self.graph_adj_export_path = "/data/{key}_{src}_graph_adj.npy"
        self.trajectory_export_path = "/data/{key}_{src}_trajectory.npy"
        self.trajectory_raw_export_path = "/data/{key}_{src}_raw_trajectory.npy"
        self.label_output_path = "/data/opt_labels.dat"
        self.connections_output_path = "/data/opt_connections.dat"
        self.degenerate_window = 10
        self.synchronization_max_diff_s = 1.0
        self.verbosity = 1
        self.warmup_nodes = 10
        self.max_iterations = -1

        # constraint construction
        self.client_mode = 'multiscale'
        self.wavelet_scales = 6
        self.classifier = 'top'
        self.top_classifier_select_n = 10
        self.top_classifier_min_threshold = 0.1
        self.large_scale_partition_method = 'id'
        self.large_scale_anchor = False
        self.n_hop_mid_constraints = 10
        self.min_dist_large_constraints = 20.0
        self.nn_neighbors = 3
        self.stop_method = 'none'
        self.stop_threshold = 0.0

        # Graph construction
        self.construction_method = 'se3'
        self.use_graph_hierarchies = True
        self.max_graph_levels = 2
        self.use_downstreaming = False
        self.graph_hierarchies_node_threshold = 100
        self.use_parallel_construction = True
        self.visualize_graph = False

        # input
        self.opt_graph_topic = "/graph_monitor/sparse_graph/graph"
        self.opt_traj_topic = "/graph_monitor/sparse_graph/trajectory"
        self.est_traj_topic = "/trajectory"
        self.est_traj_path_topic = "/incremental_trajectory"

        # output
        self.anchor_node_topic = "/graph_client/anchor_nodes"
        self.relative_node_topic = "/graph_client/relative_nodes"
        self.intra_constraint_topic = "/graph_client/intra_constraints"

        # input and output
        self.client_update_topic = "/graph_client/latest_graph"
        self.T_robot_server = np.eye(4).reshape(16).tolist()

    def init_from_config(self):
        # general config
        self.rate = self.try_get_param("update_rate", self.rate)
        self.dataroot = self.try_get_param("dataroot", self.dataroot)
        self.robot_name = self.try_get_param("robot_name", self.robot_name)
        self.enable_client_update = self.try_get_param(
            "enable_client_update", self.enable_client_update)
        self.enable_anchor_constraints = self.try_get_param(
            "enable_anchor_constraints", self.enable_anchor_constraints)
        self.enable_relative_constraints = self.try_get_param(
            "enable_relative_constraints", self.enable_relative_constraints)
        self.enable_signal_recording = self.try_get_param(
            "enable_signal_recording", self.enable_signal_recording)
        self.enable_trajectory_recording = self.try_get_param(
            "enable_trajectory_recording", self.enable_trajectory_recording)
        self.signal_export_path = self.try_get_param(
            "signal_export_path", self.signal_export_path)
        self.graph_coords_export_path = self.try_get_param(
            "graph_coords_export_path", self.graph_coords_export_path)
        self.graph_adj_export_path = self.try_get_param(
            "graph_adj_export_path", self.graph_adj_export_path)
        self.trajectory_export_path = self.try_get_param(
            "trajectory_export_path", self.trajectory_export_path)
        self.trajectory_raw_export_path = self.try_get_param(
            "trajectory_raw_export_path", self.trajectory_raw_export_path)
        self.label_output_path = self.try_get_param(
            "label_output_path", self.label_output_path)
        self.connections_output_path = self.try_get_param(
            "connections_output_path", self.connections_output_path)

        self.degenerate_window = self.try_get_param(
            "degenerate_window", self.degenerate_window)
        self.synchronization_max_diff_s = self.try_get_param(
            "synchronization_max_diff_s", self.synchronization_max_diff_s)
        self.verbosity = self.try_get_param("verbosity", self.verbosity)
        self.warmup_nodes = self.try_get_param(
            "warmup_nodes", self.warmup_nodes)
        self.max_iterations = self.try_get_param(
            "max_iterations", self.max_iterations)

        # constraint construction
        self.client_mode = self.try_get_param("client_mode", self.client_mode)
        self.wavelet_scales = self.try_get_param(
            "wavelet_scales", self.wavelet_scales)
        self.classifier = self.try_get_param("classifier", self.classifier)
        self.top_classifier_select_n = self.try_get_param(
            "top_classifier_select_n", self.top_classifier_select_n)
        self.top_classifier_min_threshold = self.try_get_param(
            "top_classifier_min_threshold", self.top_classifier_min_threshold)
        self.large_scale_partition_method = self.try_get_param(
            "large_scale_partition_method", self.large_scale_partition_method)
        self.large_scale_anchor = self.try_get_param(
            "large_scale_anchor", self.large_scale_anchor)
        self.n_hop_mid_constraints = self.try_get_param(
            "n_hop_mid_constraints", self.n_hop_mid_constraints)
        self.min_dist_large_constraints = self.try_get_param(
            "min_dist_large_constraints", self.min_dist_large_constraints)
        self.nn_neighbors = self.try_get_param(
            "nn_neighbors", self.nn_neighbors)
        self.stop_method = self.try_get_param("stop_method", self.stop_method)
        self.stop_threshold = self.try_get_param(
            "stop_threshold", self.stop_threshold)

        # Graph construction
        self.construction_method = self.try_get_param(
            "construction_method", self.construction_method)
        self.use_graph_hierarchies = self.try_get_param(
            "use_graph_hierarchies", self.use_graph_hierarchies)
        self.max_graph_levels = self.try_get_param(
            "max_graph_levels", self.max_graph_levels)
        self.use_downstreaming = self.try_get_param(
            "use_downstreaming", self.use_downstreaming)
        self.graph_hierarchies_node_threshold = self.try_get_param(
            "graph_hierarchies_node_threshold", self.graph_hierarchies_node_threshold)
        self.use_parallel_construction = self.try_get_param(
            "use_parallel_construction", self.use_parallel_construction)
        self.visualize_graph = self.try_get_param(
            "visualize_graph", self.visualize_graph)

        # input
        self.opt_graph_topic = self.try_get_param(
            "opt_graph_topic", self.opt_graph_topic)
        self.opt_traj_topic = self.try_get_param(
            "opt_traj_topic", self.opt_traj_topic)
        self.est_traj_topic = self.try_get_param(
            "est_traj_topic", self.est_traj_topic)
        self.est_traj_path_topic = self.try_get_param(
            "est_traj_path_topic", self.est_traj_path_topic)

        # output
        self.anchor_node_topic = self.try_get_param(
            "anchor_node_topic", self.anchor_node_topic)
        self.relative_node_topic = self.try_get_param(
            "relative_node_topic", self.relative_node_topic)
        self.intra_constraint_topic = self.try_get_param(
            "intra_constraints", self.intra_constraint_topic)

        # input and output
        self.client_update_topic = self.try_get_param(
            "client_update_topic", self.client_update_topic)
        self.T_robot_server = np.array(self.try_get_param(
            "T_robot_server", self.T_robot_server)).reshape(4, 4)


class DebugConfig(BaseConfig):
    def __init__(self):
        # Reduction settings.
        self.reduce_global_graph = True
        self.reduction_method = 'largest_ev'
        # self.reduction_method = 'every_other'
        self.reduce_to_n_percent = 0.4


if __name__ == '__main__':
    from plotter import Plotter

    # cfg = MonitorConfig()
    # Plotter.PrintMonitorConfig(cfg)

    cfg = ClientConfig()
    Plotter.PrintClientConfig(cfg)
