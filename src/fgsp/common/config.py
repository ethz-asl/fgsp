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
        self.enable_submap_constraints = True
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
        self.enable_submap_map_publishing = False
        self.compute_poses_in_LiDAR = False

        # input
        self.in_graph_topic = '/maplab_server/sparse_graph/graph'
        self.in_traj_opt_topic = '/maplab_server/sparse_graph/trajectory'
        self.verification_service_topic = '/grap_monitor/verification'
        self.opt_pc_topic = '/maplab_server/sparse_graph/submap'

        # output
        self.out_graph_topic = '/graph_monitor/sparse_graph/graph'
        self.out_traj_opt_topic = '/graph_monitor/sparse_graph/trajectory'
        self.submap_topic = '/graph_monitor/submaps'
        self.accumulated_map_topic = '/graph_monitor/map'

    def init_from_config(self):
        # general config
        self.rate = self.try_get_param("update_rate", self.rate)
        self.enable_submap_constraints = self.try_get_param(
            "enable_submap_constraints", self.enable_submap_constraints)
        self.enable_graph_building = self.try_get_param(
            "enable_graph_building", self.enable_graph_building)
        self.min_node_count = self.try_get_param(
            "min_node_count", self.min_node_count)
        self.submap_min_count = self.try_get_param(
            "submap_min_count", self.submap_min_count)
        self.send_separate_traj_msgs = self.try_get_param(
            "send_separate_traj_msgs", self.send_separate_traj_msgs)

        # Reduction settings.
        self.reduce_global_graph = self.try_get_param(
            "reduce_global_graph", self.reduce_global_graph)
        self.reduction_method = self.try_get_param(
            "reduction_method", self.reduction_method)
        self.reduce_to_n_percent = self.try_get_param(
            "reduce_to_n_percent", self.reduce_to_n_percent)

        # submap constraints
        self.pivot_distance = self.try_get_param(
            "submap_constraint_pivot_distance", self.pivot_distance)
        self.min_pivot_distance = self.try_get_param(
            "submap_constraint_min_distance", self.min_pivot_distance)
        self.n_nearest_neighbors = self.try_get_param(
            "submap_constraint_knn", self.n_nearest_neighbors)
        self.p_norm = self.try_get_param(
            "submap_constraint_p_norm", self.p_norm)
        self.enable_submap_map_publishing = self.try_get_param(
            "enable_submap_map_publishing", self.enable_submap_map_publishing)
        self.compute_poses_in_LiDAR = self.try_get_param(
            "submap_constraint_export_lidar_poses", self.compute_poses_in_LiDAR)

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
        self.submap_topic = self.try_get_param(
            "submap_constraint_topic", self.submap_topic)
        self.accumulated_map_topic = self.try_get_param(
            "accumulated_map_topic", self.accumulated_map_topic)


class ClientConfig(BaseConfig):
    def __init__(self, node):
        super().__init__(node)

        # general config
        self.rate = 0.5
        self.dataroot = '/home/berlukas/Documents/workspace/fgsp_ws/src/fgsp'
        self.robot_name = 'cerberus'
        self.enable_client_update = True
        self.enable_submap_constraints = True
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
        self.degenerate_window = 10
        self.synchronization_max_diff_s = 1.0
        self.verbosity = 1

        # constraint construction
        self.client_mode = 'multiscale'
        self.classifier = 'top'
        self.top_classifier_select_n = 10

        # Graph construction
        self.include_rotational_weight = False
        self.include_temporal_decay_weight = False
        self.use_se3_computation = False
        self.use_so3_computation = False
        self.use_graph_hierarchies = False

        # input
        self.opt_graph_topic = "/graph_monitor/sparse_graph/graph"
        self.opt_traj_topic = "/graph_monitor/sparse_graph/trajectory"
        self.est_traj_topic = "/trajectory"
        self.est_traj_path_topic = "/incremental_trajectory"
        self.submap_constraint_topic = "/graph_monitor/submaps"

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
        self.enable_submap_constraints = self.try_get_param(
            "enable_submap_constraints", self.enable_submap_constraints)
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
        self.degenerate_window = self.try_get_param(
            "degenerate_window", self.degenerate_window)
        self.synchronization_max_diff_s = self.try_get_param(
            "synchronization_max_diff_s", self.synchronization_max_diff_s)
        self.verbosity = self.try_get_param("verbosity", self.verbosity)

        # constraint construction
        self.client_mode = self.try_get_param("client_mode", self.client_mode)
        self.classifier = self.try_get_param("classifier", self.classifier)
        self.top_classifier_select_n = self.try_get_param(
            "top_classifier_select_n", self.top_classifier_select_n)

        # Graph construction
        self.include_rotational_weight = self.try_get_param(
            "include_rotational_weight", self.include_rotational_weight)
        self.include_temporal_decay_weight = self.try_get_param(
            "include_temporal_decay_weight", self.include_temporal_decay_weight)
        self.use_se3_computation = self.try_get_param(
            "use_se3_computation", self.use_se3_computation)
        self.use_so3_computation = self.try_get_param(
            "use_so3_computation", self.use_so3_computation)
        self.use_graph_hierarchies = self.try_get_param(
            "use_graph_hierarchies", self.use_graph_hierarchies)

        # input
        self.opt_graph_topic = self.try_get_param(
            "opt_graph_topic", self.opt_graph_topic)
        self.opt_traj_topic = self.try_get_param(
            "opt_traj_topic", self.opt_traj_topic)
        self.est_traj_topic = self.try_get_param(
            "est_traj_topic", self.est_traj_topic)
        self.est_traj_path_topic = self.try_get_param(
            "est_traj_path_topic", self.est_traj_path_topic)
        self.submap_constraint_topic = self.try_get_param(
            "opt_submap_constraint_topic", self.submap_constraint_topic)

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
        # Graph construction
        self.include_rotational_weight = False
        self.include_temporal_decay_weight = False
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
