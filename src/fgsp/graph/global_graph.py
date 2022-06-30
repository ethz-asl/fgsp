#! /usr/bin/env python3
import time

import numpy as np
from scipy import spatial
from maplab_msgs.msg import Graph
from geometry_msgs.msg import Point
from pygsp import graphs, filters, reduction, utils
from functools import partial
from multiprocessing import Pool
from liegroups import SE3

from src.fgsp.common.logger import Logger
from src.fgsp.common.utils import Utils
from src.fgsp.common.visualizer import Visualizer
from src.fgsp.graph.base_graph import BaseGraph
from src.fgsp.common.utils import Utils
from src.fgsp.common.visualizer import Visualizer


class GlobalGraph(BaseGraph):
    def __init__(self, config, reduced=False):
        BaseGraph.__init__(self, config)
        self.adj = None
        self.coords = np.array([])
        self.G = None
        self.is_reduced = reduced
        self.reduced_ind = []
        self.skip_ind = []
        self.submap_ind = []
        self.graph_seq = -1
        self.latest_graph_msg = None
        Logger.LogInfo('GlobalGraph: Initialized.')

    def msg_contains_updates(self, graph_msg):
        if self.is_built is False:
            return True

        if graph_msg.header.frame_id != '':
            return int(graph_msg.header.frame_id) > self.graph_seq
        else:
            return graph_msg.header.seq > self.graph_seq

    def graph_size(self):
        if self.G is not None:
            return self.G.N
        else:
            return 0

    def build(self, graph_msg):
        start_time = time.time()
        self.coords = self.read_coordinates(graph_msg)
        Logger.LogDebug(
            f'GlobalGraph: Building with coords {self.coords.shape}.')
        self.adj = self.read_adjacency(graph_msg)
        Logger.LogDebug(f'GlobalGraph: Building with adj: {self.adj.shape}.')
        self.submap_ind = self.read_submap_indices(graph_msg)
        Logger.LogDebug(
            f'GlobalGraph: Building with ind: {len(self.submap_ind)}.')

        self.adj = utils.symmetrize(self.adj, method='average')
        if not self.build_graph():
            self.G = None
            self.is_built = False
            return

        if graph_msg.header.frame_id != "":
            self.graph_seq = int(graph_msg.header.frame_id)
        else:
            self.graph_seq = graph_msg.header.seq
        self.is_built = True
        execution_time = (time.time() - start_time)
        Logger.LogInfo(
            f'GlobalGraph: Building complete ({execution_time} sec)')
        self.latest_graph_msg = graph_msg

    def build_graph(self):
        if len(self.adj.tolist()) == 0:
            Logger.LogInfo(
                'GlobalGraph: Path adjacency matrix is empty. Aborting graph building.')
            return False
        self.G = graphs.Graph(self.adj)

        if self.G.N != self.coords.shape[0]:
            Logger.LogError(
                f'GlobalGraph: Path graph size is {self.G.N} but coords are {self.coords.shape}')
            return False
        if self.G.N <= 1:
            Logger.LogDebug(
                'GlobalGraph: Path graph vertex count is less than 2.')
            return False

        self.G.set_coordinates(self.coords[:, [0, 1]])
        self.G.compute_fourier_basis()

        if (self.is_reduced):
            self.reduce_graph()

        self.is_built = True
        return True

    def build_graph_from_coords_and_adj(self, coords, adj):
        self.coords = coords
        self.adj = adj
        self.build_graph()

    def build_from_path(self, path_msg):
        return self.build_from_pose_msgs(path_msg.poses)

    def build_from_pose_msgs(self, poses):
        n_poses = len(poses)
        if n_poses <= 0:
            Logger.LogError('GlobalGraph: Received empty path message.')
            return
        poses = self.read_coordinates_from_poses(poses)
        self.coords = poses[:, 0:3]
        Logger.LogDebug(
            f'GlobalGraph: Building with coords {self.coords.shape}.')
        self.adj = self.create_adjacency_from_poses(poses)
        Logger.LogDebug(f'GlobalGraph: Building with adj: {self.adj.shape}.')
        return self.build_graph()

    def build_from_poses(self, poses):
        n_poses = poses.shape[0]
        if n_poses <= 0:
            Logger.LogError('GlobalGraph: Received empty path message.')
            return
        self.coords = poses
        Logger.LogDebug(
            f'GlobalGraph Building with coords {self.coords.shape}.')
        self.adj = self.create_adjacency_from_poses(self.coords)
        Logger.LogDebug(f'GlobalGraph: Building with adj: {self.adj.shape}.')
        return self.build_graph()

    def skip_jumping_coords(self, prev_coords, next_coords):
        n_coords_to_check = len(prev_coords)
        skip_indices = []
        for i in range(0, n_coords_to_check):
            diff = np.linalg.norm(prev_coords[i, :] - next_coords[i, :])
            if diff < 0.5:
                continue
            next_coords = np.delete(next_coords, i, 0)
            skip_indices.append(i)

        return next_coords, skip_indices

    def has_skipped(self):
        return len(self.skip_ind) > 0

    def read_coordinates(self, graph_msg):
        n_coords = len(graph_msg.coords)
        coords = np.zeros((n_coords, 3))
        for i in range(n_coords):
            coords[i, 0] = graph_msg.coords[i].x
            coords[i, 1] = graph_msg.coords[i].y
            coords[i, 2] = graph_msg.coords[i].z

        return coords

    def read_coordinates_from_poses(self, poses):
        n_coords = len(poses)
        coords = np.zeros((n_coords, 8))
        for i in range(0, n_coords):
            coords[i, 0] = poses[i].pose.position.x
            coords[i, 1] = poses[i].pose.position.y
            coords[i, 2] = poses[i].pose.position.z
            coords[i, 3] = poses[i].pose.orientation.x
            coords[i, 4] = poses[i].pose.orientation.y
            coords[i, 5] = poses[i].pose.orientation.z
            coords[i, 6] = poses[i].pose.orientation.w
            coords[i, 7] = Utils.ros_time_msg_to_ns(poses[i].pose.header.stamp)
        return coords

    def read_adjacency(self, graph_msg):
        n_coords = len(graph_msg.coords)
        adj = np.zeros((n_coords, n_coords))
        for i in range(n_coords):
            for j in range(n_coords):
                adj[i, j] = graph_msg.adjacency_matrix[j + i * n_coords]

        return adj

    def get_graph(self):
        return self.G

    def compute_temporal_decay(self, timestmap_lhs, timestamp_rhs):
        ts_diff_s = Utils.ts_ns_to_seconds(
            np.absolute(timestmap_lhs - timestamp_rhs))
        alpha = 1.5
        beta = 1000
        return alpha * np.exp(-ts_diff_s / beta)

    def read_submap_indices(self, graph_msg):
        return graph_msg.submap_indices

    def reduce_graph(self):
        if self.config.reduction_method == 'every_other':
            self.reduced_ind = self.reduce_every_other(self.coords)
        elif self.config.reduction_method == 'positive_ev':
            self.reduced_ind = self.reduce_largest_ev_positive(
                self.G.N, self.G)
        elif self.config.reduction_method == 'negative_ev':
            self.reduced_ind = self.reduce_largest_ev_negative(
                self.G.N, self.G)
        elif self.config.reduction_method == 'largest_ev':
            take_n = int(round(self.config.reduce_to_n_percent * self.G.N))
            if take_n >= self.G.N:
                Logger.LogWarn(
                    'GlobalGraph: Requested reduction amount is equal or greater than the graph size.')
                print(take_n)
                print(self.G.N)
                return
            self.reduced_ind = self.reduce_largest_ev(take_n)
        else:
            Logger.LogError(
                f'GlobalGraph: Unknown graph reduction method: {self.config.reduction_method}. Aborting reduction.')
            return
        self.reduce_graph_using_indices(self.reduced_ind)

    def reduce_graph_using_indices(self, reduced_ind):
        Logger.LogInfo(
            f'GlobalGraph: Reducing graph using {len(reduced_ind)}/{self.G.N} indices.')
        self.coords = self.coords[reduced_ind]
        self.G = reduction.kron_reduction(self.G, reduced_ind)
        self.adj = self.G.W.toarray()

        # TODO(lbern): check why kron results in some negative weights sometimes.
        # self.adj[self.adj < 0] = 0
        # self.G = graphs.Graph(self.adj)

        assert np.all(self.adj >= 0)
        self.G.compute_fourier_basis()

    def to_graph_msg(self):
        graph_msg = Graph()
        graph_msg.header.seq = self.graph_seq
        graph_msg.header.frame_id = str(self.graph_seq)
        n_coords = self.G.N

        # Write coordinates and adjacency.
        graph_msg.coords = [None] * n_coords
        graph_msg.adjacency_matrix = [0] * (n_coords*n_coords)
        for i in range(0, n_coords):
            graph_msg.coords[i] = Point()
            graph_msg.coords[i].x = self.coords[i, 0]
            graph_msg.coords[i].y = self.coords[i, 1]
            graph_msg.coords[i].z = self.coords[i, 2]
            for j in range(0, n_coords):
                graph_msg.adjacency_matrix[j + i * n_coords] = self.adj[i, j]

        graph_msg.submap_indices = self.submap_ind
        graph_msg.reduced_indices = self.reduced_ind

        return graph_msg

    def write_graph_to_disk(self, coords_file, adj_file):
        np.save(coords_file, self.coords)
        np.save(adj_file, self.adj)

    def publish(self):
        if not self.is_built:
            return
        viz = Visualizer(self.config)

        n_coords = self.G.N
        if n_coords > self.coords.shape[0] or n_coords > self.adj.shape[0]:
            Logger.LogError(
                f'Size mismatch in global graph {n_coords} vs. {self.coords.shape[0]} vs. {self.adj.shape[0]}.')
            return

        # Publish the coordinates of the global graph along with the adjacency matrix
        for i in range(0, n_coords):
            pt_h_i = np.ones((4, 1), dtype=np.float32)
            pt_h_i[0:3, 0] = self.coords[i, 0:3]
            pt_i = np.dot(self.config.T_robot_server, pt_h_i)
            viz.add_graph_coordinate(pt_i)

            for j in range(0, n_coords):
                pt_h_j = np.ones((4, 1), dtype=np.float32)
                pt_h_j[0:3, 0] = self.coords[j, 0:3]
                pt_j = np.dot(self.config.T_robot_server, pt_h_j)
                if i >= n_coords or j >= self.coords.shape[0]:
                    continue
                if i >= self.adj.shape[0] or j >= self.adj.shape[1]:
                    continue
                if self.adj[i, j] <= 0.0:
                    continue

                viz.add_graph_adjacency(pt_i, pt_j)
        viz.visualize_coords()
        viz.visualize_adjacency()

        Logger.LogInfo('GlobalGraph: Visualized global graph.')
