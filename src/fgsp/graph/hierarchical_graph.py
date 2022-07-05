#! /usr/bin/env python3

import numpy as np
from pygsp import graphs, reduction

from src.fgsp.graph.base_graph import BaseGraph
from src.fgsp.common.logger import Logger
from src.fgsp.common.visualizer import Visualizer


class HierarchicalGraph(BaseGraph):
    def __init__(self, config):
        BaseGraph.__init__(self, config)
        self.G = [None]
        self.adj = [None]
        self.coords = [None]
        self.indices = [None]
        self.idx = 0
        self.node_threshold = 20
        Logger.LogInfo(
            f'HierarchicalGraph: Initialized with a threshold of {self.node_threshold}.')

    def build(self, graph_msg):
        pass

    def build_graph(self):
        if len(self.adj[self.idx].tolist()) == 0:
            Logger.LogInfo(
                f'HierarchicalGraph: Path adjacency matrix is empty. Aborting graph building.')
            return False
        self.G[self.idx] = graphs.Graph(self.adj[self.idx])

        n_nodes = self.G[self.idx].N

        if n_nodes != self.coords[self.idx].shape[0]:
            Logger.LogInfo(
                f'HierarchicalGraph: Path graph size is {n_nodes} but coords are {self.coords[self.idx].shape}')
            return False
        if n_nodes <= 1:
            Logger.LogError(
                f'HierarchicalGraph: Path graph vertex count is less than 2.')
            return False

        self.indices[self.idx] = np.arange(n_nodes)
        self.G[self.idx].set_coordinates(self.coords[self.idx][:, [0, 1]])
        self.G[self.idx].compute_fourier_basis()
        self.is_built = True
        print(f'Building is {self.is_built}.')

        return True

    def build_from_poses(self, poses):
        self.idx = 0
        self.coords[self.idx] = poses
        self.adj[self.idx] = self.create_adjacency_from_poses(
            self.coords[self.idx])
        self.build_graph()

    def build_hierarchies(self):
        while self.build_hierarchy():
            pass

    def build_hierarchy(self):
        current_n = self.G[self.idx].N
        if current_n < self.node_threshold:
            return False
        self.coords.append(None)
        self.adj.append(None)
        self.G.append(None)
        self.indices.append(None)

        indices = self.reduce_every_other(self.coords[self.idx])
        G_next = reduction.kron_reduction(self.G[self.idx], indices)

        self.idx = self.idx + 1
        self.indices[self.idx] = indices
        self.G[self.idx] = G_next
        self.adj[self.idx] = G_next.W.toarray()
        self.coords[self.idx] = self.coords[self.idx - 1][indices]

        return True

    def get_graph(self):
        return self.G[self.idx]

    def get_coords(self):
        return self.coords[self.idx]

    def get_indices(self):
        return self.indices[self.idx]

    def write_graph_to_disk(self, coords_file, adj_file):
        np.save(coords_file, self.coords[0])
        np.save(adj_file, self.adj[0])

    def publish(self):
        print(f'PUBLISH: Building is {self.is_built}.')
        if not self.is_built:
            print(f'BUILT IS FALSE??')
            return

        print(f'Visualizing graph levels: {self.idx}.')
        for i in range(self.idx+1):
            self.publish_graph_level(
                self.coords[i], self.adj[i], self.G[i].N, i)

    def publish_graph_level(self, coords, adj, n_nodes, level):
        viz = Visualizer()
        if n_nodes > coords.shape[0] or n_nodes > adj.shape[0]:
            Logger.LogError(
                f'Size mismatch in global graph {n_nodes} vs. {coords.shape[0]} vs. {adj.shape[0]}.')
            return

        color = self.get_level_color(level)
        z = np.array([0, 0, 5 * level])
        for i in range(0, n_nodes):
            pt_h_i = np.ones((4, 1), dtype=np.float32)
            pt_h_i[0:3, 0] = coords[i, 0:3] + z
            pt_i = np.dot(self.config.T_robot_server, pt_h_i)
            viz.add_graph_coordinate(pt_i, color)

            for j in range(0, n_nodes):
                pt_h_j = np.ones((4, 1), dtype=np.float32)
                pt_h_j[0:3, 0] = coords[j, 0:3]
                pt_j = np.dot(self.config.T_robot_server, pt_h_j)
                if i >= n_nodes or j >= coords.shape[0]:
                    continue
                if i >= adj.shape[0] or j >= adj.shape[1]:
                    continue
                if adj[i, j] <= 0.0:
                    continue
                viz.add_graph_adjacency(pt_i, pt_j)
        viz.visualize_coords()
        viz.visualize_adjacency()
        Logger.LogInfo(f'HierarchicalGraph: Visualized graph level {level}.')

    def get_level_color(self, idx):
        max_idx = 6
        norm_idx = idx % max_idx
        if norm_idx == 0:
            return [0.95, 0.05, 0.05]
        elif norm_idx == 1:
            return [0.05, 0.95, 0.05]
        elif norm_idx == 2:
            return [0.05, 0.05, 0.95]
        elif norm_idx == 3:
            return [0.95, 0.05, 0.95]
        elif norm_idx == 4:
            return [0.95, 0.95, 0.05]
        elif norm_idx == 5:
            return [0.05, 0.95, 0.95]
        return [0.0, 0.0, 0.0]
