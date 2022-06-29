#! /usr/bin/env python3

import numpy as np
from pygsp import graphs, reduction

from src.fgsp.graph.base_graph import BaseGraph
from src.fgsp.common.logger import Logger


class HierarchicalGraph(BaseGraph):
    def __init__(self, config):
        BaseGraph.__init__(self, config)
        self.G = [None]
        self.adj = [None]
        self.coords = [None]
        self.idx = 0
        self.node_threshold = 200
        Logger.LogInfo(
            f'HierarchicalGraph: Initialized with a threshold of {self.node.threshold}.')

    def build(self, graph_msg):
        pass

    def build_graph(self, graph_msg):
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

        self.G[self.idx].set_coordinates(self.coords[self.idx][:, [0, 1]])
        self.G[self.idx].compute_fourier_basis()

        return self.build_hierarchy()

    def build_from_poses(self, poses):
        self.idx = 0
        self.coords[self.idx] = poses
        self.adj[self.idx] = self.create_adjacency_from_poses(
            self.coords[self.idx])
        self.build_graph()

    def build_hierarchy(self):
        current_n = self.G[self.idx].N
        if current_n < self.node_threshold:
            return True
        self.coords.append(None)
        self.adj.append(None)
        self.G.append(None)

        indices = self.reduce_every_other(self.coords[self.idx])
        G_next = reduction.kron_reduction(self.G[self.idx], indices)

        self.idx = self.idx + 1
        self.G[self.idx] = G_next
        self.adj[self.idx] = G_next.W.toarray()
        self.coords[self.idx] = self.coords[indices]

        return True

    def get_graph(self):
        return self.G[self.idx]

    def write_graph_to_disk(self, coords_file, adj_file):
        np.save(coords_file, self.coords[0])
        np.save(adj_file, self.adj[0])
