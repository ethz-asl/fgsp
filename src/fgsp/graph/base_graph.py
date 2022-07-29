#! /usr/bin/env python3

import multiprocessing
import numpy as np
from liegroups import SE3
from scipy import spatial
from functools import partial
from multiprocessing import Pool

from src.fgsp.common.utils import Utils
from src.fgsp.common.logger import Logger


def process_poses(poses, tree, w_func, i):
    if len(poses) == 0:
        return
    max_pos_dist = 6.0
    nn_indices = tree.query_ball_point(
        poses[i, 0:3], r=max_pos_dist, p=2)

    return nn_indices, [w_func(poses[i, :], poses[nn_i, :]) for nn_i in nn_indices]


def compute_distance_weights(coords_lhs, coords_rhs):
    sigma = 1.0
    normalization = 2.0*(sigma**2)
    dist = spatial.distance.euclidean(coords_lhs[0:3], coords_rhs[0:3])

    return np.exp(-dist/normalization)


def compute_so3_weights(pose_lhs, pose_rhs):
    R_lhs = Utils.convert_quat_to_rotation(pose_lhs[3:7])
    R_rhs = Utils.convert_quat_to_rotation(pose_rhs[3:7])
    rot_diff = np.matmul(R_lhs, R_rhs.transpose())
    return np.trace(rot_diff)


def compute_se3_weights(poses_lhs, poses_rhs):
    T_G_lhs = Utils.convert_pos_quat_to_transformation(
        poses_lhs[0:3], poses_lhs[3:7])
    T_G_rhs = Utils.convert_pos_quat_to_transformation(
        poses_rhs[0:3], poses_rhs[3:7])

    pose1 = SE3.from_matrix(T_G_lhs)
    pose2 = SE3.from_matrix(T_G_rhs)

    Xi_12 = (pose1.inv().dot(pose2)).log()
    W = np.eye(4, 4)
    W[0, 0] = 5
    W[1, 1] = 5
    W[2, 2] = 0.1
    W[3, 3] = 0.1
    inner = np.trace(
        np.matmul(np.matmul(SE3.wedge(Xi_12), W), SE3.wedge(Xi_12).transpose()))

    # Equal weighting for rotation and translation.
    # inner = np.matmul(Xi_12.transpose(),Xi_12)

    dist = np.sqrt(inner)
    sigma = 1.0
    normalization = 2.0*(sigma**2)
    return np.exp(-dist/normalization)


class BaseGraph(object):
    def __init__(self, config):
        self.config = config
        self.is_built = False
        self.graph_seq = -1
        self.latest_graph_msg = None

    def build(self, graph_msg):
        Logger.LogFatal('Called method in BaseGraph')

    def build_from_poses(self, poses):
        Logger.LogFatal('Called method in BaseGraph')

    def get_graph(self):
        Logger.LogFatal('Called method in BaseGraph')

    def get_coords(self):
        Logger.LogFatal('Called method in BaseGraph')

    def write_graph_to_disk(self, coords_file, adj_file):
        Logger.LogFatal('Called method in BaseGraph')

    def publish(self):
        Logger.LogFatal('Called method in BaseGraph')

    def create_adjacency_from_poses(self, poses):
        n_coords = poses.shape[0]
        adj = np.zeros((n_coords, n_coords))
        tree = spatial.KDTree(poses[:, 0:3])

        indices = np.arange(0, n_coords)

        if self.config.construction_method == 'se3':
            func = partial(process_poses, poses, tree, compute_se3_weights)
        elif self.config.construction_method == 'so3':
            func = partial(process_poses, poses, tree, compute_so3_weights)
        elif self.config.construction_method == 'r3':
            func = partial(process_poses, poses, tree,
                           compute_distance_weights)
        else:
            Logger.LogError(
                f'Unknown construction method: {self.config.construction_method}. Using default SE(3).')
            func = partial(process_poses, poses, tree, compute_se3_weights)

        n_cores = multiprocessing.cpu_count()
        with Pool(n_cores) as p:
            for idx, (nn_indices, weights) in zip(indices, p.map(func, indices)):
                for nn_i, w in zip(nn_indices, weights):
                    if nn_i != idx:
                        adj[idx, nn_i] = w

        adj[adj < 0] = 0
        assert np.all(adj >= 0)
        return adj

    def reduce_every_other(self, coords):
        n_nodes = coords.shape[0]
        return np.arange(0, n_nodes, 2)

    def reduce_largest_ev_positive(self, take_n, G):
        idx = np.argmax(np.abs(G.U))
        _, idx_fourier = np.unravel_index(idx, G.U.shape)
        indices = []
        for i in range(0, take_n):
            if (G.U[i, idx_fourier] >= 0):
                indices.append(i)
        return indices

    def reduce_largest_ev_negative(self, take_n, G):
        idx = np.argmax(np.abs(G.U))
        _, idx_fourier = np.unravel_index(idx, G.U.shape)
        indices = []
        for i in range(0, take_n):
            if (G.U[i, idx_fourier] < 0):
                indices.append(i)
        return indices

    def reduce_largest_ev(self, take_n, G):
        Logger.LogInfo(f'BaseGraph: Reducing to largest {take_n} EVs')
        indices = []
        ev = np.abs(G.U)
        for i in range(0, take_n):
            idx = np.argmax(ev)
            idx_vertex, idx_fourier = np.unravel_index(idx, G.U.shape)
            if ev[idx_vertex, idx_fourier] == -1:
                Logger.LogWarn(
                    f'BaseGraph: Could not reduce to requested number of nodes: {len(indices)}/{take_n}')
                return indices
            ev[idx_vertex, :] = -1
            indices.append(idx_vertex)
        return indices
