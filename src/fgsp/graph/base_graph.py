#! /usr/bin/env python2
import rospy
import time
import sys

import numpy as np
from liegroups import SE3
from pygsp import graphs, filters, reduction, utils
from geometry_msgs.msg import Point
from maplab_msgs.msg import Graph
from scipy import spatial
from scipy.spatial.transform import Rotation

from fgsp.common.visualizer import Visualizer
from fgsp.common.utils import Utils

class BaseGraph(object):
    def __init__(self, config):
        self.config = config
        self.is_built = False
        self.graph_seq = -1
        self.latest_graph_msg = None

    def build(self, graph_msg):
        rospy.logfatal("Called method in HierarchicalGraph")

    def get_graph(self):
        rospy.logfatal("Called method in HierarchicalGraph")

    def build_from_poses(self, poses):
        rospy.logfatal("Called method in HierarchicalGraph")

    def write_graph_to_disk(self, coords_file, adj_file):
        rospy.logfatal("Called method in HierarchicalGraph")

    def create_adjacency_from_poses(self, poses):
        n_coords = poses.shape[0]
        adj = np.zeros((n_coords, n_coords))
        tree = spatial.KDTree(poses[:,0:3])
        max_pos_dist = 6.0
        # n_nearest_neighbors = min(20, n_coords)
        for i in range(n_coords):
            # nn_dists, nn_indices = tree.query(coords[i,:], p = 2, k = n_nearest_neighbors)
            # nn_indices = [nn_indices] if n_nearest_neighbors == 1 else nn_indices
            nn_indices = tree.query_ball_point(poses[i,0:3], r = max_pos_dist, p = 2)

            # print(f'Found the following indices: {nn_indices} / {n_coords}')
            for nn_i in nn_indices:
                if nn_i == i:
                    continue
                if self.config.use_se3_computation:
                    adj[i, nn_i] = self.compute_se3_weights(poses[i,:], poses[nn_i,:])
                elif self.config.use_so3_computation:
                    adj[i, nn_i] = self.compute_so3_weights(poses[i,:], poses[nn_i,:])
                else:
                    adj[i, nn_i] = self.compute_simple_weights(poses[i,:], poses[nn_i,:])

        adj[adj < 0] = 0
        assert np.all(adj >= 0)
        return adj

    def compute_simple_weights(self, poses_lhs, poses_rhs):
        w_d = self.compute_distance_weight(poses_lhs[0:3], poses_rhs[0:3])
        if self.config.include_rotational_weight:
            w_r = self.compute_rotation_weight(poses_lhs, poses_rhs)
        else:
            w_r = 0.0
        if self.config.include_temporal_decay_weight:
            w_t = self.compute_temporal_decay(poses_lhs[7], poses_rhs[7])
        else:
            w_t = 1.0
        return w_t * (w_d + w_r)

    def compute_se3_weights(self, poses_lhs, poses_rhs):
        T_G_lhs = Utils.convert_pos_quat_to_transformation(poses_lhs[0:3], poses_lhs[3:7])
        T_G_rhs = Utils.convert_pos_quat_to_transformation(poses_rhs[0:3], poses_rhs[3:7])

        pose1 = SE3.from_matrix(T_G_lhs)
        pose2 = SE3.from_matrix(T_G_rhs)

        Xi_12 = (pose1.inv().dot(pose2)).log()
        W = np.eye(4,4)
        W[0,0] = 10
        W[1,1] = 10
        W[2,2] = 1
        W[3,3] = 3
        inner = np.trace(np.matmul(np.matmul(SE3.wedge(Xi_12),W),SE3.wedge(Xi_12).transpose()))

        # Equal weighting for rotation and translation.
        # inner = np.matmul(Xi_12.transpose(),Xi_12)

        dist = np.sqrt(inner)
        sigma = 1.0
        normalization = 2.0*(sigma**2)
        return np.exp(-dist/normalization)

    def compute_so3_weights(self, pose_lhs, pose_rhs):
        R_lhs = Utils.convert_quat_to_rotation(pose_lhs[3:7])
        R_rhs = Utils.convert_quat_to_rotation(pose_rhs[3:7])
        rot_diff = np.matmul(R_lhs, R_rhs.transpose())
        return np.trace(rot_diff)

    def compute_distance_weight(self, coords_lhs, coords_rhs):
        sigma = 1.0
        normalization = 2.0*(sigma**2)
        dist = spatial.distance.euclidean(coords_lhs, coords_rhs)
        return np.exp(-dist/normalization)

    def compute_rotation_weight(self, coords_lhs, coords_rhs):
        import eigenpy
        # angle_lhs = np.linalg.norm(Rotation.from_quat(coords_lhs[3:]).as_rotvec())
        # angle_rhs = np.linalg.norm(Rotation.from_quat(coords_rhs[3:]).as_rotvec())
        # angle = angle_lhs - angle_rhs
        T_G_lhs = Utils.convert_pos_quat_to_transformation(coords_lhs[0:3], coords_lhs[3:7])
        T_G_rhs = Utils.convert_pos_quat_to_transformation(coords_rhs[0:3], coords_rhs[3:7])
        T_lhs_rhs = np.matmul(np.linalg.inv(T_G_lhs), T_G_rhs)
        rotation = eigenpy.AngleAxis(T_lhs_rhs[0:3,0:3]).angle

        eps = 0.001
        return 0.5 * (1 + np.cos(rotation)) + eps

    def compute_temporal_decay(self, timestmap_lhs, timestamp_rhs):
        ts_diff_s = Utils.ts_ns_to_seconds(np.absolute(timestmap_lhs - timestamp_rhs))
        alpha = 1.5
        beta = 1000
        return alpha * np.exp(-ts_diff_s / beta)

    def reduce_every_other(self):
        n_nodes = self.coords.shape[0]
        return np.arange(0, n_nodes, 2)

    def reduce_largest_ev_positive(self, take_n):
        idx = np.argmax(np.abs(self.G.U))
        idx_vertex, idx_fourier = np.unravel_index(idx, self.G.U.shape)
        indices = []
        for i in range(0, take_n):
            if (self.G.U[i,idx_fourier] >= 0):
                indices.append(i)
        return indices

    def reduce_largest_ev_negative(self, take_n):
        idx = np.argmax(np.abs(self.G.U))
        idx_vertex, idx_fourier = np.unravel_index(idx, self.G.U.shape)
        indices = []
        for i in range(0, take_n):
            if (self.G.U[i,idx_fourier] < 0):
                indices.append(i)
        return indices

    def reduce_largest_ev(self, take_n):
        rospy.loginfo('[GlobalGraph] Reducing to largest {n} EVs'.format(n=take_n))
        indices = []
        ev = np.abs(self.G.U)
        for i in range(0, take_n):
            idx = np.argmax(ev)
            idx_vertex, idx_fourier = np.unravel_index(idx, self.G.U.shape)
            if ev[idx_vertex, idx_fourier] == -1:
                rospy.logwarn('[GlobalGraph] Warning Could not reduce to requested number of nodes: {indices}/{take_n}'.format(indices=len(indices),take_n=take_n))
                return indices
            ev[idx_vertex, :] = -1
            indices.append(idx_vertex)
        return indices
