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
from fgsp.graph.base_graph import BaseGraph

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
        rospy.loginfo("[GlobalGraph] Initialized.")

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
        rospy.logdebug("[GlobalGraph] Building with coords " + str(self.coords.shape))
        self.adj = self.read_adjacency(graph_msg)
        rospy.logdebug("[GlobalGraph] Building with adj: " + str(self.adj.shape))
        self.submap_ind = self.read_submap_indices(graph_msg)
        rospy.logdebug("[GlobalGraph] Building with ind: " + str(len(self.submap_ind)))

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
        rospy.loginfo('[GlobalGraph] Building complete ({execution_time} sec)'.format(execution_time=execution_time))
        self.latest_graph_msg = graph_msg

    def build_graph(self):
        if len(self.adj.tolist()) == 0:
            rospy.loginfo("[GlobalGraph] Path adjacency matrix is empty. Aborting graph building.")
            return False
        self.G = graphs.Graph(self.adj)

        if self.G.N != self.coords.shape[0]:
            rospy.logerr("[GlobalGraph] Path graph size is {coords} but coords are {coords}".format(n=self.G.N, coords=self.coords.shape))
            return False
        if self.G.N <= 1:
            rospy.logdebug("[GlobalGraph] Path graph vertex count is less than 2.")
            return False

        self.G.set_coordinates(self.coords[:,[0,1]])
        self.G.compute_fourier_basis()

        if (self.is_reduced):
            self.reduce_graph()
        return True

    def build_graph_from_coords_and_adj(self, coords, adj):
        self.coords = coords
        self.adj = adj
        self.build_graph()

    def build_from_path(self, path_msg):
        start_time = time.time()
        self.build_from_pose_msgs(path_msg.poses)
        return self.build_graph()

    def build_from_pose_msgs(self, poses):
        start_time = time.time()
        n_poses = len(poses)
        if n_poses <= 0:
            rospy.logerr("[GlobalGraph] Received empty path message.")
            return
        poses = self.read_coordinates_from_poses(poses)
        self.coords = poses[:,0:3]
        rospy.logdebug("[GlobalGraph] Building with coords " + str(self.coords.shape))
        self.adj = self.create_adjacency_from_poses(poses)
        rospy.logdebug("[GlobalGraph] Building with adj " + str(self.adj.shape))

    def build_from_poses(self, poses):
        start_time = time.time()
        n_poses = poses.shape[0]
        if n_poses <= 0:
            rospy.logerr("[GlobalGraph] Received empty path message.")
            return
        self.coords = poses
        rospy.logdebug("[GlobalGraph] Building with coords " + str(self.coords.shape))
        self.adj = self.create_adjacency_from_poses(self.coords)
        rospy.logdebug("[GlobalGraph] Building with adj " + str(self.adj.shape))
        self.build_graph()

    def skip_jumping_coords(self, prev_coords, next_coords):
        n_coords_to_check = len(prev_coords)
        skip_indices = []
        for i in range(0, n_coords_to_check):
            diff = np.linalg.norm(prev_coords[i,:] - next_coords[i,:])
            if diff < 0.5:
                continue
            next_coords = np.delete(next_coords,i,0)
            skip_indices.append(i)

        return next_coords, skip_indices

    def has_skipped(self):
        return len(self.skip_ind) > 0

    def read_coordinates(self, graph_msg):
        n_coords = len(graph_msg.coords)
        coords = np.zeros((n_coords,3))
        for i in range(n_coords):
            coords[i,0] = graph_msg.coords[i].x
            coords[i,1] = graph_msg.coords[i].y
            coords[i,2] = graph_msg.coords[i].z

        return coords

    def read_coordinates_from_poses(self, poses):
        n_coords = len(poses)
        coords = np.zeros((n_coords, 8))
        for i in range(0, n_coords):
            coords[i,0] = poses[i].pose.position.x
            coords[i,1] = poses[i].pose.position.y
            coords[i,2] = poses[i].pose.position.z
            coords[i,3] = poses[i].pose.orientation.x
            coords[i,4] = poses[i].pose.orientation.y
            coords[i,5] = poses[i].pose.orientation.z
            coords[i,6] = poses[i].pose.orientation.w
            coords[i,7] = Utils.ros_time_to_ns(poses[i].pose.header.stamp)
        return coords

    def read_adjacency(self, graph_msg):
        n_coords = len(graph_msg.coords)
        adj = np.zeros((n_coords, n_coords))
        for i in range(n_coords):
            for j in range(n_coords):
                adj[i,j] = graph_msg.adjacency_matrix[j + i * n_coords]

        return adj

    def get_graph(self):
        return self.G


    def read_submap_indices(self, graph_msg):
        return graph_msg.submap_indices

    def reduce_graph(self):
        if self.config.reduction_method == 'every_other':
            self.reduced_ind = self.reduce_every_other()
        elif self.config.reduction_method == 'positive_ev':
            self.reduced_ind = self.reduce_largest_ev_positive(self.G.N)
        elif self.config.reduction_method == 'negative_ev':
            self.reduced_ind = self.reduce_largest_ev_negative(self.G.N)
        elif self.config.reduction_method == 'largest_ev':
            take_n = int(round(self.config.reduce_to_n_percent * self.G.N))
            if take_n >= self.G.N:
                rospy.logwarn('[GlobalGraph] Requested reduction amount is equal or greater than the graph size.')
                print(take_n)
                print(self.G.N)
                return
            self.reduced_ind = self.reduce_largest_ev(take_n)
        else:
            rospy.logerr('[GlobalGraph] Unknown graph reduction method: {method}. Aborting reduction.'.format(method=self.config.reduction_method))
            return
        self.reduce_graph_using_indices(self.reduced_ind)

    def reduce_graph_using_indices(self, reduced_ind):
        rospy.loginfo('[GlobalGraph] Reducing graph using {reduced}/{coords} indices.'.format(reduced=len(reduced_ind), coords=self.G.N))
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
            graph_msg.coords[i].x = self.coords[i,0]
            graph_msg.coords[i].y = self.coords[i,1]
            graph_msg.coords[i].z = self.coords[i,2]
            for j in range(0, n_coords):
                graph_msg.adjacency_matrix[j + i * n_coords] = self.adj[i,j]

        graph_msg.submap_indices = self.submap_ind
        graph_msg.reduced_indices = self.reduced_ind

        return graph_msg

    def write_graph_to_disk(self, coords_file, adj_file):
        np.save(coords_file, self.coords)
        np.save(adj_file, self.adj)

    def publish(self):
        if not self.is_built:
            return
        viz = Visualizer()

        n_coords = self.G.N
        if n_coords > self.coords.shape[0] or n_coords > self.adj.shape[0]:
            rospy.logerr('Size mismatch in global graph {n_global} vs. {n_coords} vs. {n_adj}'.format(n_global=n_coords, n_coords=self.coords.shape[0], n_adj=self.adj.shape[0]))
            return

        # First publish the coordinates of the global graph.
        for i in range(0, n_coords):
            viz.add_graph_coordinate(self.coords[i,:])
        viz.visualize_coords()

        # Next publish the adjacency matrix of the global graph.
        for i in range(0, n_coords):
            for j in range(0, n_coords):
                if i >= n_coords or j >= self.coords.shape[0]:
                    continue
                if i >= self.adj.shape[0] or j >= self.adj.shape[1]:
                    continue
                if self.adj[i,j] <= 0.0:
                    continue

                viz.add_graph_adjacency(self.coords[i,:], self.coords[j,:])
        viz.visualize_adjacency()

        rospy.loginfo("[GlobalGraph] Visualized global graph.")
