#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction
from geometry_msgs.msg import Point
from maplab_msgs.msg import Graph
from scipy import spatial

class GlobalGraph(object):
    def __init__(self, reduced=False):
        self.adj = None
        self.coords = None
        self.G = None
        self.is_reduced = reduced
        self.is_built = False
        self.reduced_ind = []
        self.submap_ind = []
        self.graph_seq = None
        self.latest_graph_msg = None
        rospy.loginfo("[GlobalGraph] Initialized.")

    def msg_contains_updates(self, graph_msg):
        if self.is_built is False:
            return True

        return graph_msg.header.seq > self.graph_seq

    def graph_size(self):
        if self.G is not None:
            return self.G.N
        else:
            return 0

    def build(self, graph_msg):
        self.coords = self.read_coordinates(graph_msg)
        rospy.logdebug("[GlobalGraph] Building with coords " + str(self.coords.shape))
        self.adj = self.read_adjacency(graph_msg)
        rospy.logdebug("[GlobalGraph] Building with adj: " + str(self.adj.shape))
        self.submap_ind = self.read_submap_indices(graph_msg)
        rospy.logdebug("[GlobalGraph] Building with ind: " + str(len(self.submap_ind)))

        if len(self.adj.tolist()) == 0:
            rospy.loginfo(f"[GlobalGraph] Adjacency matrix is empty. Aborting graph building.")
            return
        self.G = graphs.Graph(self.adj)
        if self.G.N != self.coords.shape[0]:
            rospy.logerr(f"[GlobalGraph] Graph size is {self.G.N} but coords are {self.coords.shape}")
            return
        if self.G.N <= 1:
            rospy.logdebug("[GlobalGraph] Graph vertex count is less than 2.")
            return

        self.G.set_coordinates(self.coords[:,[0,1]])
        self.G.compute_fourier_basis()

        if (self.is_reduced):
            self.reduce_graph()

        self.graph_seq = graph_msg.header.seq
        self.is_built = True
        rospy.loginfo("[GlobalGraph] Building complete")
        self.latest_graph_msg = graph_msg

    def build_from_path(self, path_msg):
        n_poses = len(path_msg.poses)
        if n_poses <= 0:
            rospy.logerr(f"[GlobalGraph] Received empty path message.")
            return
        self.coords = self.read_coordinates_from_poses(path_msg.poses)
        rospy.loginfo("[GlobalGraph] Building with coords " + str(self.coords.shape))
        self.is_built = True

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
        coords = np.zeros((n_coords, 3))
        for i in range(0, n_coords):
            coords[i,0] = poses[i].pose.position.x
            coords[i,1] = poses[i].pose.position.y
            coords[i,2] = poses[i].pose.position.z
        return coords

    def read_adjacency(self, graph_msg):
        n_coords = len(graph_msg.coords)
        adj = np.zeros((n_coords, n_coords))
        for i in range(n_coords):
            for j in range(n_coords):
                adj[i,j] = graph_msg.adjacency_matrix[j + i * n_coords]

        return adj

    def create_adjacency_from_poses(self, coords):
        n_coords = coords.shape[0]
        adj = np.zeros((n_coords, n_coords))
        tree = spatial.KDTree(coords)
        max_pos_dist = 5
        n_nearest_neighbors = 5
        sigma = 1
        normlization = 2*(sigma**2)
        for i in range(n_coords):
            nn_dists, nn_indices = tree.query(coords[i,:], p = 2, k = n_nearest_neighbors)
            nn_indices = [nn_indices] if n_nearest_neighbors == 1 else nn_indices
            for nn_i in range(nn_indices):
                if nn_i == i:
                    continue
                dist = spatial.distance.euclidean(coords[nn_i,:], coords[i,:])
                if dist <= max_pos_dist:
                    adj[i,nn_i] = np.exp(dist/normalization)

        return adj

    def read_submap_indices(self, graph_msg):
        return graph_msg.submap_indices

    def reduce_graph(self):
        #self.reduced_ind = self.reduce_every_other()
        self.reduced_ind = self.reduce_largest_ev_positive()

        self.coords = self.coords[self.reduced_ind]
        self.G = reduction.kron_reduction(self.G, self.reduced_ind)
        self.G.compute_fourier_basis()
        self.adj = self.G.W.toarray()

    def reduce_every_other(self):
        n_nodes = np.shape(self.coords)[0]
        return np.arange(0, n_nodes, 2)

    def reduce_largest_ev_positive(self):
        idx = np.argmax(np.abs(self.G.U))
        idx_vertex, idx_fourier = np.unravel_index(idx, self.G.U.shape)
        indices = []
        for i in range(0, self.G.N):
            if (self.G.U[i,idx_fourier] >= 0):
                indices.append(i)
        return indices

    def reduce_largest_ev_negative(self):
        idx = np.argmax(np.abs(self.G.U))
        idx_vertex, idx_fourier = np.unravel_index(idx, self.G.U.shape)
        indices = []
        for i in range(0, self.G.N):
            if (self.G.U[i,idx_fourier] < 0):
                indices.append(i)
        return indices

    def to_graph_msg(self):
        graph_msg = Graph()
        graph_msg.header.seq = self.graph_seq
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
