#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs, filters, reduction
from geometry_msgs.msg import Point
from maplab_msgs.msg import Graph


class GlobalGraph(object):
    def __init__(self, reduced=False):
        rospy.loginfo("[Graph] Initializing graph builder..")
        self.adj = None
        self.coords = None
        self.G = None
        self.is_reduced = reduced
        self.is_built = False
        self.reduced_ind = []
        self.submap_ind = []
        self.graph_seq = None
        self.latest_graph_msg = None

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
        rospy.logdebug("[Graph] Building with coords " + str(self.coords.shape))
        self.adj = self.read_adjacency(graph_msg)
        rospy.logdebug("[Graph] Building with adj: " + str(self.adj.shape))
        self.submap_ind = self.read_submap_indices(graph_msg)
        rospy.logdebug("[Graph] Building with ind: " + str(len(self.submap_ind)))

        self.G = graphs.Graph(self.adj)
        if self.G.N != self.coords.shape[0]:
            rospy.logerr(f"[Graph] Graph size is {self.G.N} but coords are {self.coords.shape}")
            return
        if self.G.N <= 1:
            rospy.logdebug("[Graph] Graph vertex count is less than 2.")
            return


        self.G.set_coordinates(self.coords[:,[0,1]])
        self.G.compute_fourier_basis()

        if (self.is_reduced):
            self.reduce_graph()

        self.graph_seq = graph_msg.header.seq
        self.is_built = True
        rospy.loginfo("[Graph] Building complete")
        self.latest_graph_msg = graph_msg

    def read_coordinates(self, graph_msg):
        n_coords = len(graph_msg.coords)
        coords = np.zeros((n_coords,3))
        for i in range(n_coords):
            coords[i,0] = graph_msg.coords[i].x
            coords[i,1] = graph_msg.coords[i].y
            coords[i,2] = graph_msg.coords[i].z

        return coords

    def read_adjacency(self, graph_msg):
        n_coords = len(graph_msg.coords)
        adj = np.zeros((n_coords, n_coords))
        for i in range(n_coords):
            for j in range(n_coords):
                adj[i,j] = graph_msg.adjacency_matrix[j + i * n_coords]

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
        graph_msg.coords = [Point()] * n_coords
        graph_msg.adjacency_matrix = [0] * (n_coords*n_coords)
        for i in range(n_coords):
            graph_msg.coords[i].x = self.coords[i,0]
            graph_msg.coords[i].y = self.coords[i,1]
            graph_msg.coords[i].z = self.coords[i,2]
            for j in range(n_coords):
                graph_msg.adjacency_matrix[j + i * n_coords] = self.adj[i,j]

        graph_msg.submap_indices = self.submap_ind
        graph_msg.reduced_indices = self.reduced_ind

        return graph_msg
