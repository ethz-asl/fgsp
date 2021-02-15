#! /usr/bin/env python3
import rospy
import numpy as np
from pygsp import graphs
from pygsp import graphs, filters, reduction


class GlobalGraph(object):

    def __init__(self, reduced=False):
        rospy.loginfo("[Graph] Initializing graph builder..")
        self.adj = None
        self.coords = None
        self.G = None
        self.is_reduced = reduced
        self.is_built = False

    def build(self, graph_msg):
        self.coords = self.read_coordinates(graph_msg)
        rospy.loginfo("[Graph] Building with coords " + str(self.coords.shape))
        self.adj = self.read_adjacency(graph_msg)
        rospy.loginfo("[Graph] Building with adj: " + str(self.adj.shape))

        self.G = graphs.Graph(self.adj)
        self.G.set_coordinates(self.coords[:,[0,1]])
        self.G.compute_fourier_basis()

        if (self.is_reduced):
            self.reduce_graph()

        self.is_built = True
        rospy.loginfo("[Graph] Building complete")


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

    def reduce_graph(self):
        # TODO(lbern): find a more intelligent way to select vertices.
        n_nodes = np.shape(self.coords)[0]
        ind = np.arange(0, n_nodes, 2)

        self.coords = self.coords[ind]
        self.G = reduction.kron_reduction(self.G, ind)
        self.G.compute_fourier_basis()
        self.adj = self.G.W.toarray()
