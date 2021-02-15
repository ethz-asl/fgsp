#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from pygsp import graphs, plotting

plotting.BACKEND = 'matplotlib'

class GraphVisualizer(object):

    @staticmethod
    def visualize_adjacency(graph):
        rospy.loginfo("[GraphVisualizer] Visualizing adjacency")
        adj_plot = plt.imshow(np.real(graph.adj))
        plt.show()

    @staticmethod
    def visualize_graph(graph):
        rospy.loginfo("[GraphVisualizer] Visualizing graph")
        graph.G.plot()
        plt.show()
