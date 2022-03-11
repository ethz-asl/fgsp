#! /usr/bin/env python2
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

    @staticmethod
    def visualize_trajectory(trajectory):
        rospy.loginfo("[GraphVisualizer] Visualizing trajectory")
        plt.plot(trajectory[:,0], trajectory[:,1])
        plt.show()

    @staticmethod
    def visualize_signal(graph, signal):
        rospy.loginfo("[GraphVisualizer] Visualizing signal on graph")
        fig, axes = plt.subplots(1, 1, figsize=(24, 8))
        graph.G.plot(signal, ax=axes)
        plt.show()
