#! /usr/bin/env python3

import numpy as np
from src.fgsp.controller.signal_handler import SignalHandler
from src.fgsp.graph.wavelet_evaluator import WaveletEvaluator
from src.fgsp.classifier import ClassificationResult


class WindowedResult(ClassificationResult):
    def __init__(self, config, robot_name, opt_nodes, est_nodes, features, labels, graph):
        super().__init__(config, robot_name, opt_nodes, features, labels)
        assert config.use_graph_hierarchies
        assert not config.use_downstreaming

        self.indices = graph.get_indices()
        self.graph = graph
        self.est_nodes = est_nodes
        n_nodes = len(self.opt_nodes)
        self.labels = self.create_windowed_labels(n_nodes, labels)

    def create_windowed_labels(self, n_nodes, labels):
        # Labels in the current hierarchy
        n_labels = len(labels)
        last_label = n_labels - 1

        signal = SignalHandler(self.config)
        x_est = signal.compute_signal(self.est_nodes)
        x_opt = signal.compute_signal(self.opt_nodes)

        downstream_labels = [None] * n_nodes
        for idx in range(0, last_label):
            if len(labels[idx]) == 0:
                continue

            # Compute the windowed wavelets in the initial graph.
            node_range = np.arange(
                self.indices[idx], self.indices[idx+1], dtype=int)
            features = self.evaluate_node_range(node_range, x_est, x_opt)

            for lbl in labels[idx]:
                max_n = node_range[np.argmax(features[node_range, lbl-1])]
                if downstream_labels[max_n] is None:
                    downstream_labels[max_n] = [lbl]
                else:
                    downstream_labels[max_n].append(lbl)

        # Fix remainder
        node_range = np.arange(self.indices[last_label], n_nodes, dtype=int)
        features = self.evaluate_node_range(node_range, x_est, x_opt)
        for lbl in labels[last_label]:
            max_n = node_range[np.argmax(features[node_range, lbl-1])]
            if downstream_labels[max_n] is None:
                downstream_labels[max_n] = [lbl]
            else:
                downstream_labels[max_n].append(lbl)

        downstream_labels = [lbl if lbl is not None else []
                             for lbl in downstream_labels]
        return downstream_labels

    def evaluate_node_range(self, node_range, x_est, x_opt):
        print(f'Computing wavelets for {node_range[0]} to {node_range[-1]}')

        # Compute the windowed wavelets in the initial graph.
        eval = WaveletEvaluator()
        eval.compute_wavelets(self.graph.get_graph(0), node_range)

        # Compute the windowed features for the windowed wavelets.
        W_est = eval.compute_wavelet_coeffs(x_est)
        W_opt = eval.compute_wavelet_coeffs(x_opt)
        return eval.compute_features(W_opt, W_est)
