#! /usr/bin/env python3

from src.fgsp.controller.signal_handler import SignalHandler
from src.fgsp.graph.wavelet_evaluator import WaveletEvaluator
from src.fgsp.classifier import ClassificationResult


class WindowedResult(ClassificationResult):
    def __init__(self, config, robot_name, opt_nodes, est_nodes, features, labels, graph):
        super().__init__(config, robot_name, opt_nodes, features, labels)
        self.indices = graph.get_indices()
        self.graph = graph
        self.est_nodes = est_nodes
        n_nodes = len(self.opt_nodes)
        self.labels = self.create_windowed_labels(n_nodes, labels, graph)

    def create_windowed_labels(self, n_nodes, labels, graph):
        # Labels in the current hierarchy
        n_labels = len(labels)
        last_label = n_labels - 1

        eval = WaveletEvaluator()
        signal = SignalHandler(self.config)
        x_est = signal.compute_signal(self.est_nodes)
        x_opt = signal.compute_signal(self.opt_nodes)

        downstream_labels = [None] * n_nodes
        for idx in range(0, last_label):
            print(
                f'Computing wavelets for {self.indices[idx]} to {self.indices[idx+1]}')

            eval.compute_wavelets(self.graph.get_graph(),
                                  self.indices[idx], self.indices[idx+1])
            W_est = eval.compute_wavelet_coeffs(x_est)
            W_opt = eval.compute_wavelet_coeffs(x_opt)
            features = eval.compute_features(W_opt, W_est)
            print(f'=== Features ===== \n {features}')

            # downstream_labels[i] = labels[idx]

        # Fix remainder
        # for i in range(self.indices[last_label], n_nodes):
            # downstream_labels[i] = labels[last_label]

        return downstream_labels
