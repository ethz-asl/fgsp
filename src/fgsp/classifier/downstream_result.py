#! /usr/bin/env python3

from src.fgsp.classifier import ClassificationResult


class DownstreamResult(ClassificationResult):
    def __init__(self, config, robot_name, opt_nodes, features, labels, indices):
        super().__init__(config, robot_name, opt_nodes, features, labels)
        self.indices = indices
        n_nodes = len(self.opt_nodes)
        self.labels = self.create_downstream_labels(n_nodes, labels)

    def create_downstream_labels(self, n_nodes, labels):
        # Labels in the current hierarchy
        n_labels = len(labels)
        last_label = n_labels - 1

        downstream_labels = [None] * n_nodes
        for idx in range(0, last_label):
            for i in range(self.indices[idx], self.indices[idx+1]):
                downstream_labels[i] = labels[idx]

        # Fix remainder
        for i in range(self.indices[last_label], n_nodes):
            downstream_labels[i] = labels[last_label]

        return downstream_labels
