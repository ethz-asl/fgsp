#! /usr/bin/env python3

import numpy as np

from src.fgsp.common.logger import Logger


class SimpleClassifier(object):

    def classify(self, data):
        n_nodes = data.shape[0]
        labels = []
        for i in range(0, n_nodes):
            low_mean = data[i, 0]
            mid_mean = data[i, 1]
            high_mean = data[i, 2]
            dists = np.array([low_mean, mid_mean, high_mean])

            np.set_printoptions(suppress=True)
            local_labels = []

            if dists[0] > 0.005:
                local_labels.append(1)
            if dists[1] > 0.002:
                local_labels.append(2)
            if dists[2] > 0.001:
                local_labels.append(3)

            Logger.LogDebug(f'WaveletEvaluator: dists are {dists}')
            Logger.LogDebug(
                f'WaveletEvaluator: Local labels are {local_labels}')
            labels.append(local_labels)
        return labels
