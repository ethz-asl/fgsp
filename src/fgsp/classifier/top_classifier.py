#! /usr/bin/env python3

import numpy as np
from src.fgsp.common.logger import Logger


class TopClassifier(object):

    def __init__(self, config):
        self.top_n = config.top_classifier_select_n
        self.threshold = config.top_classifier_min_threshold

    def classify(self, data):
        n_nodes = data.shape[0]
        labels = [None] * n_nodes

        Logger.LogDebug(f'TopClassifier: data shape is {data.shape}')
        Logger.LogDebug('--- DATA ---------------------------------')
        Logger.LogDebug(data)
        Logger.LogDebug('------------------------------------------')

        top_n = min(self.top_n, n_nodes)

        # Find top n entries in the data.
        xy_indices = np.unravel_index(np.argsort(
            data.ravel())[-top_n:], data.shape)
        Logger.LogDebug('--- TOP N ---------------------------------')
        for i in range(0, top_n):
            row_idx = xy_indices[0][i]
            col_idx = xy_indices[1][i]

            if data[row_idx, col_idx] < self.threshold:
                Logger.LogDebug(
                    f'skip {data[row_idx, col_idx]} < {self.threshold}')
                continue
            else:
                Logger.LogDebug(data[row_idx, col_idx])

            if labels[row_idx] == None:
                labels[row_idx] = []
            labels[row_idx].append(col_idx + 1)
        Logger.LogDebug('------------------------------------------')

        Logger.LogDebug('--- LABELS ---------------------------------')
        Logger.LogDebug(labels)
        Logger.LogDebug('------------------------------------------')

        return labels


if __name__ == '__main__':
    classifier = TopClassifier(3)

    data = np.random.random((5, 3))
    print('Classifying the following data:')
    print(data)
    print('\n')

    labels = classifier.classify(data)

    print('Got the following labels:')
    print(labels)
