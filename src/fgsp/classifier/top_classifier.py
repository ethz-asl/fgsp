#! /usr/bin/env python3

import numpy as np


class TopClassifier(object):

    def __init__(self, top_n=5):
        self.top_n = top_n
        self.threshold = 0.2

    def classify(self, data):
        n_nodes = data.shape[0]
        labels = [None] * n_nodes

        np.set_printoptions(suppress=True)
        print('--- DATA ---------------------------------')
        print(data)
        print('------------------------------------------')

        top_n = min(self.top_n, n_nodes)

        # Find top n entries in the data.
        xy_indices = np.unravel_index(np.argsort(
            data.ravel())[-top_n:], data.shape)
        for i in range(0, top_n):
            row_idx = xy_indices[0][i]
            col_idx = xy_indices[1][i]

            if data[row_idx, col_idx] < self.threshold:
                continue

            if labels[row_idx] == None:
                labels[row_idx] = []
            labels[row_idx].append(col_idx + 1)

        print('--- LABELS ---------------------------------')
        print(labels)
        print('------------------------------------------')

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
