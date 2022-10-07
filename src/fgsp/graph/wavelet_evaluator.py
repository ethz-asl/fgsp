#! /usr/bin/env python3

from platform import node
import numpy as np
from pygsp import graphs, filters, reduction
from enum import Enum

import pandas
import scipy.spatial


from src.fgsp.common.logger import Logger


class SubmapState(Enum):
    ALL_GOOD = 1
    LOW_GOOD = 2
    HIGH_GOOD = 3
    NO_GOOD = 4


class WaveletEvaluator(object):

    def __init__(self, n_scales=6):
        assert n_scales >= 3, 'n_scales must be at least 3.'
        self.n_scales = n_scales
        self.psi = None
        self.G = None
        self.feature_names = ['Euclidean_L', 'Euclidean_B', 'Euclidean_H', 'Correlation_L', 'Correlation_B',
                              'Correlation_H', 'Manhattan_L', 'Manhattan_B', 'Manhattan_H', 'Chebyshev_L', 'Chebyshev_B', 'Chebyshev_H']
        self.ranges = self.compute_constraint_ranges()
        Logger.LogInfo(
            f'WaveletEvaluator: Created with ranges: {self.ranges}.')

    def set_scales(self, n_scales):
        self.n_scales = n_scales

    def compute_constraint_ranges(self):
        step_size = self.n_scales // 3
        ranges = np.arange(0, self.n_scales, step_size)
        low_scale_range = np.arange(ranges[0], ranges[1], dtype=int)
        mid_scale_range = np.arange(ranges[1], ranges[2], dtype=int)
        large_scale_range = np.arange(ranges[2], self.n_scales, dtype=int)
        return [low_scale_range, mid_scale_range, large_scale_range]

    def get_wavelets(self):
        return self.psi

    def compute_wavelets(self, G, node_range=None):
        Logger.LogInfo(
            f'WaveletEvaluator: Computing wavelets for {self.n_scales} scales.')
        g = filters.Meyer(G, self.n_scales)

        if node_range is None:
            node_range = np.arange(0, G.N)
        else:
            Logger.LogWarn(
                f'WaveletEvaluator: Computing wavelets for nodes {node_range}.')
        n = G.N

        # Evalute filter bank on the frequencies (eigenvalues).
        f = g.evaluate(G.e)
        f = np.expand_dims(f.T, 1)
        self.psi = np.zeros((n, n, self.n_scales))
        self.G = G

        for i in node_range:
            # Create a Dirac centered at node i.
            x = np.zeros((n, 1))
            x[i] = 1

            # Transform the signal to spectral domain.
            s = G.gft(x)

            # Multiply the transformed signal with filter.
            if s.ndim == 1:
                s = np.expand_dims(s, -1)
            s = np.expand_dims(s, 1)
            s = np.matmul(s, f)  # [nodes, features, scales]

            # Transform back the features to the vertex domain.
            self.psi[i, :, :] = G.igft(s).squeeze()

        return self.psi

    def compute_wavelet_coeffs(self, x_signal):
        return self.compute_wavelet_coeffs_using_wavelet(self.psi, x_signal)

    def compute_wavelet_coeffs_using_wavelet(self, wavelet, x_signal):
        n_values = x_signal.shape[0]
        n_dim = x_signal.shape[1] if len(x_signal.shape) >= 2 else 1
        W = np.zeros((n_values, self.n_scales, n_dim)).squeeze()

        for i in range(0, n_values):
            for j in range(0, self.n_scales):
                W[i, j] = np.matmul(wavelet[i, :, j].transpose(), x_signal)

        return W if n_dim == 1 else np.mean(W, axis=2)

    def compute_distances_1D(self, coeffs_1, coeffs_2):
        distances = np.zeros((1, self.n_scales))
        for j in range(0, self.n_scales):
            distances[0, j] = scipy.spatial.distance.euclidean(
                coeffs_1[j], coeffs_2[j])

        return distances

    def compute_features(self, submap_coeffs_1, submap_coeffs_2):
        n_nodes = submap_coeffs_1.shape[0]
        all_data = pandas.DataFrame()
        for i in range(n_nodes):
            D = self.compute_distances_1D(
                submap_coeffs_1[i, :], submap_coeffs_2[i, :])
            data = pandas.DataFrame({
                # self.feature_names[0]: [np.sum(D[0, 0:2])],
                # self.feature_names[1]: [np.sum(D[0, 2:4])],
                # self.feature_names[2]: [np.sum(D[0, 4:6])],

                # self.feature_names[0]: [np.sum(D[0, 0:3])],
                # self.feature_names[1]: [np.sum(D[0, 3:6])],
                # self.feature_names[2]: [np.sum(D[0, 6:9])],

                # self.feature_names[0]: [np.sum(D[0, 0:4])],
                # self.feature_names[1]: [np.sum(D[0, 4:8])],
                # self.feature_names[2]: [np.sum(D[0, 8:12])],

                self.feature_names[0]: [np.sum(D[0, self.ranges[0]])],
                self.feature_names[1]: [np.sum(D[0, self.ranges[1]])],
                self.feature_names[2]: [np.sum(D[0, self.ranges[2]])],

                # self.feature_names[0]: [np.sum(D[0, 0])],
                # self.feature_names[1]: [np.sum(D[0, 1])],
                # self.feature_names[2]: [np.sum(D[0, 2])],
            })
            all_data = pandas.concat([all_data, data])
        return np.nan_to_num(all_data)


if __name__ == '__main__':
    print(" --- Test Driver for the Wavelet Evaluator ----------------------")
    eval = WaveletEvaluator()

    # Create a reduced graph for quicker tests.
    G = graphs.Bunny()
    ind = np.arange(0, G.N, 10)
    Gs = reduction.kron_reduction(G, ind)
    Gs.compute_fourier_basis()

    # Compute wavelets.
    psi = eval.compute_wavelets(Gs)

    # Compute wavelet coefficients for two signals.
    x_1 = Gs.coords
    x_1 = np.linalg.norm(x_1, ord=2, axis=1)
    x_2 = Gs.coords + 10
    x_2 = np.linalg.norm(x_2, ord=2, axis=1)

    W_1 = eval.compute_wavelet_coeffs_using_wavelet(psi, x_1)
    W_2 = eval.compute_wavelet_coeffs_using_wavelet(psi, x_2)
    features = eval.compute_features(W_1, W_2)
    label = eval.classify_simple(features)
