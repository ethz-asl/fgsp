#! /usr/bin/env python3

import numpy as np

from src.fgsp.common.utils import Utils
from src.fgsp.common.logger import Logger

class SignalSynchronizer(object):

    def __init__(self, config):
        self.config = config


    def synchronize(self, optimized, estimated):
        ts_opt = self.extract_timestamps(optimized)
        ts_est = self.extract_timestamps(estimated)

        opt_size = ts_opt.shape[0]
        est_size = ts_est.shape[0]
        min_size = min(opt_size, est_size)
        est_idx = []
        opt_idx = []

        if min_size != opt_size:
            Logger.LogError(f'SignalSynchronizer: min size is {min_size} and opt is {opt_size}.')

        for i in range(0, min_size):
            cur_ts = ts_opt[i,0]

            # TODO(lbern): Check for a max difference.
            ts_diff = np.absolute(ts_est - cur_ts)
            ts_min = np.amin(ts_diff)
            diff_s = Utils.ts_ns_to_seconds(ts_min)
            if diff_s > self.config.synchronization_max_diff_s:
                Logger.LogWarn(f'SignalSynchronizer: closest TS is {diff_s} away.')
                continue
            cur_min_index = np.where(ts_diff == ts_min)[0]
            est_idx.append(cur_min_index[0])
            opt_idx.append(i)

        est_nodes = [estimated[i] for i in est_idx]
        opt_nodes = [optimized[i] for i in opt_idx]

        return (opt_nodes, est_nodes, opt_idx, est_idx)

    def extract_timestamps(self, signals):
        n_nodes = len(signals)
        ts = np.zeros((n_nodes, 1))
        for i in range(0, n_nodes):
            ts[i] = Utils.ros_time_to_ns(signals[i].ts)

        return ts
