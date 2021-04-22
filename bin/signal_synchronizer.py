#! /usr/bin/env python3

import rospy
import numpy as np

from utils import Utils

class SignalSynchronizer(object):

    def __init__(self):
        rospy.loginfo("[SignalSynchronizer] Initialized")

    def synchronize(self, optimized, estimated):
        rospy.loginfo(f"[SignalSynchronizer] Computing optimized {len(optimized)}")
        ts_opt = self.extract_timestamps(optimized)
        rospy.loginfo(f"[SignalSynchronizer] Computing estimated {len(estimated)}")
        ts_est = self.extract_timestamps(estimated)

        opt_size = ts_opt.shape[0]
        est_size = ts_est.shape[0]
        min_size = min(opt_size, est_size)
        est_idx = []
        opt_idx = []

        for i in range(0, min_size):
            cur_ts = ts_opt[i,0]

            # TODO(lbern): Check for a max difference.
            ts_diff = np.absolute(ts_est - cur_ts)
            cur_min_index = np.where(ts_diff == np.amin(ts_diff))[0]
            est_idx.append(cur_min_index[0])
            opt_idx.append(i)

        est_nodes = [estimated[i] for i in est_idx]
        opt_nodes = [optimized[i] for i in opt_idx]

        return (opt_nodes, est_nodes)


    def extract_timestamps(self, signals):
        n_nodes = len(signals)
        ts = np.zeros((n_nodes, 1))
        for i in range(0, n_nodes):
            ts[i] = Utils.ros_time_to_ns(signals[i].ts)

        return ts
