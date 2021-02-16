#! /usr/bin/env python3

import rospy
import numpy as np

class SignalSynchronizer(object):

    def __init__(self):
        rospy.loginfo("[SignalSynchronizer] Initialized")

    def syncrhonize(self, optimized, estimated):
        ts_opt = self.extract_timestamps(optimized)
        ts_est = self.extract_timestamps(estimated)

        opt_size = ts_opt.shape[0]
        est_idx = []

        for i in range(0, opt_size):
            cur_ts = ts_opt[i,0]

            # TODO(lbern): Check for a max difference.
            ts_diff = np.absolute(ts_est - cur_ts)
            cur_min_index = np.where(ts_diff == np.amin(ts_diff))[0]
            est_idx.append(cur_min_index[0])

        return [estimated[i] for i in est_idx]


    def extract_timestamps(self, signals):
        n_nodes = len(signals)
        ts = np.zeros((n_nodes, 1))
        for i in range(0, n_nodes):
            ts[i] = self.ros_time_to_ns(signals[i].ts)

        return ts

    def ros_time_to_ns(self, time):
        k_s_to_ns = 1e9
        return time.secs * k_s_to_ns + time.nsecs
