#! /usr/bin/env python3

import numpy as np
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from evo.tools import file_interface
from evo.core import sync
from evo.core import metrics
from evo.tools import plot
import matplotlib.pyplot as plt


from src.fgsp.common.logger import Logger


class LookupAlignedPose(Node):
    def __init__(self):
        super().__init__('lookup_aligned_pose')
        Logger.Verbosity = 5
        Logger.LogInfo('LookupAlingedPose: Initializing...')

        input_file = self.get_param('input_file', '')
        align_file = self.get_param('align_to_file', '')

        if (input_file == '' or not os.path.exists(input_file)):
            Logger.LogError('LookupAlignedPose: Invalid input file')
            return

        should_align = align_file == '' or not os.path.exists(align_file)
        if (should_align):
            Logger.LogWarn(
                'LookupAlignedPose: No alignement file provided or file does not exist.')

        lookup_ts_ns = self.get_param('timestamp_ns', 0)
        if (lookup_ts_ns == 0):
            Logger.LogError('LookupAlignedPose: Invalid timestamp')

        Logger.LogInfo(
            'LookupAlingedPose: Initializing done. Processing data...')
        input_traj = self.synchronize_and_align(
            input_file, align_file, should_align)
        pose = self.lookup_aligned_pose(input_traj, lookup_ts_ns)
        if pose is None:
            Logger.LogError('LookupAlignedPose: Could not find matching pose.')
        else:
            self.print_pose(pose)

    def print_pose(self, pose):
        Logger.LogInfo(f'LookupAlignedPose: Pose:')
        Logger.LogInfo(f'LookupAlignedPose: xyz: {pose[1:4]}')
        Logger.LogInfo(f'LookupAlignedPose: wxyz: {pose[4:8]}')

    def lookup_aligned_pose(self, input_traj, lookup_ts_ns):
        k_ns_per_s = 1e9
        lookup_ts_s = lookup_ts_ns / k_ns_per_s
        idx = self.lookup_closest_ts_idx(input_traj.timestamps, lookup_ts_s)
        if idx < 0:
            return None

        return np.array((input_traj.timestamps[idx], *input_traj.positions_xyz[idx, :],
                         *input_traj.orientations_quat_wxyz[idx, :]))

    def lookup_closest_ts_idx(self, timestamps_s, ts_s, eps=1e-2):
        print(f'Looking for closest timestamp to {ts_s}...')
        print(f'first then ts {timestamps_s[0]}')
        diff_timestamps = np.abs(timestamps_s - ts_s)
        minval = np.amin(diff_timestamps)
        if (minval > eps):
            return -1

        return np.where(diff_timestamps == minval)[0][0]

    def synchronize_and_align(self, input_file, align_file, should_align):
        input_traj = self.read_csv_file(input_file)
        if should_align:
            return input_traj

        align_traj = self.read_csv_file(align_file)

        print(
            f'Found {input_traj.num_poses} and {align_traj.num_poses} poses for input and align file, respectively.')

        align_traj, input_traj = sync.associate_trajectories(
            align_traj, input_traj, max_diff=0.1)

        alignment = input_traj.align(align_traj, correct_scale=False,
                                     correct_only_scale=False, n=1000)
        print('--- Alignement -----------------')
        print('Rotation:')
        print(alignment[0])
        print('Translation:')
        print(alignment[1])
        print('Scale:')
        print(alignment[2])

        # self.evaluate(align_traj, input_traj, True)

        print(f'Aligned {input_traj.num_poses} synchronized poses.')
        return input_traj

    def perform_evaluation(self, data):
        pose_relation = metrics.PoseRelation.translation_part
        ape_metric_trans = metrics.APE(pose_relation)
        ape_metric_trans.process_data(data)
        ape_stats_trans = ape_metric_trans.get_all_statistics()
        print('--- APE ---------------------------')
        print(ape_stats_trans)
        return ape_stats_trans, ape_metric_trans

    def plot_trajectories(self, data, ape_metric, ape_stats):
        fig = plt.figure(figsize=(8, 6), dpi=160)
        traj_by_label = {
            "estimate": data[1],
            "reference": data[0]
        }
        plot.trajectories(fig, traj_by_label, plot.PlotMode.xy)
        plt.show()

    def evaluate(self, gt_traj, est_traj, should_plot):
        data = (gt_traj, est_traj)
        stats, metric = self.perform_evaluation(data)
        if should_plot:
            self.plot_trajectories(data, metric, stats)

    def read_csv_file(self, filename):
        Logger.LogDebug(f'LookupAlignedPose: Reading file {filename}')
        if os.path.exists(filename):
            return file_interface.read_tum_trajectory_file(filename)
        else:
            Logger.LogError(f'LookupAlignedPose: File does not exist!')
            return np.array([])

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value


def main(args=None):
    rclpy.init(args=args)
    lookup = LookupAlignedPose()
    lookup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
