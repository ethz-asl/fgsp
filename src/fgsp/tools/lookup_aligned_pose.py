#! /usr/bin/env python3

import numpy as np
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from evo.tools import file_interface
from evo.core import sync


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
        if (align_file == '' or not os.path.exists(align_file)):
            Logger.LogError('LookupAlignedPose: Invalid align file')

        lookup_ts_ns = self.get_param('timestamp_ns', 0)
        if (lookup_ts_ns == 0):
            Logger.LogError('LookupAlignedPose: Invalid timestamp')

        Logger.LogInfo(
            'LookupAlingedPose: Initializing done. Processing data...')
        input_traj = self.synchronize_and_align(input_file, align_file)
        pose = self.lookup_aligned_pose(input_traj, lookup_ts_ns)
        self.print_pose(pose)

    def print_pose(self, pose):
        Logger.LogInfo(f'LookupAlignedPose: Pose: {pose}')

    def lookup_aligned_pose(self, input_traj, lookup_ts_ns):
        k_ns_per_s = 1e9
        lookup_ts_s = lookup_ts_ns / k_ns_per_s
        idx = self.lookup_closest_ts_idx(input_traj.timestamps, lookup_ts_s)
        if idx < 0:
            return None
        return input_traj.poses[idx]

    def lookup_closest_ts_idx(self, timestamps_s, ts_s, eps=1e-4):
        diff_timestamps = np.abs(timestamps_s - ts_s)
        minval = np.amin(diff_timestamps)
        if (minval > eps):
            return -1

        return np.where(diff_timestamps == minval)[0][0]

    def synchronize_and_align(self, input_file, align_file):
        input_traj = self.read_csv_file(input_file)
        align_traj = self.read_csv_file(align_file)

        print(
            f'Found {input_traj.num_poses} and {align_traj.num_poses} poses for input and align file, respectively.')

        align_traj, input_traj = sync.associate_trajectories(
            align_traj, input_traj, max_diff=0.1)

        input_traj.align(align_traj, correct_scale=False,
                         correct_only_scale=False, n=-1)

        print(f'Aligned {input_traj.num_poses} synchronized poses.')
        return input_traj

    def read_csv_file(self, filename):
        if os.path.exists(filename):
            return file_interface.read_tum_trajectory_file(filename)
        else:
            Logger.LogError(f'LookupAlignedPose: File does not exist!')
            return np.array([])

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def cloud_callback(self, msg):
        cloud = point_cloud2.read_points_numpy(
            msg, field_names=self.field_names, skip_nans=True, reshape_organized_cloud=False)
        n_dims = len(self.field_names)
        cloud = np.reshape(cloud, (-1, n_dims))

        export_path = self.export_path
        if self.should_store_sequentially:
            export_path = self.export_path.format(n_clouds=self.n_clouds)
            self.n_clouds += 1
        np.save(export_path, cloud)
        Logger.LogInfo(f'CloudSaver: Saved cloud to {export_path}')


def main(args=None):
    rclpy.init(args=args)
    lookup = LookupAlignedPose()
    rclpy.spin(lookup)
    lookup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
