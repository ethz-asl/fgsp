#! /usr/bin/env python3

import rclpy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2 as pc2


class Utils(object):

    @staticmethod
    def ros_time_to_ns(time):
        return time.nanoseconds

    @staticmethod
    def ros_time_msg_to_ns(time):
        k_s_to_ns = 1e9
        return time.sec * k_s_to_ns + time.nanosec

    @staticmethod
    def ts_ns_to_ros_time(ts_ns):
        k_ns_per_s = 1e9
        ros_timestamp_sec = ts_ns / k_ns_per_s
        ros_timestamp_nsec = ts_ns - (ros_timestamp_sec * k_ns_per_s)
        return rclpy.time.Time(seconds=ros_timestamp_sec,
                               nanoseconds=ros_timestamp_nsec)

    @staticmethod
    def ts_ns_to_seconds(ts_ns):
        k_ns_per_s = 1e9
        return ts_ns / k_ns_per_s

    @staticmethod
    def convert_pointcloud2_msg_to_array(cloud_msg):
        points_list = []
        for data in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])
        return np.array(points_list)

    @staticmethod
    def convert_pose_stamped_msg_to_array(pose_msg):
        position = np.array(
            [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        orientation = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                               pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        return position, orientation

    @staticmethod
    def convert_quat_to_rotation(quat):
        R = Rotation.from_quat(np.array([quat[1], quat[2], quat[3], quat[0]]))
        return R.as_matrix()

    @staticmethod
    def convert_pos_quat_to_transformation(pos, quat):
        # takes xyzw as input
        R = Rotation.from_quat(np.array([quat[1], quat[2], quat[3], quat[0]]))
        T = np.empty((4, 4))
        T[0:3, 0:3] = R.as_matrix()
        T[0:3, 3] = pos
        T[3, :] = [0, 0, 0, 1]

        return T

    @staticmethod
    def convert_pointcloud2_msg_to_array(cloud_msg):
        points_list = []
        for data in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])
        return np.array(points_list)

    @staticmethod
    def transform_pointcloud(cloud, T):
        return (np.dot(cloud, T[0:3, 0:3].T) + T[0:3, 3].T)

    @staticmethod
    def downsample_pointcloud(cloud, voxel_size=0.15):
        # TODO: not yet implemented
        return cloud

    @staticmethod
    def fix_nn_output(n_neighbors, idx, nn_dists, nn_indices):
        self_idx = np.where(nn_indices == idx)[0][0]
        nn_dists = np.delete(nn_dists, [self_idx])
        nn_indices = np.delete(nn_indices, [self_idx])

        nn_indices = [nn_indices] if n_neighbors == 1 else nn_indices
        nn_dists = [nn_dists] if n_neighbors == 1 else nn_dists

        mask = np.isnan(nn_indices.astype(float))
        mask = mask | np.isnan(nn_dists.astype(float))
        mask = mask | np.isinf(nn_indices.astype(float))
        mask = mask | np.isinf(nn_dists.astype(float))
        mask = np.logical_not(mask)
        return nn_dists[mask], nn_indices[mask]
