#! /usr/bin/env python3

import numpy as np

from .utils import Utils

class SubmapModel(object):
    def __init__(self):
        self.submap_ts = 0
        self.ts = []
        self.seq_nr = 0
        self.robot_name = ""
        self.id = 0
        self.poses = []
        self.pointclouds = []
        self.mission_id = ""
        self.T_B_L = np.array(
            [[1, 0, 0, 0.005303],
             [0, 1, 0, 0.037340],
             [0, 0, 1, 0.063319],
             [0, 0, 0, 1]])

    def construct_data(self, submap_msg):
        n_data = len(submap_msg.nodes)

        # Parse general information
        self.parse_information(submap_msg)

        for i in range(0, n_data):
            dense_node = submap_msg.nodes[i]

            # Parse pose.
            pos, orien = Utils.convert_pose_stamped_msg_to_array(dense_node.pose)
            T_G_B = Utils.convert_pos_quat_to_transformation(pos, orien)
            self.poses.append(T_G_B)

            # Parse pointcloud
            cloud = Utils.convert_pointcloud2_msg_to_array(dense_node.cloud)
            self.pointclouds.append(cloud)

            # Parse ts.
            self.ts.append(dense_node.pose.header.stamp)

    def parse_information(self, msg):
        ts = msg.header.stamp
        seq = msg.header.seq
        robot_name = msg.robot_name
        mission_id = msg.mission_id
        id = msg.id

        self.set_submap_information(ts, seq, robot_name, mission_id, id)

    def set_submap_information(self, ts, seq_nr, robot_name, mission_id, id):
        self.submap_ts = ts
        self.seq_nr = seq_nr
        self.robot_name = robot_name
        self.id = id

    def compute_dense_map(self):
        n_poses = len(self.poses)
        if n_poses == 0:
            return

        pivot = n_poses // 2
        T_G_L_pivot = np.matmul(self.poses[pivot], self.T_B_L)
        T_L_pivot_G = np.linalg.inv(T_G_L_pivot)

        acc_points = self.pointclouds[pivot]
        for i in range(0, n_poses):
            if i == pivot:
                continue

            T_G_L = np.matmul(self.poses[i], self.T_B_L)
            T_L_pivot_L = np.matmul(T_L_pivot_G, T_G_L)

            points = Utils.transform_pointcloud(self.pointclouds[i], T_L_pivot_L)
            acc_points = np.append(acc_points, points, axis=0)

        return acc_points

    def get_pivot_pose_IMU(self):
        n_poses = len(self.poses)
        if n_poses == 0:
            return None
        return self.poses[n_poses // 2]

    def get_pivot_pose_LiDAR(self):
        T_G_B = self.get_pivot_pose_IMU()
        return np.matmul(T_G_B, self.T_B_L)

    def get_pivot_timestamp_ros(self):
        n_poses = len(self.poses)
        if n_poses == 0:
            return None
        return self.ts[n_poses // 2]