#! /usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from maplab_msgs.msg import Submap, DenseNode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

from utils import Utils
from visualize import Visualize

class SubmapModel(object):
    def __init__(self):
        self.submap_ts = 0
        self.ts = []
        self.seq_nr = 0
        self.robot_name = ""
        self.id = 0
        self.poses = []
        self.pointclouds = []
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
        id = msg.id

        self.set_submap_information(ts, seq, robot_name, id)

    def set_submap_information(self, ts, seq_nr, robot_name, id):
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

if __name__ == "__main__":
    rospy.init_node('foo')

    submap_msg = Submap()
    submap_msg.header.stamp = rospy.get_rostime()
    submap_msg.header.seq = 0

    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.get_rostime()
    pose_msg.header.seq = 0
    pose_msg.pose.position.x = 1
    pose_msg.pose.position.y = 2
    pose_msg.pose.position.z = 3
    pose_msg.pose.orientation.w = 1
    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0
    pose_msg.pose.orientation.z = 0

    cloud_msg = PointCloud2()
    points = np.random.random_sample((3, 3))
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    cloud_msg.data = points.tostring()

    dense_node = DenseNode()
    dense_node.pose = pose_msg
    dense_node.cloud = cloud_msg
    submap_msg.nodes.append(dense_node)

    model = SubmapModel()
    model.construct_data(submap_msg)

    print(f"Model for robot {model.robot_name} contains {len(model.poses)} poses and {len(model.pointclouds)} clouds.")
