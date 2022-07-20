#! /usr/bin/env python3

import open3d as o3d
import numpy as np
import pathlib
from os.path import exists

from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class ReprojectPub(Node):
    def __init__(self):
        super().__init__('reproject_viz')
        self.gt_traj = self.read_traj_file('traj_file')
        if (len(self.gt_traj) == 0):
            print('Error occurred while reading the trajectory file.')
            return
        print(f'Read {self.gt_traj.shape[0]} poses.')

        self.T_O_L = np.array(self.try_get_param(
            'T_O_L', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)

        self.map = o3d.geometry.PointCloud()
        self.voxel_size = 0.1
        self.map_pub = self.create_publisher(PointCloud2, 'map_cloud', 10)
        self.path_pub = self.create_publisher(Path, 'trajectory', 10)
        self.latest_idx = -1

        cloud_topic = self.try_get_param('cloud_in', '/cloud')
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.cloud_counter = 0
        print(f'Subscribed to {cloud_topic}.')
        print('------------------------------------------------')
        self.timer = self.create_timer(5, self.publish_map)

    def publish_map(self):
        n_points = len(self.map.points)
        if n_points <= 0:
            return

        print(f'Publishing cloud msg with {n_points} points.')
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        map = self.map.voxel_down_sample(voxel_size=self.voxel_size)
        map_ros = point_cloud2.create_cloud(
            header, FIELDS_XYZ, map.points)
        self.map_pub.publish(map_ros)

        if self.latest_idx > 0:
            print(f'Publishing path msg up to index {self.latest_idx}.')
            path_msg = self.create_path_up_to_idx(self.latest_idx)
            self.path_pub.publish(path_msg)

    def create_path_up_to_idx(self, idx):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for i in range(idx):
            ts_s = self.gt_traj[i, 0]
            t = self.gt_traj[i, 1:4]
            q = self.gt_traj[i, 4:8]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = t[0]
            pose_msg.pose.position.y = t[1]
            pose_msg.pose.position.z = t[2]
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]
            pose_msg.pose.orientation.w = q[0]

            path_msg.poses.append(pose_msg)

        return path_msg

    def cloud_callback(self, cloud_msg):
        self.cloud_counter += 1
        if self.cloud_counter % 2 != 0:
            return

        ts_s = self.ros_time_msg_to_s(cloud_msg.header.stamp)
        pose = self.lookup_closest_pose(ts_s)
        if (len(pose) == 0):
            return
        pose, T_M_L = self.transform_pose_to_sensor_frame(pose)
        cloud = self.parse_cloud(cloud_msg)

        cloud.transform(T_M_L)
        self.accumulate_cloud(cloud)

    def accumulate_cloud(self, pcd):
        self.map += pcd

    def parse_cloud(self, cloud_msg: PointCloud2):
        cloud = point_cloud2.read_points_numpy(
            cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True, reshape_organized_cloud=True)
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud))
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd

    def lookup_closest_pose(self, ts_s):
        timestamps = np.abs(self.gt_traj[:, 0] - ts_s)
        minval = np.amin(timestamps)
        if (minval > 1e-4):
            print(f'Timestamp {ts_s} is not available (min diff: {minval}).')
            return np.array([])

        self.latest_idx = np.where(timestamps == minval)[0][0]
        return self.gt_traj[self.latest_idx, :]

    def transform_pose_to_sensor_frame(self, pose):
        qxyzw = pose[[5, 6, 7, 4]]
        rot_mat = Rotation.from_quat(qxyzw).as_matrix().reshape([3, 3])
        transl = pose[1:4].reshape([3, 1])
        T_M_O = np.vstack(
            [np.hstack([rot_mat, transl]), np.array([0, 0, 0, 1])])
        T_M_L = np.matmul(T_M_O, self.T_O_L)

        result = np.zeros_like(pose)
        result[0] = pose[0]
        result[1:4] = T_M_L[0:3, 3].reshape([1, 3])
        result[4:] = Rotation.from_matrix(T_M_L[0:3, 0:3]).as_quat()
        result[4:] = result[[7, 4, 5, 6]]

        return result, T_M_L

    def try_get_param(self, key, default=None):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def read_traj_file(self, traj_key):
        traj_file_path = self.try_get_param(traj_key, '')
        ext = pathlib.Path(traj_file_path).suffix
        if ext == '.csv':
            print(f'Reading CSV file: {traj_file_path}')
            return self.read_csv_file(traj_file_path)
        elif ext == '.npy':
            print(f'Reading NPY file: {traj_file_path}')
            return self.read_npy_file(traj_file_path)
        print(f'Unknown file extension: {ext}')
        return np.array([])

    def read_csv_file(self, filename):
        print(f'Simulation: Reading file {filename}.')
        if exists(filename):
            robot_traj = file_interface.read_tum_trajectory_file(filename)
            return np.column_stack((robot_traj.timestamps, robot_traj.positions_xyz,
                                    robot_traj.orientations_quat_wxyz))

        else:
            print(f'Simulation: File does not exist!')
            return np.array([])

    def read_npy_file(self, filename):
        print(f'Simulation: Reading file {filename}.')
        if exists(filename):
            return self.convert_to_traj(np.load(filename))
        else:
            print(f'Simulation: File does not exist!')
            return np.array([])

    def ros_time_msg_to_s(self, time):
        k_s_to_ns = 1e9
        return (time.sec * k_s_to_ns + time.nanosec) / k_s_to_ns

    def ts_ns_to_ros_time(self, ts_ns):
        k_ns_per_s = 1e9
        ros_timestamp_sec = ts_ns / k_ns_per_s
        ros_timestamp_nsec = ts_ns - (ros_timestamp_sec * k_ns_per_s)
        return rclpy.time.Time(seconds=ros_timestamp_sec,
                               nanoseconds=ros_timestamp_nsec)

    def ts_sec_to_ns(self, ts_s):
        k_ns_per_s = 1e9
        return ts_s * k_ns_per_s


def main(args=None):
    rclpy.init(args=args)
    pub = ReprojectPub()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
