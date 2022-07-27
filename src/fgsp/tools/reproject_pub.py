#! /usr/bin/env python3

import os
import open3d as o3d
import numpy as np
import pathlib
import copy
import pickle
from os.path import exists

from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from evo.tools import file_interface
from scipy.spatial.transform import Rotation

from src.fgsp.common.utils import Utils
from src.fgsp.common.comms import Comms
from src.fgsp.common.visualizer import Visualizer

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class ReprojectPub(Node):
    def __init__(self):
        super().__init__('reproject_viz')
        self.dataroot = self.try_get_param('dataroot', './')

        # GT Publisher
        self.enable_gt = self.try_get_param('enable_gt', False)
        if self.enable_gt:
            self.create_gt_pub()

        # Est Publisher
        self.enable_est = self.try_get_param('enable_est', False)
        if self.enable_est:
            self.create_est_pub()

        # Corr Publisher
        self.enable_corr = self.try_get_param('enable_corr', False)
        if self.enable_corr:
            self.create_corr_pub()

        self.constraints_file = self.try_get_param('constraints_file', '')
        self.enable_constraints = self.constraints_file != ''
        if self.enable_constraints:
            self.create_constraints_pub()
            self.comms = Comms()
            self.comms.node = self
            self.vis_helper = Visualizer()

        print(f'Enable GT: {self.enable_gt}')
        print(f'Enable EST: {self.enable_est}')
        print(f'Enable CORR: {self.enable_corr}')
        print(f'Enable CONSTR: {self.enable_constraints}')

        # Configuration
        self.T_O_L = np.array(self.try_get_param(
            'T_O_L', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.voxel_size = 0.1

        # Cloud subscriber
        cloud_topic = self.try_get_param('cloud_in', '/cloud')
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.ts_cloud_map = {}
        print(f'Subscribed to {cloud_topic}. Waiting for clouds...')
        print('------------------------------------------------')

        # Updater
        self.timer = self.create_timer(1, self.publish_all_maps)

    def create_gt_pub(self):
        self.gt_traj = self.read_traj_file('gt_traj_file')
        if (len(self.gt_traj) == 0):
            print('Error occurred while reading the trajectory file.')
            self.enable_gt = False
            return
        print(f'Read {self.gt_traj.shape[0]} gt poses.')

        self.gt_map_pub = self.create_publisher(
            PointCloud2, 'gt_map_cloud', 10)
        self.gt_path_pub = self.create_publisher(Path, 'gt_trajectory', 10)

    def create_est_pub(self):
        self.est_traj = self.read_traj_file('est_traj_file')
        if (len(self.est_traj) == 0):
            print('Error occurred while reading the trajectory file.')
            self.enable_est = False
            return
        self.T_GT_EST = np.array(self.try_get_param(
            'T_GT_EST', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.est_traj = self.align_poses(self.est_traj, self.T_GT_EST)
        print(f'Read {self.est_traj.shape[0]} est poses.')

        self.est_map_pub = self.create_publisher(
            PointCloud2, 'est_map_cloud', 10)
        self.est_path_pub = self.create_publisher(Path, 'est_trajectory', 10)

    def create_corr_pub(self):
        self.corr_traj = self.read_traj_file('corr_traj_file')
        if (len(self.corr_traj) == 0):
            print('Error occurred while reading the trajectory file.')
            self.enable_corr = False
            return
        self.T_GT_CORR = np.array(self.try_get_param(
            'T_GT_CORR', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.corr_traj = self.align_poses(self.corr_traj, self.T_GT_CORR)
        print(f'Read {self.corr_traj.shape[0]} corr poses.')
        self.corr_map_pub = self.create_publisher(
            PointCloud2, 'corr_map_cloud', 10)
        self.corr_path_pub = self.create_publisher(Path, 'corr_trajectory', 10)

    def create_constraints_pub(self):
        self.ts_constraint_map = self.read_constraints_file(
            self.constraints_file)
        print(f'Read {len(self.ts_constraint_map.keys())} constraints.')

        self.constraint_markers = MarkerArray()
        self.constraints_pub = self.create_publisher(
            MarkerArray, 'constraints', 10)

    def create_transformation(self, pose):
        qxyzw = pose[[5, 6, 7, 4]]
        rot_mat = Rotation.from_quat(qxyzw).as_matrix().reshape([3, 3])
        transl = pose[1:4].reshape([3, 1])
        return np.vstack([np.hstack([rot_mat, transl]), np.array([0, 0, 0, 1])])

    def align_poses(self, poses, T_G_M):
        n_poses = len(poses)
        for i in range(n_poses):
            T_M_B = self.create_transformation(poses[i])
            T_G_B = np.matmul(T_G_M, T_M_B)

            poses[i, 1:4] = T_G_B[0:3, 3]
            xyzw = Rotation.from_matrix(T_G_B[0:3, 0:3]).as_quat()
            poses[i, 4:] = xyzw[[3, 0, 1, 2]]

        return poses

    def publish_all_maps(self):
        if len(self.ts_cloud_map) == 0:
            print(f'Received no clouds yet.')
            return

        if self.enable_gt:
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            gt_map, gt_idx = self.accumulate_cloud(self.gt_traj, ts_cloud_map)
            self.publish_map(self.gt_map_pub, self.gt_path_pub,
                             gt_map, self.gt_traj[0:gt_idx+1, :])
            print(f'Published gt map with {len(gt_map.points)} points.')

        if self.enable_est:
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            est_map, est_idx = self.accumulate_cloud(
                self.est_traj, ts_cloud_map)
            self.publish_map(self.est_map_pub, self.est_path_pub,
                             est_map, self.est_traj[0:est_idx+1, :])
            print(f'Published est map with {len(est_map.points)} points.')

        if self.enable_corr:
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            corr_map, corr_idx = self.accumulate_cloud(
                self.corr_traj, ts_cloud_map)
            corr_traj = self.corr_traj[0:corr_idx+1, :]
            self.publish_map(self.corr_map_pub,
                             self.corr_path_pub, corr_map, corr_traj)
            print(f'Published corr map with {len(corr_map.points)} points.')
            if self.enable_constraints:
                self.publish_constraints(corr_traj)

    def publish_constraints(self, traj):
        self.constraint_markers.markers = []
        for ts_ns, labels in self.ts_constraint_map.items():
            ctr_timestamps_s = Utils.ts_ns_to_seconds(ts_ns)
            idx = self.lookup_closest_ts_idx(traj[:, 0], ctr_timestamps_s)
            if idx == -1:
                continue

            if 1 in labels:
                self.add_constraint_at(traj, idx, idx-1)
                self.add_constraint_at(traj, idx, idx+1)

            if 2 in labels:
                self.add_constraint_at(traj, idx, idx-5)
                self.add_constraint_at(traj, idx, idx+5)

        self.constraints_pub.publish(self.constraint_markers)

    def add_constraint_at(self, traj, idx_a, idx_b):
        n_poses = traj.shape[0]
        if (idx_a < 0 or idx_a >= n_poses):
            return
        if (idx_b < 0 or idx_b >= n_poses):
            return
        points = [traj[idx_a, 0:3], traj[idx_b, 0:3]]
        line_marker = self.vis_helper.create_line_marker(points)
        self.constraint_markers.markers.append(line_marker)

    def publish_map(self, map_pub, path_pub, map, traj):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        map = map.voxel_down_sample(voxel_size=self.voxel_size)
        map_ros = point_cloud2.create_cloud(
            header, FIELDS_XYZ, map.points)
        map_pub.publish(map_ros)

        path_msg = self.create_path_msg(traj)
        path_pub.publish(path_msg)

    def create_path_msg(self, trajectory):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        n_poses = trajectory.shape[0]
        for i in range(n_poses):
            t = trajectory[i, 1:4]
            q = trajectory[i, 4:8]

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
        ts_s = Utils.ros_time_msg_to_s(cloud_msg.header.stamp)
        cloud = self.parse_cloud(cloud_msg)
        self.ts_cloud_map[ts_s] = cloud

    def accumulate_cloud(self, trajectory, ts_cloud_map):
        map = o3d.geometry.PointCloud()
        idx = -1
        for ts_s, cloud in ts_cloud_map.items():
            idx = self.lookup_closest_ts_idx(trajectory[:, 0], ts_s)
            if idx < 0:
                continue

            T_M_L = self.transform_pose_to_sensor_frame(trajectory[idx, :])
            cloud = cloud.transform(T_M_L)
            map += cloud

        return map, idx

    def parse_cloud(self, cloud_msg: PointCloud2):
        cloud = point_cloud2.read_points_numpy(
            cloud_msg, field_names=['x', 'y', 'z'], skip_nans=True, reshape_organized_cloud=True)
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud))
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd

    def lookup_closest_ts_idx(self, timestamps_s, ts_s):
        diff_timestamps = np.abs(timestamps_s - ts_s)
        minval = np.amin(diff_timestamps)
        if (minval > 1e-4):
            print(f'Timestamp {ts_s} is not available (min diff: {minval}).')
            return -1

        return np.where(diff_timestamps == minval)[0][0]

    def transform_pose_to_sensor_frame(self, pose):
        qxyzw = pose[[5, 6, 7, 4]]
        rot_mat = Rotation.from_quat(qxyzw).as_matrix().reshape([3, 3])
        transl = pose[1:4].reshape([3, 1])
        T_M_O = np.vstack(
            [np.hstack([rot_mat, transl]), np.array([0, 0, 0, 1])])
        T_M_L = np.matmul(T_M_O, self.T_O_L)

        return T_M_L

    def try_get_param(self, key, default=None):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def read_traj_file(self, traj_key):
        traj_file_path = self.try_get_param(traj_key, '')
        traj_full_path = os.path.join(self.dataroot, traj_file_path)
        ext = pathlib.Path(traj_file_path).suffix
        if ext == '.csv':
            print(f'Reading CSV file: {traj_full_path}')
            return self.read_csv_file(traj_full_path)
        elif ext == '.npy':
            print(f'Reading NPY file: {traj_full_path}')
            return self.read_npy_file(traj_full_path)
        print(f'Unknown file extension: {ext}')
        return np.array([])

    def read_csv_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        print(f'ReprojectPub: Reading file {filename}.')
        if exists(filename):
            robot_traj = file_interface.read_tum_trajectory_file(filename)
            return np.column_stack((robot_traj.timestamps, robot_traj.positions_xyz,
                                    robot_traj.orientations_quat_wxyz))

        else:
            print(f'ReprojectPub: File does not exist!')
            return np.array([])

    def read_npy_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        print(f'ReprojectPub: Reading file {filename}.')
        if exists(filename):
            return self.convert_to_traj(np.load(filename))
        else:
            print(f'ReprojectPub: File does not exist!')
            return np.array([])

    def read_constraints_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        print(f'ReprojectPub: Reading constraints file {filename}.')
        dict = {}
        if exists(filename):
            input_file = open(filename, 'rb')
            dict = pickle.load(input_file)
            input_file.close()
        else:
            print(f'ReprojectPub: Constraints file does not exist!')
        return dict


def main(args=None):
    rclpy.init(args=args)
    pub = ReprojectPub()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
