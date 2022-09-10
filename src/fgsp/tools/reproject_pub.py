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
from scipy import interpolate

from src.fgsp.common.utils import Utils
from src.fgsp.common.comms import Comms
from src.fgsp.common.logger import Logger
from src.fgsp.common.visualizer import Visualizer
from src.fgsp.common.transform_history import ConstraintType

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class ReprojectPub(Node):
    def __init__(self):
        super().__init__('reproject_viz')
        Logger.Verbosity = 7
        self.dataroot = self.try_get_param('dataroot', './')

        # Configuration
        self.T_O_L = np.array(self.try_get_param(
            'T_O_L', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.voxel_size = 0.1
        self.vis_helper = Visualizer()

        # GT Publisher
        self.enable_gt = self.try_get_param('enable_gt', False)
        if self.enable_gt:
            self.create_gt_pub()

        # Est Publisher
        self.enable_est = self.try_get_param('enable_est', False)
        if self.enable_est:
            self.create_est_pub()

        self.enable_graph = self.try_get_param('enable_graph', False)
        if self.enable_graph:
            self.create_graph_pub()

        # Corr Publisher
        self.enable_corr = self.try_get_param('enable_corr', False)
        if self.enable_corr:
            self.create_corr_pub()

        self.constraints_file = self.try_get_param('constraints_file', '')
        self.connections_file = self.try_get_param('connections_file', '')
        self.enable_constraints = self.constraints_file != '' and self.connections_file != ''
        if self.enable_constraints:
            self.create_constraints_pub()
            self.comms = Comms()
            self.comms.node = self
            self.compute_splines = self.try_get_param('compute_splines', False)
            if self.compute_splines:
                self.spline_points = self.try_get_param('spline_points', 10)
                self.spline_low_height = self.try_get_param(
                    'spline_low_height', 1)
                self.spline_mid_height = self.try_get_param(
                    'spline_mid_height', 5)
                self.spline_large_height = self.try_get_param(
                    'spline_large_height', 10)

        Logger.LogDebug(f'Enable GT: {self.enable_gt}')
        Logger.LogDebug(f'Enable EST: {self.enable_est}')
        Logger.LogDebug(f'Enable CORR: {self.enable_corr}')
        Logger.LogDebug(f'Enable CONSTR: {self.enable_constraints}')

        # Cloud subscriber
        cloud_topic = self.try_get_param('cloud_in', '/cloud')
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.ts_cloud_map = {}

        Logger.LogInfo(f'Subscribed to {cloud_topic}. Waiting for clouds...')
        Logger.LogInfo('------------------------------------------------')

        # Updater
        self.timer = self.create_timer(1, self.publish_all_maps)

    def create_gt_pub(self):
        self.gt_traj = self.read_traj_file('gt_traj_file')
        if (len(self.gt_traj) == 0):
            Logger.LogError(
                'Error occurred while reading the trajectory file.')
            self.enable_gt = False
            return
        Logger.LogInfo(f'Read {self.gt_traj.shape[0]} gt poses.')

        self.gt_map_pub = self.create_publisher(
            PointCloud2, 'gt_map_cloud', 10)
        self.gt_path_pub = self.create_publisher(Path, 'gt_trajectory', 10)

    def create_est_pub(self):
        self.est_traj = self.read_traj_file('est_traj_file')
        if (len(self.est_traj) == 0):
            Logger.LogError(
                'Error occurred while reading the trajectory file.')
            self.enable_est = False
            return
        self.T_GT_EST = np.array(self.try_get_param(
            'T_GT_EST', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.est_traj = self.align_poses(self.est_traj, self.T_GT_EST)
        Logger.LogInfo(f'Read {self.est_traj.shape[0]} est poses.')

        self.est_map_pub = self.create_publisher(
            PointCloud2, 'est_map_cloud', 10)
        self.est_path_pub = self.create_publisher(Path, 'est_trajectory', 10)

    def create_corr_pub(self):
        self.corr_traj = self.read_traj_file('corr_traj_file')
        if (len(self.corr_traj) == 0):
            Logger.LogError(
                'Error occurred while reading the trajectory file.')
            self.enable_corr = False
            return
        self.T_GT_CORR = np.array(self.try_get_param(
            'T_GT_CORR', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)
        self.corr_traj = self.align_poses(self.corr_traj, self.T_GT_CORR)
        Logger.LogInfo(f'Read {self.corr_traj.shape[0]} corr poses.')
        self.corr_map_pub = self.create_publisher(
            PointCloud2, 'corr_map_cloud', 10)
        self.corr_path_pub = self.create_publisher(Path, 'corr_trajectory', 10)

    def create_graph_pub(self):
        self.graph_coords = self.read_traj_file('graph_coords_file')
        self.graph_coords = self.graph_coords[:, [7, 0, 1, 2, 3, 4, 5, 6]]
        n_coords = self.graph_coords.shape[0]
        if (n_coords == 0):
            Logger.LogError(
                'Error occurred while reading the graph coords file.')
            self.enable_graph = False
            return
        for i in range(0, n_coords):
            self.graph_coords[i, 0] = Utils.ts_ns_to_seconds(
                self.graph_coords[i, 0])
        self.hierarchy_levels = self.try_get_param('hierarchy_levels', 1)
        self.z_offset = self.try_get_param('z_offset', 5.0)
        self.graph_map_pub = [None] * self.hierarchy_levels
        for i in range(self.hierarchy_levels):
            self.graph_map_pub[i] = self.create_publisher(
                PointCloud2, f'graph_map_cloud_{i}', 10)

        Logger.LogInfo(f'Read {n_coords} graph coords.')
        if self.hierarchy_levels < 1:
            Logger.LogError(
                'The number of hierarchy levels must be at least 1.')
            self.enable_graph = False
            return

    def create_constraints_pub(self):
        self.ts_constraint_map = self.read_constraints_file(
            self.constraints_file)
        self.ts_connections_map = self.read_constraints_file(
            self.connections_file)
        n_constraints = len(self.ts_constraint_map.keys())
        if n_constraints != len(self.ts_connections_map.keys()):
            Logger.LogError(
                'Error occurred while reading the constraints file.')
            Logger.LogError(
                f'{n_constraints} vs. {len(self.ts_connections_map.keys())}')
            self.enable_constraints = False
            return
        Logger.LogInfo(f'Read {n_constraints} constraints.')

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

    def should_publish(self, map_pub, traj_pub):
        has_map_subs = True
        has_traj_subs = True
        if (map_pub is not None):
            has_map_subs = map_pub.get_subscription_count() > 0
        if (traj_pub is not None):
            has_traj_subs = traj_pub.get_subscription_count() > 0
        return has_map_subs & has_traj_subs

    def publish_all_maps(self):
        if len(self.ts_cloud_map) == 0:
            Logger.LogWarn(f'Received no clouds yet.')
            return

        if self.enable_gt and self.should_publish(self.gt_map_pub, self.gt_path_pub):
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            gt_map, gt_idx = self.accumulate_cloud(self.gt_traj, ts_cloud_map)
            if gt_idx >= 0:
                self.publish_map(self.gt_map_pub, self.gt_path_pub,
                                 gt_map, self.gt_traj[0:gt_idx+1, :])
                Logger.LogInfo(
                    f'Published gt map with {len(gt_map.points)} points.')
        if self.enable_graph:
            for lvl in range(self.hierarchy_levels):
                if not self.should_publish(self.graph_map_pub[lvl], None):
                    continue
                ts_cloud_map = copy.deepcopy(self.ts_cloud_map)

                indices = np.arange(0, self.graph_coords.shape[0], lvl + 1)
                cur_coords = self.graph_coords[indices, :]

                graph_map, graph_idx = self.accumulate_cloud(
                    cur_coords, ts_cloud_map, 0.5)

                if graph_idx >= 0:
                    print(f'Publishing graph to {graph_idx}')
                    self.publish_map(
                        self.graph_map_pub[lvl], None, graph_map, None)
                    self.create_sphere_pub(cur_coords[0:graph_idx, 1:4], lvl)
                    Logger.LogInfo(
                        f'Published graph map with {len(graph_map.points)} points.')
            self.vis_helper.visualize_coords()

        if self.enable_est and self.should_publish(self.est_map_pub, self.est_path_pub):
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            est_map, est_idx = self.accumulate_cloud(
                self.est_traj, ts_cloud_map)
            if est_idx >= 0:
                self.publish_map(self.est_map_pub, self.est_path_pub,
                                 est_map, self.est_traj[0:est_idx+1, :])
                Logger.LogInfo(
                    f'Published est map with {len(est_map.points)} points.')

        if self.enable_corr and self.should_publish(self.corr_map_pub, self.corr_path_pub):
            ts_cloud_map = copy.deepcopy(self.ts_cloud_map)
            corr_map, corr_idx = self.accumulate_cloud(
                self.corr_traj, ts_cloud_map)
            if corr_idx >= 0:
                corr_traj = self.corr_traj[0:corr_idx+1, :]
                self.publish_map(self.corr_map_pub,
                                 self.corr_path_pub, corr_map, corr_traj)
                Logger.LogInfo(
                    f'Published corr map with {len(corr_map.points)} points.')
                if self.enable_constraints and self.constraints_pub.get_subscription_count() > 0:
                    self.publish_constraints(corr_traj)

    def publish_constraints(self, traj):
        if len(traj) == 0:
            return

        self.constraint_markers.markers = []
        for ts_ns, labels in self.ts_constraint_map.items():
            ts_s = Utils.ts_ns_to_seconds(ts_ns)
            idx = self.lookup_closest_ts_idx(traj[:, 0], ts_s)
            if idx == -1:
                continue

            n_labels = len(labels)
            for i in range(n_labels):
                label = labels[i]
                child_ts_ns = self.ts_connections_map[ts_ns][i]
                child_ts_s = Utils.ts_ns_to_seconds(child_ts_ns)
                target_idx = self.lookup_closest_ts_idx(traj[:, 0], child_ts_s)
                if target_idx == -1:
                    continue

                if label == ConstraintType.SMALL.value:
                    color = [0.9, 0.05, 0.05]
                elif label == ConstraintType.MID.value:
                    color = [0.05, 0.9, 0.05]
                elif label == ConstraintType.LARGE.value:
                    color = [0.05, 0.05, 0.9]
                else:
                    color = [0.0, 0.0, 0.0]

                self.add_constraint_at(traj, idx, target_idx, label, color)

        self.constraints_pub.publish(self.constraint_markers)

    def add_constraint_at(self, traj, idx_a, idx_b, label, color):
        n_poses = traj.shape[0]
        if (idx_a < 0 or idx_a >= n_poses):
            return
        if (idx_b < 0 or idx_b >= n_poses):
            return
        if self.compute_splines:
            self.add_spline_constraints(traj, idx_a, idx_b, label, color)
        else:
            self.add_linear_constraints(traj, idx_a, idx_b, color)

    def add_linear_constraints(self, traj, idx_a, idx_b, color):
        points = [traj[idx_a, 1:4], traj[idx_b, 1:4]]
        line_marker = self.vis_helper.create_point_line_markers(points, color)
        self.constraint_markers.markers.append(line_marker)

    def interpolate_spline(self, xyz, n_points=20):
        tck, _ = interpolate.splprep(
            [xyz[:, 0], xyz[:, 1], xyz[:, 2]], k=2, s=2)
        u = np.linspace(0, 1, n_points)
        x, y, z = interpolate.splev(u, tck)
        xyz_interp = np.array([x, y, z]).T
        return np.reshape(xyz_interp, (n_points, 3))

    def add_spline_constraints(self, traj, idx_a, idx_b, label, color):
        origin = traj[idx_a, 1:4]
        dest = traj[idx_b, 1:4]

        z = 0
        if label == ConstraintType.SMALL.value:
            z = self.spline_low_height
        elif label == ConstraintType.MID.value:
            z = self.spline_mid_height
        elif label == ConstraintType.LARGE.value:
            z = self.spline_large_height

        diff = (origin + dest) / 2
        pivot = np.array([diff[0], diff[1], diff[2] + z])
        xyz = np.concatenate((origin, pivot, dest))
        xyz = np.reshape(xyz, (3, -1))
        xyz_interp = self.interpolate_spline(xyz, self.spline_points)

        prev = xyz_interp[0, :]
        for i in range(1, self.spline_points):
            cur = xyz_interp[i, :]

            points = [prev, cur]
            line_marker = self.vis_helper.create_point_line_markers(
                points, color)
            self.constraint_markers.markers.append(line_marker)

            prev = cur

    def publish_map(self, map_pub, path_pub, map, traj):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        if map_pub is not None and map is not None:
            map = map.voxel_down_sample(voxel_size=self.voxel_size)
            map_ros = point_cloud2.create_cloud(
                header, FIELDS_XYZ, map.points)
            map_pub.publish(map_ros)

        if path_pub is not None and traj is not None:
            path_msg = self.create_path_msg(traj)
            path_pub.publish(path_msg)

    def get_level_color(self, idx):
        max_idx = 6
        norm_idx = idx % max_idx
        if norm_idx == 0:
            return [0.95, 0.05, 0.05]
        elif norm_idx == 1:
            return [0.05, 0.95, 0.05]
        elif norm_idx == 2:
            return [0.05, 0.05, 0.95]
        elif norm_idx == 3:
            return [0.95, 0.05, 0.95]
        elif norm_idx == 4:
            return [0.95, 0.95, 0.05]
        elif norm_idx == 5:
            return [0.05, 0.95, 0.95]
        return [0.0, 0.0, 0.0]

    def create_sphere_pub(self, traj, level):
        n_poses = traj.shape[0]
        level_z = np.array([0, 0, level * self.z_offset])
        step = level + 1
        for i in range(0, n_poses, step):
            xyz = traj[i, :] + level_z
            color = self.get_level_color(level)
            self.vis_helper.add_graph_coordinate(xyz, color)

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

    def accumulate_cloud(self, trajectory, ts_cloud_map, eps=1e-4):
        map = o3d.geometry.PointCloud()
        idx = -1
        for ts_s, cloud in ts_cloud_map.items():
            idx = self.lookup_closest_ts_idx(trajectory[:, 0], ts_s, eps)
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

    def lookup_closest_ts_idx(self, timestamps_s, ts_s, eps=1e-4):
        diff_timestamps = np.abs(timestamps_s - ts_s)
        minval = np.amin(diff_timestamps)
        if (minval > eps):
            return -1

        return np.where(diff_timestamps == minval)[0][0]

    def lookup_closest_position_idx(self, coords, position):
        diff_positions = np.linalg.norm(coords[:, 0:3] - position, axis=1)
        minval = np.amin(diff_positions)
        print(f'minval: {minval}')
        if (minval > 0.1):
            return -1

        return np.where(diff_positions == minval)[0][0]

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
            Logger.LogDebug(f'Reading CSV file: {traj_full_path}')
            return self.read_csv_file(traj_full_path)
        elif ext == '.npy':
            Logger.LogDebug(f'Reading NPY file: {traj_full_path}')
            return self.read_npy_file(traj_full_path)
        Logger.LogError(f'Unknown file extension: {ext}')
        return np.array([])

    def read_csv_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        Logger.LogDebug(f'ReprojectPub: Reading file {filename}.')
        if exists(filename):
            robot_traj = file_interface.read_tum_trajectory_file(filename)
            return np.column_stack((robot_traj.timestamps, robot_traj.positions_xyz,
                                    robot_traj.orientations_quat_wxyz))
        else:
            Logger.LogError(f'ReprojectPub: File does not exist!')
            return np.array([])

    def read_npy_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        Logger.LogDebug(f'ReprojectPub: Reading file {filename}.')
        if exists(filename):
            return np.load(filename)
        else:
            Logger.LogError(f'ReprojectPub: File does not exist!')
            return np.array([])

    def read_constraints_file(self, filename):
        filename = os.path.join(self.dataroot, filename)
        Logger.LogDebug(f'ReprojectPub: Reading constraints file {filename}.')
        dict = {}
        if exists(filename):
            input_file = open(filename, 'rb')
            dict = pickle.load(input_file)
            input_file.close()
        else:
            Logger.LogError(f'ReprojectPub: Constraints file does not exist!')
        return dict

    def publish_map_for_level(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    pub = ReprojectPub()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
