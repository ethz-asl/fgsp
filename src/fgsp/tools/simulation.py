#! /usr/bin/env python3

from os.path import exists

import rclpy
import pathlib
from rclpy.node import Node
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D
from maplab_msgs.msg import Graph, Trajectory, SubmapConstraint

import numpy as np

from rosbags.serde import serialize_cdr
from rosbags.rosbag2 import Writer as Rosbag2Writer
from rosbags.typesys import get_types_from_msg, register_types

from src.fgsp.common.logger import Logger
from src.fgsp.controller.signal_handler import SignalHandler


class Simulation(Node):
    def __init__(self):
        super().__init__('simulation')
        Logger.Verbosity = 5
        Logger.LogInfo('Simulation: Initializing...')

        self.server_traj = self.get_traj_file('server_file')
        self.robot_traj = self.get_traj_file('robot_file')

        if self.server_traj is None and self.robot_traj is None:
            Logger.LogError(
                'Simulation: No trajectory file has been parsed. Aborting.!')
            return

        self.odom_topic = self.get_param('odom_topic', '/odometry')
        self.monitor_topic = self.get_param('monitor_topic', '/monitor')
        self.robot_name = self.get_param('robot_name', 'robot')
        self.map_frame = self.get_param('map_frame', 'map')
        self.child_frame = self.get_param('child_frame', 'base')
        self.graph_threshold_dist = self.get_param('graph_threshold_dist', 0.5)
        self.out_bag_file = self.get_param(
            'out_bag_file', '/tmp/odom_and_monitor.bag')

        Logger.LogInfo(
            f'Simulation: Writing bag file to: {self.out_bag_file}')
        ros2_bag_out = Rosbag2Writer(self.out_bag_file)
        ros2_bag_out.open()

        ros_time_odom = None
        if self.robot_traj is not None:
            Logger.LogInfo(
                f'Simulation: Robot trajectory has {self.robot_traj.num_poses} poses')
            Logger.LogDebug(
                f'Simulation: Robot trajectory: {self.robot_traj.get_infos()}')
            ros_time_odom = self.write_odometry(self.robot_traj, ros2_bag_out)

        ros_time_traj = None
        if self.server_traj is not None:
            Logger.LogInfo(
                f'Simulation: Server trajectory has {self.server_traj.num_poses} poses')
            Logger.LogDebug(
                f'Simulation: Server trajectory: {self.server_traj.get_infos()}')
            ros_time_traj = self.write_server_trajectory(
                self.server_traj, ros2_bag_out)

        self.write_delayed_msg(ros_time_odom, ros_time_traj, ros2_bag_out)

        ros2_bag_out.close()
        Logger.LogInfo('Simulation: Writing bag file done.')

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def get_traj_file(self, traj_key):
        traj_file_path = self.get_param(traj_key, '')
        ext = pathlib.Path(traj_file_path).suffix
        if ext == '.csv':
            Logger.LogInfo('Simulation: Reading CSV file.')
            return self.read_csv_file(traj_file_path)
        elif ext == '.npy':
            Logger.LogInfo('Simulation: Reading NPY file.')
            return self.read_npy_file(traj_file_path)
        Logger.LogError(f'Simulation: Unknown file extension: {ext}')
        return None

    def read_csv_file(self, filename):
        Logger.LogInfo(f'Simulation: Reading file {filename}.')
        if exists(filename):
            return file_interface.read_tum_trajectory_file(filename)
        else:
            Logger.LogError(f'Simulation: File does not exist!')
            return None

    def convert_to_traj(self, trajectory):
        k_ns_per_s = 1e9
        ts = trajectory[:, 0] / k_ns_per_s
        xyz = trajectory[:, 1:4]
        wxyz = trajectory[:, 4:8]
        return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=wxyz, timestamps=ts)

    def read_npy_file(self, filename):
        Logger.LogInfo(f'Simulation: Reading file {filename}.')
        if exists(filename):
            return self.convert_to_traj(np.load(filename))
        else:
            Logger.LogError(f'Simulation: File does not exist!')
            return None

    def write_odometry(self, robot_traj, ros2_bag_out):

        from rosbags.typesys.types import (
            nav_msgs__msg__Odometry as Odometry,
            geometry_msgs__msg__PoseWithCovariance as PoseWithCovariance,
            geometry_msgs__msg__TwistWithCovariance as TwistWithCovariance,
            geometry_msgs__msg__Twist as Twist,
            geometry_msgs__msg__Vector3 as Vector3,
            std_msgs__msg__Header as Header,
            geometry_msgs__msg__Pose as Pose,
            geometry_msgs__msg__Point as Position,
            geometry_msgs__msg__Quaternion as Quaternion,
            builtin_interfaces__msg__Time as Time)

        msg_type = Odometry.__msgtype__
        connection = ros2_bag_out.add_connection(self.odom_topic, msg_type)

        Logger.LogDebug(
            f'Writing odometry to bag file to: {self.out_bag_file}')
        for stamp, xyz, quat in zip(robot_traj.timestamps, robot_traj.positions_xyz,
                                    robot_traj.orientations_quat_wxyz):
            sec = int(stamp // 1)
            nanosec = int((stamp - sec) * 1e9)
            ros_time = Time(sec, nanosec)
            header = Header(ros_time, self.map_frame)
            position = Position(x=xyz[0], y=xyz[1], z=xyz[2])
            quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
            pose = Pose(position, quaternion)
            pose_with_cov = PoseWithCovariance(pose, np.array([0.0] * 36))
            vec = Vector3(0.0, 0.0, 0.0)
            twist = Twist(vec, vec)
            twist_with_cov = TwistWithCovariance(twist, np.array([0.0] * 36))
            odom = Odometry(header, self.child_frame,
                            pose_with_cov, twist_with_cov)

            serialized_msg = serialize_cdr(odom, msg_type)
            ros2_bag_out.write(connection, int(stamp * 1e9), serialized_msg)
        Logger.LogInfo(f'Simulation: Wrote topic: {self.odom_topic}')

        Logger.LogInfo('Simulation: Writing odometry to bag file done.')
        return ros_time

    def write_server_trajectory(self, server_traj, ros2_bag_out):
        TRAJECTORY_MSG = """
        std_msgs/Header header
        maplab_msgs/TrajectoryNode[] nodes
        """
        TRAJECTORY_NODE_MSG = """
        string robot_name
        int64 id
        geometry_msgs/PoseStamped pose
        float32 signal
        """

        register_types(get_types_from_msg(
            TRAJECTORY_NODE_MSG, 'maplab_msgs/msg/TrajectoryNode'))
        register_types(get_types_from_msg(
            TRAJECTORY_MSG, 'maplab_msgs/msg/Trajectory'))

        from rosbags.typesys.types import (maplab_msgs__msg__TrajectoryNode as TrajectoryNode,
                                           maplab_msgs__msg__Trajectory as Trajectory,
                                           geometry_msgs__msg__Pose as Pose,
                                           geometry_msgs__msg__PoseStamped as PoseStamped,
                                           geometry_msgs__msg__Point as Position,
                                           geometry_msgs__msg__Quaternion as Quaternion,
                                           std_msgs__msg__Header as Header,
                                           builtin_interfaces__msg__Time as Time)

        msg_type = Trajectory.__msgtype__
        connection = ros2_bag_out.add_connection(self.monitor_topic, msg_type)

        Logger.LogDebug(
            f'Writing server trajectory to bag file to: {self.out_bag_file}')
        i = 0
        k = 0
        update_every_n_poses = 10
        nodes = []
        n_poses = len(server_traj.timestamps)
        last_pos = np.array([])
        last_idx = n_poses - 1
        dist = 0.0
        Logger.LogDebug(
            f'Using a threshold of {self.graph_threshold_dist}m for distance.')
        for stamp, xyz, quat in zip(server_traj.timestamps, server_traj.positions_xyz,
                                    server_traj.orientations_quat_wxyz):
            i += 1
            if i > 1 and i != last_idx:
                dist += np.linalg.norm(xyz - last_pos)
                if self.graph_threshold_dist > 0.0 and dist < self.graph_threshold_dist:
                    continue

            dist = 0.0
            last_pos = np.copy(xyz)
            sec = int(stamp // 1)
            nanosec = int((stamp - sec) * 1e9)
            ros_time = Time(sec, nanosec)
            header = Header(ros_time, self.map_frame)

            position = Position(x=xyz[0], y=xyz[1], z=xyz[2])
            quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
            pose = Pose(position, quaternion)
            pose_stamped = PoseStamped(header, pose)

            node = TrajectoryNode(self.robot_name, k, pose_stamped, 0.0)
            nodes.append(node)

            if i == 1 or i % update_every_n_poses == 0 or i == last_idx:
                k = k + 1
                traj_msg = Trajectory(header, nodes)
                serialized_msg = serialize_cdr(traj_msg, msg_type)
                ros2_bag_out.write(connection, int(
                    stamp * 1e9), serialized_msg)
                print(f'Wrote pose {i} of {n_poses} with current dist {dist}')

        for i in range(0, 10):
            ros2_bag_out.write(connection, int(
                (stamp+i*20) * 1e9), serialized_msg)
        Logger.LogInfo(
            f'Simulation: Wrote monitor topic {self.monitor_topic} to the bag.')

        Logger.LogInfo(
            'Simulation: Writing server trajectory to bag file done.')

        return ros_time

    def write_delayed_msg(self, ros_time_odom, ros_time_traj, ros2_bag_out):
        from rosbags.typesys.types import (std_msgs__msg__Header as Header,
                                           std_msgs__msg__String as String,
                                           builtin_interfaces__msg__Time as Time)
        if ros_time_odom is None:
            max_time_s = ros_time_traj.sec
        elif ros_time_traj is None:
            max_time_s = ros_time_odom.sec
        else:
            max_time_s = max(ros_time_odom.sec, ros_time_traj.sec)

        delay_s = 600

        str_msg = String(data=f'Delaying by {delay_s} seconds.')
        msg_type = String.__msgtype__
        connection = ros2_bag_out.add_connection('delay', msg_type)
        serialized_msg = serialize_cdr(str_msg, msg_type)

        ros2_bag_out.write(connection, int(
            (max_time_s + delay_s) * 1e9), serialized_msg)
        Logger.LogInfo(f'Simulation: Wrote delayed topic to the bag.')


def main(args=None):
    rclpy.init(args=args)
    server = Simulation()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
