#! /usr/bin/env python3

from os.path import exists

import rclpy
from rclpy.node import Node
from evo.tools import file_interface
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
        Logger.LogInfo('Simulation: Initializing...')

        self.server_traj = self.get_traj_file('server_file')
        self.robot_traj = self.get_traj_file('robot_file')
        if self.server_traj is None or self.robot_traj is None:
            Logger.LogError('Simulation: Trajectory files not found!')
            return

        print(
            f'Simulation: Server trajectory: {self.server_traj.get_infos()}')
        print(
            f'Simulation: Robot trajectory: {self.robot_traj.get_infos()}')

        # self.write_odometry(self.robot_traj)
        self.write_server_trajectory(self.server_traj)

    def get_traj_file(self, traj_key):
        print(f'declaring param {traj_key}')
        self.declare_parameter(traj_key, 'foo')
        return self.read_trajectory_file(self.get_parameter(traj_key).value)

    def read_trajectory_file(self, filename):
        Logger.LogInfo(f'Simulation: Reading file {filename}.')
        if exists(filename):
            return file_interface.read_tum_trajectory_file(filename)
        else:
            Logger.LogError(f'Simulation: File does not exist!')
            return None

    def write_odometry(self, robot_traj):
        bag_file = '/tmp/odometry.bag'
        ros2_bag_out = Rosbag2Writer(bag_file)
        ros2_bag_out.open()

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
        topic = '/odometry'
        connection = ros2_bag_out.add_connection(topic, msg_type)

        Logger.LogDebug(f'Writing odometry to bag file to: {bag_file}')
        for stamp, xyz, quat in zip(robot_traj.timestamps, robot_traj.positions_xyz,
                                    robot_traj.orientations_quat_wxyz):
            sec = int(stamp // 1)
            nanosec = int((stamp - sec) * 1e9)
            time = Time(sec, nanosec)
            header = Header(time, "map")
            position = Position(x=xyz[0], y=xyz[1], z=xyz[2])
            quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
            pose = Pose(position, quaternion)
            pose_with_cov = PoseWithCovariance(pose, np.array([0.0] * 36))
            vec = Vector3(0.0, 0.0, 0.0)
            twist = Twist(vec, vec)
            twist_with_cov = TwistWithCovariance(twist, np.array([0.0] * 36))
            odom = Odometry(header, "base", pose_with_cov, twist_with_cov)

            serialized_msg = serialize_cdr(odom, msg_type)
            ros2_bag_out.write(connection, int(stamp * 1e9), serialized_msg)
        Logger.LogInfo(f'Wrote topic: {topic}')
        ros2_bag_out.close()

        Logger.LogInfo('Simulation: Writing odometry to bag file done.')

    def create_submap_ids(self, n_poses):
        submap_ids = []
        k = 0
        for i in range(n_poses):
            submap_ids.append(k)
            if i % 20 == 0:
                k = k + 1
        return np.array(submap_ids)

    def convert_traj_to_signal(self, traj):
        handler = SignalHandler(None)
        submap_ids = self.create_submap_ids(len(traj.timestamps))
        poses = np.column_stack(
            [submap_ids, traj.timestamps, traj.positions_xyz, traj.orientations_quat_wxyz])
        handler.convert_signal_from_poses(poses, 'foo')
        return handler.to_signal_msg('foo')

    def write_server_trajectory(self, server_traj):
        TRAJECTORY_MSG = """
        std_msgs/Header header
        maplab_msgs/TrajectoryNode[] nodes
        """
        TRAJECTORY_NODE_MSG = """
        string robot_name
        int64 id
        geometry_msgs/Pose pose
        float32 signal
        """

        register_types(get_types_from_msg(
            TRAJECTORY_NODE_MSG, 'maplab_msgs/msg/TrajectoryNode'))
        register_types(get_types_from_msg(
            TRAJECTORY_MSG, 'maplab_msgs/msg/Trajectory'))

        from rosbags.typesys.types import maplab_msgs__msg__TrajectoryNode as TrajectoryNode  # type: ignore  # noqa
        from rosbags.typesys.types import maplab_msgs__msg__Trajectory as Trajectory  # type: ignore  # noqa

        print(f'timestamp shape {server_traj.timestamps.shape}')
        print(f'positions shape {server_traj.positions_xyz.shape}')

        trajectory_msg = self.convert_traj_to_signal(server_traj)

        # bag_file = '/tmp/server_traj.bag'
        # ros2_bag_out = Rosbag2Writer(bag_file)
        # ros2_bag_out.open()
        # ros2_bag_out.close()


def main(args=None):
    rclpy.init(args=args)
    server = Simulation()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
