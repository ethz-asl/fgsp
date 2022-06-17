#! /usr/bin/env python3

from os.path import exists

import rclpy
from rclpy.node import Node
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from maplab_msgs.msg import Graph, Trajectory, SubmapConstraint
from nav_msgs.msg import Path, Odometry

from src.fgsp.common.logger import Logger


class Simulation(Node):
    def __init__(self):
        super().__init__('simulation')
        Logger.LogInfo('Simulation: Initializing...')

        self.server_traj_tum = self.get_traj_file('server_file')
        self.robot_traj_tum = self.get_traj_file('robot_file')
        if self.server_traj_tum is None or self.robot_traj_tum is None:
            Logger.LogError('Simulation: Trajectory files not found!')
            return

        print(
            f'Simulation: Server trajectory: {self.server_traj_tum.get_infos()}')
        print(
            f'Simulation: Robot trajectory: {self.robot_traj_tum.get_infos()}')

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


def main(args=None):
    rclpy.init(args=args)
    server = Simulation()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
