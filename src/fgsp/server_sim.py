#! /usr/bin/env python3

from os.path import exists

import rclpy
from rclpy.node import Node
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from maplab_msgs.msg import Graph, Trajectory, SubmapConstraint
from nav_msgs.msg import Path

from src.fgsp.common.logger import Logger



class ServerSimulation(Node):
    def __init__(self):
        super().__init__('FGSP_TrajPublisher')
        Logger.LogInfo('ServerSimulation: Initializing...')
        self.traj_file = self.get_traj_file()

    def get_traj_file(self):
        traj_key = 'traj_file'
        self.declare_parameter(traj_key, '')
        return self.read_trajectory_file(self.get_parameter(traj_key).value)

    def read_trajectory_file(self, filename):
        Logger.LogInfo(f'ServerSimulation: Reading file {filename}.')
        if exists(filename):
            return file_interface.read_tum_trajectory_file(filename)
        else:
            Logger.LogError(f'ServerSimulation: File does not exist!')
            return None

def main(args=None):
    rclpy.init(args=args)
    server = ServerSimulation()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()