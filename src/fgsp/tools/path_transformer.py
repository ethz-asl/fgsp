#! /usr/bin/env python3

import os
import numpy as np

import rclpy
from rclpy.node import Node
from evo.tools import file_interface


from src.fgsp.common.logger import Logger


class PathTransformer(Node):
    def __init__(self):
        super().__init__('path_transformer')
        Logger.Verbosity = 5
        Logger.LogInfo('PathTransformer: Initializing...')

        input_path = self.get_param('input_path', '')
        export_path = self.get_param(
            'export_path', '/tmp/transformed.npy')
        T = np.array(self.get_param(
            'T', np.eye(4, 4).reshape(16).tolist())).reshape(4, 4)

        Logger.LogInfo(f'PathTransformer: Reading poses from {input_path}')
        Logger.LogInfo(f'PathTransformer: Processing poses...')
        transformed_path = self.transform_path(input_path, T)
        np.save(export_path, transformed_path)
        Logger.LogInfo(f'PathTransformer: Saved cloud to {export_path}')

    def transform_path(self, input_path, T):
        path = self.read_csv_file(input_path)

        # Transform path
        ones = np.ones_like(path[:, 0])
        path = np.column_stack((path, ones))
        path = np.column_stack((path[:, 0], np.matmul(T, path[:, 1:5].T).T))

        return path[:, 0:4]

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def read_csv_file(self, filename):
        Logger.LogDebug(f'PathTransformer: Reading file {filename}.')
        if os.path.exists(filename):
            robot_traj = file_interface.read_tum_trajectory_file(filename)
            return np.column_stack((robot_traj.timestamps, robot_traj.positions_xyz))
        else:
            Logger.LogError(f'PathTransformer: File does not exist!')
            return np.array([])


def main(args=None):
    rclpy.init(args=args)
    transformer = PathTransformer()
    rclpy.spin(transformer)
    transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
