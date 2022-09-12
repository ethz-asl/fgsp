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


class CloudSaver(Node):
    def __init__(self):
        super().__init__('cloud_saver')
        Logger.Verbosity = 5
        Logger.LogInfo('CloudSaver: Initializing...')

        self.cloud_topic = self.get_param('odom_topic', '/point_cloud')
        self.export_path = self.get_param('monitor_topic', '/tmp/cloud.npy')

        Logger.LogInfo('Simulation: Initializing done.')

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value


def main(args=None):
    rclpy.init(args=args)
    server = CloudSaver()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
