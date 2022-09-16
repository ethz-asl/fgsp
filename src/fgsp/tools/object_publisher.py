#! /usr/bin/env python3

import numpy as np
import os
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from evo.tools import file_interface
from evo.core import sync
from evo.core import metrics
from evo.tools import plot
import matplotlib.pyplot as plt


from src.fgsp.common.logger import Logger


class ObjectPublisher(Node):
    def __init__(self):
        super().__init__('object_publisher')
        Logger.Verbosity = 5
        Logger.LogInfo('ObjectPublisher: Initializing...')

        color = self.get_param('marker_color', [0.8, 0.8, 0.1])
        print(f'color: {color}')
        object_file = self.get_param('object_file', '')
        with open(object_file, 'r') as f:
            configuration = yaml.safe_load(f)
        print(f'configuration: {configuration}')

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value


def main(args=None):
    rclpy.init(args=args)
    pub = ObjectPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
