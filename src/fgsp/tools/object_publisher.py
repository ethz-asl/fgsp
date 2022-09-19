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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


from src.fgsp.common.logger import Logger
from src.fgsp.common.utils import Utils


class ObjectPublisher(Node):
    def __init__(self):
        super().__init__('object_publisher')
        Logger.Verbosity = 5
        Logger.LogInfo('ObjectPublisher: Initializing...')

        self.color = self.get_param('marker_color', [0.8, 0.8, 0.1])
        object_file = self.get_param('object_file', '')
        with open(object_file, 'r') as f:
            configuration = yaml.safe_load(f)
        self.artifact_pub = self.create_publisher(MarkerArray, 'artifacts', 10)

        self.publish_artifacts(configuration["objects"])

        Logger.LogInfo(
            'ObjectPublisher: Initializing done. Publishing objects...')

    def publish_artifacts(self, artifacts):
        markers = self.create_artifacts(artifacts)
        self.artifact_pub.publish(markers)

    def create_artifacts(self, artifacts):
        if artifacts is None and not isinstance(artifacts, list):
            return
        ts_now = self.get_clock().now()
        index = 0

        marker_color = ColorRGBA()
        marker_color.r = self.color[0]
        marker_color.g = self.color[1]
        marker_color.b = self.color[2]
        marker_color.a = 1.0

        artifact_markers = MarkerArray()
        for art in artifacts:
            cube = self.create_artifact(art, ts_now, index, marker_color)
            artifact_markers.markers.append(cube)
            index += 1
        return artifact_markers

    def create_artifact(self, artifact, ts_now, index, color):
        (x, y, z) = artifact['position']

        cube = Marker()
        cube.header.frame_id = 'map'
        cube.header.stamp = ts_now.to_msg()

        cube.id = index
        cube.action = Marker.ADD
        cube.pose.position.x = x
        cube.pose.position.y = y
        cube.pose.position.z = z
        cube.pose.orientation.w = 1.0

        cube.color = color
        cube.scale.x = cube.scale.y = cube.scale.z = 3.0

        cube.type = Marker.CUBE
        cube.frame_locked = True

        return cube

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
