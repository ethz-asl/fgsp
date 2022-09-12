#! /usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


from src.fgsp.common.logger import Logger


class CloudSaver(Node):
    def __init__(self):
        super().__init__('cloud_saver')
        Logger.Verbosity = 5
        Logger.LogInfo('CloudSaver: Initializing...')

        self.cloud_topic = self.get_param('cloud_topic', '/point_cloud')
        self.export_path = self.get_param('export_path', '/tmp/cloud.npy')

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, 10)

        Logger.LogInfo(f'CloudSaver: Subscribed to {self.cloud_topic}')
        Logger.LogInfo(f'CloudSaver: Writing cloud to {self.export_path}')
        Logger.LogInfo('CloudSaver: Initializing done. Waiting for clouds...')

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def cloud_callback(self, msg):
        cloud = np.array(list(point_cloud2.read_points(msg, skip_nans=True)))
        np.save(self.export_path, cloud)
        Logger.LogInfo(f'CloudSaver: Saved cloud to {self.export_path}')


def main(args=None):
    rclpy.init(args=args)
    server = CloudSaver()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
