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
        self.should_store_sequentially = self.get_param('sequential', False)
        if self.should_store_sequentially:
            self.n_clouds = 0
            self.export_path = self.export_path.replace(
                '.npy', '_{n_clouds}.npy')

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, 10)

        Logger.LogInfo(f'CloudSaver: Subscribed to {self.cloud_topic}')
        Logger.LogInfo(f'CloudSaver: Writing cloud to {self.export_path}')
        Logger.LogInfo('CloudSaver: Initializing done. Waiting for clouds...')

    def get_param(self, key, default):
        self.declare_parameter(key, default)
        return self.get_parameter(key).value

    def cloud_callback(self, msg):
        cloud = point_cloud2.read_points_numpy(
            msg, field_names=['x', 'y', 'z', "intensity"], skip_nans=True, reshape_organized_cloud=False)
        cloud = np.reshape(cloud, (-1, 4))

        export_path = self.export_path
        if self.should_store_sequentially:
            export_path = self.export_path.format(n_clouds=self.n_clouds)
            self.n_clouds += 1
        np.save(export_path, cloud)
        Logger.LogInfo(f'CloudSaver: Saved cloud to {export_path}')


def main(args=None):
    rclpy.init(args=args)
    server = CloudSaver()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
