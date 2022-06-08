#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class Comms(Node):
    def __init__(self):
        super().__init__('fgsp_comms')

    def send(self, msg, type, topic):
        publisher = self.create_publisher(type, topic, 10)
        publisher.publish(msg)


if __name__ == '__main__':
    from std_msgs.msg import String

    rclpy.init()

    msg = String()
    msg.data = "Foo"

    comms = Comms()
    comms.send(msg, String, "foo")

    rclpy.spin_once(comms, timeout_sec=1)
    comms.destroy_node()
    rclpy.shutdown()
