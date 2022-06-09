#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class Comms(object):
    _instance = None

    def __new__(self):
        if self._instance is None:
            print(f'Creating the object')
            self._instance = super(Comms, self).__new__(self)
            self._instance.node = Node('fgsp_comms')
        return self._instance

    def send(self, msg, type, topic):
        publisher = self.node.create_publisher(type, topic, 10)
        publisher.publish(msg)


if __name__ == '__main__':
    from std_msgs.msg import String

    rclpy.init()

    msg = String()
    msg2 = String()
    msg.data = "Foo"
    msg2.data = "Bar"

    comms = Comms()
    comms.send(msg, String, "/foo")

    comms2 = Comms()
    comms2.send(msg2, String, "/foo")

    print(f'Singleton? {comms is comms2}')

    rclpy.spin_once(comms.node, timeout_sec=1)
    comms.node.destroy_node()
    rclpy.shutdown()
