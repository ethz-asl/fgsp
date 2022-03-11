#! /usr/bin/env python2

import rospy
from maplab_msgs.msg import VerificationCheckRequest

class VerificationHandler(object):
    def __init__(self):
        self.robot_requests = {}
        self.server_acks = []
        verification_server_topic = rospy.get_param("~verification_server_request")
        self.pub_to_server = rospy.Publisher(verification_server_topic, VerificationCheckRequest, queue_size=10)
        rospy.loginfo("[VerificationHandler] Initialized.")

    def handle_verification(self, msg):
        rospy.loginfo("[VerificationHandler] From {robot_name}  with {submap_ids}".format(robot_name=msg.robot_name, submap_ids=msg.submap_ids))
        if msg.robot_name not in self.robot_requests:
            self.robot_requests[msg.robot_name] = []

        for submap in msg.submap_ids:
            if submap not in self.robot_requests[msg.robot_name]:
                self.robot_requests[msg.robot_name].append(submap)
        rospy.loginfo("[VerificationHandler] requests {requests}".format(requests=self.robot_requests))

    def send_verification_request(self):
        submap_ids = []
        for k,v in self.robot_requests.items():
            submap_ids = submap_ids + v
        msg = VerificationCheckRequest()
        msg.robot_name = 'monitor'
        msg.submap_ids = list(set(submap_ids))

        self.pub_to_server.publish(msg)
