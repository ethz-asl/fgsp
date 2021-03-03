#! /usr/bin/env python3

import rospy
from maplab_msgs.srv import Verification

class VerificationHandler(object):
    def __init__(self):
        self.robot_requests = {}
        self.server_acks = []
        self.verification_server_topic = rospy.get_param("~verification_server_request")
        #self.pub_to_server = rospy.Publisher(verification_server_topic, Path, queue_size=10)
        rospy.loginfo("[VerificationHandler] Handler initialized.")

    def handle_verification(self, msg):
        rospy.loginfo(f"[VerificationHandler] From {msg.robot_name}  with {msg.submap_ids}")
        if msg.robot_name not in self.robot_requests:
            self.robot_requests[msg.robot_name] = []

        for submap in msg.submap_ids:
            if submap not in self.robot_requests[msg.robot_name]:
                self.robot_requests[msg.robot_name].append(submap)
        rospy.loginfo(f"[VerificationHandler] requests {self.robot_requests}")

    def send_verification_request(self):
        submap_ids = []
        for k,v in self.robot_requests.items():
            submap_ids = submap_ids + v
        msg = VerificationCheckRequest()
        msg.robot_name = 'monitor'
        msg.submap_ids = list(set(submap_ids))

        verification_proxy = rospy.ServiceProxy(verification_server_topic, Verification)
        verification_proxy(msg)
