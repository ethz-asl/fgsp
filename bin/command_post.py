#! /usr/bin/env python3

import rospy
import numpy as np
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode, VerificationCheckRequest
from maplab_msgs.srv import Verification, VerificationResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommandPost(object):
    def __init__(self):
        anchor_node_topic = rospy.get_param("~anchor_node_topic")
        relative_node_topic = rospy.get_param("~relative_node_topic")
        verification_service = rospy.get_param("~verification_service")
        self.pub_anchor = rospy.Publisher(anchor_node_topic, Path, queue_size=10)
        self.pub_relative = rospy.Publisher(relative_node_topic, Path, queue_size=10)
        self.pub_verify = rospy.Publisher(verification_service, VerificationCheckRequest, queue_size=10)
        self.pub_degenerate = rospy.Publisher('/graph_client/degenerate_anchors', Path, queue_size=10)
        #self.verification_proxy = rospy.ServiceProxy(verification_service, Verification)

        self.good_path_msg = None
        self.bad_path_msg = None
        self.degenerate_path_msg = None
        self.verification_request = VerificationCheckRequest()
        self.degenerate_indices = []
        rospy.loginfo("[CommandPost] Initialized command post center.")
        self.previous_relatives = {}

    def reset_msgs(self):
        self.good_path_msg = Path()
        self.bad_path_msg = Path()
        self.degenerate_path_msg = Path()
        self.verification_request = VerificationCheckRequest()

    def evaluate_labels_per_node(self, labels):
        # Should always publish for all states as we don't know
        # whether they reached the clients.
        n_nodes = labels.size()
        for i in range(0, n_nodes):
            if labels.labels[i] == 0 and i in self.previous_relatives.keys():
                labels.labels[i] = self.previous_relatives[i]
            relative_constraint = labels.check_and_construct_constraint_at(i)
            if relative_constraint is None:
                continue # no-op
            self.previous_relatives[i] = labels.labels[i]
            self.pub_relative.publish(relative_constraint)

    def create_pose_msg_from_node(self, cur_opt):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = cur_opt.ts
        pose_msg.pose.position.x = cur_opt.position[0]
        pose_msg.pose.position.y = cur_opt.position[1]
        pose_msg.pose.position.z = cur_opt.position[2]
        pose_msg.pose.orientation.w = cur_opt.orientation[0]
        pose_msg.pose.orientation.x = cur_opt.orientation[1]
        pose_msg.pose.orientation.y = cur_opt.orientation[2]
        pose_msg.pose.orientation.z = cur_opt.orientation[3]
        return pose_msg

    def append_verification_request(self, submap_features):
        if not self.verification_request.robot_name:
            self.verification_request.robot_name = submap_features.robot_name
        elif submap_features.robot_name and self.verification_request.robot_name != submap_features.robot_name:
            rospy.logerror(f"[CommandPost] Trying to add verifiction for {submap_features.robot_name} but previous was {self.verification_request.robot_name}")
            return False
        if submap_features.id not in self.verification_request.submap_ids:
            self.verification_request.submap_ids.append(submap_features.id)
        return True

    def send_verification_request(self):
        if len(self.verification_request.submap_ids) == 0:
            return
        rospy.loginfo(f"[CommandPost] Service call: {self.verification_request.robot_name} with ids: {self.verification_request.submap_ids}")
        #self.verification_proxy(self.verification_request)
        self.verification_request.header.stamp = rospy.Time.now()
        self.pub_verify.publish(self.verification_request)

    def send_degenerate_anchors(self, all_opt_nodes, begin_send, end_send):
        rospy.logerr(f'Sending degenerate anchors for {end_send - begin_send} nodes.')
        indices = np.arange(begin_send, end_send, 1)
        self.send_degenerate_anchors_based_on_indices(all_opt_nodes, indices)

    def send_degenerate_anchors_based_on_indices(self, opt_nodes, indices):
        rospy.logerr(f'Sending degenerate anchors for {len(indices)} nodes.')
        for i in indices:
            pose_msg = self.create_pose_msg_from_node(opt_nodes[i])
            self.degenerate_path_msg.poses.append(pose_msg)

            # Update the degenerate anchor indices
            if not i in self.degenerate_indices:
                self.degenerate_indices.append(i)

        self.degenerate_path_msg.header.stamp = rospy.Time.now()
        self.pub_degenerate.publish(self.degenerate_path_msg)

    def update_degenerate_anchors(self, all_opt_nodes):
        if len(self.degenerate_indices) == 0:
            return
        rospy.logerr(f'Sending degenerate anchor update for {self.degenerate_indices}')
        self.send_degenerate_anchors_based_on_indices(all_opt_nodes, self.degenerate_indices)
