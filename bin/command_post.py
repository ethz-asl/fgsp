#! /usr/bin/env python3

import rospy
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommandPost(object):
    def __init__(self):
        update_good_topic = rospy.get_param("~update_good_topic")
        update_bad_topic = rospy.get_param("~update_bad_topic")
        self.pub_good = rospy.Publisher(update_good_topic, Path, queue_size=10)
        self.pub_bad = rospy.Publisher(update_bad_topic, Path, queue_size=10)
        self.good_path_msg = None
        self.bad_path_msg = None
        rospy.loginfo("[CommandPost] Initialized command post center.")

    def reset_msgs(self):
        self.good_path_msg = Path()
        self.bad_path_msg = Path()


    def accumulate_update_messages(self, submap_features):
        # Should always publish for all states as we don't know
        # whether they reached the clients.
        n_nodes = len(submap_features.nodes)
        for i in range(0, n_nodes):
            cur_opt = submap_features.nodes[i]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = cur_opt.ts
            pose_msg.pose.position.x = cur_opt.position[0]
            pose_msg.pose.position.y = cur_opt.position[1]
            pose_msg.pose.position.z = cur_opt.position[2]
            pose_msg.pose.orientation.w = cur_opt.orientation[0]
            pose_msg.pose.orientation.x = cur_opt.orientation[1]
            pose_msg.pose.orientation.y = cur_opt.orientation[2]
            pose_msg.pose.orientation.z = cur_opt.orientation[3]

            if submap_features.label == 0:
                self.good_path_msg.poses.append(pose_msg)
            elif submap_features.label == 1:
                self.bad_path_msg.poses.append(pose_msg)
            else:
                rospy.logerror(f"Found an unknown label {submap_features.label}")

    def publish_update_messages(self):
        if self.good_path_msg == None or self.bad_path_msg == None:
            return
        n_good = len(self.good_path_msg.poses)
        n_bad = len(self.bad_path_msg.poses)
        rospy.loginfo(f"[CommandPost] Publishing evaluation results ({n_good}/{n_bad}).")
        if n_good > 0:
            self.good_path_msg.header.stamp = rospy.Time.now()
            self.pub_good.publish(self.good_path_msg)
        if n_bad > 0:
            self.bad_path_msg.header.stamp = rospy.Time.now()
            self.pub_bad.publish(self.bad_path_msg)
