#! /usr/bin/env python3

import rospy
from maplab_msgs.msg import Graph, Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class CommandPost(object):
    def __init__(self):
        update_good_topic = rospy.get_param("~update_good_topic")
        self.pub_good = rospy.Publisher(update_good_topic, Path, queue_size=10)
        rospy.loginfo("[CommandPost] Initialed command post center.")

    def publish_update_messages(self, est_nodes, opt_nodes, predictions):
        # Should always publish for all states as we don't know
        # whether they reached the clients.
        rospy.loginfo("[CommandPost] Publishing evaluation results.")
        n_nodes = len(predictions)
        path_msg = Path()
        for i in range(0, n_nodes):
            if predictions[i] is not 0.0:
                continue

            cur_opt = opt_nodes[i]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = cur_opt.ts
            pose_msg.pose.position.x = cur_opt.position[0]
            pose_msg.pose.position.y = cur_opt.position[1]
            pose_msg.pose.position.z = cur_opt.position[2]
            pose_msg.pose.orientation.w = cur_opt.orientation[0]
            pose_msg.pose.orientation.x = cur_opt.orientation[1]
            pose_msg.pose.orientation.y = cur_opt.orientation[2]
            pose_msg.pose.orientation.z = cur_opt.orientation[3]

            path_msg.poses.append(pose_msg)

        self.pub_good.publish(path_msg)
