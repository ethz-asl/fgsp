#! /usr/bin/env python2

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from fgsp.common.lc_model import LcModel
from fgsp.common.utils import Utils

class RobotConstraints(object):
    def __init__(self):
        self.submap_constraints = {}
        self.previous_timestamps = []

    def add_submap_constraints(self, ts_from, ts_to, T_a_b):
        lc = LcModel(ts_from, ts_to, T_a_b)
        ts_from_ns = Utils.ros_time_to_ns(ts_from)
        ts_to_ns = Utils.ros_time_to_ns(ts_to)
        if ts_from_ns == ts_to_ns:
            print('Timestamp from and to are identical.')
            return

        if ts_from_ns not in self.submap_constraints:
            self.submap_constraints[ts_from_ns] = []
        for i in range(len(self.submap_constraints[ts_from_ns])):
            if self.submap_constraints[ts_from_ns][i].ts_to == ts_to:
                self.submap_constraints[ts_from_ns][i] = lc
                return

        self.submap_constraints[ts_from_ns].append(lc)

    def construct_path_msgs(self):
        path_msgs = []
        print('Constructing path message for {n_submaps} different submaps.'.format(n_submaps=len(self.submap_constraints)))
        for ts_ns_from in self.submap_constraints:
            loop_closures = list(self.submap_constraints[ts_ns_from])
            path_msg = self.construct_path_msg_for_submap(ts_ns_from, loop_closures)
            path_msgs.append(path_msg)
        return path_msgs

    def construct_path_msgs_using_ts(self, timestamps):
        path_msgs = []
        print('Constructing path message for {n_submaps} different submaps.'.format(n_submaps=len(self.submap_constraints)))
        for ts_from_ns in self.submap_constraints:
            if not self.should_publish_map(ts_from_ns, timestamps):
                continue
            if ts_from_ns not in self.previous_timestamps:
                self.previous_timestamps.append(ts_from_ns)

            rospy.logwarn('[RobotConstraints] Found a valid timestamp!!! ')
            loop_closures = list(self.submap_constraints[ts_from_ns])
            path_msg = self.construct_path_msg_for_submap(ts_from_ns, loop_closures)
            path_msgs.append(path_msg)
        return path_msgs

    def should_publish_map(self, ts_from_ns, timestamps):
        if ts_from_ns in self.previous_timestamps:
            return True

        ts_diff = np.absolute(timestamps - ts_from_ns)
        ts_min = np.amin(ts_diff)
        diff_s = Utils.ts_ns_to_seconds(ts_min)
        rospy.logwarn('[RobotConstraints] min diff ts is {diff}'.format(diff=diff_s))

        return diff_s < 3

    def construct_path_msg_for_submap(self, ts_ns_from, loop_closures):
        path_msg = Path()
        path_msg.header.stamp = Utils.ts_ns_to_ros_time(ts_ns_from)

        for lc in loop_closures:
            t_a_b = lc.get_translation()
            q_a_b = lc.get_rotation_quat()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = lc.ts_to
            pose_msg.pose.position.x = t_a_b[0]
            pose_msg.pose.position.y = t_a_b[1]
            pose_msg.pose.position.z = t_a_b[2]
            pose_msg.pose.orientation.x = q_a_b[0]
            pose_msg.pose.orientation.y = q_a_b[1]
            pose_msg.pose.orientation.z = q_a_b[2]
            pose_msg.pose.orientation.w = q_a_b[3]

            path_msg.poses.append(pose_msg)

        return path_msg
