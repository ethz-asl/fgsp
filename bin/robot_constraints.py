#! /usr/bin/env python3

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from lc_model import LcModel
from utils import Utils

class RobotConstraints(object):
    def __init__(self):
        self.submap_constraints = {}

    def add_submap_constraints(self, ts_from, ts_to, T_a_b):
        lc = LcModel(ts_from, ts_to, T_a_b)
        ts_from_ns = Utils.ros_time_to_ns(ts_from)
        ts_to_ns = Utils.ros_time_to_ns(ts_to)
        if ts_from_ns == ts_to_ns:
            print(f'Timestamp from and to are identical.')
            return

        print(f'Received submap constraint at time {ts_from_ns}')
        if ts_from_ns not in self.submap_constraints:
            self.submap_constraints[ts_from_ns] = []
        self.submap_constraints[ts_from_ns].append(lc)

    def construct_path_msgs(self):
        path_msgs = []
        print(f'Constructing path message for {len(self.submap_constraints)} different submaps.')
        for ts_ns_from in self.submap_constraints:
            loop_closures = list(self.submap_constraints[ts_ns_from])
            path_msg = self.construct_path_msg_for_submap(ts_ns_from, loop_closures)
            path_msgs.append(path_msg)
        return path_msgs

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
