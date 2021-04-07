#! /usr/bin/env python3

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from lc_model import LcModel

class RobotConstraints(object):
    def __init__(self):
        print(f"foo")
        self.submap_constraints = {}

    def add_submap_constraints(self, ts_from, ts_to, T_a_b):
        lc = LcModel(ts_from, ts_to, T_a_b)
        self.submap_constraints[ts_from].append(lc)

    def construct_path_msg(self):
        for ts_from in self.submap_constraints:
            loop_closures = list(self.submap_constraints[ts_from])
            self.construct_path_msg_for_submap(ts_from, loop_closures)

    def construct_path_msg_for_submap(self, ts_from, loop_closures):
        path_msg = Path()
        path_msg.header.stamp = ts_from

        for lc in loop_closures:
            t_a_b = lc.get_translation()
            q_a_b = lc.get_rotation_quat()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = nodes[i].ts
            pose_msg.pose.position.x = t_a_b[0]
            pose_msg.pose.position.y = t_a_b[1]
            pose_msg.pose.position.z = t_a_b[2]
            pose_msg.pose.orientation.x = q_a_b[0]
            pose_msg.pose.orientation.y = q_a_b[1]
            pose_msg.pose.orientation.z = q_a_b[2]
            pose_msg.pose.orientation.w = q_a_b[3]

            path_msg.poses.append(pose_msg)

        return path_msg
