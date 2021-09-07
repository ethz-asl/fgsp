#! /usr/bin/env python3

import rospy
import numpy as np
from robot_constraints import RobotConstraints
from utils import Utils

class ConstraintHandler(object):
    def __init__(self):
        self.intra_contraints = {}
        self.inter_contraints = {}

    def add_constraints(self, constraint_msg):
        if not self.verify_constraint_msg(constraint_msg):
            rospy.logerr("Submap constraint verfication failed.")
            return False
        n_constraints  = len(constraint_msg.id_from)
        rospy.loginfo(f"Submap constraint message is verified. Processing {n_constraints} constraints")

        # Received new message, reinitialize the constraints.
        # Each message contains all currently used constraints.
        # self.intra_contraints = {}
        # self.inter_contraints = {}

        for i in range(0, n_constraints):
            if constraint_msg.robot_name_to[i] == constraint_msg.robot_name_from[i]:
                self.process_intra_constraints(constraint_msg, i)
        return True

    def verify_constraint_msg(self, constraint_msg):
        n_id_from = len(constraint_msg.id_from)
        n_id_to = len(constraint_msg.id_to)

        n_ts_from = len(constraint_msg.timestamp_from)
        n_ts_to = len(constraint_msg.timestamp_to)

        n_robot_name_to = len(constraint_msg.robot_name_to)
        n_robot_name_from = len(constraint_msg.robot_name_from)

        n_poses = len(constraint_msg.T_a_b)

        if n_id_from != n_id_to:
            rospy.logerr("[ConstraintHandler] We have a ID mismatch.")
            return False
        if n_id_from != n_ts_from:
            rospy.logerr("[ConstraintHandler] We have a TS (from) mismatch.")
            return False
        if n_id_from != n_ts_to:
            rospy.logerr("[ConstraintHandler] We have a TS (to) mismatch.")
            return False
        if n_id_from != n_poses:
            rospy.logerr("[ConstraintHandler] We have a poses mismatch.")
            return False

        time_now = rospy.Time.now()
        for i in range(0, n_ts_from):
            diff_from = time_now - constraint_msg.timestamp_from[i]
            diff_to = time_now - constraint_msg.timestamp_to[i]
            if diff_from.to_nsec() < 0 or diff_to.to_nsec() < 0:
                rospy.logerr(f"[ConstraintHandler] Difference is negative. From is {diff_from.to_nsec()} and to is {diff_to.to_nsec()}.")
                return False
        return True

    def process_intra_constraints(self, constraint_msg, i):
        robot_name = constraint_msg.robot_name_from[i]
        # Add the key if it is not present in the dict.
        if robot_name not in self.intra_contraints:
            self.intra_contraints[robot_name] = RobotConstraints()

        # Add the constraint to the respective robot.
        self.intra_contraints[robot_name].add_submap_constraints(
            constraint_msg.timestamp_from[i], constraint_msg.timestamp_to[i], constraint_msg.T_a_b[i])

    def get_constraints_from(self, robot_name):
        if robot_name not in self.intra_contraints:
            return []
        else:
            return self.intra_contraints[robot_name]

    def filter_large_constraints(self, labels, all_opt_nodes):
        timestamps = []
        if labels == None:
            return np.array(timestamps)
        n_labels = labels.size()
        for i in range(n_labels):
            if not 3 in labels.labels[i]:
                continue
            timestamps.append(Utils.ros_time_to_ns(all_opt_nodes[i].ts))
        return np.array(timestamps)

    def create_msg_for_intra_constraints(self, robot_name, labels, all_opt_nodes):
        if robot_name not in self.intra_contraints:
            rospy.logerr(f"Robot {robot_name} does not have intra mission constraints.")
            return []
        timestamps = self.filter_large_constraints(labels, all_opt_nodes)
        if timestamps.shape[0] > 0:
            return self.intra_contraints[robot_name].construct_path_msgs_using_ts(timestamps)
        else:
            return []
