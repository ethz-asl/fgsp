#! /usr/bin/env python3

import rospy

from robot_constraints import RobotConstraints

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
        self.intra_contraints = {}
        self.inter_contraints = {}

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
            return False
        if n_id_from != n_ts_from:
            return False
        if n_id_from != n_ts_to:
            return False
        if n_id_from != n_poses:
            return False

        time_now = rospy.Time.now()
        rospy.loginfo(f'Current time is {time_now.to_nsec()}')
        for i in range(0, n_ts_from):
            rospy.loginfo(f' - Time from {constraint_msg.timestamp_from[i].to_nsec()}')
            rospy.loginfo(f' - Time to {constraint_msg.timestamp_to[i].to_nsec()}')

            diff_from = time_now - constraint_msg.timestamp_from[i]
            diff_to = time_now - constraint_msg.timestamp_to[i]
            rospy.loginfo(f'diff_from is {diff_from} and diff_to is {diff_to}')
            if diff_from.to_nsec() < 0 or diff_to.to_nsec() < 0:
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

    def create_msg_for_intra_constraints(self, robot_name):
        if robot_name not in self.intra_contraints:
            rospy.logerr(f"Robot {robot_name} does not have intra mission constraints.")
            return []
        return self.intra_contraints[robot_name].construct_path_msgs()
