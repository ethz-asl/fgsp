#! /usr/bin/env python3

import rospy

from robot_constraints import RobotConstraints

class ConstraintHandler(object):
    def __init__(self):
        self.intra_contraints = {}
        self.inter_contraints = {}

    def add_constraints(self, constraint_msg):
        self.verify_constraint_msg(constraint_msg)
        n_constraints  = len(constraint_msg.id_from)
        rospy.loginfo(f"Submap constraint message is verified. Processing {n_constraints} constraints")

        # Received new message, reinitialize the constraints.
        # Each message contains all currently used constraints.
        self.intra_contraints = {}
        self.inter_contraints = {}

        for i in range(0, n_constraints):
            if constraint_msg.robot_name_to[i] == constraint_msg.robot_name_from[i]:
                self.process_intra_constraints(constraint_msg, i)

    def verify_constraint_msg(self, constraint_msg):
        n_id_from = len(constraint_msg.id_from)
        n_id_to = len(constraint_msg.id_to)

        n_ts_from = len(constraint_msg.timestamp_from)
        n_ts_to = len(constraint_msg.timestamp_to)

        n_robot_name_to = len(constraint_msg.robot_name_to)
        n_robot_name_from = len(constraint_msg.robot_name_from)

        n_poses = len(constraint_msg.T_a_b)

        assert(n_id_from == n_id_to)
        assert(n_id_from == n_ts_from)
        assert(n_id_from == n_ts_to)
        assert(n_id_from == n_poses)

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
