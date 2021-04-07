#! /usr/bin/env python3

class ConstraintHandler(object):
    def __init__(self):
        self.intra_contraints = {}
        self.inter_contraints = {}

    def add_constraints(self, constraint_msg):
        self.verify_constraint_msg(constraint_msg)
        n_constraints  = len(constraint_msg.id_from)
        for i in range(0, n_constraints):
            if constraint_msg.robot_name_to[i] == constraint_msg.robot_name_from[i]:
                self.process_intra_constraints(constraint_msg, i)

    def verify_constraint_msg(self, constraint_msg):
        n_id_from = len(constraint_msg.id_from)
        n_id_to = len(constraint_msg.id_to)

        n_ts_from = len(constraint_msg.timestamp_from)
        n_ts_to = len(constraint_msg.timestamp_to)

        n_robot_name_to = len(submap_msg.robot_name_to)
        n_robot_name_from = len(submap_msg.robot_name_from)

        n_poses = len(constraint_msg.T_a_b)

        assert(n_id_from == n_id_to)
        assert(n_id_from == n_ts_from)
        assert(n_id_from == n_ts_to)
        assert(n_id_from == n_poses)

    def process_intra_constraints(self, constraint_msg, i):
        robot_name = constraint_msg.robot_name_from[i]
        
