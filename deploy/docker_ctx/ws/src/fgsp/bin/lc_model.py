#! /usr/bin/env python2

import numpy as np
from scipy.spatial.transform import Rotation

class LcModel(object):
    def __init__(self, ts_from, ts_to, T_a_b):
        self.ts_from = ts_from
        self.ts_to = ts_to
        self.T_a_b = self.convert_pose_msg(T_a_b)

    def convert_pose_msg(self, T_a_b):
        t_a_b = T_a_b.pose.position
        q_a_b = T_a_b.pose.orientation
        R_a_b = Rotation.from_quat([q_a_b.x, q_a_b.y, q_a_b.z, q_a_b.w])

        T_a_b = np.eye(4,4)
        # T_a_b[0:3,0:3] = R_a_b.as_matrix()
        T_a_b[0:3,0:3] = R_a_b.as_dcm()
        T_a_b[0:3, 3] = np.array([t_a_b.x, t_a_b.y, t_a_b.z])
        return T_a_b

    def get_translation(self):
        return self.T_a_b[0:3, 3]

    def get_rotation_quat(self):
        R = self.T_a_b[0:3,0:3]
        # return Rotation.from_matrix(R).as_quat() # x, y, z, w
        return Rotation.from_dcm(R).as_quat() # x, y, z, w
