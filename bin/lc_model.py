#! /usr/bin/env python3

from scipy.spatial.transform import Rotation

class LcModel(object):
    def __init__(self, ts_from, ts_to, T_a_b):
        self.ts_from = None
        self.ts_to = None
        self.T_a_b = None

    def get_translation(self):
        return self.T_a_b[0:3, 3]

    def get_rotation_quat(self):
        R = self.T_a_b[0:3,0:3]
        return Rotation.from_matrix(R).as_quat() # x, y, z, w
