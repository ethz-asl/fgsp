#! /usr/bin/env python3
import rospy
import numpy as np

class SignalNode(object):
    def __init__(self):
        self.ts = None
        self.id = None
        self.robot_name = None
        self.pose = None
        self.residual = None

    def init(self, ts, id, robot_name, pose, residual):
        self.ts = ts
        self.id = id
        self.robot_name = robot_name
        self.pose = pose
        self.residual = residual
