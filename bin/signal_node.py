#! /usr/bin/env python3
import rospy
import numpy as np

class SignalNode(object):
    def __init__(self):
        self.ts = None
        self.id = None
        self.robot_name = None
        self.position = None
        self.orientation = None
        self.residual = None

    def init(self, ts, id, robot_name, position, orientation, residual):
        self.ts = ts
        self.id = id
        self.robot_name = robot_name
        self.position = position
        self.orientation = orientation
        self.residual = residual

    def init_onboard(self, ts, robot_name, position, orientation):
        self.ts = ts
        self.robot_name = robot_name
        self.position = position
        self.orientation = orientation
        
