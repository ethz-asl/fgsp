#! /usr/bin/env python3
from enum import Enum

import numpy as np
from src.fgsp.common.logger import Logger


class ConstraintType(Enum):
    SMALL = 1
    MID = 2
    LARGE = 3


class TransformHistory(object):
    def __init__(self):
        self.children = []
        self.transforms = []
        self.largest_diff = 0.1
        self.types = []

    def size(self):
        n_children = len(self.children)
        assert n_children == len(self.transforms)
        assert n_children == len(self.types)
        return n_children

    def add_record(self, child, T, constraint_type):
        if type(constraint_type) is not ConstraintType:
            Logger.LogError(
                'TransformHistory: Constraint type is not of type ConstraintType.')
            return
        if self.has_child(child):
            self.remove_child(child)

        self.children.append(child)
        self.transforms.append(T)
        self.types.append(constraint_type.value)

    def has_child(self, child):
        return child in self.children

    def remove_child(self, child):
        res = [x for x, z in enumerate(self.children) if z == child]
        if len(res) == 0:
            Logger.LogWarn(
                'TransformHistory: Index retrieval failed for removal.')
            return
        res = res[0]
        self.children.pop(res)
        self.transforms.pop(res)
        self.types.pop(res)

    def has_different_transform(self, child, T):
        if not self.has_child(child):
            return True

        res = [x for x, z in enumerate(self.children) if z == child]
        n_res = len(res)
        if n_res == 0:
            Logger.LogWarn(
                '[TransformHistory] Index retrieval failed for comparison.')
            return False

        if len(res) > 1:
            Logger.LogWarn(
                '[TransformHistory] Found multiple children for parent.')
        res = res[0]
        T_prev = self.transforms[res]
        T_diff = np.abs(T_prev - T)
        largest_diff = np.amax(T_diff)
        return largest_diff > self.largest_diff
