#! /usr/bin/env python2
import os

import rospy
import numpy as np
from pygsp import graphs, filters, reduction

import scipy.spatial
from sklearn.ensemble import RandomForestClassifier
from sklearn import metrics
from joblib import dump, load

class RandomForestPredictor(object):
    def __init__(self, use_ros = False):
        if use_ros and rospy.has_param('~random_forest_model'):
            dataroot = rospy.get_param("~dataroot")
            path_to_model = rospy.get_param("~random_forest_model")
            random_forest_model = dataroot + path_to_model
        else:
            random_forest_model = "../config/forest.joblib"

        rospy.loginfo(f"[RandomForestPredictor] Loading model from {random_forest_model}")
        try:
            self.clf = load(random_forest_model)
            self.initialized = True
        except:
            self.initialized = False
            rospy.logerr(f'[RandomForestPredictor] Failed to load the model.')


    def predict(self, X):
        X = X.fillna(0)
        X = X.replace(float('inf'), 1)

        prediction = self.clf.predict(X)
        prediction_prob = self.clf.predict_proba(X)

        return (prediction, prediction_prob)
