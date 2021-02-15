#! /usr/bin/env python3

import rospy
import pandas
import numpy as np

from maplab_msgs.msg import Trajectory, TrajectoryNode

class TestDriver(object):

    def __init__(self):
        # Send every 10 seconds.
        self.rate = rospy.Rate(0.1)

    def update(self):
        (loam_df, rovio_df) = self.read_estimations()
        rospy.loginfo(f'[TestDriver] Read {loam_df.size} (loam) and {rovio_df.size} (rovio) entries.')

    def read_estimations(self):
        dataset_path = '/home/berlukas/Documents/workspace/fgsp_ws/src/fgsp/data/mission_03/'
        loam_file = dataset_path + 'loam_header_less.csv'
        rovio_file = dataset_path + 'rovio_header_less.csv'

        labels = ['ts', 'vertex-id', 'mission_id', 'p_G_Ix', 'p_G_Iy', 'p_G_Iz', 'q_G_Iw', 'q_G_Ix', 'q_G_Iy', 'q_G_Iz', 'p_M_Ix', 'p_M_Iy', 'p_M_Iz', 'q_M_Iw', 'q_M_Ix', 'q_M_Iy', 'q_M_Iz', 'v_Mx', 'v_My', 'v_Mz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz']
        loam_df = pandas.read_csv(loam_file, names=labels, delimiter=',', header=None)
        rovio_df = pandas.read_csv(rovio_file, names=labels, delimiter=',', header=None)
        return (loam_df, rovio_df)



if __name__ == '__main__':
    rospy.init_node('test_driver')
    node = TestDriver()
    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
