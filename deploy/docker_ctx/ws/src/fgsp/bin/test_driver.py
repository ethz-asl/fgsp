#! /usr/bin/env python2

import rospy
import pandas
import numpy as np

from maplab_msgs.msg import Trajectory, TrajectoryNode
from geometry_msgs.msg import Pose
from utils import Utils

class TestDriver(object):

    def __init__(self):
        # Send every 10 seconds.
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))
        traj_topic = rospy.get_param("~traj_topic")

        self.pub = rospy.Publisher(traj_topic, Trajectory, queue_size=10)


    def update(self):
        (loam_df, rovio_df) = self.read_estimations()
        rospy.logdebug(f'[TestDriver] Read {loam_df.size} (loam) and {rovio_df.size} (rovio) entries.')

        loam_msg = self.create_trajectory_message(loam_df, "cerberus")
        #rovio_msg = self.create_trajectory_message(rovio_df, "penguin")
        self.pub.publish(loam_msg)
        rospy.logdebug(f'[TestDriver] Published trajectory message.')

    def read_estimations(self):
        dataset_path = '/home/berlukas/Documents/workspace/fgsp_ws/src/fgsp/data/mission_03/'
        loam_file = dataset_path + 'loam_header_less.csv'
        rovio_file = dataset_path + 'rovio_header_less.csv'

        labels = ['ts', 'vertex-id', 'mission_id', 'p_G_Ix', 'p_G_Iy', 'p_G_Iz', 'q_G_Iw', 'q_G_Ix', 'q_G_Iy', 'q_G_Iz', 'p_M_Ix', 'p_M_Iy', 'p_M_Iz', 'q_M_Iw', 'q_M_Ix', 'q_M_Iy', 'q_M_Iz', 'v_Mx', 'v_My', 'v_Mz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz']
        loam_df = pandas.read_csv(loam_file, names=labels, delimiter=',', header=None)
        rovio_df = pandas.read_csv(rovio_file, names=labels, delimiter=',', header=None)

        return (loam_df, rovio_df)

    def create_trajectory_message(self, df, robot_name):
        nodes = df[['ts', 'p_G_Ix', 'p_G_Iy', 'p_G_Iz']].to_numpy()
        traj_msg = Trajectory()

        n_nodes = nodes.shape[0]
        ts = 0
        for i in range(0, n_nodes):
            cur_node = nodes[i,:]
            ts = Utils.ts_ns_to_ros_time(cur_node[0])

            pose_msg = PoseStamped()
            pose_msg.header.stamp = ts
            pose_msg.pose.position.x = cur_node[1]
            pose_msg.pose.position.y = cur_node[2]
            pose_msg.pose.position.z = cur_node[3]

            node_msg = TrajectoryNode()
            node_msg.robot_name = robot_name
            node_msg.pose = pose_msg

            traj_msg.nodes.append(node_msg)

        traj_msg.header.stamp = ts
        return traj_msg

if __name__ == '__main__':
    rospy.init_node('test_driver')
    node = TestDriver()
    while not rospy.is_shutdown():
        node.rate.sleep()
        node.update()
