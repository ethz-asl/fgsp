import math
import os

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseWithCovariance
import numpy as np
from scipy import spatial
from scipy.spatial.transform import Rotation

import torch
from base_controller import BaseController
from evaluation_set import EvaluationSet
from lc_candidate import LcCandidate
from model import Model
from reg_box import RegBox
from utils import Utils
from visualize import Visualize

from maplab_msgs.msg import SubmapConstraint
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='i', offset=12, datatype=PointField.FLOAT32, count=1),
]

class SubmapHandler(object):
    def __init__(self):
        self.pivot_distance = 8
        self.n_nearest_neighbors = 50
        self.p_norm = 2
        self.reg_box = RegBox()

        #submap_topic = rospy.get_param("~submap_constraint_topic")
        map_topic = '/s2loc/map'
        self.map_pub = rospy.Publisher(map_topic, PointCloud2, queue_size=10)
        self.visualizer = Visualize()
        self.submap_seq = 0
        self.compute_poses_in_LiDAR = False
        self.refine_with_ICP = False

    def publish_submaps(self, submaps):
        n_submaps = len(submaps)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        map_points = np.zeros((1,4))

        for i in range(0, n_submaps):
            T_G_L = submaps[i].get_pivot_pose_LiDAR()
            submap = submaps[i].compute_dense_map()
            submap[:,3] = i
            submap_points = Utils.transform_pointcloud(submap, T_G_L)
            map_points = np.append(map_points, submap_points, axis=0)

        map_points = Utils.downsample_pointcloud(map_points)
        n_points = map_points.shape[0]
        if n_points > 1:
            map_points = map_points[1:,:]
            map_pointcloud_ros = pc2.create_cloud(header, FIELDS_XYZ, map_points)
            self.map_pub.publish(map_pointcloud_ros)
            rospy.loginfo(f"Published map with {n_points} points.")

    def compute_constraints(self, submaps):
        candidates = self.find_close_submaps(submaps)
        if np.count_nonzero(candidates) == 0:
            rospy.logerr("Unable to find any close submaps.")
            return
        return self.evaluate_candidates(submaps, candidates)

    def find_close_submaps(self, submaps):
        n_submaps = len(submaps)
        candidates = np.zeros((n_submaps, n_submaps))
        if n_submaps <= 1:
            rospy.logerr("Not enough submaps to find candidates.")
            return candidates
        submap_positions = self.get_all_positions(submaps)
        tree = spatial.KDTree(submap_positions)

        for i in range(0, n_submaps):
            nn_dists, nn_indices = self.lookup_closest_submap(submap_positions, tree, i)
            if len(nn_indices) == 0 or len(nn_dists) == 0:
                continue
            for nn_i in nn_indices:
                candidates[i, nn_i] = 1

        # filter duplicates by zeroing the lower triangle
        return np.triu(candidates)

    def lookup_closest_submap(self, submaps, tree, idx):
        current_submap = submaps[idx, :]
        n_neighbors = min(len(submaps), self.n_nearest_neighbors)
        nn_dists, nn_indices = tree.query(
            current_submap,
            p=self.p_norm,
            k=n_neighbors,
            distance_upper_bound=self.pivot_distance)

        # Remove self and fix output.
        nn_dists, nn_indices = Utils.fix_nn_output(n_neighbors, idx, nn_dists, nn_indices)

        return nn_dists, nn_indices

    def get_all_positions(self, submaps):
        n_submaps = len(submaps)
        if n_submaps == 0:
            return
        positions = np.empty((n_submaps, 3))
        for i in range(0, n_submaps):
            positions[i, 0:3] = np.transpose(submaps[i].get_pivot_pose_IMU()[0:3, 3])
        return positions

    def evaluate_candidates(self, submaps, candidates):
        n_submaps = len(submaps)
        if n_submaps == 0 or len(candidates) == 0:
            return
        self.visualizer.resetConstraintVisualization()
        submap_msg = SubmapConstraint()
        for i in range(0, n_submaps):
            submap_msg = self.evaluate_neighbors_for(submaps, candidates, i, submap_msg)

        submap_msg.header.stamp = rospy.get_rostime()
        submap_msg.header.seq = self.submap_seq
        self.submap_seq += 1
        return submap_msg

    def evaluate_neighbors_for(self, submaps, candidates, i, submap_msg):
        neighbors = candidates[i,:]
        nnz = np.count_nonzero(neighbors)
        if nnz == 0:
            return submap_msg

        candidate_a = submaps[i]
        n_neighbors = len(neighbors)
        for j in range(0, n_neighbors):
            if neighbors[j] > 0:
                candidate_b = submaps[j]

                # Compute the alignment between the two submaps.
                T_L_a_L_b = self.compute_alignment(candidate_a, candidate_b)
                self.visualizer.visualizeCandidates(candidate_a, candidate_b, T_L_a_L_b)

                # Create a submap constraint message
                submap_msg = self.create_and_append_submap_constraint_msg(
                                candidate_a, candidate_b, T_L_a_L_b, submap_msg)
                self.verify_submap_message(submap_msg)

        return submap_msg

    def compute_alignment(self, candidate_a, candidate_b):
        points_a = candidate_a.compute_dense_map()
        points_b = candidate_b.compute_dense_map()

        # Compute prior transformation.
        T_a_b = None
        if self.compute_poses_in_LiDAR:
            T_L_G_a = np.linalg.inv(candidate_a.get_pivot_pose_LiDAR())
            T_G_L_b = candidate_b.get_pivot_pose_LiDAR()
            T_a_b = np.matmul(T_L_G_a, T_G_L_b)
        else:
            T_B_G_a = np.linalg.inv(candidate_a.get_pivot_pose_IMU())
            T_G_B_b = candidate_b.get_pivot_pose_IMU()
            T_a_b = np.matmul(T_B_G_a, T_G_B_b)

        if not self.refine_with_ICP:
            return T_a_b

        # Register the submaps.
        T = self.reg_box.register(points_a, points_b, T_a_b)
        #self.reg_box.draw_registration_result(points_a, points_b, T)
        #self.reg_box.draw_registration_result(points_a, points_b, T_L_a_L_b)
        if self.reg_box.verify_registration_result(T, T_a_b):
            return T
        else:
            return T_L_a_L_b

    def create_and_append_submap_constraint_msg(self, candidate_a, candidate_b, T_L_a_L_b, submap_msg):
        submap_msg.id_from.append(candidate_a.id)
        submap_msg.timestamp_from.append(candidate_a.get_pivot_timestamp_ros())
        submap_msg.robot_name_from.append(candidate_a.robot_name)

        submap_msg.id_to.append(candidate_b.id)
        submap_msg.timestamp_to.append(candidate_b.get_pivot_timestamp_ros())
        submap_msg.robot_name_to.append(candidate_b.robot_name)

        t = T_L_a_L_b[0:3,3]
        q = Rotation.from_matrix(T_L_a_L_b[0:3,0:3]).as_quat() # x, y, z, w

        pose_cov_msg = PoseWithCovariance()
        pose_msg = Pose()
        pose_msg.position.x = t[0]
        pose_msg.position.y = t[1]
        pose_msg.position.z = t[2]
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        pose_cov_msg.pose = pose_msg
        submap_msg.T_a_b.append(pose_cov_msg)

        return submap_msg

    def verify_submap_message(self, submap_msg):
        n_id_from = len(submap_msg.id_from)
        n_id_to = len(submap_msg.id_to)

        n_ts_from = len(submap_msg.timestamp_from)
        n_ts_to = len(submap_msg.timestamp_to)

        n_robot_name_to = len(submap_msg.robot_name_to)
        n_robot_name_from = len(submap_msg.robot_name_from)

        n_poses = len(submap_msg.T_a_b)

        assert(n_id_from == n_id_to)
        assert(n_id_from == n_ts_from)
        assert(n_id_from == n_ts_to)
        assert(n_id_from == n_robot_name_to)
        assert(n_id_from == n_robot_name_from)
        assert(n_id_from == n_poses)

if __name__ == "__main__":
    submap_handler = SubmapHandler()
