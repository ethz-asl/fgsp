#! /usr/bin/env python3

import copy
import numpy as np
import open3d as o3d

class RegBox(object):

    def __init__(self):
        self.threshold = 0.02
        self.t_verification_threshold = 1
        self.R_verification_threshold = 0.3

    def register(self, points_a, points_b, T_prior=None):
        if T_prior is None:
            T_prior = np.eye(4,4)
        cloud_a = self.create_point_cloud(points_a)
        cloud_b = self.create_point_cloud(points_b)
        return self.apply_point_to_point(cloud_a, cloud_b, T_prior)
        #return self.apply_point_to_plane(cloud_a, cloud_b, T_prior)


    def create_point_cloud(self, points):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
        return pcd

    def apply_point_to_point(self, source, target, T_prior):
        source = source.voxel_down_sample(voxel_size=0.05)
        target = source.voxel_down_sample(voxel_size=0.05)
        result = o3d.pipelines.registration.registration_icp(
            source, target, self.threshold, T_prior,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        return result.transformation

    def apply_point_to_plane(self, source, target, T_prior):
        source = source.voxel_down_sample(voxel_size=0.05)
        target = source.voxel_down_sample(voxel_size=0.05)

        source.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        result = o3d.pipelines.registration.registration_icp(
            source, target, self.threshold, T_prior,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        return result.transformation

    def draw_registration_result(self, points_a, points_b, T_a_b):
        cloud_a = self.create_point_cloud(points_a)
        cloud_b = self.create_point_cloud(points_b)
        cloud_a.paint_uniform_color([1, 0.706, 0])
        cloud_b.paint_uniform_color([0, 0.651, 0.929])
        cloud_b.transform(T_a_b)
        o3d.visualization.draw_geometries([cloud_a, cloud_b])

    def verify_registration_result(self, T_reg_a_b, T_prior_a_b):
        t_reg = T_reg_a_b[0:3,3]
        t_prior = T_reg_a_b[0:3,3]
        R_reg = T_reg_a_b[0:3,0:3]
        R_prior = T_prior_a_b[0:3,0:3]

        is_valid = True
        t_diff = np.linalg.norm(t_reg - t_prior)
        if (t_diff > self.t_verification_threshold):
            is_valid = False

        R_diff = np.trace(np.linalg.inv(R_reg) - R_prior)
        if R_diff > self.R_verification_threshold:
            is_valid = False
        return is_valid


def random_test(regbox):
    points_a = np.random.rand(100,4)
    points_b = np.random.rand(100,4)
    T_reg = regbox.register(points_a, points_b)
    print(f"Registration result:\n {T_reg}")


if __name__ == "__main__":
    regbox = RegBox()
    random_test(regbox)
