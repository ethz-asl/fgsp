import rospy

from utils import Utils

class MonitorConfig(object):
    def __init__(self):
        # general config
        self.rate = 0.1
        self.enable_submap_constraints = True
        self.min_node_count = 10
        self.reduce_global_graph = False
        self.submap_min_ts_diff_ns = 10000000000

        # submap constraints
        self.pivot_distance = 20
        self.n_nearest_neighbors = 50
        self.p_norm = 2
        self.enable_submap_map_publishing = False
        self.compute_poses_in_LiDAR = False
        self.refine_with_ICP = False

        # input
        self.in_graph_topic = '/maplab_server/sparse_graph/graph'
        self.in_traj_opt_topic = '/maplab_server/sparse_graph/trajectory'
        self.verification_service_topic = '/grap_monitor/verification'
        self.opt_pc_topic = '/maplab_server/sparse_graph/submap'

        # output
        self.out_graph_topic = '/graph_monitor/sparse_graph/graph'
        self.out_traj_opt_topic = '/graph_monitor/sparse_graph/trajectory'
        self.submap_topic = '/graph_monitor/submaps'

    def init_from_config(self):
        # general config
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))
        self.enable_submap_constraints = rospy.get_param("~enable_submap_constraints")
        self.min_node_count = rospy.get_param("~min_node_count")
        self.reduce_global_graph = rospy.get_param("~reduce_global_graph")
        self.submap_min_count = rospy.get_param("~submap_min_count")

        # submap constraints
        self.pivot_distance = rospy.get_param("~submap_constraint_pivot_distance")
        self.n_nearest_neighbors = rospy.get_param("~submap_constraint_knn")
        self.p_norm = rospy.get_param("~submap_constraint_p_norm")
        self.enable_submap_map_publishing = rospy.get_param("~enable_submap_map_publishing")
        self.compute_poses_in_LiDAR = rospy.get_param("~submap_constraint_export_lidar_poses")
        self.refine_with_ICP = rospy.get_param("~submap_constraint_refine_icp")

        # input
        self.in_graph_topic = rospy.get_param("~in_graph_topic")
        self.in_traj_opt_topic = rospy.get_param("~in_traj_opt_topic")
        self.verification_service_topic = rospy.get_param("~verification_service")
        self.opt_pc_topic = rospy.get_param("~opt_pc_topic")

        # output
        self.out_graph_topic = rospy.get_param("~out_graph_topic")
        self.out_traj_opt_topic = rospy.get_param("~out_traj_opt_topic")
        self.submap_topic = rospy.get_param("~submap_constraint_topic")
        self.accumulated_map_topic = rospy.get_param("~accumulated_map_topic")



if __name__ == '__main__':
    from plotter import Plotter

    monitor_cfg = MonitorConfig()
    Plotter.PrintMonitorConfig(monitor_cfg)
