import rospy

from utils import Utils

class MonitorConfig(object):
    def __init__(self):
        # general config
        self.rate = rospy.Rate(0.1)
        self.enable_graph_building = True
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
        self.enable_graph_building = rospy.get_param("~enable_graph_building")
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


class ClientConfig(object):
    def __init__(self):
        # general config
        self.rate = rospy.Rate(0.2)
        self.dataroot = "/home/berlukas/Documents/workspace/fgsp_ws/src/fgsp"
        self.random_forest_model = "/config/forest.joblib"
        self.robot_name = "cerberus"
        self.enable_submap_constraints = True
        self.enable_anchor_constraints = False
        self.enable_signal_recording = False
        self.enable_trajectory_recording = False
        self.signal_export_path = "/data/{key}_{src}_signal.npy"
        self.trajectory_export_path = "/data/{key}_{src}_trajectory.npy"


        # input
        self.opt_graph_topic = "/graph_monitor/sparse_graph/graph"
        self.opt_traj_topic = "/graph_monitor/sparse_graph/trajectory"
        self.est_traj_topic = "/trajectory"
        self.est_traj_path_topic = "/incremental_trajectory"
        self.submap_constraint_topic = "/graph_monitor/submaps"

        # output
        self.anchor_node_topic = "/graph_client/anchor_nodes"
        self.relative_node_topic = "/graph_client/relative_nodes"
        self.intra_constraint_topic = "/graph_client/intra_constraints"
        self.verification_service_topic = "/graph_monitor/verification"

        # input and output
        self.client_update_topic = "/graph_client/latest_graph"

    def init_from_config(self):
        # general config
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))
        self.dataroot = rospy.get_param("~dataroot")
        self.random_forest_model = rospy.get_param("~random_forest_model")
        self.robot_name = rospy.get_param("~robot_name")
        self.enable_submap_constraints = rospy.get_param("~enable_submap_constraints")
        self.enable_anchor_constraints = rospy.get_param("~enable_anchor_constraints")
        self.enable_signal_recording = rospy.get_param("~enable_signal_recording")
        self.enable_trajectory_recording = rospy.get_param("~enable_trajectory_recording")
        self.signal_export_path = rospy.get_param("~signal_export_path")
        self.trajectory_export_path = rospy.get_param("~trajectory_export_path")


        # input
        self.opt_graph_topic = rospy.get_param("~opt_graph_topic")
        self.opt_traj_topic = rospy.get_param("~opt_traj_topic")
        self.est_traj_topic = rospy.get_param("~est_traj_topic")
        self.est_traj_path_topic = rospy.get_param("~est_traj_path_topic")
        self.submap_constraint_topic = rospy.get_param("~opt_submap_constraint_topic")

        # output
        self.anchor_node_topic = rospy.get_param("~anchor_node_topic")
        self.relative_node_topic = rospy.get_param("~relative_node_topic")
        self.intra_constraint_topic = rospy.get_param("~intra_constraints")
        self.verification_service_topic = rospy.get_param("~verification_service")

        # input and output
        self.client_update_topic = rospy.get_param("~client_update_topic")

if __name__ == '__main__':
    from plotter import Plotter

    # cfg = MonitorConfig()
    # Plotter.PrintMonitorConfig(cfg)

    cfg = ClientConfig()
    Plotter.PrintClientConfig(cfg)
