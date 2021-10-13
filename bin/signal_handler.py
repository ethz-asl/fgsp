#! /usr/bin/env python2
import rospy
import numpy as np
from pygsp import graphs, filters, reduction
from maplab_msgs.msg import Trajectory, TrajectoryNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from utils import Utils
from signal_node import SignalNode
from visualizer import Visualizer

class SignalHandler(object):
    def __init__(self):
        self.signals = {}

    def group_robots(self, signals):
        n_nodes = len(signals)
        grouped_signals = {}
        for i in range(0, n_nodes):
            robot = signals[i].robot_name
            if not robot in grouped_signals.keys():
                grouped_signals[robot] = []

            grouped_signals[robot].append(signals[i])
        return grouped_signals

    def publish_grouped_robots(self, grouped_signals):
        for key, nodes in grouped_signals.items():
            path_msg = Path()
            for node in nodes:
                path_msg.poses.append(node.pose)
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = 'darpa'
            pub = rospy.Publisher('/graph_monitor/{key}/monitor_path'.format(key=key), Path, queue_size=10)
            pub.publish(path_msg)

    def convert_signal(self, signal_msg):
        grouped_signals = self.group_robots(signal_msg.nodes)
        self.publish_grouped_robots(grouped_signals)
        rospy.loginfo('[SignalHandler] Grouped signals are {keys}'.format(keys=grouped_signals.keys()))

        for key, nodes in grouped_signals.items():
            n_nodes = len(nodes)
            rospy.logwarn('[SignalHandler] For {key} we have {n_nodes} nodes.'.format(key=key, n_nodes=n_nodes))
            if (n_nodes <= 0):
                continue

            signals = [None] * n_nodes
            for i in range(0, n_nodes):
                signals[i] = self.convert_trajectory_node(nodes[i])
            self.signals[key] = signals

        return grouped_signals.keys()

    def convert_signal_from_path(self, path_msg, robot_name):
        n_poses = len(path_msg.poses)
        if (n_poses <= 0):
            return ""

        # key = path_msg.header.frame_id
        key = robot_name
        signals = [None] * n_poses
        for i in range(0, n_poses):
            signals[i] = self.convert_path_node(path_msg.poses[i], key)

        self.signals[key] = signals
        return key

    def get_all_nodes(self, key):
        if key in self.signals:
            return self.signals[key]
        else:
            return []

    def get_number_of_submaps(self, key):
        if key in self.signals:
            return self.signals[key][-1].id + 1
        else:
            return 0

    def get_nodes_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        filtered_nodes = []
        for node in nodes:
            if node.id == id:
                filtered_nodes.append(node)
        return filtered_nodes

    def get_mask_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)
        mask = [False] * n_nodes
        for i in range(0, n_nodes):
            if nodes[i].id == id:
                mask[i] = True
        return mask

    def get_indices_for_submap(self, key, id):
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)
        indices = []
        for i in range(0, n_nodes):
            if nodes[i].id == id:
                indices.append(i)
        return indices

    def convert_trajectory_node(self, node_msg):
        id = node_msg.id;
        robot_name = node_msg.robot_name
        pose_msg = node_msg.pose
        ts = pose_msg.header.stamp
        position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        orientation = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        residual = node_msg.signal

        signal = SignalNode()
        signal.init(ts, id, robot_name, position, orientation, residual)
        return signal

    def convert_path_node(self, pose_msg, robot_name):
        ts = pose_msg.header.stamp
        position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        orientation = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        degenerate = pose_msg.header.frame_id.lower() == 'degenerate'

        signal = SignalNode()
        signal.init_onboard(ts, robot_name, position, orientation, degenerate)
        return signal

    def compute_signal_from_key(self, key):
        nodes = self.signals[key]
        traj = self.compute_trajectory(nodes)
        traj_origin = traj[0,1:4]

        pos_signal = (traj[:,1:4] - traj_origin).squeeze()

        x = np.linalg.norm(pos_signal, ord=2, axis=1)
        return x

    def compute_pos_signal(self, nodes):
        traj = self.compute_trajectory(nodes)
        traj_origin = traj[0,1:4]

        pos_signal = (traj[:,4:8] - traj_origin).squeeze()

        return np.linalg.norm(pos_signal, ord=2, axis=1)

    def compute_rot_signal(self, nodes):
        traj = self.compute_trajectory(nodes)
        wxyz = traj[0,4:8]
        traj_origin = Rotation.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]).as_dcm()

        n_nodes = len(nodes)
        x_rot = [0] * n_nodes
        for i in range(0, n_nodes):
            wxyz = traj[i,4:8]
            rot_diff = np.matmul(opt_origin, Rotation.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]).as_matrix().transpose())
            x_rot[i] = np.trace(rot_diff)
        return np.array(x_rot)

    def compute_signal(self, nodes):
        positions = self.compute_pos_signal(nodes)
        rotations = self.compute_rot_signal(nodes)
        return np.column_stack((positions, rotations))

    def compute_trajectory(self, nodes):
        n_nodes = len(nodes)
        trajectory = np.zeros((n_nodes, 8))
        for i in range(n_nodes):
            trajectory[i,0] = Utils.ros_time_to_ns(nodes[i].ts)
            trajectory[i,1:4] = nodes[i].position
            trajectory[i,4:8] = nodes[i].orientation

        return trajectory

    def to_signal_msg(self, key):
        traj_msg = Trajectory()
        node_msg = TrajectoryNode()
        nodes = self.get_all_nodes(key)
        n_nodes = len(nodes)

        for i in range(n_nodes):
            node_msg = TrajectoryNode()
            node_msg.id = nodes[i].id;
            node_msg.robot_name = nodes[i].robot_name

            pose_msg = PoseStamped()
            pose_msg.header.stamp = nodes[i].ts
            pose_msg.pose.position.x = nodes[i].position[0]
            pose_msg.pose.position.y = nodes[i].position[1]
            pose_msg.pose.position.z = nodes[i].position[2]
            pose_msg.pose.orientation.w = nodes[i].orientation[0]
            pose_msg.pose.orientation.x = nodes[i].orientation[1]
            pose_msg.pose.orientation.y = nodes[i].orientation[2]
            pose_msg.pose.orientation.z = nodes[i].orientation[3]

            node_msg.pose = pose_msg
            node_msg.signal = nodes[i].residual
            traj_msg.nodes.append(node_msg)

        return traj_msg

    def publish(self):
        topic_fmt = '/graph_monitor/signal/{key}_trajectory'
        color_idx = 0
        viz = Visualizer()
        for robot, nodes in self.signals.items():
            rospy.logwarn('[SignalHandler] Publishing signal for {robot}'.format(robot=robot))
            topic = topic_fmt.format(key=robot)
            for node in nodes:
                viz.add_signal_coordinate(node.position, color_idx)
            viz.visualize_signals(topic)
            viz.resetConstraintVisualization()
            color_idx += 1


if __name__ == '__main__':
    sh = SignalHandler()
    sh.to_signal_msg("foo")
