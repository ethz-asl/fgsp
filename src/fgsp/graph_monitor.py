#! /usr/bin/env python3

import copy
import time
from maplab_msgs.msg import Graph, Trajectory, Submap, SubmapConstraint
from multiprocessing import Lock

import rclpy
from rclpy.node import Node

from src.fgsp.graph.global_graph import GlobalGraph
from src.fgsp.controller.signal_handler import SignalHandler
from src.fgsp.controller.submap_handler import SubmapHandler
from src.fgsp.common.submap_model import SubmapModel
from src.fgsp.common.config import MonitorConfig
from src.fgsp.common.plotter import Plotter
from src.fgsp.common.logger import Logger


class GraphMonitor(Node):
    def __init__(self):
        super().__init__('graph_monitor')
        self.is_initialized = False

        Plotter.PlotMonitorBanner()
        self.config = MonitorConfig(self)
        self.config.init_from_config()
        Plotter.PrintMonitorConfig(self.config)
        Plotter.PrintSeparator()

        self.mutex = Lock()
        self.mutex.acquire()
        # Publishers and subscribers.
        if self.config.enable_submap_constraints:
            self.submap_sub = self.create_subscription(
                Submap, self.config.opt_pc_topic, self.submap_callback, 10)
            self.submap_pub = self.create_publisher(
                SubmapConstraint, self.config.submap_topic, 10)
        if self.config.enable_graph_building:
            self.graph_sub = self.create_subscription(
                Graph, self.config.in_graph_topic, self.graph_callback, 10)
            self.traj_sub = self.create_subscription(
                Trajectory, self.config.in_traj_opt_topic, self.traj_opt_callback, 10)
            self.graph_pub = self.create_publisher(
                Graph, self.config.out_graph_topic, 10)
            self.traj_pub = self.create_publisher(
                Trajectory, self.config.out_traj_opt_topic, 10)

        # Handlers and evaluators.
        self.graph = GlobalGraph(
            self.config, reduced=self.config.reduce_global_graph)
        self.optimized_signal = SignalHandler(self.config)
        self.submap_handler = SubmapHandler(self.config)

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.submaps = {}
        self.submap_counter = {}
        self.is_initialized = True
        self.latest_opt_traj_msg = None
        self.mutex.release()
        self.timer = self.create_timer(1 / self.config.rate, self.update)

    def graph_callback(self, msg):
        if self.is_initialized is False:
            return
        Logger.LogInfo('GraphMonitor: Received graph message from server.')
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.graph.msg_contains_updates(msg) is True:
            self.graph.build(msg)

        self.mutex.release()

    def traj_opt_callback(self, msg):
        if self.is_initialized is False:
            return

        keys = self.optimized_signal.convert_signal(msg)

        # If the graph is reduced, we need to reduce the optimized nodes too.
        if self.graph.is_reduced:
            msg.nodes = [msg.nodes[i] for i in self.graph.reduced_ind]

        if self.graph.has_skipped():
            msg.nodes = [element for i, element in enumerate(
                msg.nodes) if i not in self.graph.skip_ind]

        for key in keys:
            if self.key_in_optimized_keys(key):
                continue
            self.optimized_keys.append(key)
        self.latest_opt_traj_msg = msg

    def update(self):
        # Compute the submap constraints and publish them if enabled.
        if self.config.enable_submap_constraints:
            self.compute_and_publish_submaps()

        # Compute the global graph and signal, then publish it
        if self.config.enable_graph_building:
            self.compute_and_publish_graph()

        try:
            self.graph.publish()
            self.optimized_signal.publish()
        except Exception as e:
            Logger.LogError(
                'GraphMonitor: Unable to publish results to client.')

    def compute_and_publish_graph(self):
        self.mutex.acquire()
        if self.graph.is_built is False:
            Logger.LogWarn('GraphMonitor: Graph is not built yet!')
            self.mutex.release()
            return
        Logger.LogInfo(
            f'GraphMonitor: Computing global graph with {self.graph.graph_size()}.')
        if self.graph.graph_size() < self.config.min_node_count:
            Logger.LogWarn(
                f'GraphMonitor: Not enough nodes ({self.graph.graph_size()} < {self.config.min_node_count}')
            self.mutex.release()
            return
        self.mutex.release()

        # Publish the graph to the clients.
        self.publish_graph_and_traj()

    def submap_callback(self, submap_msg):
        submap = SubmapModel()
        submap.construct_data(submap_msg)
        submap.compute_dense_map()

        id = submap.id
        self.mutex.acquire()
        self.submaps[id] = submap
        if not id in self.submap_counter:
            self.submap_counter[id] = 0
        else:
            self.submap_counter[id] += 1

        self.mutex.release()

    def compute_and_publish_submaps(self):
        n_submaps = len(self.submaps)
        if n_submaps != len(self.submap_counter) or n_submaps == 0:
            return

        self.mutex.acquire()
        submaps = copy.deepcopy(self.submaps)
        submap_counter = copy.deepcopy(self.submap_counter)
        self.mutex.release()

        # Filter out submaps based on min count.
        if self.config.submap_min_count > 0:
            for k, v in submap_counter.items():
                if v < self.config.submap_min_count:
                    submaps.pop(k)

        self.publish_all_submaps(submaps)
        msg = self.compute_submap_constraints(submaps)
        if msg is not None:
            self.submap_pub.publish(msg)

    def compute_submap_constraints(self, submaps):
        n_submaps = len(submaps)
        if n_submaps == 0:
            return None
        Logger.LogWarn(
            f'GraphMonitor: Computing constraints for {n_submaps} submaps.')
        return self.submap_handler.compute_constraints(submaps)

    def publish_graph_and_traj(self):
        graph_msg = self.graph.to_graph_msg()
        self.pub_graph.publish(graph_msg)
        Logger.LogInfo('GraphMonitor: Published global graph.')

        if self.config.send_separate_traj_msgs:
            self.send_separate_traj_msgs()
        elif self.latest_opt_traj_msg is not None:
            self.pub_traj.publish(self.latest_opt_traj_msg)
            Logger.LogInfo(
                f'GraphMonitor: Published trajectory for keys {self.optimized_keys}.')

    def send_separate_traj_msgs(self):
        for key in self.optimized_keys:
            traj_msg = self.optimized_signal.to_signal_msg(key)
            self.pub_traj.publish(traj_msg)
            time.sleep(0.10)
            Logger.LogInfo(
                f'GraphMonitor: Published separate trajectory for {key}.')

    def publish_all_submaps(self, submaps):
        self.submap_handler.publish_submaps(submaps)

    def key_in_optimized_keys(self, key):
        return any(key in k for k in self.optimized_keys)


def main(args=None):
    rclpy.init(args=args)
    monitor = GraphMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
