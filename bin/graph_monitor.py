#! /usr/bin/env python2

import rospy
import copy
import time
from maplab_msgs.msg import *
from multiprocessing import Lock

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from submap_handler import SubmapHandler
from submap_model import SubmapModel
from config import MonitorConfig
from plotter import Plotter

class GraphMonitor(object):
    def __init__(self):
        self.is_initialized = False

        Plotter.PlotMonitorBanner()
        self.config = MonitorConfig()
        self.config.init_from_config()
        Plotter.PrintMonitorConfig(self.config)
        Plotter.PrintSeparator()

        self.mutex = Lock()
        self.mutex.acquire()
        # Publishers and subscribers.
        if self.config.enable_submap_constraints:
            rospy.Subscriber(self.config.opt_pc_topic, Submap, self.submap_callback)
            self.submap_pub = rospy.Publisher(self.config.submap_topic, SubmapConstraint, queue_size=10)
        if self.config.enable_graph_building:
            rospy.Subscriber(self.config.in_graph_topic, Graph, self.graph_callback)
            rospy.Subscriber(self.config.in_traj_opt_topic, Trajectory, self.traj_opt_callback)
            self.pub_graph = rospy.Publisher(self.config.out_graph_topic, Graph, queue_size=10)
            self.pub_traj = rospy.Publisher(self.config.out_traj_opt_topic, Trajectory, queue_size=10)

        # Handlers and evaluators.
        self.graph = GlobalGraph(self.config, reduced=self.config.reduce_global_graph)
        self.optimized_signal = SignalHandler(self.config)
        self.submap_handler = SubmapHandler(self.config)

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.submaps = {}
        self.submap_counter = {}
        self.is_initialized = True
        self.latest_opt_traj_msg = None
        self.mutex.release()

    def graph_callback(self, msg):
        if self.is_initialized is False:
            return
        rospy.loginfo("[GraphMonitor] Received graph message from server.")
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
            # rospy.logwarn('[GraphMonitor] Reducing trajectory with {len(self.graph.reduced_ind)} indices.')
            msg.nodes = [msg.nodes[i] for i in self.graph.reduced_ind]

        if self.graph.has_skipped():
            # rospy.logwarn('[GraphMonitor] Skipping trajectory with {len(self.graph.skip_ind)} indices.')
            msg.nodes = [element for i,element in enumerate(msg.nodes) if i not in self.graph.skip_ind]

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
            rospy.logerr('[GraphMonitor] Unable to publish results to client.')

    def compute_and_publish_graph(self):
        self.mutex.acquire()
        if self.graph.is_built is False:
            rospy.logwarn("[GraphMonitor] Graph is not built yet!")
            self.mutex.release()
            return
        rospy.loginfo("[GraphMonitor] Computing global graph with {graph_size}.".format(graph_size=self.graph.graph_size()))
        if self.graph.graph_size() < self.config.min_node_count:
            rospy.logwarn("[GraphMonitor] Not enough nodes ({graph_size} < {min_node_count})".format(graph_size=self.graph.graph_size(), min_node_count=self.config.min_node_count))
            self.mutex.release()
            return;
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
        rospy.loginfo("[GraphMonitor] Computing constraints for {n_submaps} submaps.".format(n_submaps=n_submaps))
        return self.submap_handler.compute_constraints(submaps)

    def publish_graph_and_traj(self):
        graph_msg = self.graph.to_graph_msg()
        self.pub_graph.publish(graph_msg)
        rospy.loginfo("[GraphMonitor] Published global graph.")

        if self.config.send_separate_traj_msgs:
            self.send_separate_traj_msgs()
        elif self.latest_opt_traj_msg is not None:
            self.pub_traj.publish(self.latest_opt_traj_msg)
            rospy.loginfo("[GraphMonitor] Published trajectory for keys {key}.".format(key=self.optimized_keys))

    def send_separate_traj_msgs(self):
        for key in self.optimized_keys:
            traj_msg = self.optimized_signal.to_signal_msg(key)
            self.pub_traj.publish(traj_msg)
            time.sleep(0.10)
            rospy.loginfo("[GraphMonitor] Published separate trajectory for {key}.".format(key=key))

    def publish_all_submaps(self, submaps):
        self.submap_handler.publish_submaps(submaps)

    def key_in_optimized_keys(self, key):
       return any(key in k for k in self.optimized_keys)

if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    while not rospy.is_shutdown():
        node.update()
        node.config.rate.sleep()
