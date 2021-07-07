#! /usr/bin/env python3

import rospy
import copy
import time
from maplab_msgs.msg import *
from multiprocessing import Lock

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from verification_handler import VerificationHandler
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
            rospy.Subscriber(self.config.verification_service_topic, VerificationCheckRequest, self.verification_callback)
            self.pub_graph = rospy.Publisher(self.config.out_graph_topic, Graph, queue_size=10)
            self.pub_traj = rospy.Publisher(self.config.out_traj_opt_topic, Trajectory, queue_size=10)

        # Handlers and evaluators.
        self.graph = GlobalGraph(reduced=self.config.reduce_global_graph)
        self.optimized_signal = SignalHandler()
        self.verification_handler = VerificationHandler()
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
        rospy.loginfo(f"[GraphMonitor] Received graph message from server.")
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.graph.msg_contains_updates(msg) is True:
            self.graph.build(msg)

        self.mutex.release()

    def traj_opt_callback(self, msg):
        if self.is_initialized is False:
            return

        keys = self.optimized_signal.convert_signal(msg)
        print(f'Got optimized version of robot {keys}')

        for key in keys:
            if self.key_in_optimized_keys(key):
                continue
            self.optimized_keys.append(key)
        self.latest_opt_traj_msg = msg

    def verification_callback(self, msg):
        self.verification_handler.handle_verification(msg)

    def update(self):
        # Publish verifications to the server.
        self.verification_handler.send_verification_request()

        # Compute the submap constraints and publish them if enabled.
        if self.config.enable_submap_constraints:
            self.compute_and_publish_submaps()

        # Compute the global graph and signal, then publish it
        if self.config.enable_graph_building:
            self.compute_and_publish_graph()

    def compute_and_publish_graph(self):
        rospy.loginfo(f"[GraphMonitor] Computing global graph.")
        self.mutex.acquire()
        if self.graph.is_built is False:
            rospy.logwarn(f"[GraphMonitor] Graph is not built yet!")
            self.mutex.release()
            return
        rospy.loginfo(f"[GraphMonitor] Checking size")
        if self.graph.graph_size() < self.config.min_node_count:
            rospy.logwarn(f"[GraphMonitor] Not enough nodes ({self.graph.graph_size()})")
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
        rospy.loginfo(f"[GraphMonitor] Computing constraints for {n_submaps} submaps.")
        return self.submap_handler.compute_constraints(submaps)

    def publish_graph_and_traj(self):
        graph_msg = self.graph.to_graph_msg()
        self.pub_graph.publish(graph_msg)
        rospy.loginfo(f"[GraphMonitor] Published global graph.")

        if self.config.send_separate_traj_msgs:
            self.send_separate_traj_msgs()
        else:
            self.pub_traj.publish(self.latest_opt_traj_msg)
            rospy.loginfo(f"[GraphMonitor] Published trajectory for keys {self.optimized_keys}.")


    def send_separate_traj_msgs(self):
        for key in self.optimized_keys:
            traj_msg = self.optimized_signal.to_signal_msg(key)
            self.pub_traj.publish(traj_msg)
            time.sleep(0.10)
            rospy.loginfo(f"[GraphMonitor] Published separate trajectory for {key}.")

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
