#! /usr/bin/env python3

import rospy
import copy
from maplab_msgs.msg import *
from multiprocessing import Lock

from global_graph import GlobalGraph
from signal_handler import SignalHandler
from verification_handler import VerificationHandler
from submap_handler import SubmapHandler
from submap_model import SubmapModel

class GraphMonitor(object):

    def __init__(self):
        self.is_initialized = False
        self.mutex = Lock()
        self.mutex.acquire()
        rospy.loginfo("[GraphMonitor] Initializing monitor node...")
        self.rate = rospy.Rate(rospy.get_param("~update_rate"))

        in_graph_topic = rospy.get_param("~in_graph_topic")
        in_traj_opt_topic = rospy.get_param("~in_traj_opt_topic")
        out_graph_topic = rospy.get_param("~out_graph_topic")
        out_traj_opt_topic = rospy.get_param("~out_traj_opt_topic")
        self.min_node_count = rospy.get_param("~min_node_count")
        verification_service_topic = rospy.get_param("~verification_service")
        pc_topic = rospy.get_param("~opt_pc_topic")
        submap_topic = rospy.get_param("~submap_constraint_topic")

        rospy.Subscriber(pc_topic, Submap, self.submap_callback)
        rospy.Subscriber(in_graph_topic, Graph, self.graph_callback)
        rospy.Subscriber(in_traj_opt_topic, Trajectory, self.traj_opt_callback)
        rospy.Subscriber(verification_service_topic, VerificationCheckRequest, self.verification_callback)
        self.pub_graph = rospy.Publisher(out_graph_topic, Graph, queue_size=10)
        self.pub_traj = rospy.Publisher(out_traj_opt_topic, Trajectory, queue_size=10)
        self.submap_pub = rospy.Publisher(submap_topic, SubmapConstraint, queue_size=10)
        rospy.loginfo("[GraphMonitor] Listening for graphs from " + in_graph_topic)
        rospy.loginfo("[GraphMonitor] Listening for trajectory from " + in_traj_opt_topic)

        # Handlers and evaluators.
        self.graph = GlobalGraph(reduced=False)
        self.optimized_signal = SignalHandler()
        self.verification_handler = VerificationHandler()
        self.submap_handler = SubmapHandler()

        # Key management to keep track of the received messages.
        self.optimized_keys = []
        self.submaps = {}

        rospy.loginfo("[GraphMonitor] Graph monitor is set up.")
        self.is_initialized = True
        self.mutex.release()

    def graph_callback(self, msg):
        rospy.loginfo(f"[GraphMonitor] Received graph message.")
        if self.is_initialized is False:
            return
        self.mutex.acquire()

        # We only trigger the graph building if the msg contains new information.
        if self.graph.msg_contains_updates(msg) is True:
            self.graph.build(msg)

        self.mutex.release()

    def traj_opt_callback(self, msg):
        if self.is_initialized is False:
            return

        key = self.optimized_signal.convert_signal(msg)
        rospy.loginfo(f"[GraphMonitor] Received opt trajectory message from {key}.")

        if self.key_in_optimized_keys(key):
            return
        self.optimized_keys.append(key)

    def verification_callback(self, msg):
        self.verification_handler.handle_verification(msg)

    def update(self):
        # Compute the submap constraints and publish them.
        self.compute_and_publish_submaps()

        self.mutex.acquire()
        if self.graph.is_built is False:
            self.mutex.release()
            return
        if self.graph.graph_size() < self.min_node_count:
            rospy.loginfo(f"[GraphMonitor] Not enough nodes ({self.graph.graph_size()})")
            self.mutex.release()
            return;
        self.mutex.release()

        # Publish the graph to the clients.
        self.publish_graph_and_traj()

        # Publish verifications to the server.
        self.verification_handler.send_verification_request()

    def submap_callback(self, submap_msg):
        submap = SubmapModel()
        submap.construct_data(submap_msg)
        submap.compute_dense_map()
        print(f'Received submap from {submap_msg.robot_name} with {len(submap_msg.nodes)} nodes and id {submap_msg.id}.')
        #self.submap_handler.add_submap(submap)
        id = submap.id
        self.submaps[id] = submap

    def compute_and_publish_submaps(self):
        submaps = copy.deepcopy(self.submaps)

        self.publish_all_submaps(submaps)
        msg = self.compute_submap_constraints(submaps)
        if msg is not None:
            self.submap_pub.publish(msg)

    def compute_submap_constraints(self, submaps):
        n_submaps = len(submaps)
        if n_submaps == 0:
            return None
        print(f"Computing constraints for {n_submaps} submaps.")
        return self.submap_handler.compute_constraints(submaps)

    def publish_graph_and_traj(self):
        graph_msg = self.graph.to_graph_msg()
        self.pub_graph.publish(graph_msg)

        for key in self.optimized_keys:
            traj_msg = self.optimized_signal.to_signal_msg(key)
            self.pub_traj.publish(traj_msg)

    def publish_all_submaps(self, submaps):
        self.submap_handler.publish_submaps(submaps)

    def key_in_optimized_keys(self, key):
       return any(key in k for k in self.optimized_keys)


if __name__ == '__main__':
    rospy.init_node('graph_monitor')
    node = GraphMonitor()
    while not rospy.is_shutdown():
        node.update()
        node.rate.sleep()
