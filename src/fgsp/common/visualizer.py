#! /usr/bin/env python3

import copy

from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from src.fgsp.common.comms import Comms


class Visualizer(object):
    def __init__(self, config=None):
        self.config = config
        self.line_marker = self.create_line_marker()
        self.sphere_graph_marker = self.create_sphere_marker()
        self.signal_graph_marker = self.create_sphere_marker()
        self.resetConstraintVisualization()

        self.comms = Comms()
        self.robot_colors = self.generate_robot_colors()
        self.resetConstraintVisualization()

    def create_line_marker(self, frame_id='map'):
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.ns = "Line"
        line_marker.action = Marker().ADD
        line_marker.type = Marker().LINE_STRIP
        line_marker.scale.x = 0.5
        return line_marker

    def create_sphere_marker(self, frame_id='map'):
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.action = Marker.ADD
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
        sphere.color = self.get_color(0.9, 0.05, 0.05)
        sphere.type = Marker.SPHERE
        sphere.frame_locked = True
        return sphere

    def generate_robot_colors(self):
        robots = []
        robots.append(self.get_color(0.8, 0.1, 0.1))
        robots.append(self.get_color(0.4, 0.4, 0.2))
        robots.append(self.get_color(0.1, 0.1, 0.8))
        robots.append(self.get_color(0.2, 0.4, 0.4))
        robots.append(self.get_color(0.4, 0.2, 0.4))
        return robots

    def get_color(self, r, g, b):
        result = ColorRGBA()
        result.a = 1.0
        result.r = r
        result.g = g
        result.b = b
        return result

    def resetConstraintVisualization(self):
        self.line_marker.id = 0
        self.sphere_graph_marker.id = 0
        self.signal_graph_marker.id = 0
        self.spheres = MarkerArray()
        self.adjacency = MarkerArray()
        self.signals = MarkerArray()

    def create_sphere(self, sphere, point):
        sphere.header.stamp = self.time_now().to_msg()
        sphere.pose.position.x = float(point[0])
        sphere.pose.position.y = float(point[1])
        sphere.pose.position.z = float(point[2])
        return sphere

    def add_graph_coordinate(self, point, color=[0.9, 0.05, 0.05]):
        self.sphere_graph_marker.id += 1
        sphere = copy.deepcopy(self.sphere_graph_marker)
        sphere = self.create_sphere(sphere, point)
        sphere.color = self.get_color(color[0], color[1], color[2])
        self.spheres.markers.append(sphere)

    def add_signal_coordinate(self, point, robot_idx):
        self.signal_graph_marker.id += 1
        sphere = copy.deepcopy(self.signal_graph_marker)
        sphere = self.create_sphere(sphere, point)
        sphere.color = self.robot_colors[robot_idx]
        self.signals.markers.append(sphere)

    def add_graph_adjacency(self, point_a, point_b):
        points = [point_a, point_b]
        color = [0.9, 0.05, 0.05]
        line_marker = self.create_point_line_markers(points, color)

        self.adjacency.markers.append(line_marker)

    def create_point_line_markers(self, points, color=[0.9, 0.05, 0.05]):
        line_point_a = Point()
        line_point_a.x = float(points[0][0])
        line_point_a.y = float(points[0][1])
        line_point_a.z = float(points[0][2])
        line_point_b = Point()
        line_point_b.x = float(points[1][0])
        line_point_b.y = float(points[1][1])
        line_point_b.z = float(points[1][2])

        self.line_marker.id += 1
        line_marker = copy.deepcopy(self.line_marker)
        line_marker.header.stamp = self.time_now().to_msg()
        line_marker.color = self.get_color(color[0], color[1], color[2])

        line_marker.points[:] = []
        line_marker.points.append(line_point_a)
        line_marker.points.append(line_point_b)

        return line_marker

    def visualize_coords(self):
        self.comms.publish(self.spheres, MarkerArray, '/graph/coords')

    def visualize_adjacency(self):
        self.comms.publish(self.adjacency, MarkerArray, '/graph/adjacency')

    def visualize_signals(self, topic):
        self.comms.publish(self.signals, MarkerArray, topic)

    def time_now(self):
        return self.comms.node.get_clock().now()
