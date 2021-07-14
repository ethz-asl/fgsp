import rospy
import copy

from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Visualizer(object):
    def __init__(self):
        self.line_id = 0
        self.sphere_id = 0

        self.spheres = MarkerArray()
        self.adjacency = MarkerArray()
        self.line_marker = self.create_line_marker()
        self.sphere_marker = self.create_sphere_marker()

        self.pub_adjacency = rospy.Publisher("/graph_monitor/graph/adjacency", MarkerArray, queue_size=10)
        self.pub_coords = rospy.Publisher('/graph_monitor/graph/coords', MarkerArray, queue_size=10)
        self.robot_colors = self.generate_robot_colors()
        self.resetConstraintVisualization()

    def create_line_marker(self, frame_id = 'darpa'):
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.ns = "Line"
        line_marker.action = Marker().ADD
        line_marker.type = Marker().LINE_STRIP
        line_marker.lifetime = rospy.Duration(0.0)
        line_marker.scale.x = 0.05
        return line_marker

    def create_sphere_marker(self, frame_id = 'darpa'):
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.action = Marker.ADD
        sphere.pose.orientation.w = 1
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
        result.a = 1
        result.r = r
        result.g = g
        result.b = b
        return result

    def resetConstraintVisualization(self):
        self.line_marker.id = 0
        self.sphere_marker.id = 0
        self.spheres = MarkerArray()
        self.adjacency = MarkerArray()

    def add_graph_coordinate(self, point):
        self.sphere_marker.id += 1
        sphere = copy.deepcopy(self.sphere_marker)
        sphere.header.stamp = rospy.Time.now()
        sphere.pose.position.x = point[0]
        sphere.pose.position.y = point[1]
        sphere.pose.position.z = point[2]

        self.spheres.markers.append(sphere)
        self.sphere_id = self.sphere_id + 1

    def add_graph_adjacency(self, point_a, point_b):
        line_point_a = Point(point_a[0], point_a[1], point_a[2])
        line_point_b = Point(point_b[0], point_b[1], point_b[2])

        self.line_marker.id += 1
        line_marker = copy.deepcopy(self.line_marker)
        line_marker.header.stamp = rospy.Time.now()
        line_marker.color = self.get_color(0.1, 0.8, 0.1)

        line_marker.points[:] = []
        line_marker.points.append(line_point_a)
        line_marker.points.append(line_point_b)

        self.adjacency.markers.append(line_marker)

    def visualize_coords(self):
        self.pub_coords.publish(self.spheres)

    def visualize_adjacency(self):
        self.pub_adjacency.publish(self.adjacency)
