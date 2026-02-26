#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import SetParametersResult

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class InteractivePoints(Node):

    def __init__(self):
        super().__init__('interactive_points')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('mode', 'green')   # green | red
        self.declare_parameter('save', False)     # trigger save

        self.mode = self.get_parameter('mode').value

        # -----------------------------
        # Storage
        # -----------------------------
        self.green_points = []
        self.red_points = []

        # -----------------------------
        # YAML file path
        # -----------------------------
        pkg_path = get_package_share_directory('diff_drive_robot')  # CHANGE THIS
        self.yaml_path = os.path.join(pkg_path, 'config', 'points.yaml')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            '/interactive_points',
            10
        )

        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info(
            "\n=== Interactive Points Node ===\n"
            "RViz tool  : Publish Point\n"
            "Modes      : green | red\n"
            "Commands   :\n"
            "  ros2 param set /interactive_points mode green\n"
            "  ros2 param set /interactive_points mode red\n"
            "  ros2 param set /interactive_points save true\n"
        )

    # --------------------------------------------------
    # Handle parameter updates
    # --------------------------------------------------
    def param_callback(self, params):
        for param in params:
            if param.name == 'mode':
                self.mode = param.value
                self.get_logger().info(f"Mode changed to: {self.mode}")

            if param.name == 'save' and param.value is True:
                self.save_to_yaml()

        return SetParametersResult(successful=True)

    # --------------------------------------------------
    # Handle RViz clicks
    # --------------------------------------------------
    def click_callback(self, msg):
        x = msg.point.x
        y = msg.point.y

        if self.mode == 'green':
            self.green_points.append((x, y))
            self.get_logger().info(f"Added GREEN point: ({x:.2f}, {y:.2f})")
        else:
            self.red_points.append((x, y))
            self.get_logger().info(f"Added RED point: ({x:.2f}, {y:.2f})")

        self.publish_markers()

    # --------------------------------------------------
    # Publish all markers
    # --------------------------------------------------
    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for x, y in self.green_points:
            marker_array.markers.append(
                self.create_marker(x, y, marker_id, 'green')
            )
            marker_id += 1

        for x, y in self.red_points:
            marker_array.markers.append(
                self.create_marker(x, y, marker_id, 'red')
            )
            marker_id += 1

        self.pub.publish(marker_array)

    # --------------------------------------------------
    # Create marker helper
    # --------------------------------------------------
    def create_marker(self, x, y, mid, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = color
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0

        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3

        m.color.a = 1.0
        if color == 'green':
            m.color.g = 1.0
        else:
            m.color.r = 1.0

        return m

    # --------------------------------------------------
    # Save points to YAML
    # --------------------------------------------------
    def save_to_yaml(self):
        data = {
            'green_points': [{'x': x, 'y': y} for x, y in self.green_points],
            'red_points': [{'x': x, 'y': y} for x, y in self.red_points],
        }

        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)

        with open(self.yaml_path, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Points saved to: {self.yaml_path}")


def main():
    rclpy.init()
    node = InteractivePoints()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
