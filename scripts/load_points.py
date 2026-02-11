#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import yaml
import os

class LoadPoints(Node):

    def __init__(self):
        super().__init__('load_points')

        self.pub = self.create_publisher(Marker, '/interactive_points', 10)

        self.yaml_path = os.path.expanduser(
            '~/ros2_ws/src/diff_drive_robot/config/points.yaml'
        )

        self.green = []
        self.red = []

        self.load_yaml()
        self.timer = self.create_timer(1.0, self.publish)

    def load_yaml(self):
        if not os.path.exists(self.yaml_path):
            self.get_logger().warn('YAML not found')
            return

        with open(self.yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.green = data.get('green_points', [])
        self.red = data.get('red_points', [])

    def publish(self):
        self.draw(self.green, 0, 0, 1, 0)
        self.draw(self.red, 100, 1, 0, 0)

    def draw(self, points, base_id, r, g, b):
        for i, p in enumerate(points):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'points'
            m.id = base_id + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = p['x']
            m.pose.position.y = p['y']
            m.pose.position.z = 0.1

            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 1.0
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)

            self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(LoadPoints())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
