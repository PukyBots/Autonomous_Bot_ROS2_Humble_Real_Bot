#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.pub = self.create_publisher(MarkerArray, '/points', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()

        # BONUS POINT (green)
        bonus = Marker()
        bonus.header.frame_id = "map"
        bonus.header.stamp = self.get_clock().now().to_msg()
        bonus.ns = "bonus"
        bonus.id = 0
        bonus.type = Marker.SPHERE
        bonus.action = Marker.ADD
        bonus.pose.position.x = 2.0
        bonus.pose.position.y = 1.0
        bonus.pose.position.z = 0.0
        bonus.pose.orientation.w = 1.0
        bonus.scale.x = 0.3
        bonus.scale.y = 0.3
        bonus.scale.z = 0.3
        bonus.color.r = 0.0
        bonus.color.g = 1.0
        bonus.color.b = 0.0
        bonus.color.a = 1.0

        # NEGATIVE POINT (red)
        negative = Marker()
        negative.header.frame_id = "map"
        negative.header.stamp = bonus.header.stamp
        negative.ns = "negative"
        negative.id = 1
        negative.type = Marker.SPHERE
        negative.action = Marker.ADD
        negative.pose.position.x = -1.0
        negative.pose.position.y = -0.5
        negative.pose.position.z = 0.0
        negative.pose.orientation.w = 1.0
        negative.scale.x = 0.3
        negative.scale.y = 0.3
        negative.scale.z = 0.3
        negative.color.r = 1.0
        negative.color.g = 0.0
        negative.color.b = 0.0
        negative.color.a = 1.0

        marker_array.markers.append(bonus)
        marker_array.markers.append(negative)

        self.pub.publish(marker_array)

def main():
    rclpy.init()
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
