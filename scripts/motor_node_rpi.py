#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests


RPI_IP = "172.21.7.4"  # Change this
def send_to_rpi(cmd):
   try:
       requests.get(f"http://{RPI_IP}:5000/cmd/{cmd}")
   except:
       print("RPi not reachable")



class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node_rpi')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_cmd = 'S'
        self.get_logger().info("Motor control node started")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < 0.01 and abs(angular) < 0.01:
            cmd = 'S'
        elif linear > 0.0:
            cmd = 'F'
        elif linear < 0.0:
            cmd = 'B'
        elif angular > 0.0:
            cmd = 'L'
        elif angular < 0.0:
            cmd = 'R'
        else:
            cmd = 'S'

        if cmd != self.last_cmd:
            send_to_rpi(cmd)
            self.last_cmd = cmd

    def destroy_node(self):
        send_to_rpi('S')
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  