#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):
    def __init__(self):
        super().__init__('circle')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.radius = 2.0
        self.linear_speed = 1.0
        self.angular_speed = self.linear_speed / self.radius

    def timer_callback(self):
        t = Twist()
        t.linear.x = self.linear_speed
        t.angular.z = self.angular_speed
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = Circle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
