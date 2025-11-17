#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwoCircles(Node):
    def __init__(self):
        super().__init__('two_circles')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Speeds
        self.linear_speed = 2.0
        self.angular_speed = 1.0

        # Duration for each full circle
        self.circle_duration = 6.28  # seconds for one circle (approx 2π)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        t = Twist()
        t.linear.x = self.linear_speed

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        # Determine which circle we are in
        phase = int(elapsed // self.circle_duration) % 2

        # First circle clockwise, second circle counter-clockwise
        t.angular.z = self.angular_speed if phase == 0 else -self.angular_speed

        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = TwoCircles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
