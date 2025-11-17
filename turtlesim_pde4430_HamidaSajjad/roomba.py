#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Roomba(Node):
    def __init__(self):
        super().__init__('roomba')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pose = None
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.speed = 1.0
        self.bound = 0.5

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def timer_cb(self):
        if not self.pose:
            return
        x, y = self.pose.x, self.pose.y
        t = Twist()
        t.linear.x = self.speed
        t.angular.z = 0.0
        if x < self.bound or x > 11.0 - self.bound or y < self.bound or y > 11.0 - self.bound:
            t.linear.x = 0.0
            t.angular.z = 2.0
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = Roomba()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
