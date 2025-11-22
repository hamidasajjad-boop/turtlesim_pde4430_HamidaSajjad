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
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.speed = 2.0
        self.turn_speed = 2.0
        self.step_y = 1.0  # distance to move down after each row
        self.bound_x = 0.5
        self.bound_y = 0.5
        self.direction = 1  # 1 = move right, -1 = move left
        self.state = 'forward'  # 'forward' or 'turn'
        self.turning_steps = 0

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def timer_cb(self):
        if not self.pose:
            return

        t = Twist()
        x, y = self.pose.x, self.pose.y

        if self.state == 'forward':
            t.linear.x = self.speed
            t.angular.z = 0.0

            # Check if we reached the wall
            if (self.direction == 1 and x >= 11.0 - self.bound_x) or \
               (self.direction == -1 and x <= self.bound_x):
                self.state = 'turn'
                self.turning_steps = int(3.1415 / 0.1)  # approx 180° turn

        elif self.state == 'turn':
            t.linear.x = 0.5  # small forward while turning
            t.angular.z = self.turn_speed * self.direction
            self.turning_steps -= 1

            if self.turning_steps <= 0:
                # Shift Y for next row
                if y + self.step_y <= 11.0 - self.bound_y:
                    t.linear.x = self.speed
                    self.pose.y += self.step_y
                self.direction *= -1
                self.state = 'forward'

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
