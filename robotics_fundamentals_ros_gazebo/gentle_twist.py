import sys
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist


class GentleTwist(Node):
    '''
    Example of generating acceleration-limited twist commands.
    '''
    class State(Enum):
        ACCELERATE = auto()
        CONST_VELOCITY = auto()
        DECELERATE = auto()
        FINISHED = auto()

    def __init__(self):
        super().__init__('gentle_twist')
        self.declare_parameter('linear_acceleration', 0.1)  # m/s^2
        self.declare_parameter('linear_velocity', 0.5)  # m/s
        self.declare_parameter('full_velocity_duration', 5.0)  # s
        self.linear_acceleration = self.get_parameter(
            'linear_acceleration').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.full_velocity_duration = self.get_parameter(
            'full_velocity_duration').value
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_twist = Twist()
        self.full_velocity_start_time = self.get_clock().now()
        self.state = self.State.ACCELERATE
        self.timer_period = 0.1  # s
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.linear_acceleration = self.get_parameter(
            'linear_acceleration').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.full_velocity_duration = self.get_parameter(
            'full_velocity_duration').value

        twist = self.prev_twist
        if self.state == self.State.ACCELERATE:
            twist.linear.x = min(self.linear_velocity,
                                 self.prev_twist.linear.x + self.linear_acceleration * self.timer_period)
            if self.prev_twist.linear.x == self.linear_velocity:
                self.state = self.State.CONST_VELOCITY
                self.full_velocity_start_time = self.get_clock().now()
        elif self.state == self.State.CONST_VELOCITY:
            if (self.get_clock().now() - self.full_velocity_start_time).nanoseconds / 1e9 >= self.full_velocity_duration:
                self.state = self.State.DECELERATE
        elif self.state == self.State.DECELERATE:
            twist.linear.x = max(
                0.0, self.prev_twist.linear.x - self.linear_acceleration * self.timer_period)
            if self.prev_twist.linear.x == 0:
                self.state = self.State.FINISHED
        elif self.state == self.State.FINISHED:
            sys.exit(0)  # NOT a clean way to exit a ROS node!

        self.twist_pub.publish(twist)
        self.prev_twist = twist


def main(args=None):
    rclpy.init(args=args)

    gentle_twist = GentleTwist()

    rclpy.spin(gentle_twist)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
