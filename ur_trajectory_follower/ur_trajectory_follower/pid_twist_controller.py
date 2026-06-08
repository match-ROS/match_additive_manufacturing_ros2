#!/usr/bin/env python3
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

from ur_trajectory_follower.ros2_utils import as_bool


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    prev_error: float = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PIDTwistController(Node):
    def __init__(self) -> None:
        super().__init__('pid_twist_controller')
        self.declare_parameter('stamped', False)
        self.declare_parameter('input_twist_topic', '/input_twist')
        self.declare_parameter('output_twist_topic', '/output_twist')
        for prefix in ('linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'):
            self.declare_parameter(f'Kp_{prefix}', 1.0)
            self.declare_parameter(f'Ki_{prefix}', 0.0)
            self.declare_parameter(f'Kd_{prefix}', 0.0)

        self.stamped = as_bool(self.get_parameter('stamped').value)
        self.pids = {
            prefix: PID(
                float(self.get_parameter(f'Kp_{prefix}').value),
                float(self.get_parameter(f'Ki_{prefix}').value),
                float(self.get_parameter(f'Kd_{prefix}').value),
            )
            for prefix in ('linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z')
        }
        self.last_time = self.get_clock().now()

        input_topic = str(self.get_parameter('input_twist_topic').value)
        output_topic = str(self.get_parameter('output_twist_topic').value)
        if self.stamped:
            self.pub = self.create_publisher(TwistStamped, output_topic, 10)
            self.create_subscription(TwistStamped, input_topic, self.twist_stamped_callback, 10)
        else:
            self.pub = self.create_publisher(Twist, output_topic, 10)
            self.create_subscription(Twist, input_topic, self.twist_callback, 10)
        self.get_logger().info(f"PID twist controller: {input_topic} -> {output_topic}")

    def _control(self, twist: Twist) -> Twist:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        out = Twist()
        out.linear.x = self.pids['linear_x'].update(twist.linear.x, dt)
        out.linear.y = self.pids['linear_y'].update(twist.linear.y, dt)
        out.linear.z = self.pids['linear_z'].update(twist.linear.z, dt)
        out.angular.x = self.pids['angular_x'].update(twist.angular.x, dt)
        out.angular.y = self.pids['angular_y'].update(twist.angular.y, dt)
        out.angular.z = self.pids['angular_z'].update(twist.angular.z, dt)
        return out

    def twist_callback(self, msg: Twist) -> None:
        self.pub.publish(self._control(msg))

    def twist_stamped_callback(self, msg: TwistStamped) -> None:
        msg.twist = self._control(msg.twist)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PIDTwistController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
