#!/usr/bin/env python3
"""Compensate Cartesian arm motion for motion of the mobile base.

The arm follower produces a TCP velocity in ``world_frame``. A mobile base
adds its own velocity at the TCP, including the yaw-rate lever-arm term. This
node publishes the negative of that induced velocity so the resulting TCP
motion remains on the world-frame print path.
"""

from __future__ import annotations

import signal
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix


def planar_tcp_induced_velocity(
    base_linear: np.ndarray,
    base_angular: np.ndarray,
    tcp_offset_in_base: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the TCP twist caused by a base twist.

    Both vectors are expressed in the same base frame. The expression is the
    standard rigid-body relation ``v_tcp = v_base + omega x r`` and therefore
    also covers the ROS1 planar Jacobian used by ``ur_vel_induced_by_mir``.
    """

    linear = np.asarray(base_linear, dtype=float) + np.cross(
        np.asarray(base_angular, dtype=float),
        np.asarray(tcp_offset_in_base, dtype=float),
    )
    return linear, np.asarray(base_angular, dtype=float)


def world_compensation(
    base_linear: np.ndarray,
    base_angular: np.ndarray,
    tcp_offset_in_base: np.ndarray,
    rotation_world_from_base: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the negative induced TCP twist in the world frame."""

    induced_linear, induced_angular = planar_tcp_induced_velocity(
        base_linear, base_angular, tcp_offset_in_base
    )
    rotation = np.asarray(rotation_world_from_base, dtype=float)
    return -(rotation @ induced_linear), -(rotation @ induced_angular)


def _rotation_from_transform(transform) -> np.ndarray:
    q = transform.transform.rotation
    return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]


def _translation_from_transform(transform) -> np.ndarray:
    t = transform.transform.translation
    return np.array([t.x, t.y, t.z], dtype=float)


class BaseMotionCompensation(Node):
    """Publish a safe, world-frame correction for a moving mobile base."""

    def __init__(self) -> None:
        super().__init__('ur_vel_induced_by_base')
        self.declare_parameter('base_velocity_topic', '/odom')
        self.declare_parameter('base_velocity_type', 'odometry')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'tool0')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('output_topic', '/ur_twist_base_compensation_world')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('output_smoothing_coeff', 0.0)

        self.base_frame = self._clean_frame(str(self.get_parameter('base_frame').value))
        self.tcp_frame = self._clean_frame(str(self.get_parameter('tcp_frame').value))
        self.world_frame = self._clean_frame(str(self.get_parameter('world_frame').value))
        self.velocity_type = str(self.get_parameter('base_velocity_type').value).strip().lower()
        self.stale_timeout = max(0.0, float(self.get_parameter('stale_timeout').value))
        self.smoothing_coeff = max(
            0.0, min(1.0, float(self.get_parameter('output_smoothing_coeff').value))
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_velocity: Optional[Tuple[np.ndarray, np.ndarray, str]] = None
        self.last_velocity_time = None
        self.previous_output = Twist()
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10
        )

        velocity_topic = str(self.get_parameter('base_velocity_topic').value)
        if self.velocity_type in {'odometry', 'odom'}:
            self.create_subscription(Odometry, velocity_topic, self._odom_cb, 10)
        elif self.velocity_type in {'twist_stamped', 'stamped'}:
            self.create_subscription(TwistStamped, velocity_topic, self._stamped_cb, 10)
        elif self.velocity_type == 'twist':
            self.create_subscription(Twist, velocity_topic, self._twist_cb, 10)
        else:
            raise ValueError(
                "base_velocity_type must be 'odometry', 'twist_stamped', or 'twist'"
            )

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Base-motion compensation: {velocity_topic} -> '
            f'{self.get_parameter("output_topic").value}, '
            f'TF {self.world_frame} <- {self.base_frame} <- {self.tcp_frame}.'
        )

    @staticmethod
    def _clean_frame(frame: str) -> str:
        return frame.strip().lstrip('/')

    def _set_velocity(self, msg: Twist, frame: str) -> None:
        self.latest_velocity = (
            np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float),
            np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=float),
            self._clean_frame(frame) or self.base_frame,
        )
        self.last_velocity_time = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        self._set_velocity(msg.twist.twist, msg.child_frame_id or self.base_frame)

    def _stamped_cb(self, msg: TwistStamped) -> None:
        self._set_velocity(msg.twist, msg.header.frame_id or self.base_frame)

    def _twist_cb(self, msg: Twist) -> None:
        self._set_velocity(msg, self.base_frame)

    def _lookup(self, target: str, source: str):
        return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())

    def _compute(self) -> Optional[Twist]:
        if self.latest_velocity is None or self.last_velocity_time is None:
            return None
        age = (self.get_clock().now() - self.last_velocity_time).nanoseconds / 1e9
        if age > self.stale_timeout:
            return None

        linear, angular, velocity_frame = self.latest_velocity
        if not np.all(np.isfinite(linear)) or not np.all(np.isfinite(angular)):
            return None
        try:
            base_from_velocity = self._lookup(self.base_frame, velocity_frame)
            world_from_base = self._lookup(self.world_frame, self.base_frame)
            base_from_tcp = self._lookup(self.base_frame, self.tcp_frame)
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for compensation TF ({self.world_frame}, '
                f'{self.base_frame}, {self.tcp_frame}): {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        base_rotation = _rotation_from_transform(base_from_velocity)
        world_rotation = _rotation_from_transform(world_from_base)
        base_linear = base_rotation @ linear
        base_angular = base_rotation @ angular
        tcp_offset = _translation_from_transform(base_from_tcp)
        compensated_linear, compensated_angular = world_compensation(
            base_linear, base_angular, tcp_offset, world_rotation
        )

        output = Twist()
        output.linear.x, output.linear.y, output.linear.z = map(float, compensated_linear)
        output.angular.x, output.angular.y, output.angular.z = map(float, compensated_angular)
        return output

    def _smooth(self, twist: Twist) -> Twist:
        coeff = self.smoothing_coeff
        output = Twist()
        output.linear.x = coeff * self.previous_output.linear.x + (1.0 - coeff) * twist.linear.x
        output.linear.y = coeff * self.previous_output.linear.y + (1.0 - coeff) * twist.linear.y
        output.linear.z = coeff * self.previous_output.linear.z + (1.0 - coeff) * twist.linear.z
        output.angular.x = coeff * self.previous_output.angular.x + (1.0 - coeff) * twist.angular.x
        output.angular.y = coeff * self.previous_output.angular.y + (1.0 - coeff) * twist.angular.y
        output.angular.z = coeff * self.previous_output.angular.z + (1.0 - coeff) * twist.angular.z
        self.previous_output = output
        return output

    def _publish(self) -> None:
        output = self._compute()
        if output is None:
            # Never let smoothing keep a non-zero correction alive after a
            # stale velocity or missing TF has removed the valid input.
            self.previous_output = Twist()
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self._smooth(output))

    def publish_zero(self) -> None:
        """Clear the correction before this independently managed node exits."""
        self.previous_output = Twist()
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseMotionCompensation()

    def _stop(_signum, _frame) -> None:
        # ProcessRegistry terminates this process group with SIGTERM.  Publish
        # through the already matched publisher before exiting so the combiner
        # cannot retain a stale non-zero correction.
        node.publish_zero()
        raise KeyboardInterrupt

    previous_sigterm_handler = signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
