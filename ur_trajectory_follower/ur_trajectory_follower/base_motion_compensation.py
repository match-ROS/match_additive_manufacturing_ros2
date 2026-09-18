#!/usr/bin/env python3
"""Compensate Cartesian arm motion for motion of the mobile base.

The arm follower commands the deposition-point velocity in ``world_frame``.
This node subtracts the motion induced there by the mobile base, including
the angular-velocity lever arm, fixed nozzle offset and dynamic spray distance.
"""

from __future__ import annotations

import signal
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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


def _rigid_matrix(position, orientation) -> np.ndarray:
    xyz = np.array([position.x, position.y, position.z], dtype=float)
    quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-9:
        raise ValueError('pose/transform contains invalid position or orientation')
    result = quaternion_matrix(quat / np.linalg.norm(quat))
    result[:3, 3] = xyz
    return result


def _transform_matrix(transform) -> np.ndarray:
    return _rigid_matrix(transform.transform.translation, transform.transform.rotation)


class BaseMotionCompensation(Node):
    """Publish a safe, world-frame correction for a moving mobile base."""

    def __init__(self, **kwargs) -> None:
        super().__init__('ur_vel_induced_by_base', **kwargs)
        self.declare_parameter('base_velocity_topic', '/odom')
        self.declare_parameter('base_velocity_type', 'odometry')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'tool0')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('output_topic', '/ur_twist_base_compensation_world')
        self.declare_parameter('fixed_tool_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('fixed_tool_offset_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('spray_distance_topic', '/spray_distance_smoothed')
        self.declare_parameter('spray_distance_initial', 0.0)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('pose_timeout', 1.5)
        self.declare_parameter('tf_timeout', 1.5)
        self.declare_parameter('startup_wait_timeout', 45.0)
        self.declare_parameter('output_smoothing_coeff', 0.0)
        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('tcp_pose_topic', '/robot/arm/tcp_pose_broadcaster/pose')
        self.declare_parameter('base_pose_topic', '/robot_pose')
        self.declare_parameter('controller_tcp_frame', 'robot_arm_tool0_controller_raw')
        self.declare_parameter('translation_only', False)
        self.translation_only = bool(self.get_parameter('translation_only').value)
        self.translation_frame_rotation = None

        self.base_frame = self._clean_frame(str(self.get_parameter('base_frame').value))
        self.tcp_frame = self._clean_frame(str(self.get_parameter('tcp_frame').value))
        self.world_frame = self._clean_frame(str(self.get_parameter('world_frame').value))
        self.velocity_type = str(self.get_parameter('base_velocity_type').value).strip().lower()
        self.pose_source = str(self.get_parameter('pose_source').value).strip().lower()
        if self.pose_source not in {'tf', 'tcp_pose'}:
            raise ValueError("pose_source must be 'tf' or 'tcp_pose'")
        self.controller_tcp_frame = self._clean_frame(str(self.get_parameter('controller_tcp_frame').value))
        self.tcp_pose_topic = str(self.get_parameter('tcp_pose_topic').value)
        self.base_pose_topic = str(self.get_parameter('base_pose_topic').value)
        self.tcp_pose = self.base_pose = None
        self.tcp_pose_received = self.base_pose_received = None
        self.cached_geometry = None
        self.stale_timeout = max(0.0, float(self.get_parameter('stale_timeout').value))
        self.pose_timeout = self._timeout_parameter('pose_timeout')
        self.tf_timeout = self._timeout_parameter('tf_timeout')
        self.startup_wait_timeout = self._timeout_parameter('startup_wait_timeout')
        self.started_at = self.get_clock().now().nanoseconds
        self.ever_ready = False
        self.connection_state = None
        self.unavailable_reason = 'waiting for inputs'
        # Only the newest sample matters to this controller. Do not build up a
        # queue of old poses while a busy executor or network recovers.
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.smoothing_coeff = max(
            0.0, min(1.0, float(self.get_parameter('output_smoothing_coeff').value))
        )

        self.tool_xyz = np.asarray(self.get_parameter('fixed_tool_offset_xyz').value, dtype=float)
        tool_q = np.asarray(self.get_parameter('fixed_tool_offset_quaternion_xyzw').value, dtype=float)
        if (self.tool_xyz.shape != (3,) or tool_q.shape != (4,)
                or not np.all(np.isfinite(self.tool_xyz)) or not np.all(np.isfinite(tool_q))
                or np.linalg.norm(tool_q) < 1e-9):
            raise ValueError('fixed tool offset requires finite XYZ and a nonzero XYZW quaternion')
        self.tool_rotation = quaternion_matrix(tool_q / np.linalg.norm(tool_q))[:3, :3]
        self.spray_distance = float(self.get_parameter('spray_distance_initial').value)
        if not np.isfinite(self.spray_distance):
            raise ValueError('spray_distance_initial must be finite')
        self.create_subscription(Float32, str(self.get_parameter('spray_distance_topic').value),
                                 self._distance_cb, 10)
        self.velocity_stamp = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.pose_source == 'tcp_pose':
            self.create_subscription(PoseStamped, self.tcp_pose_topic, self._tcp_pose_cb, latest_qos)
            self.create_subscription(PoseStamped, self.base_pose_topic, self._base_pose_cb, latest_qos)
        self.latest_velocity: Optional[Tuple[np.ndarray, np.ndarray, str]] = None
        self.last_velocity_time = None
        self.previous_output = Twist()
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            Bool, '/am/base_compensation/translation_only', self._translation_only_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        velocity_topic = str(self.get_parameter('base_velocity_topic').value)
        if self.velocity_type in {'odometry', 'odom'}:
            self.create_subscription(Odometry, velocity_topic, self._odom_cb, latest_qos)
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
            f'pose_source={self.pose_source}, translation_only={self.translation_only}.'
        )
        if self.pose_source == 'tcp_pose' and not self.translation_only:
            self.get_logger().info(
                f'Using {self.tcp_pose_topic} and {self.base_pose_topic}; '
                'capturing mounting/tool transforms once. Keep lift height and UR TCP configuration '
                'fixed until this follower is stopped; restart after changing either.'
            )

    @staticmethod
    def _clean_frame(frame: str) -> str:
        return frame.strip().lstrip('/')

    def _timeout_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _distance_cb(self, msg: Float32) -> None:
        self.spray_distance = float(msg.data)

    def _translation_only_cb(self, msg: Bool) -> None:
        if self.translation_only == bool(msg.data):
            return
        self.translation_only = bool(msg.data)
        self.cached_geometry = None
        self.translation_frame_rotation = None
        self.previous_output = Twist()
        self.publisher.publish(Twist())
        self.ever_ready = False
        self.connection_state = None
        self.started_at = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f'Base compensation mode: {"translation only" if self.translation_only else "translation and rotation"}.'
        )

    def _translation_rotation(self, velocity_frame):
        if self.pose_source == 'tf':
            return _rotation_from_transform(self._lookup(self.world_frame, velocity_frame))
        world_from_base = self._fresh_pose(self.base_pose, self.base_pose_received, self.base_pose_topic)
        if self._clean_frame(self.base_pose.header.frame_id) != self.world_frame:
            raise ValueError(f'{self.base_pose_topic} must express {self.base_frame} in {self.world_frame}')
        if velocity_frame == self.base_frame:
            # Normal hardware path: no TCP pose, mount, tool TF, or lever arm.
            return world_from_base[:3, :3]
        if self.translation_frame_rotation is None:
            self.translation_frame_rotation = (
                velocity_frame, _rotation_from_transform(self._lookup(self.base_frame, velocity_frame))
            )
        frame, rotation = self.translation_frame_rotation
        if frame != velocity_frame:
            raise ValueError('velocity frame changed; toggle compensation mode or restart follower')
        return world_from_base[:3, :3] @ rotation

    def _tcp_pose_cb(self, msg: PoseStamped) -> None:
        self._accept_pose('tcp_pose', msg, self.tcp_pose_topic)

    def _base_pose_cb(self, msg: PoseStamped) -> None:
        self._accept_pose('base_pose', msg, self.base_pose_topic)

    def _accept_pose(self, attribute, msg, topic):
        now = self.get_clock().now().nanoseconds
        try:
            self._fresh_pose(msg, now, topic)
        except ValueError as exc:
            self.get_logger().warn(f'Ignoring pose: {exc}', throttle_duration_sec=10.0)
            return
        previous = getattr(self, attribute)
        received = getattr(self, attribute + '_received')
        if previous is not None and received is not None and now >= received:
            previous_stamp = rclpy.time.Time.from_msg(previous.header.stamp).nanoseconds
            stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
            if stamp <= previous_stamp:
                # Duplicates/reordered messages must not refresh the receipt
                # timeout or overwrite a newer valid geometry sample.
                return
        setattr(self, attribute, msg)
        setattr(self, attribute + '_received', now)

    def _fresh_pose(self, pose, received, topic):
        if pose is None or received is None:
            raise ValueError(f'waiting for {topic}')
        now = self.get_clock().now().nanoseconds
        age = (now - rclpy.time.Time.from_msg(pose.header.stamp).nanoseconds) / 1e9
        received_age = (now - received) / 1e9
        if not (0.0 <= age <= self.pose_timeout and 0.0 <= received_age <= self.pose_timeout):
            raise ValueError(f'stale {topic}: source age={age:.3f}s, receipt age={received_age:.3f}s')
        if not self._clean_frame(pose.header.frame_id):
            raise ValueError(f'{topic} has an empty frame_id')
        return _rigid_matrix(pose.pose.position, pose.pose.orientation)

    def _topic_geometry(self, velocity_frame):
        arm_from_controller_tcp = self._fresh_pose(self.tcp_pose, self.tcp_pose_received, self.tcp_pose_topic)
        world_from_base = self._fresh_pose(self.base_pose, self.base_pose_received, self.base_pose_topic)
        if self._clean_frame(self.base_pose.header.frame_id) != self.world_frame:
            raise ValueError(f'{self.base_pose_topic} must express {self.base_frame} in {self.world_frame}')
        arm_frame = self._clean_frame(self.tcp_pose.header.frame_id)
        frames = (velocity_frame, arm_frame)
        if self.cached_geometry is None:
            # Capture only once: the mount (including lift height) and active
            # controller TCP configuration must remain fixed during a run.
            base_from_arm = _transform_matrix(self._lookup(self.base_frame, arm_frame))
            base_from_velocity = _transform_matrix(self._lookup(self.base_frame, velocity_frame))
            tip_from_controller_tcp = _transform_matrix(self._lookup(self.tcp_frame, self.controller_tcp_frame))
            tip_from_nozzle = np.eye(4)
            tip_from_nozzle[:3, :3] = self.tool_rotation
            tip_from_nozzle[:3, 3] = self.tool_xyz
            controller_tcp_from_nozzle = np.linalg.inv(tip_from_controller_tcp) @ tip_from_nozzle
            self.cached_geometry = (frames, base_from_arm, base_from_velocity, controller_tcp_from_nozzle)
            self.get_logger().info(
                f'Captured fixed mounting geometry {self.base_frame} <- {arm_frame} and '
                f'{self.controller_tcp_frame} -> nozzle; compensation now uses pose topics without TF lookups.'
            )
        captured_frames, base_from_arm, base_from_velocity, controller_tcp_from_nozzle = self.cached_geometry
        if frames != captured_frames:
            raise ValueError('pose/velocity frame changed after mounting capture; restart the follower')
        velocity_from_nozzle = (np.linalg.inv(base_from_velocity) @ base_from_arm
                               @ arm_from_controller_tcp @ controller_tcp_from_nozzle)
        deposition_offset = (velocity_from_nozzle[:3, 3]
                             + velocity_from_nozzle[:3, :3] @ np.array([0.0, 0.0, self.spray_distance]))
        world_from_velocity_rotation = world_from_base[:3, :3] @ base_from_velocity[:3, :3]
        return deposition_offset, world_from_velocity_rotation

    def _set_velocity(self, msg: Twist, frame: str, stamp=None) -> None:
        self.latest_velocity = (
            np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float),
            np.array([msg.angular.x, msg.angular.y, msg.angular.z], dtype=float),
            self._clean_frame(frame) or self.base_frame,
        )
        self.last_velocity_time = self.get_clock().now()
        self.velocity_stamp = rclpy.time.Time.from_msg(stamp) if stamp is not None else None

    def _odom_cb(self, msg: Odometry) -> None:
        self._set_velocity(msg.twist.twist, msg.child_frame_id or self.base_frame, msg.header.stamp)

    def _stamped_cb(self, msg: TwistStamped) -> None:
        self._set_velocity(msg.twist, msg.header.frame_id or self.base_frame, msg.header.stamp)

    def _twist_cb(self, msg: Twist) -> None:
        self._set_velocity(msg, self.base_frame)

    def _lookup(self, target: str, source: str):
        transform = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        stamp = rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds
        # A zero stamp denotes a timeless static TF. Dynamic TF must stay fresh.
        if stamp:
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if age < 0.0 or age > self.tf_timeout:
                raise TransformException(f'Stale TF {target} <- {source}: age={age:.3f}s, limit={self.tf_timeout:.3f}s')
        return transform

    def _compute(self) -> Optional[Twist]:
        if self.latest_velocity is None or self.last_velocity_time is None:
            self.unavailable_reason = 'waiting for base velocity'
            return None
        age = (self.get_clock().now() - self.last_velocity_time).nanoseconds / 1e9
        if self.velocity_stamp is not None:
            stamp_age = (self.get_clock().now().nanoseconds - self.velocity_stamp.nanoseconds) / 1e9
            if stamp_age < 0.0 or stamp_age > self.stale_timeout:
                self.unavailable_reason = f'base velocity source age={stamp_age:.3f}s exceeds validity window'
                return None
        if age < 0.0 or age > self.stale_timeout or (not self.translation_only and not np.isfinite(self.spray_distance)):
            self.unavailable_reason = f'base velocity receipt age={age:.3f}s or invalid spray distance'
            return None

        linear, angular, velocity_frame = self.latest_velocity
        if self.translation_only:
            angular = np.zeros(3)
        if not np.all(np.isfinite(linear)) or not np.all(np.isfinite(angular)):
            self.unavailable_reason = 'invalid base velocity'
            return None
        try:
            # The velocity frame origin is the physical point at which its
            # linear velocity is measured (Odometry.child_frame_id). Compute
            # the lever arm from that origin, not from an arbitrary base frame.
            if self.translation_only:
                deposition_offset = np.zeros(3)
                world_rotation = self._translation_rotation(velocity_frame)
            elif self.pose_source == 'tcp_pose':
                deposition_offset, world_rotation = self._topic_geometry(velocity_frame)
            else:
                world_from_velocity = self._lookup(self.world_frame, velocity_frame)
                velocity_from_tip = self._lookup(velocity_frame, self.tcp_frame)
                tip_offset = self.tool_xyz + self.tool_rotation @ np.array([0.0, 0.0, self.spray_distance])
                deposition_offset = (_translation_from_transform(velocity_from_tip)
                                     + _rotation_from_transform(velocity_from_tip) @ tip_offset)
                world_rotation = _rotation_from_transform(world_from_velocity)
        except (TransformException, ValueError) as exc:
            self.unavailable_reason = str(exc)
            return None

        compensated_linear, compensated_angular = world_compensation(
            linear, angular, deposition_offset, world_rotation
        )
        if not (np.all(np.isfinite(compensated_linear)) and np.all(np.isfinite(compensated_angular))):
            self.unavailable_reason = 'invalid compensation geometry'
            return None

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
        self._report_connection_state(output is not None)
        if output is None:
            # Never let smoothing keep a non-zero correction alive after a
            # stale velocity or missing TF has removed the valid input.
            self.previous_output = Twist()
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self._smooth(output))

    def _report_connection_state(self, ready):
        elapsed = (self.get_clock().now().nanoseconds - self.started_at) / 1e9
        if ready:
            if self.connection_state != 'ready':
                self.get_logger().info('Base compensation ready; valid inputs available, compensation active.')
            self.ever_ready = True
            self.connection_state = 'ready'
        elif not self.ever_ready and elapsed < self.startup_wait_timeout:
            if self.connection_state != 'connecting':
                self.get_logger().info(
                    f'Base compensation connecting (up to {self.startup_wait_timeout:.0f}s startup grace): '
                    f'{self.unavailable_reason}. Publishing zero until inputs are valid.'
                )
            self.connection_state = 'connecting'
        else:
            self.get_logger().warn(
                f'Base compensation unavailable: {self.unavailable_reason}; publishing zero. '
                'Will resume automatically when valid inputs arrive.', throttle_duration_sec=10.0,
            )
            self.connection_state = 'unavailable'

    def publish_zero(self) -> None:
        """Clear the correction before the follower stack exits."""
        self.previous_output = Twist()
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseMotionCompensation()

    def _stop(_signum, _frame) -> None:
        # The launch process may terminate this node with SIGTERM. Publish
        # through the already matched publisher before exiting so the combiner
        # cannot retain a stale non-zero correction.
        node.publish_zero()
        raise KeyboardInterrupt

    previous_sigterm_handler = signal.signal(signal.SIGTERM, _stop)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        node.destroy_node()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
