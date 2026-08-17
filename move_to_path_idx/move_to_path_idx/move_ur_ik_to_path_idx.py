#!/usr/bin/env python3
"""Move a MuR-mounted UR to a path pose with IK, before twist correction.

The path and feedback conventions deliberately stay in deposition-point space.
MoveIt is asked for the corresponding ``tool0`` pose and the normal
``move_ur_to_path_idx`` node then removes the remaining deposition-pose error.
"""

import math
from enum import Enum
from typing import Optional

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class State(Enum):
    WAITING = 0
    WAITING_FOR_IK = 1
    WAITING_FOR_TRAJECTORY_SWITCH = 2
    WAITING_FOR_TRAJECTORY = 3
    WAITING_FOR_VELOCITY_SWITCH = 4
    DONE = 5


def q_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in q))
    return (0.0, 0.0, 0.0, 1.0) if length < 1.0e-9 else tuple(value / length for value in q)


def q_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    rotated = q_multiply(q_multiply(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return rotated[0], rotated[1], rotated[2]


def pose_from_transform(transform: TransformStamped) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation = transform.transform.rotation
    return pose


def compose(a: Pose, b: Pose) -> Pose:
    """Return the pose produced by applying b in a's local frame."""
    qa = q_normalize((a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w))
    qb = q_normalize((b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w))
    bx, by, bz = q_rotate(qa, (b.position.x, b.position.y, b.position.z))
    result = Pose()
    result.position.x = a.position.x + bx
    result.position.y = a.position.y + by
    result.position.z = a.position.z + bz
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = q_multiply(qa, qb)
    return result


def inverse(pose: Pose) -> Pose:
    q = q_normalize((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    qi = (-q[0], -q[1], -q[2], q[3])
    x, y, z = q_rotate(qi, (-pose.position.x, -pose.position.y, -pose.position.z))
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = qi
    return result


class MoveUrIkToPathIdx(Node):
    """Request a seeded IK solution and publish it as one joint trajectory."""

    def __init__(self) -> None:
        super().__init__('move_ur_ik_to_path_idx')
        self.declare_parameter('path_topic', '/ur_path_transformed')
        self.declare_parameter('path_index', 0)
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('wait_for_start_condition', True)
        self.declare_parameter('start_condition_topic', '/start_pose_reached')
        self.declare_parameter('correction_ready_topic', '/am/ik_correction_ready')
        self.declare_parameter('ik_service', '/mur620a/compute_ik')
        self.declare_parameter('ik_group_name', 'UR_arm_r')
        self.declare_parameter('ik_link_name', 'UR10_r/tool0')
        # TF carries the MuR namespace; MoveIt uses unnamespaced link names.
        self.declare_parameter('tf_arm_base_frame', 'mur620a/base_footprint')
        self.declare_parameter('ik_pose_frame', 'base_footprint')
        self.declare_parameter('joint_states_topic', '/mur620a/joint_states')
        self.declare_parameter('joint_names', [
            'UR10_r/shoulder_pan_joint', 'UR10_r/shoulder_lift_joint', 'UR10_r/elbow_joint',
            'UR10_r/wrist_1_joint', 'UR10_r/wrist_2_joint', 'UR10_r/wrist_3_joint',
        ])
        # This seed selects the normal right-shoulder, elbow-up, unflipped-wrist
        # branch.  Supplying ik_seed_positions selects a different configuration.
        self.declare_parameter('ik_configuration', 'shoulder_right_elbow_up_wrist_unflip')
        self.declare_parameter('ik_seed_positions', [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        self.declare_parameter('fixed_tool_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('fixed_tool_offset_quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('spray_distance', 0.1)
        self.declare_parameter('trajectory_topic', '/mur620a/joint_trajectory_controller_r/joint_trajectory')
        self.declare_parameter('controller_manager', '/mur620a/controller_manager')
        self.declare_parameter('joint_trajectory_controller', 'joint_trajectory_controller_r')
        self.declare_parameter('velocity_controller', 'forward_velocity_controller_r')
        self.declare_parameter('trajectory_duration', 8.0)
        self.declare_parameter('settle_time', 1.0)

        self.path: Optional[Path] = None
        self.joints: dict[str, float] = {}
        self.started = not bool(self.get_parameter('wait_for_start_condition').value)
        self.state = State.WAITING
        self.trajectory_deadline = None
        self.ik_solution: Optional[dict[str, float]] = None
        self.last_transform_warning = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, str(self.get_parameter('path_topic').value), self._path_cb, qos)
        self.create_subscription(Bool, str(self.get_parameter('start_condition_topic').value), self._start_cb, qos)
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value), self._joint_cb, 10)
        self.ready_pub = self.create_publisher(Bool, str(self.get_parameter('correction_ready_topic').value), qos)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('trajectory_topic').value), 10)
        self.ik_client = self.create_client(GetPositionIK, str(self.get_parameter('ik_service').value))
        self.switch_client = self.create_client(
            SwitchController, str(self.get_parameter('controller_manager').value) + '/switch_controller')
        self.create_timer(0.1, self._tick)

    def _path_cb(self, message: Path) -> None:
        self.path = message if message.poses else None

    def _start_cb(self, message: Bool) -> None:
        self.started = bool(message.data) or not bool(self.get_parameter('wait_for_start_condition').value)

    def _joint_cb(self, message: JointState) -> None:
        self.joints.update({name: position for name, position in zip(message.name, message.position)})

    def _target_tool_pose(self) -> Optional[PoseStamped]:
        if self.path is None:
            return None
        index = int(self.get_parameter('path_index').value)
        if index < 0 or index >= len(self.path.poses):
            self.get_logger().error(f'Path index {index} is outside the received path.')
            self._finish()
            return None
        fixed_xyz = list(self.get_parameter('fixed_tool_offset_xyz').value)
        fixed_q = list(self.get_parameter('fixed_tool_offset_quaternion_xyzw').value)
        if len(fixed_xyz) != 3 or len(fixed_q) != 4:
            self.get_logger().error('Fixed tool offset must contain xyz and xyzw values.')
            self._finish()
            return None
        tool_to_nozzle = Pose()
        tool_to_nozzle.position.x, tool_to_nozzle.position.y, tool_to_nozzle.position.z = fixed_xyz
        tool_to_nozzle.orientation.x, tool_to_nozzle.orientation.y, tool_to_nozzle.orientation.z, tool_to_nozzle.orientation.w = fixed_q
        nozzle_to_deposition = Pose()
        nozzle_to_deposition.position.z = float(self.get_parameter('spray_distance').value)
        nozzle_to_deposition.orientation.w = 1.0
        tool_to_deposition = compose(tool_to_nozzle, nozzle_to_deposition)
        target_in_path = compose(self.path.poses[index].pose, inverse(tool_to_deposition))
        path_frame = self.path.header.frame_id or str(self.get_parameter('path_frame').value)
        tf_arm_base = str(self.get_parameter('tf_arm_base_frame').value)
        ik_pose_frame = str(self.get_parameter('ik_pose_frame').value)
        try:
            base_from_path = pose_from_transform(
                self.tf_buffer.lookup_transform(tf_arm_base, path_frame, rclpy.time.Time()))
        except Exception as error:
            now = self.get_clock().now().nanoseconds / 1.0e9
            if now - self.last_transform_warning >= 3.0:
                self.last_transform_warning = now
                self.get_logger().warn(
                    f'Waiting for IK frame transform {tf_arm_base} <- {path_frame}: {error}')
            return None
        result = PoseStamped()
        result.header.frame_id = ik_pose_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose = compose(base_from_path, target_in_path)
        return result

    def _ik_seed(self) -> RobotState:
        names = list(self.get_parameter('joint_names').value)
        requested = list(self.get_parameter('ik_seed_positions').value)
        configuration = str(self.get_parameter('ik_configuration').value).strip().lower()
        if configuration == 'keep_current' and all(name in self.joints for name in names):
            requested = [self.joints[name] for name in names]
        if len(requested) != len(names):
            self.get_logger().warn('Invalid IK seed length; using current joint positions where available.')
            requested = [self.joints.get(name, 0.0) for name in names]
        state = RobotState()
        state.joint_state.name = names
        state.joint_state.position = [float(value) for value in requested]
        return state

    def _request_ik(self, target: PoseStamped) -> None:
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = str(self.get_parameter('ik_group_name').value)
        request.ik_request.ik_link_name = str(self.get_parameter('ik_link_name').value)
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state = self._ik_seed()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        self.ik_client.call_async(request).add_done_callback(self._ik_response)
        self.state = State.WAITING_FOR_IK
        self.get_logger().info(
            f'Requesting IK for path index {self.get_parameter("path_index").value} using '
            f'{self.get_parameter("ik_configuration").value}.'
        )

    def _ik_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'IK service call failed: {error}; using twist correction only.')
            self._finish()
            return
        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(f'IK has no solution ({response.error_code.val}); using twist correction only.')
            self._finish()
            return
        names = list(self.get_parameter('joint_names').value)
        solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        if not all(name in solution for name in names):
            self.get_logger().error('IK solution is missing one or more UR joints; using twist correction only.')
            self._finish()
            return
        self.ik_solution = solution
        self._switch_to_trajectory_controller()

    def _switch_to_trajectory_controller(self) -> None:
        if not self.switch_client.service_is_ready():
            self.get_logger().error('Trajectory controller switch service is unavailable; using twist correction only.')
            self._finish()
            return
        request = SwitchController.Request()
        request.activate_controllers = [str(self.get_parameter('joint_trajectory_controller').value)]
        request.deactivate_controllers = [str(self.get_parameter('velocity_controller').value)]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        self.switch_client.call_async(request).add_done_callback(self._trajectory_switch_response)
        self.state = State.WAITING_FOR_TRAJECTORY_SWITCH

    def _trajectory_switch_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Could not activate trajectory controller: {error}; using twist correction only.')
            self._finish()
            return
        if not response.ok or self.ik_solution is None:
            self.get_logger().error(
                f'Could not activate trajectory controller: {response.message}; using twist correction only.')
            self._finish()
            return
        names = list(self.get_parameter('joint_names').value)
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [self.ik_solution[name] for name in names]
        duration = max(0.1, float(self.get_parameter('trajectory_duration').value))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)
        self.trajectory_deadline = self.get_clock().now().nanoseconds / 1.0e9 + duration + max(0.0, float(self.get_parameter('settle_time').value))
        self.state = State.WAITING_FOR_TRAJECTORY
        self.get_logger().info('Published IK joint trajectory; waiting before twist correction.')

    def _switch_to_velocity_controller(self) -> None:
        if not self.switch_client.service_is_ready():
            self.get_logger().error('Velocity controller switch service is unavailable; cannot start correction.')
            self._finish()
            return
        request = SwitchController.Request()
        request.activate_controllers = [str(self.get_parameter('velocity_controller').value)]
        request.deactivate_controllers = [str(self.get_parameter('joint_trajectory_controller').value)]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        self.switch_client.call_async(request).add_done_callback(self._velocity_switch_response)
        self.state = State.WAITING_FOR_VELOCITY_SWITCH

    def _velocity_switch_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Could not reactivate velocity controller: {error}.')
            self._finish()
            return
        if not response.ok:
            self.get_logger().error(f'Could not reactivate velocity controller: {response.message}.')
        self._finish()

    def _finish(self) -> None:
        if self.state == State.DONE:
            return
        self.ready_pub.publish(Bool(data=True))
        self.state = State.DONE
        self.get_logger().info('IK stage complete; enabled deposition-pose twist correction.')

    def _tick(self) -> None:
        if self.state == State.DONE:
            return
        if self.state == State.WAITING_FOR_TRAJECTORY:
            if self.trajectory_deadline is not None and self.get_clock().now().nanoseconds / 1.0e9 >= self.trajectory_deadline:
                self._switch_to_velocity_controller()
            return
        if not self.started or self.path is None:
            return
        if self.state == State.WAITING:
            if not self.ik_client.service_is_ready():
                self.get_logger().info('Waiting for MoveIt IK service.', throttle_duration_sec=3.0)
                return
            target = self._target_tool_pose()
            if target is not None:
                self._request_ik(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveUrIkToPathIdx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
