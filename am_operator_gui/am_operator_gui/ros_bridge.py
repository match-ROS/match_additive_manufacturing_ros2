import threading
import math
from copy import deepcopy
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import Buffer, TransformException, TransformListener


StatusCallback = Callable[[bool, bool, bool, bool, bool], None]
PathIndexCallback = Callable[[int], None]


class OperatorGuiNode(Node):
    def __init__(
        self,
        status_callback: Optional[StatusCallback] = None,
        path_index_callback: Optional[PathIndexCallback] = None,
    ) -> None:
        super().__init__('am_operator_gui')
        self._status_callback = status_callback
        self._path_index_callback = path_index_callback
        self._has_path = False
        self._has_base_path = False
        self._has_arm_path = False
        self._has_robot_pose = False
        self._has_arm_pose = False
        self._jparse_ready = False
        self._controller_ready = False
        self._last_robot_pose_time = None
        self._last_arm_pose_time = None
        self._last_base_path_time = None
        self._last_arm_path_time = None
        self._last_jparse_ready_time = None
        self._last_controller_ready_time = None
        self._latest_base_path: Optional[Path] = None
        self._latest_arm_path: Optional[Path] = None
        self._latest_tracking_arm_path: Optional[Path] = None
        self._latest_robot_pose: Optional[PoseStamped] = None
        self._latest_pose_lock = threading.Lock()

        path_index_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._path_index_pub = self.create_publisher(
            Int32,
            '/path_index_command',
            path_index_qos,
        )
        self._start_condition_pub = self.create_publisher(Bool, '/start_condition', path_index_qos)
        self._velocity_override_pub = self.create_publisher(Float32, '/velocity_override', 10)
        self._desired_arm_speed_pub = self.create_publisher(Float32, '/desired_arm_speed', path_index_qos)
        self._nozzle_height_pub = self.create_publisher(Float32, '/nozzle_height_override', 10)
        self._spray_distance_pub = self.create_publisher(Float32, '/spray_distance', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Int32, '/path_index', self._path_index_cb, path_index_qos)
        self.create_subscription(Path, '/base_path', self._base_path_cb, 10)
        self.create_subscription(Path, '/ur_path_transformed', self._ur_path_cb, path_index_qos)
        self.create_subscription(Path, '/ur_path_tracking', self._tracking_arm_path_cb, path_index_qos)
        self.create_subscription(PoseStamped, '/robot_pose', self._robot_pose_cb, 10)
        self.create_subscription(PoseStamped, '/current_deposition_pose', self._arm_pose_cb, 10)
        self.create_subscription(Bool, '/am/jparse_ready', self._jparse_ready_cb, path_index_qos)
        self.create_subscription(
            Bool,
            '/am/arm_controller_ready',
            self._controller_ready_cb,
            path_index_qos,
        )
        self._base_stop_pub = self.create_publisher(
            Twist,
            '/robot/robotnik_base_control/cmd_vel_unstamped',
            10,
        )
        self._arm_stop_pub = self.create_publisher(
            TwistStamped,
            '/jparse_velocity_controller_ur/twist_cmd_world',
            10,
        )
        self.create_timer(0.1, self._freshness_tick)

    def publish_path_index(self, value: int) -> None:
        self._path_index_pub.publish(Int32(data=int(value)))

    def publish_start_condition(self, value: bool = True) -> None:
        self._start_condition_pub.publish(Bool(data=bool(value)))

    def publish_velocity_override(self, value: float) -> None:
        self._velocity_override_pub.publish(Float32(data=float(value)))

    def publish_desired_arm_speed(self, value: float) -> None:
        self._desired_arm_speed_pub.publish(Float32(data=max(0.0, float(value))))

    def publish_nozzle_height(self, value: float) -> None:
        self._nozzle_height_pub.publish(Float32(data=float(value)))

    def publish_spray_distance(self, value: float) -> None:
        self._spray_distance_pub.publish(Float32(data=float(value)))

    def lookup_tool_offset(self, tool_frame: str, controller_frame: str):
        try:
            return self._tf_buffer.lookup_transform(tool_frame, controller_frame, rclpy.time.Time())
        except TransformException:
            return None

    def reset_tf_buffer(self) -> None:
        """Discard transforms from a previous simulation clock epoch."""
        self._tf_buffer.clear()

    def publish_stop_commands(self, arm_frame: str) -> None:
        self._base_stop_pub.publish(Twist())
        arm_stop = TwistStamped()
        arm_stop.header.stamp = self.get_clock().now().to_msg()
        arm_stop.header.frame_id = arm_frame
        self._arm_stop_pub.publish(arm_stop)

    @property
    def has_path(self) -> bool:
        return self._has_path

    @property
    def has_robot_pose(self) -> bool:
        return self._has_robot_pose

    @property
    def has_arm_pose(self) -> bool:
        return self._has_arm_pose

    @property
    def jparse_ready(self) -> bool:
        return self._jparse_ready

    @property
    def controller_ready(self) -> bool:
        return self._controller_ready

    def latest_base_path_pose(self, index: int) -> Optional[PoseStamped]:
        with self._latest_pose_lock:
            if self._latest_base_path is None:
                return None
            if index < 0 or index >= len(self._latest_base_path.poses):
                return None
            return deepcopy(self._latest_base_path.poses[index])

    def latest_robot_pose(self) -> Optional[PoseStamped]:
        with self._latest_pose_lock:
            return deepcopy(self._latest_robot_pose)

    def original_arm_index_for_tracking_index(self, index: int) -> int:
        with self._latest_pose_lock:
            return self._index_for_relative_progress(
                self._latest_tracking_arm_path,
                self._latest_arm_path,
                index,
            )

    def tracking_arm_index_for_original_index(self, index: int) -> int:
        with self._latest_pose_lock:
            return self._index_for_relative_progress(
                self._latest_arm_path,
                self._latest_tracking_arm_path,
                index,
            )

    @staticmethod
    def _index_for_relative_progress(
        source: Optional[Path], target: Optional[Path], index: int,
    ) -> int:
        if source is None or target is None or not source.poses or not target.poses:
            return max(0, int(index))
        source_index = max(0, min(int(index), len(source.poses) - 1))

        def cumulative_lengths(path: Path) -> list[float]:
            lengths = [0.0]
            for previous, current in zip(path.poses, path.poses[1:]):
                start = previous.pose.position
                end = current.pose.position
                lengths.append(lengths[-1] + math.dist(
                    (start.x, start.y, start.z),
                    (end.x, end.y, end.z),
                ))
            return lengths

        source_lengths = cumulative_lengths(source)
        target_lengths = cumulative_lengths(target)
        source_total = source_lengths[-1]
        target_total = target_lengths[-1]
        if source_total <= 1e-9 or target_total <= 1e-9:
            return int(round(source_index * (len(target.poses) - 1) / max(1, len(source.poses) - 1)))
        target_distance = source_lengths[source_index] / source_total * target_total
        return min(
            range(len(target_lengths)),
            key=lambda candidate: abs(target_lengths[candidate] - target_distance),
        )

    def _base_path_cb(self, msg: Path) -> None:
        self._has_base_path = self._is_map_path(msg)
        self._last_base_path_time = self.get_clock().now() if self._has_base_path else None
        self._has_path = self._has_base_path and self._has_arm_path
        with self._latest_pose_lock:
            self._latest_base_path = msg
        self._emit_status()

    def _robot_pose_cb(self, msg: PoseStamped) -> None:
        self._has_robot_pose = self._is_fresh_map_pose(msg)
        self._last_robot_pose_time = self.get_clock().now() if self._has_robot_pose else None
        with self._latest_pose_lock:
            self._latest_robot_pose = msg
        self._emit_status()

    def _arm_pose_cb(self, msg: PoseStamped) -> None:
        self._has_arm_pose = self._is_fresh_map_pose(msg)
        self._last_arm_pose_time = self.get_clock().now() if self._has_arm_pose else None
        self._emit_status()

    def _ur_path_cb(self, msg: Path) -> None:
        self._has_arm_path = self._is_map_path(msg)
        self._last_arm_path_time = self.get_clock().now() if self._has_arm_path else None
        self._has_path = self._has_base_path and self._has_arm_path
        with self._latest_pose_lock:
            self._latest_arm_path = msg

    def _tracking_arm_path_cb(self, msg: Path) -> None:
        with self._latest_pose_lock:
            self._latest_tracking_arm_path = msg

    def _path_index_cb(self, msg: Int32) -> None:
        if self._path_index_callback is not None:
            self._path_index_callback(int(msg.data))

    def _jparse_ready_cb(self, msg: Bool) -> None:
        self._jparse_ready = bool(msg.data)
        self._last_jparse_ready_time = self.get_clock().now()
        self._emit_status()

    def _controller_ready_cb(self, msg: Bool) -> None:
        self._controller_ready = bool(msg.data)
        self._last_controller_ready_time = self.get_clock().now()
        self._emit_status()

    def _freshness_tick(self) -> None:
        now = self.get_clock().now()
        if self._last_robot_pose_time is not None:
            self._has_robot_pose = (now - self._last_robot_pose_time).nanoseconds / 1e9 <= 0.75
        if self._last_arm_pose_time is not None:
            self._has_arm_pose = (now - self._last_arm_pose_time).nanoseconds / 1e9 <= 0.75
        if self._last_base_path_time is not None:
            self._has_base_path = (now - self._last_base_path_time).nanoseconds / 1e9 <= 0.75
        if self._last_arm_path_time is not None:
            self._has_arm_path = (now - self._last_arm_path_time).nanoseconds / 1e9 <= 0.75
        self._has_path = self._has_base_path and self._has_arm_path
        if self._last_jparse_ready_time is not None:
            fresh = (now - self._last_jparse_ready_time).nanoseconds / 1e9 <= 2.5
            self._jparse_ready = self._jparse_ready and fresh
        if self._last_controller_ready_time is not None:
            fresh = (now - self._last_controller_ready_time).nanoseconds / 1e9 <= 2.5
            self._controller_ready = self._controller_ready and fresh
        self._emit_status()

    def _is_fresh_map_pose(self, msg: PoseStamped) -> bool:
        if msg.header.frame_id.strip().lstrip('/') != 'map':
            return False
        stamp = msg.header.stamp
        # Freshness is measured from the local subscription receipt time in
        # _freshness_tick().  Comparing the message stamp here is invalid when
        # the GUI uses wall time and the robot/simulation publishes ROS time.
        return bool(stamp.sec or stamp.nanosec)

    @staticmethod
    def _is_map_path(msg: Path) -> bool:
        if (not msg.poses or not (msg.header.stamp.sec or msg.header.stamp.nanosec) or
                msg.header.frame_id.strip().lstrip('/') != 'map'):
            return False
        return all(
            (pose.header.frame_id or msg.header.frame_id).strip().lstrip('/') == 'map'
            for pose in msg.poses
        )

    def _emit_status(self) -> None:
        if self._status_callback is not None:
            self._status_callback(
                self._has_path,
                self._has_robot_pose,
                self._has_arm_pose,
                self._jparse_ready,
                self._controller_ready,
            )


class RosBridge:
    def __init__(
        self,
        status_callback: Optional[StatusCallback] = None,
        path_index_callback: Optional[PathIndexCallback] = None,
    ) -> None:
        self._status_callback = status_callback
        self._path_index_callback = path_index_callback
        self._node: Optional[OperatorGuiNode] = None
        self._executor_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = OperatorGuiNode(self._status_callback, self._path_index_callback)
        self._executor_thread = threading.Thread(
            target=self._spin_node,
            args=(self._node,),
            daemon=True,
        )
        self._executor_thread.start()

    @staticmethod
    def _spin_node(node: OperatorGuiNode) -> None:
        try:
            rclpy.spin(node)
        except Exception:
            # Ctrl-C may shut down rclpy before an ASGI/Qt lifecycle hook has
            # joined this daemon thread. There is no recovery work to do then.
            if rclpy.ok():
                raise

    def stop(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)
            self._executor_thread = None

    def publish_path_index(self, value: int) -> None:
        if self._node is not None:
            try:
                self._node.publish_path_index(value)
            except Exception:
                pass

    def reset_tf_buffer(self) -> None:
        if self._node is not None:
            try:
                self._node.reset_tf_buffer()
            except Exception:
                pass

    def publish_start_condition(self, value: bool = True) -> None:
        if self._node is not None:
            try:
                self._node.publish_start_condition(value)
            except Exception:
                pass

    def publish_velocity_override(self, value: float) -> None:
        if self._node is not None:
            try:
                self._node.publish_velocity_override(value)
            except Exception:
                pass

    def publish_nozzle_height(self, value: float) -> None:
        if self._node is not None:
            try:
                self._node.publish_nozzle_height(value)
            except Exception:
                pass

    def publish_spray_distance(self, value: float) -> None:
        if self._node is not None:
            try:
                self._node.publish_spray_distance(value)
            except Exception:
                pass

    def lookup_tool_offset(self, tool_frame: str, controller_frame: str):
        if self._node is None:
            return None
        return self._node.lookup_tool_offset(tool_frame, controller_frame)

    def publish_stop_commands(self, arm_frame: str) -> None:
        if self._node is not None:
            try:
                self._node.publish_stop_commands(arm_frame)
            except Exception:
                pass

    def check_topic_contract(self, requirements: list[tuple[str, str, str]]) -> list[str]:
        """Return compact pass/fail messages for (topic, type, endpoint) checks.

        ``endpoint`` is ``publisher`` for a hardware/Vicon input and
        ``subscriber`` for a command endpoint.  This uses the ROS graph only;
        freshness and controller state remain separate runtime checks.
        """
        if self._node is None:
            return ['ROS bridge is unavailable']
        messages: list[str] = []
        for topic, expected_type, endpoint in requirements:
            try:
                # get_topic_names_and_types() returns (topic_name, [type, ...]),
                # so flatten the advertised type list rather than accidentally
                # displaying the topic name as a type.
                types = {
                    topic_type
                    for topic_name, topic_types in self._node.get_topic_names_and_types()
                    if topic_name == topic
                    for topic_type in topic_types
                }
                infos = (
                    self._node.get_publishers_info_by_topic(topic)
                    if endpoint == 'publisher'
                    else self._node.get_subscriptions_info_by_topic(topic)
                )
            except Exception as exc:
                messages.append(f'FAIL {topic}: graph query failed ({exc})')
                continue
            if expected_type not in types:
                found = ', '.join(sorted(types)) or 'not advertised'
                messages.append(f'FAIL {topic}: expected {expected_type}, found {found}')
            elif not infos:
                messages.append(f'FAIL {topic}: no {endpoint} found')
            else:
                messages.append(f'OK {topic}: {expected_type}, {len(infos)} {endpoint}(s)')
        return messages

    @property
    def has_path(self) -> bool:
        return bool(self._node and self._node.has_path)

    @property
    def has_robot_pose(self) -> bool:
        return bool(self._node and self._node.has_robot_pose)

    @property
    def has_arm_pose(self) -> bool:
        return bool(self._node and self._node.has_arm_pose)

    @property
    def jparse_ready(self) -> bool:
        return bool(self._node and self._node.jparse_ready)

    @property
    def controller_ready(self) -> bool:
        return bool(self._node and self._node.controller_ready)

    def latest_base_path_pose(self, index: int) -> Optional[PoseStamped]:
        if self._node is None:
            return None
        return self._node.latest_base_path_pose(index)

    def latest_robot_pose(self) -> Optional[PoseStamped]:
        if self._node is None:
            return None
        return self._node.latest_robot_pose()

    def original_arm_index_for_tracking_index(self, index: int) -> int:
        if self._node is None:
            return max(0, int(index))
        return self._node.original_arm_index_for_tracking_index(index)

    def tracking_arm_index_for_original_index(self, index: int) -> int:
        if self._node is None:
            return max(0, int(index))
        return self._node.tracking_arm_index_for_original_index(index)
