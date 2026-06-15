import threading
import math
import statistics
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


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
        self._last_jparse_ready_time = None
        self._last_controller_ready_time = None
        self._latest_ur_path_rate: Optional[float] = None
        self._latest_ur_path_median_segment_length: Optional[float] = None
        self._path_rate_lock = threading.Lock()

        path_index_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._path_index_pub = self.create_publisher(Int32, '/path_index', path_index_qos)
        self._start_condition_pub = self.create_publisher(Bool, '/start_condition', 10)
        self._velocity_override_pub = self.create_publisher(Float32, '/velocity_override', 10)
        self._nozzle_height_pub = self.create_publisher(Float32, '/nozzle_height_override', 10)
        self.create_subscription(Int32, '/path_index', self._path_index_cb, path_index_qos)
        self.create_subscription(Path, '/base_path', self._base_path_cb, 10)
        self.create_subscription(Path, '/ur_path_transformed', self._ur_path_cb, path_index_qos)
        self.create_subscription(PoseStamped, '/robot_pose', self._robot_pose_cb, 10)
        self.create_subscription(PoseStamped, '/current_tcp_pose', self._arm_pose_cb, 10)
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

    def publish_nozzle_height(self, value: float) -> None:
        self._nozzle_height_pub.publish(Float32(data=float(value)))

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

    @property
    def latest_ur_path_rate(self) -> Optional[float]:
        with self._path_rate_lock:
            return self._latest_ur_path_rate

    @property
    def latest_ur_path_median_segment_length(self) -> Optional[float]:
        with self._path_rate_lock:
            return self._latest_ur_path_median_segment_length

    def _base_path_cb(self, msg: Path) -> None:
        self._has_base_path = bool(msg.poses)
        self._has_path = self._has_base_path and self._has_arm_path
        self._emit_status()

    def _robot_pose_cb(self, _msg: PoseStamped) -> None:
        self._has_robot_pose = True
        self._last_robot_pose_time = self.get_clock().now()
        self._emit_status()

    def _arm_pose_cb(self, _msg: PoseStamped) -> None:
        self._has_arm_pose = True
        self._last_arm_pose_time = self.get_clock().now()
        self._emit_status()

    def _ur_path_cb(self, msg: Path) -> None:
        self._has_arm_path = bool(msg.poses)
        self._has_path = self._has_base_path and self._has_arm_path
        rate = self._mean_path_timestamp_rate(msg)
        median_segment_length = self._median_path_segment_length(msg)
        with self._path_rate_lock:
            self._latest_ur_path_rate = rate
            self._latest_ur_path_median_segment_length = median_segment_length

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
        if self._last_jparse_ready_time is not None:
            fresh = (now - self._last_jparse_ready_time).nanoseconds / 1e9 <= 2.5
            self._jparse_ready = self._jparse_ready and fresh
        if self._last_controller_ready_time is not None:
            fresh = (now - self._last_controller_ready_time).nanoseconds / 1e9 <= 2.5
            self._controller_ready = self._controller_ready and fresh
        self._emit_status()

    def _emit_status(self) -> None:
        if self._status_callback is not None:
            self._status_callback(
                self._has_path,
                self._has_robot_pose,
                self._has_arm_pose,
                self._jparse_ready,
                self._controller_ready,
            )

    @staticmethod
    def _mean_path_timestamp_rate(msg: Path) -> Optional[float]:
        if len(msg.poses) < 2:
            return None

        deltas = []
        previous_stamp = msg.poses[0].header.stamp
        previous_time = float(previous_stamp.sec) + float(previous_stamp.nanosec) / 1e9
        for pose in msg.poses[1:]:
            stamp = pose.header.stamp
            current_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
            delta = current_time - previous_time
            if delta > 0.0:
                deltas.append(delta)
            previous_time = current_time

        if not deltas:
            return None
        return len(deltas) / sum(deltas)

    @staticmethod
    def _median_path_segment_length(msg: Path) -> Optional[float]:
        if len(msg.poses) < 2:
            return None

        lengths = []
        previous = msg.poses[0].pose.position
        for pose in msg.poses[1:]:
            current = pose.pose.position
            length = math.dist(
                (previous.x, previous.y, previous.z),
                (current.x, current.y, current.z),
            )
            if length > 0.0:
                lengths.append(length)
            previous = current

        if not lengths:
            return None
        return float(statistics.median(lengths))


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
        self._executor_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._executor_thread.start()

    def stop(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)
            self._executor_thread = None

    def publish_path_index(self, value: int) -> None:
        if self._node is not None:
            self._node.publish_path_index(value)

    def publish_start_condition(self, value: bool = True) -> None:
        if self._node is not None:
            self._node.publish_start_condition(value)

    def publish_velocity_override(self, value: float) -> None:
        if self._node is not None:
            self._node.publish_velocity_override(value)

    def publish_nozzle_height(self, value: float) -> None:
        if self._node is not None:
            self._node.publish_nozzle_height(value)

    def publish_stop_commands(self, arm_frame: str) -> None:
        if self._node is not None:
            self._node.publish_stop_commands(arm_frame)

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

    @property
    def latest_ur_path_rate(self) -> Optional[float]:
        if self._node is None:
            return None
        return self._node.latest_ur_path_rate

    @property
    def latest_ur_path_median_segment_length(self) -> Optional[float]:
        if self._node is None:
            return None
        return self._node.latest_ur_path_median_segment_length
