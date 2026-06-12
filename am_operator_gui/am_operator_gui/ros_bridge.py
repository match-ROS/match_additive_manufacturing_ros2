import threading
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


StatusCallback = Callable[[bool, bool], None]
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
        self._has_robot_pose = False

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
        self.create_subscription(PoseStamped, '/robot_pose', self._robot_pose_cb, 10)

    def publish_path_index(self, value: int) -> None:
        self._path_index_pub.publish(Int32(data=int(value)))

    def publish_start_condition(self, value: bool = True) -> None:
        self._start_condition_pub.publish(Bool(data=bool(value)))

    def publish_velocity_override(self, value: float) -> None:
        self._velocity_override_pub.publish(Float32(data=float(value)))

    def publish_nozzle_height(self, value: float) -> None:
        self._nozzle_height_pub.publish(Float32(data=float(value)))

    @property
    def has_path(self) -> bool:
        return self._has_path

    @property
    def has_robot_pose(self) -> bool:
        return self._has_robot_pose

    def _base_path_cb(self, _msg: Path) -> None:
        self._has_path = True
        self._emit_status()

    def _robot_pose_cb(self, _msg: PoseStamped) -> None:
        self._has_robot_pose = True
        self._emit_status()

    def _path_index_cb(self, msg: Int32) -> None:
        if self._path_index_callback is not None:
            self._path_index_callback(int(msg.data))

    def _emit_status(self) -> None:
        if self._status_callback is not None:
            self._status_callback(self._has_path, self._has_robot_pose)


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

    @property
    def has_path(self) -> bool:
        return bool(self._node and self._node.has_path)

    @property
    def has_robot_pose(self) -> bool:
        return bool(self._node and self._node.has_robot_pose)
