"""Readiness heartbeat for the native MuR J-Parse velocity chain."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


def endpoints_ready(jparse_subscribers, velocity_subscribers) -> bool:
    """A command path is ready only when both native controller endpoints listen."""
    return bool(jparse_subscribers) and bool(velocity_subscribers)


class MurArmReadiness(Node):
    def __init__(self):
        super().__init__('mur_arm_readiness')
        self.declare_parameter('jparse_twist_topic', '/mur620a/jparse_velocity_controller_l/twist_cmd')
        self.declare_parameter('velocity_command_topic', '/mur620a/forward_velocity_controller_l/commands')
        self.declare_parameter('jparse_ready_topic', '/am/jparse_ready')
        self.declare_parameter('controller_ready_topic', '/am/arm_controller_ready')
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self._jparse_pub = self.create_publisher(Bool, str(self.get_parameter('jparse_ready_topic').value), qos)
        self._controller_pub = self.create_publisher(Bool, str(self.get_parameter('controller_ready_topic').value), qos)
        self.create_timer(0.25, self._publish)

    def _publish(self):
        jparse = self.get_subscriptions_info_by_topic(
            str(self.get_parameter('jparse_twist_topic').value))
        controller = self.get_subscriptions_info_by_topic(
            str(self.get_parameter('velocity_command_topic').value))
        self._jparse_pub.publish(Bool(data=bool(jparse)))
        self._controller_pub.publish(Bool(data=endpoints_ready(jparse, controller)))


def main(args=None):
    rclpy.init(args=args)
    node = MurArmReadiness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
