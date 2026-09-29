"""TCP fallback variants compose the intended transforms and switch live."""
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

from am_operator_gui.pose_stamped_adapter import PoseStampedAdapter


class _Buffer:
    def __init__(self):
        self.lookups = []

    def lookup_transform(self, parent, child, _time):
        self.lookups.append((parent, child))
        offsets = {
            ('base_link', 'robot_arm_nozzle_tip'): 2.0,
            ('base_link', 'robot_arm_base'): 2.0,
            ('robot_arm_tool0_controller_raw', 'robot_arm_nozzle_tip'): 0.5,
        }
        transform = TransformStamped()
        transform.transform.translation.x = offsets[(parent, child)]
        transform.transform.rotation.w = 1.0
        return transform


def _adapter(source):
    outputs, ready, warnings = [], [], []
    tick = SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=1, nanosec=0))
    adapter = object.__new__(PoseStampedAdapter)
    adapter.source = source
    adapter.target_frame = 'map'
    adapter.buffer = _Buffer()
    adapter.latest_base_pose = None
    adapter.topic_geometry = None
    adapter.last_output_time = None
    adapter.get_parameter = lambda name: SimpleNamespace(value={
        'robot_base_frame': 'base_link',
        'robot_tcp_frame': 'robot_arm_nozzle_tip',
        'controller_tcp_frame': 'robot_arm_tool0_controller_raw',
    }[name])
    adapter.get_clock = lambda: SimpleNamespace(now=lambda: tick)
    adapter.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: warnings.append(message),
        info=lambda _message: None)
    adapter.pose_pub = SimpleNamespace(publish=outputs.append)
    adapter._set_ready = ready.append
    return adapter, outputs, ready, warnings


def _pose(frame, x, seconds=0, nanos=0):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec = seconds
    pose.header.stamp.nanosec = nanos
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def test_tf_variant_composes_pose_and_switches_live():
    adapter, outputs, ready, _warnings = _adapter('measured')
    adapter._source_mode(String(data='tf'))
    base = _pose('map', 1.0)
    q = quaternion_from_euler(0, 0, 1.5707963267948966)
    (base.pose.orientation.x, base.pose.orientation.y,
     base.pose.orientation.z, base.pose.orientation.w) = map(float, q)
    adapter._base_pose_cb(base)

    assert len(outputs) == 1
    assert abs(outputs[0].pose.position.x - 1.0) < 1e-6
    assert abs(outputs[0].pose.position.y - 2.0) < 1e-6
    assert outputs[0].header.frame_id == 'map'
    assert ready[-1] is True

    adapter._source_mode(String(data='measured'))
    adapter._base_pose_cb(base)
    assert len(outputs) == 1
    assert ready[-1] is False


def test_topic_variant_uses_latest_base_only_on_arm_messages_and_caches_tf():
    adapter, outputs, ready, warnings = _adapter('topic')
    arm = _pose('robot_arm_base', 3.0, seconds=1, nanos=500_000_000)
    adapter._controller_tcp_cb(arm)
    assert not outputs

    adapter._base_pose_cb(_pose('map', 1.0, seconds=1))
    assert not outputs
    adapter._controller_tcp_cb(arm)
    assert len(outputs) == 1
    assert abs(outputs[-1].pose.position.x - 6.5) < 1e-6
    assert outputs[-1].header.stamp.nanosec == 500_000_000
    assert len(warnings) == 2  # Missing base, then the 0.5 s timestamp mismatch.
    assert '0.500s' in warnings[-1]
    assert ready[-1] is True
    assert adapter.buffer.lookups == [
        ('base_link', 'robot_arm_base'),
        ('robot_arm_tool0_controller_raw', 'robot_arm_nozzle_tip'),
    ]

    adapter._base_pose_cb(_pose('map', 4.0, seconds=1, nanos=300_000_000))
    assert len(outputs) == 1
    adapter._controller_tcp_cb(arm)
    assert len(outputs) == 2
    assert abs(outputs[-1].pose.position.x - 9.5) < 1e-6
    assert len(warnings) == 2  # 0.2 s difference produces no warning.
    assert len(adapter.buffer.lookups) == 2  # Fixed TF was not looked up again.


def test_topic_variant_skips_timestamp_check_if_either_stamp_is_zero():
    adapter, outputs, _ready, warnings = _adapter('topic')
    adapter._base_pose_cb(_pose('map', 1.0))
    adapter._controller_tcp_cb(_pose('robot_arm_base', 3.0, seconds=10))
    assert len(outputs) == 1
    assert not warnings
