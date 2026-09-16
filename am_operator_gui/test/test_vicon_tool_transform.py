"""Check calibration composition without starting ROS discovery or hardware."""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from am_operator_gui.vicon_ee_static_tf import ViconToolTransform


@pytest.mark.parametrize('xyz, rotation', [
    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
    ([0.2, -0.1, 0.3], Rotation.from_euler('x', 45, degrees=True).as_quat().tolist()),
])
def test_configured_offset_is_composed_in_measured_frame(xyz, rotation):
    params = {
        'input_topic': '/vicon/EE/root',
        'marker_to_nozzle_xyz': xyz,
        'marker_to_nozzle_quaternion_xyzw': rotation,
    }
    outputs = []
    subscriptions = []
    def declare(_self, name, default):
        params.setdefault(name, default)
    with patch('am_operator_gui.vicon_ee_static_tf.Node.__init__', return_value=None), \
         patch.object(ViconToolTransform, 'declare_parameter', declare), \
         patch.object(ViconToolTransform, 'get_parameter', lambda self, name: SimpleNamespace(value=params[name])), \
         patch.object(ViconToolTransform, 'create_publisher', return_value=SimpleNamespace(publish=outputs.append)), \
         patch.object(ViconToolTransform, 'create_subscription', side_effect=lambda *args: subscriptions.append(args)), \
         patch('am_operator_gui.vicon_ee_static_tf.TransformBroadcaster'):
        adapter = ViconToolTransform()
        msg = PoseStamped()
        msg.header.frame_id = 'vicon_world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1.0, 2.0, 3.0
        measured_rotation = Rotation.from_euler('z', 90, degrees=True)
        q = measured_rotation.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q.tolist()
        adapter.callback(msg)
    assert subscriptions[0][1] == '/vicon/EE/root'
    out = outputs[0]
    assert out.header.frame_id == 'vicon_world'
    assert [out.pose.position.x, out.pose.position.y, out.pose.position.z] == pytest.approx(
        np.array([1, 2, 3]) + measured_rotation.apply(xyz))
    expected_rotation = measured_rotation * Rotation.from_quat(rotation)
    actual_rotation = Rotation.from_quat([out.pose.orientation.x, out.pose.orientation.y,
                                         out.pose.orientation.z, out.pose.orientation.w])
    assert actual_rotation.as_matrix() == pytest.approx(expected_rotation.as_matrix())
