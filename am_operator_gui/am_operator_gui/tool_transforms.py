"""Calibration defaults and validation shared by the GUI and Vicon adapter."""

import math


# Previous marker-to-TCP calibration, expressed as a rigid transform.
DEFAULT_VICON_NOZZLE_TRANSFORM = {
    'xyz': [0.184687295, -0.501541068, -0.126693390],
    'quaternion_xyzw': [0.0014631726632651522, -0.003424541244486294,
                        0.8435437871654295, 0.5370474939682958],
}
# Base pose calibration expressed as T_vicon_cluster__robot_base.  This is the
# inverse of the historic static TF (robot_base_footprint ->
# robot_base_vicon_reference) so existing deployments retain their calibration
# while the UI can expose the physically intuitive cluster-to-base direction.
DEFAULT_VICON_CLUSTER_TO_BASE_TRANSFORM = {
    'xyz': [-0.022345829062692456, 0.008706477947710399, 0.007544808501644998],
    'quaternion_xyzw': [-0.0044597839995404705, 0.006515751999328627,
                        -0.009033289999069223, 0.999928024896969],
}
VICON_NOZZLE_TOPIC = '/vicon/tool_transformed'
DEFAULT_VICON_INPUT_TOPIC = '/vicon/Tool_Flange/Tool_Flange'


def validated_transform(value):
    """Require finite metre/XYZW values and return a normalized quaternion."""
    if not isinstance(value, dict):
        raise ValueError('Transform must contain xyz and quaternion_xyzw arrays')
    result = {}
    for key, length in (('xyz', 3), ('quaternion_xyzw', 4)):
        values = value.get(key)
        if not isinstance(values, (list, tuple)) or len(values) != length:
            raise ValueError(f'{key} must contain {length} numbers')
        numbers = [float(item) for item in values]
        if not all(math.isfinite(item) for item in numbers):
            raise ValueError(f'{key} must contain finite numbers')
        result[key] = numbers
    norm = math.hypot(*result['quaternion_xyzw'])
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Quaternion norm must be finite and greater than zero')
    result['quaternion_xyzw'] = [item / norm for item in result['quaternion_xyzw']]
    return result


def inverted_transform(value):
    """Return the inverse of a validated rigid XYZ/XYZW transform."""
    transform = validated_transform(value)
    x, y, z, w = transform['quaternion_xyzw']
    px, py, pz = transform['xyz']
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return {
        # -R^T p
        'xyz': [-sum(rotation[row][column] * (px, py, pz)[row] for row in range(3))
                for column in range(3)],
        'quaternion_xyzw': [-x, -y, -z, w],
    }
