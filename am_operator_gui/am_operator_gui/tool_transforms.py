"""Calibration defaults and validation shared by the GUI and Vicon adapter."""

import math


# Previous marker-to-TCP calibration, expressed as a rigid transform.
DEFAULT_VICON_NOZZLE_TRANSFORM = {
    'xyz': [0.184687295, -0.501541068, -0.126693390],
    'quaternion_xyzw': [0.0014631726632651522, -0.003424541244486294,
                        0.8435437871654295, 0.5370474939682958],
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
