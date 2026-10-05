#!/usr/bin/env python3
"""Convert a rigid 4x4 matrix to the GUI's Vicon EE / marker -> nozzle fields.

The GUI expects T_marker__nozzle: p_marker = R @ p_nozzle + t (column vectors).
Use --inverse if your input is T_nozzle__marker. Translation output is metres;
quaternion output is ROS order [x, y, z, w]. Requires NumPy, but no ROS setup.

Edit TRANSFORMATION_MATRIX below and run this file, or supply --matrix with
16 row-major numbers / --file with four whitespace-separated rows. For example:

    python3 calculate_tf_quaternion_from_matrix.py --file calibration.txt --inverse
    python3 calculate_tf_quaternion_from_matrix.py --translation-unit mm

Output JSON can be merged into the GUI's advanced settings. This script only
prints values; it does not modify configuration files.
"""

import argparse
import json
from pathlib import Path

import numpy as np


# Paste the 4x4 matrix here in the format printed by the source application.
MATRIX_TEXT = """\
0.651214   -0.758768    0.013880   10.395185
0.756728    0.650627    0.063616   30.554446
-0.057301   -0.030924    0.997878  406.378555
0.000000    0.000000    0.000000    1.000000
"""
TRANSFORMATION_MATRIX = np.fromstring(MATRIX_TEXT, sep=' ').reshape((4, 4))
INVERT_MATRIX = False
TRANSLATION_UNIT = 'mm'  # Matrix translation is mm; GUI output is always metres.


def quaternion_xyzw(rotation):
    """Extract a unit quaternion, including rotations near 180 degrees."""
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            scale / 4,
        ])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = 2 * np.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        quaternion = np.zeros(4)
        quaternion[i] = scale / 4
        quaternion[j] = (rotation[j, i] + rotation[i, j]) / scale
        quaternion[k] = (rotation[k, i] + rotation[i, k]) / scale
        quaternion[3] = (rotation[k, j] - rotation[j, k]) / scale
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion *= -1
    return quaternion.tolist()


def matrix_to_gui_values(matrix, inverse=False, translation_unit='m'):
    """Validate a rigid transform and return metre XYZ and XYZW GUI values."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Matrix must be 4x4 and contain only finite numbers')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0):
        raise ValueError('Last row must be [0, 0, 0, 1] (column-vector convention)')
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
        raise ValueError('Rotation must be orthonormal with determinant +1; no scale or reflection')
    if translation_unit not in ('m', 'mm'):
        raise ValueError('Translation unit must be m or mm')
    translation = matrix[:3, 3].copy()
    if translation_unit == 'mm':
        translation /= 1000
    if inverse:
        rotation = rotation.T
        translation = -rotation @ translation
    return {'xyz': translation.tolist(), 'quaternion_xyzw': quaternion_xyzw(rotation)}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--matrix', type=float, nargs=16, metavar='N',
                        help='16 matrix entries in row-major order')
    source.add_argument('--file', type=Path, help='4x4 whitespace-separated matrix file')
    parser.add_argument('--inverse', action=argparse.BooleanOptionalAction,
                        default=INVERT_MATRIX, help='invert the complete rigid transform')
    parser.add_argument('--translation-unit', choices=('m', 'mm'), default=TRANSLATION_UNIT)
    args = parser.parse_args()
    try:
        matrix = (np.loadtxt(args.file) if args.file else
                  np.array(args.matrix).reshape(4, 4) if args.matrix else TRANSFORMATION_MATRIX)
        values = matrix_to_gui_values(matrix, args.inverse, args.translation_unit)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print('Vicon EE / marker -> nozzle: XYZ [m], quaternion [x, y, z, w]')
    print('XYZ:', values['xyz'])
    print('Quaternion XYZW:', values['quaternion_xyzw'])
    print('\nGUI advanced-settings values:')
    print(json.dumps({'vicon_nozzle_transform': values,
                      'vicon_nozzle_transform_input_mode': 'quaternion'}, indent=2))


if __name__ == '__main__':
    main()
