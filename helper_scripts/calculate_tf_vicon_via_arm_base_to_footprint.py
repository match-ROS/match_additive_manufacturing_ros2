#!/usr/bin/env python3
"""Calculate the Vicon marker cluster -> robot base footprint transform.

Convention: T_parent__child maps child coordinates into parent coordinates:
    p_parent = T_parent__child @ p_child
Therefore:
    T_cluster__footprint = T_cluster__arm_base @ T_arm_base__footprint

Paste your calibration into MATRIX_TEXT, or use --file / --matrix. Input must
be cluster -> robot_arm_base; use --inverse-input for arm_base -> cluster.
The cluster is called Base_RB in this workspace (sometimes spelled Basde_RB).
This is the cluster frame, NOT the Vicon world frame.

The second transform is read from live robot TF, including the current lift
height. Alternatively, --arm-to-footprint-file supplies it without needing ROS.
That file must map footprint coordinates into arm-base coordinates, in metres.
Results and the printed publisher command use metres and XYZW quaternions.
Nothing is published or written to configuration automatically.

Examples (run after sourcing your ROS/workspace setup for live TF):
    python3 calculate_tf_vicon_via_arm_base_to_footprint.py --file calibration.txt \
        --translation-unit mm
    python3 calculate_tf_vicon_via_arm_base_to_footprint.py --file calibration.txt \
        --footprint-frame /robot/base_footprint
    python3 calculate_tf_vicon_via_arm_base_to_footprint.py --file calibration.txt \
        --arm-to-footprint-file arm_to_footprint.txt

Use --inverse-output for footprint -> cluster (the GUI publisher's direction).
If the lift moves relative to the cluster, the result changes with lift height;
it is only a static calibration when their relative pose stays fixed.
"""

import argparse
import json
from pathlib import Path
import shlex
import time

import numpy as np

from calculate_tf_quaternion_from_matrix import matrix_to_gui_values


# Paste four whitespace-separated rows here. Leave empty to require CLI input.
# Do not use identity as a substitute for your measured calibration.
MATRIX_TEXT = """\
0.703058    0.711121   -0.003955  376.459491
-0.710826    0.702581   -0.033261  -453.478766
-0.020874    0.026196    0.999439    6.008971
0.000000    0.000000    0.000000    1.000000
"""
TRANSLATION_UNIT = 'mm'
VICON_CLUSTER_FRAME = 'Base_RB_Base_RB'
ROBOT_ARM_BASE_FRAME = 'robot_arm_base'
ROBOT_FOOTPRINT_FRAME = 'robot_base_footprint'


def rigid_matrix(matrix, translation_unit='m', inverse=False):
    """Validate, convert translation to metres, and optionally invert."""
    values = matrix_to_gui_values(matrix, inverse=inverse,
                                  translation_unit=translation_unit)
    result = np.eye(4)
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    result[:3, :3] = rotation.T if inverse else rotation
    result[:3, 3] = values['xyz']
    return result


def compose_cluster_to_footprint(cluster_to_arm, arm_to_footprint):
    """Compose two rigid transforms expressed in metres."""
    return rigid_matrix(cluster_to_arm) @ rigid_matrix(arm_to_footprint)


def lookup_arm_to_footprint(arm_frame, footprint_frame, timeout):
    """Read T_arm__footprint: tf2 target=arm, source=footprint."""
    # Lazy imports keep the offline calculation usable with only NumPy.
    try:
        import rclpy
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise RuntimeError('Source your ROS 2 setup, or use '
                           '--arm-to-footprint-file for offline calculation') from exc

    rclpy.init(args=[])
    node = rclpy.create_node('calculate_vicon_via_arm_base_to_footprint')
    listener = None
    try:
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        deadline = time.monotonic() + timeout
        last_error = 'No TF received'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
            try:
                transform = buffer.lookup_transform(arm_frame, footprint_frame, Time())
            except TransformException as exc:
                last_error = str(exc)
                continue
            t = transform.transform.translation
            q = transform.transform.rotation
            quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
            norm = np.linalg.norm(quaternion)
            if not np.isfinite(norm) or norm < 1e-12:
                raise ValueError('TF contains an invalid quaternion')
            x, y, z, w = quaternion / norm
            matrix = np.eye(4)
            matrix[:3, :3] = [
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
            ]
            matrix[:3, 3] = [t.x, t.y, t.z]
            return rigid_matrix(matrix)
        raise RuntimeError(f'Could not look up target={arm_frame}, '
                           f'source={footprint_frame} within {timeout:g}s: {last_error}')
    finally:
        if listener is not None:
            listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--file', type=Path, help='Calibration: four matrix rows')
    source.add_argument('--matrix', nargs=16, type=float, metavar='N',
                        help='Calibration: 16 row-major matrix entries')
    parser.add_argument('--translation-unit', choices=('m', 'mm'), default=TRANSLATION_UNIT,
                        help='Calibration translation units (default: mm)')
    parser.add_argument('--inverse-input', action='store_true',
                        help='Input is arm_base -> cluster; invert before composing')
    parser.add_argument('--inverse-output', action='store_true',
                        help='Print footprint -> cluster instead of cluster -> footprint')
    parser.add_argument('--cluster-frame', default=VICON_CLUSTER_FRAME)
    parser.add_argument('--arm-base-frame', default=ROBOT_ARM_BASE_FRAME)
    parser.add_argument('--footprint-frame', default=ROBOT_FOOTPRINT_FRAME)
    parser.add_argument('--arm-to-footprint-file', type=Path,
                        help='Offline T_arm__footprint matrix; translation MUST be metres')
    parser.add_argument('--timeout', type=float, default=10.0, help='TF wait in seconds')
    args = parser.parse_args()
    try:
        if not np.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError('--timeout must be finite and positive')
        cluster, arm, footprint = (name.lstrip('/') for name in
                                   (args.cluster_frame, args.arm_base_frame, args.footprint_frame))
        if not all((cluster, arm, footprint)) or len({cluster, arm, footprint}) != 3:
            raise ValueError('Frame names must be nonempty and distinct')
        if args.file is not None:
            calibration = np.loadtxt(args.file)
        elif args.matrix is not None:
            calibration = np.array(args.matrix).reshape(4, 4)
        elif MATRIX_TEXT.strip():
            calibration = np.array([float(n) for n in MATRIX_TEXT.split()]).reshape(4, 4)
        else:
            raise ValueError('Supply your measured calibration using --file, --matrix, '
                             'or MATRIX_TEXT at the top of this script')
        cluster_to_arm = rigid_matrix(calibration, args.translation_unit, args.inverse_input)
        arm_to_footprint = (rigid_matrix(np.loadtxt(args.arm_to_footprint_file))
                            if args.arm_to_footprint_file is not None else
                            lookup_arm_to_footprint(arm, footprint, args.timeout))
        result = compose_cluster_to_footprint(cluster_to_arm, arm_to_footprint)
        parent, child = cluster, footprint
        if args.inverse_output:
            result = rigid_matrix(result, inverse=True)
            parent, child = child, parent
        values = matrix_to_gui_values(result)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))

    print(f'{parent} -> {child}: XYZ [m], quaternion [x, y, z, w]')
    print('Transformation matrix (translation in metres):')
    print(np.array2string(result, precision=9, suppress_small=True))
    print(json.dumps(values, indent=2))
    command = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher']
    for flag, value in zip(('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'),
                           values['xyz'] + values['quaternion_xyzw']):
        command.extend([flag, f'{value:.12g}'])
    command.extend(['--frame-id', parent, '--child-frame-id', child])
    print('\nStatic publisher command (only if this relative pose remains fixed):')
    print(shlex.join(command))
    print('Before publishing, check that the child frame does not already have another TF parent.')


if __name__ == '__main__':
    main()
