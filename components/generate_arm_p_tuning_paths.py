#!/usr/bin/env python3
"""Generate small, closed arm tests around a pose from David's print path."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path


def unit(v):
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-9:
        raise ValueError('Zero axis in source path')
    return [x / n for x in v]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def generate(source: Path, output: Path, anchor_index: int = 0):
    arm = json.loads((source / 'arm_path.json').read_text())
    base = json.loads((source / 'base_path.json').read_text())
    if len(arm['poses']) != len(base['poses']):
        raise ValueError('Source paths must have matching pose counts')
    anchor = arm['poses'][anchor_index]
    base_anchor = base['poses'][anchor_index]
    frame = arm['frame_id']
    q = unit([anchor['orientation'][key] for key in 'xyzw'])
    x, y, z, w = q
    spray = [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]
    delta = [arm['poses'][anchor_index+1]['position'][key] - anchor['position'][key]
             for key in 'xyz']
    tangent = unit([a - sum(b*c for b, c in zip(delta, spray))*s
                    for a, s in zip(delta, spray)])
    lateral = [spray[1]*tangent[2]-spray[2]*tangent[1],
               spray[2]*tangent[0]-spray[0]*tangent[2],
               spray[0]*tangent[1]-spray[1]*tangent[0]]

    def pose(along=0., across=0., depth=0., rotation_axis=None, degrees=0.):
        result = deepcopy(anchor)
        result['position'] = {key: anchor['position'][key] + along*t + across*l + depth*s
                              for key, t, l, s in zip('xyz', tangent, lateral, spray)}
        quat = q
        if rotation_axis is not None:
            half = math.radians(degrees) / 2
            quat = multiply([*(a*math.sin(half) for a in rotation_axis), math.cos(half)], q)
        result['orientation'] = dict(zip('xyzw', quat))
        return result

    tests = {
        '01_along_track': ('along_track_kp', '30 mm reversals along the initial print tangent',
                           [(pose(), 0), (pose(), 3)] +
                           [(p, dt) for _ in range(2) for p, dt in
                            [(pose(along=.03), 3), (pose(along=.03), 3),
                             (pose(along=-.03), 6), (pose(along=-.03), 3), (pose(), 3), (pose(), 3)]]),
        '02_cross_track_corners': ('orthogonal_kp', '20 mm square with corners; cross-track disturbance recovery',
                                  [(pose(), 0), (pose(), 3), (pose(along=.02), 2),
                                   (pose(along=.02, across=.02), 2), (pose(across=.02), 2),
                                   (pose(), 2), (pose(), 3), (pose(across=-.02), 2),
                                   (pose(along=-.02, across=-.02), 2), (pose(along=-.02), 2),
                                   (pose(), 2), (pose(), 4)]),
        '03_spray_axis': ('kp_z', '10 mm spray-axis reversals at 5 mm/s',
                          [(pose(), 0), (pose(), 3), (pose(depth=.01), 2), (pose(depth=.01), 4),
                           (pose(depth=-.01), 4), (pose(depth=-.01), 4), (pose(), 2), (pose(), 4)]),
        '04_orientation': ('kp_orientation', '3 degree rotations about tangent, lateral and spray axes',
                           [(pose(), 0), (pose(), 3)] +
                           [(p, dt) for axis in (tangent, lateral, spray) for p, dt in
                            [(pose(rotation_axis=axis, degrees=3), 1.5),
                             (pose(rotation_axis=axis, degrees=3), 4),
                             (pose(rotation_axis=axis, degrees=-3), 3),
                             (pose(rotation_axis=axis, degrees=-3), 4), (pose(), 1.5), (pose(), 4)]]),
    }
    excerpt = [deepcopy(p) for p in arm['poses'][anchor_index:anchor_index+4]]
    excerpt[0] = pose()
    traversal = excerpt[1:] + list(reversed(excerpt[:-1]))
    previous = anchor
    timed = [(pose(), 0), (pose(), 3)]
    for p in traversal:
        length = math.dist([previous['position'][k] for k in 'xyz'], [p['position'][k] for k in 'xyz'])
        timed.append((p, max(2., length / .01)))
        previous = p
    timed.append((pose(), 5))
    tests['05_david_excerpt'] = ('validation', 'First four print poses, out and back at 10 mm/s; base fixed', timed)
    for name, (_gain, _description, timed_poses) in tests.items():
        folder = output / name
        folder.mkdir(parents=True, exist_ok=True)
        arm_poses, base_poses, seconds = [], [], 0.
        for p, duration in timed_poses:
            seconds += duration
            sec, ns = divmod(round(seconds*1e9), 1_000_000_000)
            stamp = {'sec': sec, 'nanosec': ns}
            p = deepcopy(p)
            b = deepcopy(base_anchor)
            for value in (p, b):
                value['stamp'] = stamp
                value['frame_id'] = frame
            arm_poses.append(p)
            base_poses.append(b)
        artifacts = {
            'arm_path.json': {'frame_id': frame, 'poses': arm_poses},
            'base_path.json': {'frame_id': frame, 'poses': base_poses},
            'normal_vector.json': dict(zip('xyz', spray)),
        }
        for filename, data in artifacts.items():
            (folder / filename).write_text(json.dumps(data, indent=2) + '\n')
    return list(tests)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parent
    parser.add_argument('--source', type=Path, default=root / 'david_path')
    parser.add_argument('--output', type=Path, default=root / 'arm_p_tuning')
    parser.add_argument('--anchor-index', type=int, default=0)
    args = parser.parse_args()
    if args.anchor_index < 0:
        parser.error('--anchor-index must be nonnegative')
    for name in generate(args.source, args.output, args.anchor_index):
        print(name)
