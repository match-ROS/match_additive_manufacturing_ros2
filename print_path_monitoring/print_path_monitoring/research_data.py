"""ROS-independent validation and statistics for reproducible measurements."""

import hashlib
import json
import math
from collections import defaultdict

from .error_metrics import summarize_distances


def path_snapshot(path):
    poses = []
    for entry in path.poses:
        p, q = entry.pose.position, entry.pose.orientation
        poses.append({'stamp_ns': entry.header.stamp.sec * 1_000_000_000 + entry.header.stamp.nanosec,
                      'frame_id': entry.header.frame_id or path.header.frame_id,
                      'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w]})
    result = {'frame_id': path.header.frame_id, 'poses': poses}
    encoded = json.dumps(result, sort_keys=True, allow_nan=False).encode()
    return result, hashlib.sha256(encoded).hexdigest()


def validate_state(state):
    if state.get('schema_version') != 1:
        raise ValueError('unsupported trajectory state schema')
    if not isinstance(state['stamp_ns'], int) or state['stamp_ns'] < 0:
        raise ValueError('invalid state timestamp')
    index, points = state['path_index'], state['path_points']
    if not isinstance(index, int) or not isinstance(points, int) or not 0 <= index < points:
        raise ValueError('invalid trajectory index')
    for key in ('start_enabled', 'path_complete'):
        if not isinstance(state[key], bool):
            raise ValueError('invalid gate')
    for key in ('segment_phase', 'velocity_override', 'desired_speed'):
        if not math.isfinite(float(state[key])):
            raise ValueError('nonfinite trajectory state')
    if not 0 <= state['segment_phase'] <= 1:
        raise ValueError('invalid segment phase')
    if state['velocity_override'] < 0 or state['path_complete'] != (index == points - 1):
        raise ValueError('invalid trajectory completion or override')
    for key in ('arm_reference', 'base_reference'):
        pose = state.get(key)
        if pose is None:
            continue
        if not pose['frame_id']:
            raise ValueError('missing reference frame')
        if not math.isfinite(float(pose['planned_stamp_sec'])):
            raise ValueError('invalid planned timestamp')
        for field, count in (('position', 3), ('orientation', 4)):
            if len(pose[field]) != count or not all(math.isfinite(float(v)) for v in pose[field]):
                raise ValueError('invalid reference pose')
        if sum(v * v for v in pose['orientation']) < 1e-12:
            raise ValueError('invalid reference quaternion')
    return state


def orientation_error(actual, reference):
    a = [actual.x, actual.y, actual.z, actual.w]
    b = [reference.x, reference.y, reference.z, reference.w]
    if not all(math.isfinite(v) for v in a + b):
        raise ValueError('nonfinite quaternion')
    denominator = math.sqrt(sum(v * v for v in a) * sum(v * v for v in b))
    if denominator < 1e-12:
        raise ValueError('invalid quaternion')
    # q and -q describe the same orientation.
    dot = abs(sum(x * y for x, y in zip(a, b)) / denominator)
    return 2.0 * math.acos(min(1.0, dot))


def summarize_window(rows, maximum_gap=0.1):
    """Keep sample, time and progress weighting explicit; never bridge pauses."""
    result = {'samples': len(rows),
              'absolute_error': summarize_distances(r['absolute_error'] for r in rows)}
    if not rows:
        return result
    weighted = []
    for previous, current in zip(rows, rows[1:]):
        dt = current['stamp_sec'] - previous['stamp_sec']
        if 0 < dt <= maximum_gap and previous.get('episode') == current.get('episode'):
            weighted.append((previous['absolute_error'], dt))
    duration = sum(dt for _, dt in weighted)
    result['observed_duration_seconds'] = duration
    if duration:
        cumulative = 0.0
        p95 = 0.0
        for error, dt in sorted(weighted):
            cumulative += dt
            p95 = error
            if cumulative >= 0.95 * duration:
                break
        result['time_weighted'] = {
            'mean': sum(error * dt for error, dt in weighted) / duration,
            'rmse': math.sqrt(sum(error ** 2 * dt for error, dt in weighted) / duration),
            'p95': p95, 'max': max(r['absolute_error'] for r in rows),
        }
    bins = defaultdict(list)
    for row in rows:
        progress = row.get('path_progress')
        if progress is not None:
            bins[min(99, int(progress * 100))].append(row['absolute_error'])
    result['progress_bins_covered'] = len(bins)
    result['progress_bin_count'] = 100
    # One statistic per bin, with equal weight per covered bin. This is a
    # progress-normalized diagnostic, not an arc-length or time percentile.
    result['progress_bin_mean_error'] = summarize_distances(
        sum(values) / len(values) for values in bins.values())
    result['orientation_error'] = summarize_distances(r['orientation_error'] for r in rows)
    result['axis_bias'] = {axis: sum(r[axis] for r in rows) / len(rows) for axis in ('dx', 'dy', 'dz')}
    for key in ('along_track_error', 'lateral_error', 'spray_axis_error', 'planar_cross_track_error'):
        result[key] = summarize_distances(abs(r[key]) for r in rows)
    return result
