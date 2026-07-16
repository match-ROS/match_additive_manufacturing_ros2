#!/usr/bin/env python3
"""Aggregate trajectory-accuracy runs and decide whether a tuning improved them."""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any

from print_path_monitoring.error_metrics import yaw_from_quaternion


def load_runs(directory: Path) -> list[dict[str, Any]]:
    runs = []
    for path in sorted(directory.glob('*.json')):
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(data, dict) and 'absolute_error' in data and 'mode' in data:
            data['_path'] = str(path)
            runs.append(data)
    return runs


def _median_metric(runs: list[dict[str, Any]], metric: str) -> float | None:
    values = [float(run.get('absolute_error', {}).get(metric)) for run in runs
              if run.get('absolute_error', {}).get(metric) is not None]
    return float(statistics.median(values)) if values else None


def _median_reach_metric(runs: list[dict[str, Any]], reach_class: str, metric: str) -> float | None:
    values = [
        float(run['reachability']['classes'][reach_class]['absolute_error'][metric])
        for run in runs
        if run.get('reachability', {}).get('classes', {}).get(reach_class, {})
        .get('absolute_error', {}).get(metric) is not None
    ]
    return float(statistics.median(values)) if values else None


def _reachability_comparison(
    baseline: list[dict[str, Any]], tuned: list[dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Keep accuracy statistics separate by planned arm-base reachability."""
    result: dict[str, dict[str, Any]] = {}
    for reach_class in ('well_reachable', 'poor_reachability', 'unreachable_planar', 'not_evaluated'):
        baseline_runs = [
            run for run in baseline
            if reach_class in run.get('reachability', {}).get('classes', {})
        ]
        tuned_runs = [
            run for run in tuned
            if reach_class in run.get('reachability', {}).get('classes', {})
        ]
        result[reach_class] = {
            'baseline_runs': len(baseline_runs),
            'tuned_runs': len(tuned_runs),
            'baseline': {metric: _median_reach_metric(baseline_runs, reach_class, metric)
                         for metric in ('rmse', 'p95', 'max')},
            'tuned': {metric: _median_reach_metric(tuned_runs, reach_class, metric)
                      for metric in ('rmse', 'p95', 'max')},
        }
    return result


def compare_runs(runs: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    candidates = [run for run in runs if run.get('mode') == mode]
    # A run that never crossed its start gate has no tracking measurement. Keep
    # its JSON as an audit record, but never let it satisfy the three-run rule.
    selected = [
        run for run in candidates
        if ('samples' not in run or int(run['samples']) > 0)
        and run.get('absolute_error', {}).get('p95') is not None
    ]
    baseline = [run for run in selected if run.get('phase') == 'baseline']
    tuned = [run for run in selected if run.get('phase') == 'tuned']
    result: dict[str, Any] = {
        'mode': mode,
        'excluded_invalid_runs': len(candidates) - len(selected),
        'baseline_runs': len(baseline),
        'tuned_runs': len(tuned),
        'baseline': {metric: _median_metric(baseline, metric) for metric in ('rmse', 'p95', 'max')},
        'tuned': {metric: _median_metric(tuned, metric) for metric in ('rmse', 'p95', 'max')},
        'accepted': False,
        'reason': 'Need three baseline and three tuned runs.',
    }
    if mode == 'tcp':
        result['reachability'] = _reachability_comparison(baseline, tuned)
    if len(baseline) < 3 or len(tuned) < 3:
        return result
    before, after = result['baseline'], result['tuned']
    if after['p95'] < before['p95'] and after['max'] <= before['max']:
        result['accepted'] = True
        result['reason'] = 'Median P95 improved without worsening median maximum error.'
    elif after['p95'] >= before['p95']:
        result['reason'] = 'Median P95 did not improve.'
    else:
        result['reason'] = 'Median maximum error worsened.'
    return result


def analyze_reach(
    trajectory_directory: Path,
    minimum: float = 0.25,
    maximum: float = 0.85,
    arm_base_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    boundary_margin: float = 0.05,
) -> dict[str, Any]:
    """Classify paired path points by planar TCP reach from the actual arm base."""
    try:
        base = json.loads((trajectory_directory / 'base_path.json').read_text(encoding='utf-8'))['poses']
        arm = json.loads((trajectory_directory / 'arm_path.json').read_text(encoding='utf-8'))['poses']
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        return {'available': False, 'reason': 'Paired exported base_path.json and arm_path.json are required.'}
    if len(base) != len(arm) or not base:
        return {'available': False, 'reason': 'Paired paths have different or empty pose counts.'}
    try:
        ox, oy, oz = arm_base_offset
        radii = []
        classes = {'well_reachable': 0, 'poor_reachability': 0, 'unreachable_planar': 0}
        for base_pose, arm_pose in zip(base, arm):
            base_position = base_pose['position']
            orientation = base_pose['orientation']
            yaw = yaw_from_quaternion(
                float(orientation['x']), float(orientation['y']),
                float(orientation['z']), float(orientation['w']),
            )
            arm_base_x = float(base_position['x']) + math.cos(yaw) * ox - math.sin(yaw) * oy
            arm_base_y = float(base_position['y']) + math.sin(yaw) * ox + math.cos(yaw) * oy
            # The vertical offset is retained in the report configuration even
            # though the conservative classifier intentionally uses XY reach.
            _ = float(base_position.get('z', 0.0)) + oz
            radius = math.hypot(
                float(arm_pose['position']['x']) - arm_base_x,
                float(arm_pose['position']['y']) - arm_base_y,
            )
            radii.append(radius)
            if radius < minimum or radius > maximum:
                classes['unreachable_planar'] += 1
            elif radius <= minimum + boundary_margin or radius >= maximum - boundary_margin:
                classes['poor_reachability'] += 1
            else:
                classes['well_reachable'] += 1
    except (KeyError, TypeError, ValueError):
        return {'available': False, 'reason': 'Paired paths contain invalid poses or arm-base offset.'}
    return {
        'available': True,
        'minimum_radius': min(radii),
        'maximum_radius': max(radii),
        'allowed_radius_range': [minimum, maximum],
        'boundary_margin': boundary_margin,
        'arm_base_offset': list(arm_base_offset),
        'reference': 'TCP position relative to planned arm base (mobile base pose plus rotated arm_base_offset).',
        'classification_counts': classes,
        'within_conservative_range': min(radii) >= minimum and max(radii) <= maximum,
        'reason': ('Within conservative planar reach.' if min(radii) >= minimum and max(radii) <= maximum
                   else 'Outside conservative planar reach; this can cause TCP tracking errors.'),
    }


def markdown_report(comparisons: list[dict[str, Any]], reach: dict[str, Any]) -> str:
    lines = [
        '# Trajectory Accuracy Report', '',
        '| Mode | Baseline RMSE/P95/Max (m) | Tuned RMSE/P95/Max (m) | Result |',
        '| --- | --- | --- | --- |',
    ]
    for item in comparisons:
        def values(section: str) -> str:
            metrics = item[section]
            return ' / '.join('-' if metrics[key] is None else f'{metrics[key]:.5f}'
                              for key in ('rmse', 'p95', 'max'))
        result = 'ACCEPTED' if item['accepted'] else f"REJECTED: {item['reason']}"
        lines.append(f"| {item['mode']} | {values('baseline')} | {values('tuned')} | {result} |")
    tcp = next((item for item in comparisons if item['mode'] == 'tcp'), None)
    if tcp and 'reachability' in tcp:
        lines.extend([
            '', '## TCP accuracy by planned reachability', '',
            '| Reachability class | Baseline runs | Baseline RMSE/P95/Max (m) | Tuned runs | Tuned RMSE/P95/Max (m) |',
            '| --- | ---: | --- | ---: | --- |',
        ])
        for reach_class, item in tcp['reachability'].items():
            def reach_values(section: str) -> str:
                return ' / '.join(
                    '-' if item[section][key] is None else f"{item[section][key]:.5f}"
                    for key in ('rmse', 'p95', 'max')
                )
            lines.append(
                f"| {reach_class} | {item['baseline_runs']} | {reach_values('baseline')} | "
                f"{item['tuned_runs']} | {reach_values('tuned')} |"
            )
    lines.extend(['', '## Planar reach'])
    if reach.get('available'):
        lines.append(
            f"Range: {reach['minimum_radius']:.3f}..{reach['maximum_radius']:.3f} m; "
            f"allowed: {reach['allowed_radius_range'][0]:.3f}..{reach['allowed_radius_range'][1]:.3f} m. "
            f"{reach['reason']}\n\n"
            f"Classification (planned arm base): {reach['classification_counts']}."
        )
    else:
        lines.append(f"Not evaluated: {reach['reason']}")
    return '\n'.join(lines) + '\n'


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-directory', default='/tmp/am_trajectory_runs')
    parser.add_argument('--output-directory', default='')
    parser.add_argument('--trajectory-directory', default='')
    parser.add_argument('--arm-base-offset', default='0,0,0',
                        help='Arm-base mount offset x,y,z in mobile-base coordinates (m).')
    parser.add_argument('--min-reachable-radius', type=float, default=0.25)
    parser.add_argument('--max-reachable-radius', type=float, default=0.85)
    parser.add_argument('--reach-boundary-margin', type=float, default=0.05)
    args = parser.parse_args(argv)
    directory = Path(args.input_directory).expanduser()
    output_directory = Path(args.output_directory).expanduser() if args.output_directory else directory
    output_directory.mkdir(parents=True, exist_ok=True)
    runs = load_runs(directory)
    comparisons = [compare_runs(runs, mode) for mode in ('base', 'tcp')]
    try:
        arm_base_offset = tuple(float(value.strip()) for value in args.arm_base_offset.split(','))
        if len(arm_base_offset) != 3:
            raise ValueError
    except ValueError:
        parser.error('--arm-base-offset must be three comma-separated numbers, e.g. 0.26,0,1.046')
    reach = analyze_reach(
        Path(args.trajectory_directory).expanduser(),
        args.min_reachable_radius,
        args.max_reachable_radius,
        arm_base_offset,
        args.reach_boundary_margin,
    ) if args.trajectory_directory else {
        'available': False, 'reason': 'No trajectory directory supplied.'
    }
    (output_directory / 'accuracy_comparison.json').write_text(
        json.dumps({'comparisons': comparisons, 'reach': reach}, indent=2) + '\n', encoding='utf-8')
    report = markdown_report(comparisons, reach)
    (output_directory / 'accuracy_comparison.md').write_text(report, encoding='utf-8')
    print(report, end='')


if __name__ == '__main__':
    main()
