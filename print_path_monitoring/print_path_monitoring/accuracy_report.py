#!/usr/bin/env python3
"""Aggregate trajectory-accuracy runs and decide whether a tuning improved them."""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any


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


def compare_runs(runs: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    selected = [run for run in runs if run.get('mode') == mode]
    baseline = [run for run in selected if run.get('phase') == 'baseline']
    tuned = [run for run in selected if run.get('phase') == 'tuned']
    result: dict[str, Any] = {
        'mode': mode,
        'baseline_runs': len(baseline),
        'tuned_runs': len(tuned),
        'baseline': {metric: _median_metric(baseline, metric) for metric in ('rmse', 'p95', 'max')},
        'tuned': {metric: _median_metric(tuned, metric) for metric in ('rmse', 'p95', 'max')},
        'accepted': False,
        'reason': 'Need three baseline and three tuned runs.',
    }
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


def analyze_reach(trajectory_directory: Path, minimum: float = 0.25, maximum: float = 0.85) -> dict[str, Any]:
    """Assess the conservative planar base-to-TCP reach of exported paired paths."""
    try:
        base = json.loads((trajectory_directory / 'base_path.json').read_text(encoding='utf-8'))['poses']
        arm = json.loads((trajectory_directory / 'arm_path.json').read_text(encoding='utf-8'))['poses']
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        return {'available': False, 'reason': 'Paired exported base_path.json and arm_path.json are required.'}
    if len(base) != len(arm) or not base:
        return {'available': False, 'reason': 'Paired paths have different or empty pose counts.'}
    try:
        radii = [
            math.hypot(
                float(arm_pose['position']['x']) - float(base_pose['position']['x']),
                float(arm_pose['position']['y']) - float(base_pose['position']['y']),
            )
            for base_pose, arm_pose in zip(base, arm)
        ]
    except (KeyError, TypeError, ValueError):
        return {'available': False, 'reason': 'Paired paths contain invalid positions.'}
    return {
        'available': True,
        'minimum_radius': min(radii),
        'maximum_radius': max(radii),
        'allowed_radius_range': [minimum, maximum],
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
    lines.extend(['', '## Planar reach'])
    if reach.get('available'):
        lines.append(
            f"Range: {reach['minimum_radius']:.3f}..{reach['maximum_radius']:.3f} m; "
            f"allowed: {reach['allowed_radius_range'][0]:.3f}..{reach['allowed_radius_range'][1]:.3f} m. "
            f"{reach['reason']}"
        )
    else:
        lines.append(f"Not evaluated: {reach['reason']}")
    return '\n'.join(lines) + '\n'


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-directory', default='/tmp/am_trajectory_runs')
    parser.add_argument('--output-directory', default='')
    parser.add_argument('--trajectory-directory', default='')
    args = parser.parse_args(argv)
    directory = Path(args.input_directory).expanduser()
    output_directory = Path(args.output_directory).expanduser() if args.output_directory else directory
    output_directory.mkdir(parents=True, exist_ok=True)
    runs = load_runs(directory)
    comparisons = [compare_runs(runs, mode) for mode in ('base', 'tcp')]
    reach = analyze_reach(Path(args.trajectory_directory).expanduser()) if args.trajectory_directory else {
        'available': False, 'reason': 'No trajectory directory supplied.'
    }
    (output_directory / 'accuracy_comparison.json').write_text(
        json.dumps({'comparisons': comparisons, 'reach': reach}, indent=2) + '\n', encoding='utf-8')
    report = markdown_report(comparisons, reach)
    (output_directory / 'accuracy_comparison.md').write_text(report, encoding='utf-8')
    print(report, end='')


if __name__ == '__main__':
    main()
