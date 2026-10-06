"""Aggregate independent paper trials, stratifying incompatible experiments."""

import argparse
import json
import random
import statistics
from collections import defaultdict
from pathlib import Path


def run_statistics(values):
    values = [float(v) for v in values]
    result = {'runs': len(values), 'values': values, 'mean': statistics.mean(values),
              'median': statistics.median(values), 'bootstrap_mean_ci95': None}
    if len(values) >= 2:
        rng = random.Random(0)
        estimates = sorted(statistics.mean(rng.choices(values, k=len(values))) for _ in range(2000))
        result['bootstrap_mean_ci95'] = [estimates[49], estimates[1949]]
    return result


def build_report(directory, minimum_valid_fraction=0.95):
    groups = defaultdict(list)
    excluded = []
    for path in sorted(directory.rglob('manifest.json')):
        try:
            manifest = json.loads(path.read_text())
            if manifest.get('schema_version') != 1 or 'session_id' not in manifest:
                continue
            settings = manifest['settings']
            for mode in ('base', 'tcp'):
                try:
                    run = json.loads((path.parent / f'{mode}.json').read_text())
                    window = run['windows']['active_tracking']
                    metrics = window.get('time_weighted')
                    reasons = []
                    if manifest['status'] != 'finished' or not manifest.get('bag_metadata_available'):
                        reasons.append('session or raw bag not finalized')
                    if not manifest.get('bag_required_topics_present'):
                        reasons.append('required measurement topics missing from raw bag')
                    if not run['reached_path_end']:
                        reasons.append('path end not observed')
                    if run.get('recording_started_before_gate') is False:
                        reasons.append('recording started after the trajectory start gate')
                    if run['path_changed']:
                        reasons.append('path changed during trial')
                    if run['valid_sample_fraction'] < minimum_valid_fraction:
                        reasons.append('valid sample fraction below threshold')
                    if not metrics:
                        reasons.append('no active tracking duration')
                    if reasons:
                        excluded.append({'session_id': manifest['session_id'], 'mode': mode, 'reasons': reasons})
                        continue
                    # Gains and compensation may deliberately vary by condition.
                    # Do not silently pool different paths, speeds or sensors.
                    design = {
                        'mode': mode, 'platform': manifest['platform'], 'simulation': manifest['simulation'],
                        'frame': manifest['control_frame'], 'path_hashes': run['path_hashes'],
                        'pose_source': {key: settings.get(key) for key in (
                            'base_pose_topic', 'vicon_input_topic', 'use_odometry_robot_pose',
                            'use_base_tcp_pose_fallback', 'use_vicon_tcp_base_pose_fallback',
                            'base_tcp_pose_fallback_source')},
                        'speed': {key: settings.get(key) for key in (
                            'default_velocity', 'default_velocity_enabled', 'velocity_override')},
                        'observed_desired_speeds': run.get('observed_desired_speeds'),
                        'observed_velocity_overrides': run.get('observed_velocity_overrides'),
                        'calibration': {key: settings.get(key) for key in (
                            'vicon_nozzle_transform', 'vicon_cluster_to_base_transform',
                            'fixed_tool_offsets_by_platform', 'spray_distance_mm', 'nozzle_offset_mm')},
                        'initial_path_index': manifest.get('initial_path_index', settings.get('path_index', 0)),
                    }
                    key = json.dumps({'design': design, 'condition': manifest['condition']}, sort_keys=True)
                    groups[key].append({'session_id': manifest['session_id'], 'metrics': metrics,
                                        'active_tracking': window,
                                        'settling': run['windows']['settling'],
                                        'endpoint': run['endpoint_window']})
                except (OSError, KeyError, ValueError, TypeError) as exc:
                    excluded.append({'session_id': manifest['session_id'], 'mode': mode, 'reasons': [str(exc)]})
        except (OSError, KeyError, ValueError, TypeError) as exc:
            excluded.append({'manifest': str(path), 'reasons': [str(exc)]})
    return {
        'schema_version': 1, 'minimum_valid_fraction': minimum_valid_fraction,
        'weighting': 'time-weighted per-run active tracking; independent runs weighted equally',
        'confidence_interval': 'percentile bootstrap of run means; 2000 resamples; seed 0',
        'groups': [dict(json.loads(key), trials=trials,
                        statistics={metric: run_statistics([r['metrics'][metric] for r in trials])
                                    for metric in ('mean', 'rmse', 'p95', 'max')})
                   for key, trials in sorted(groups.items())],
        'excluded': excluded,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-directory', required=True)
    parser.add_argument('--minimum-valid-fraction', type=float, default=0.95)
    args = parser.parse_args(argv)
    if not 0 <= args.minimum_valid_fraction <= 1:
        parser.error('--minimum-valid-fraction must be in [0, 1]')
    directory = Path(args.input_directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    report = build_report(directory, args.minimum_valid_fraction)
    (directory / 'paper_accuracy_report.json').write_text(json.dumps(report, indent=2) + '\n')
    lines = ['# Paper accuracy measurements', '',
             'Active tracking metrics use time weighting within each run. Runs have equal weight.',
             'Intervals bootstrap independent trials, not individual pose samples.', '',
             '| Experiment | Condition | Mode | Runs | RMSE (mm), mean [95% CI] | P95 (mm), mean [95% CI] | Max (mm), mean [95% CI] |',
             '| --- | --- | --- | ---: | --- | --- | --- |']
    for index, group in enumerate(report['groups'], 1):
        def display(metric):
            item = group['statistics'][metric]
            ci = item['bootstrap_mean_ci95']
            interval = f'[{ci[0] * 1000:.2f}, {ci[1] * 1000:.2f}]' if ci else '[insufficient repeats]'
            return f'{item["mean"] * 1000:.2f} {interval}'
        condition = group['condition'].replace('|', '\\|').replace('\n', ' ')
        lines.append(f'| {index} | {condition} | {group["design"]["mode"]} | {len(group["trials"])} | '
                     f'{display("rmse")} | {display("p95")} | {display("max")} |')
    lines.extend(['', f'Excluded mode/trial records: {len(report["excluded"])}. See JSON for reasons.',
                  'Different paths, speeds, initial indices, frames and sensor sources are grouped separately.',
                  'Small repeat counts produce uncertain intervals; inspect all per-run values in the JSON.'])
    markdown = '\n'.join(lines) + '\n'
    (directory / 'paper_accuracy_report.md').write_text(markdown)
    print(markdown, end='')


if __name__ == '__main__':
    main()
