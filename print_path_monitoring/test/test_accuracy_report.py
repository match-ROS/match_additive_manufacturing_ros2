import json

from print_path_monitoring.accuracy_report import analyze_reach, compare_runs


def _run(phase, p95, maximum):
    return {
        'mode': 'base',
        'phase': phase,
        'absolute_error': {'rmse': p95 / 2.0, 'p95': p95, 'max': maximum},
    }


def test_tuning_requires_better_p95_without_worse_maximum():
    runs = [_run('baseline', 0.10, 0.20) for _ in range(3)]
    runs += [_run('tuned', 0.08, 0.20) for _ in range(3)]

    comparison = compare_runs(runs, 'base')

    assert comparison['accepted'] is True


def test_tuning_is_rejected_when_only_rmse_improves():
    runs = [_run('baseline', 0.10, 0.20) for _ in range(3)]
    runs += [_run('tuned', 0.10, 0.20) for _ in range(3)]
    for run in runs[3:]:
        run['absolute_error']['rmse'] = 0.01

    comparison = compare_runs(runs, 'base')

    assert comparison['accepted'] is False
    assert 'P95' in comparison['reason']


def test_reach_analysis_reports_path_outside_conservative_radius(tmp_path):
    base = {'poses': [{'position': {'x': 0.0, 'y': 0.0}}]}
    arm = {'poses': [{'position': {'x': 1.0, 'y': 0.0}}]}
    (tmp_path / 'base_path.json').write_text(json.dumps(base), encoding='utf-8')
    (tmp_path / 'arm_path.json').write_text(json.dumps(arm), encoding='utf-8')

    reach = analyze_reach(tmp_path)

    assert reach['available'] is True
    assert reach['within_conservative_range'] is False
