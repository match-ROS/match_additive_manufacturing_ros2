"""Exercise browser alarm behavior without connecting to or commanding a robot.

Install requirements-web-test.txt to include the QuickJS audio/DOM test runtime.
"""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from am_operator_gui.ur_dashboard import UrStatusMonitor


ALARM_SCRIPT = Path(__file__).parents[1] / 'am_operator_gui/web/static/protective-stop-alarm.js'


def browser_runtime():
    quickjs = pytest.importorskip('quickjs')
    context = quickjs.Context()
    context.eval('''
      const elements = {};
      const storage = {};
      const localStorage = {
        getItem: key => storage[key] ?? null,
        setItem: (key, value) => { storage[key] = String(value); },
      };
      const document = {querySelector: id => elements[id] ??= {
        value: '60', textContent: '', disabled: false, listeners: {},
        classList: {toggle() {}},
        addEventListener(event, callback) { this.listeners[event] = callback; },
      }};
      let blockAudio = false;
      class AudioContext {
        constructor() { this.state = 'suspended'; this.sampleRate = 4000; this.currentTime = 0; this.sources = []; }
        resume() {
          if (blockAudio) return Promise.reject(new Error('blocked'));
          this.state = 'running';
          return Promise.resolve();
        }
        createGain() { return {connect() {}, gain: {
          value: 0, setTargetAtTime(value) { this.value = value; },
        }}; }
        createBuffer(channels, count, rate) {
          const samples = new Float32Array(count);
          return {getChannelData: () => samples, duration: count / rate};
        }
        createBufferSource() {
          const source = {connect() {}, disconnect() {},
            start() { this.started = true; }, stop() { this.stopped = true; }};
          this.sources.push(source);
          return source;
        }
      }
      const window = {AudioContext};
    ''')
    context.eval(ALARM_SCRIPT.read_text())
    context.eval('const alarm = new ProtectiveStopAlarm();')
    return context


def run(context, code):
    result = context.eval(code)
    while context.execute_pending_job():
        pass
    return result


def update(context, mode, **extra):
    dashboard = {'available': True, 'stale': False, 'safety_mode': mode, **extra}
    run(context, f'alarm.update({json.dumps(dashboard)})')


def test_enable_during_stop_and_repeated_updates_use_one_loop():
    browser = browser_runtime()
    update(browser, 'PROTECTIVE_STOP')
    assert run(browser, 'alarm.source === null')
    run(browser, 'alarm.enable()')
    assert run(browser, 'alarm.source.started && alarm.source.loop')
    for _ in range(5):
        update(browser, 'PROTECTIVE_STOP')
    assert run(browser, 'alarm.context.sources.length') == 1
    update(browser, 'NORMAL')
    assert run(browser, 'alarm.source === null && alarm.context.sources[0].stopped')


def test_silence_is_local_and_next_stop_rearms():
    pc, notebook = browser_runtime(), browser_runtime()
    for browser in (pc, notebook):
        run(browser, 'alarm.enable()')
        update(browser, 'PROTECTIVE_STOP')
    run(pc, "elements['#alarm-silence'].listeners.click()")
    update(pc, 'PROTECTIVE_STOP')
    assert run(pc, 'alarm.source === null && alarm.silenced')
    assert run(notebook, 'alarm.source.started && !alarm.source.stopped')
    update(pc, 'NORMAL')
    update(pc, 'PROTECTIVE_STOP')
    assert run(pc, 'alarm.source.started && !alarm.silenced')


@pytest.mark.parametrize('unknown', [None, {},
    {'available': False, 'stale': True, 'safety_mode': 'NORMAL'},
    {'available': True, 'safety_mode_available': False, 'safety_mode': 'NORMAL'}])
def test_missing_or_stale_status_preserves_confirmed_stop(unknown):
    browser = browser_runtime()
    run(browser, 'alarm.enable()')
    update(browser, 'PROTECTIVE_STOP')
    run(browser, f'alarm.update({json.dumps(unknown)})')
    assert run(browser, 'alarm.active && alarm.source.started && !alarm.source.stopped')
    assert not run(browser, 'alarm.statusKnown')


@pytest.mark.parametrize('mode', ['NORMAL', 'REDUCED', 'SAFEGUARD_STOP', 'ROBOT_EMERGENCY_STOP'])
def test_other_modes_do_not_trigger_protective_stop_alarm(mode):
    browser = browser_runtime()
    run(browser, 'alarm.enable()')
    update(browser, mode)
    assert run(browser, 'alarm.source === null && !alarm.active')


def test_live_safety_topic_works_when_dashboard_services_are_unavailable():
    browser = browser_runtime()
    run(browser, 'alarm.enable()')
    update(browser, 'PROTECTIVE_STOP', available=False, stale=True, safety_mode_available=True)
    assert run(browser, 'alarm.source.started && alarm.statusKnown')


def test_test_button_plays_one_cycle_and_volume_changes_audio_gain():
    browser = browser_runtime()
    run(browser, "elements['#alarm-test'].listeners.click()")
    assert run(browser, 'alarm.enabled && alarm.testSource.started && !alarm.testSource.loop')
    assert run(browser, 'alarm.source === null && !alarm.active')
    assert run(browser, 'alarm.buffer.duration') == pytest.approx(2.8)
    assert run(browser, 'alarm.buffer.getChannelData(0).some(value => Math.abs(value) > 0.1)')
    assert run(browser, 'alarm.buffer.getChannelData(0).every(value => Number.isFinite(value) && Math.abs(value) <= 1)')
    assert run(browser, 'alarm.buffer.getChannelData(0).slice(8400).every(value => value === 0)')
    run(browser, "elements['#alarm-volume'].value = '25'; elements['#alarm-volume'].listeners.input()")
    assert run(browser, 'alarm.gain.gain.value') == pytest.approx(0.15)
    assert run(browser, "storage['am-operator-alarm-volume']") == '25'
    run(browser, 'alarm.testSource.onended()')
    assert run(browser, 'alarm.testSource === null')


def test_disable_stops_audio_and_reenable_resumes_current_stop():
    browser = browser_runtime()
    run(browser, 'alarm.enable()')
    update(browser, 'PROTECTIVE_STOP')
    run(browser, "elements['#alarm-enable'].listeners.click()")
    assert run(browser, '!alarm.enabled && alarm.source === null && alarm.active')
    run(browser, "elements['#alarm-enable'].listeners.click()")
    assert run(browser, 'alarm.enabled && alarm.source.started')


def test_blocked_audio_can_be_retried_and_does_not_clear_stop():
    browser = browser_runtime()
    update(browser, 'PROTECTIVE_STOP')
    run(browser, 'blockAudio = true; alarm.enable()')
    assert run(browser, '!alarm.enabled && alarm.source === null && alarm.active')
    run(browser, 'blockAudio = false; alarm.enable()')
    assert run(browser, 'alarm.enabled && alarm.source.started && !alarm.error')


def test_safety_topic_overrides_older_dashboard_query_even_if_it_completes_later(monkeypatch):
    monitor = UrStatusMonitor()
    monitor._dashboard_in_flight = True
    monitor._dashboard_index = 4
    monitor._dashboard_queries = (None,) * 5
    monitor._dashboard_values = {'safety_mode': 1, 'safety_mode_checked_at': 100.0}
    monkeypatch.setattr('am_operator_gui.ur_dashboard.time.time', lambda: 110.0)
    monitor._safety_mode_callback(SimpleNamespace(mode=3))
    monitor._dashboard_response(SimpleNamespace(result=lambda: 7), 'robot_mode', lambda value: value)
    snapshot = monitor.dashboard_snapshot()
    assert snapshot['safety_mode'] == 'PROTECTIVE_STOP'
    assert snapshot['safety_mode_available']
    assert snapshot['safety_mode_source'] == 'topic'
    monitor._set_dashboard_error('unavailable')
    assert monitor.dashboard_snapshot()['safety_mode_available']


def test_lost_topic_cannot_clear_stop_using_old_dashboard_cache(monkeypatch):
    monitor = UrStatusMonitor()
    monitor._dashboard.update(available=True, safety_mode='NORMAL', safety_mode_checked_at=100.0)
    monkeypatch.setattr('am_operator_gui.ur_dashboard.time.time', lambda: 110.0)
    monitor._safety_mode_callback(SimpleNamespace(mode=3))
    monitor._safety_subscription = SimpleNamespace(get_publisher_count=lambda: 0)
    monitor._next_dashboard = monitor._next_controller = float('inf')
    monitor._tick()
    snapshot = monitor.dashboard_snapshot()
    assert snapshot['safety_mode'] == 'PROTECTIVE_STOP'
    assert not snapshot['safety_mode_available']
    monitor._dashboard.update(safety_mode='NORMAL', safety_mode_checked_at=120.0)
    snapshot = monitor.dashboard_snapshot()
    assert snapshot['safety_mode'] == 'NORMAL'
    assert snapshot['safety_mode_available']
    assert snapshot['safety_mode_source'] == 'dashboard'


def test_fresh_dashboard_poll_can_recover_a_missed_topic_change(monkeypatch):
    monitor = UrStatusMonitor()
    monkeypatch.setattr('am_operator_gui.ur_dashboard.time.time', lambda: 100.0)
    monitor._safety_mode_callback(SimpleNamespace(mode=1))
    monitor._dashboard.update(available=True, safety_mode='PROTECTIVE_STOP', safety_mode_checked_at=110.0)
    snapshot = monitor.dashboard_snapshot()
    assert snapshot['safety_mode'] == 'PROTECTIVE_STOP'
    assert snapshot['safety_mode_available']
    assert snapshot['safety_mode_source'] == 'dashboard'


def test_delayed_safety_response_retains_its_request_time(monkeypatch):
    monitor = UrStatusMonitor()
    monkeypatch.setattr('am_operator_gui.ur_dashboard.time.time', lambda: 110.0)
    monitor._safety_mode_callback(SimpleNamespace(mode=3))
    monitor._dashboard_in_flight = True
    monitor._dashboard_queries = (None,)
    monitor._dashboard_values = {'robot_mode': 7}
    monkeypatch.setattr('am_operator_gui.ur_dashboard.time.time', lambda: 120.0)
    monitor._dashboard_response(
        SimpleNamespace(result=lambda: 1), 'safety_mode', lambda value: value, queried_at=100.0,
    )
    assert monitor.dashboard_snapshot()['safety_mode'] == 'PROTECTIVE_STOP'
