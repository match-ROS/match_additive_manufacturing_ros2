"""Browser-level smoke tests for the local operator web GUI.

Run explicitly after installing the test browser:
  python -m pip install -r requirements-web-test.txt
  python -m playwright install chromium
  pytest -q test/test_web_e2e.py
"""

import json
import os
import re
import socket
import subprocess
import time
from pathlib import Path

import pytest

try:
    from playwright import sync_api
except ImportError:
    sync_api = None


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
START_SCRIPT = PACKAGE_ROOT / 'scripts' / 'start_web_gui.sh'


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return int(sock.getsockname()[1])


def wait_for_server(port: int, process: subprocess.Popen[str]) -> None:
    import urllib.request

    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            output = process.stdout.read() if process.stdout else ''
            raise RuntimeError(f'web GUI stopped during startup:\n{output}')
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state', timeout=0.5):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError('web GUI did not become reachable within 20 seconds')


def test_start_script_renders_and_saves_settings(tmp_path: Path) -> None:
    if sync_api is None:
        pytest.skip('Playwright is not installed; use requirements-web-test.txt for browser tests')
    port = free_port()
    config_path = tmp_path / 'operator.json'
    environment = os.environ | {
        'AM_OPERATOR_WEB_PORT': str(port),
        'AM_OPERATOR_WEB_NO_BROWSER': '1',
        'AM_OPERATOR_GUI_CONFIG': str(config_path),
        'ROS_LOG_DIR': str(tmp_path / 'ros_logs'),
    }
    process = subprocess.Popen(
        [str(START_SCRIPT)],
        cwd=START_SCRIPT.parent,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        wait_for_server(port, process)
        with sync_api.sync_playwright() as playwright:
            browser = playwright.chromium.launch()
            page = browser.new_page(viewport={'width': 1440, 'height': 1000})
            console_snapshot = {
                'config': {}, 'status': {'path': False, 'robot_pose': False, 'arm_pose': False, 'jparse_ready': False, 'controller_ready': False},
                'actions': {}, 'ros_error': None,
                'logs': [
                    {'source': 'ros', 'message': '[ERROR] Example error', 'level': 'error', 'timestamp': '2026-01-01T12:00:00+00:00'},
                    {'source': 'ros', 'message': '[WARN] Example warning', 'level': 'warning', 'timestamp': '2026-01-01T12:00:01+00:00'},
                    {'source': 'path_index', 'message': '[INFO] Example information', 'level': 'info', 'timestamp': '2026-01-01T12:00:02+00:00'},
                ],
            }
            page.route('**/api/state', lambda route: route.fulfill(json=console_snapshot))
            # The page polls /api/state once per second, so networkidle is not
            # a meaningful readiness signal for this live operator console.
            page.goto(f'http://127.0.0.1:{port}', wait_until='domcontentloaded')
            expect = sync_api.expect
            expect(page.get_by_role('heading', name='AM Operator')).to_be_visible()
            expect(page.get_by_alt_text('MATCH')).to_be_visible()
            expect(page.get_by_alt_text('Additive Manufacturing Center Aachen')).to_be_visible()
            assert page.get_by_alt_text('MATCH').evaluate('(image) => image.naturalWidth') > 0
            assert page.get_by_alt_text('Additive Manufacturing Center Aachen').evaluate('(image) => image.naturalWidth') > 0
            assert 32 <= page.get_by_alt_text('MATCH').bounding_box()['width'] <= 48
            expect(page.locator('#log')).to_contain_text('Example error')
            page.locator('[data-console-level="error"]').uncheck()
            expect(page.locator('#log')).to_contain_text('Example warning')
            expect(page.locator('#log')).not_to_contain_text('Example error')
            page.locator('[data-console-level="info"]').uncheck()
            expect(page.locator('#log')).not_to_contain_text('Example information')
            page.locator('#console-source').select_option('path_index')
            expect(page.locator('#log')).to_have_text('')
            page.locator('[data-console-level="info"]').check()
            expect(page.locator('#log')).to_contain_text('Example information')
            page.unroute('**/api/state')
            page.reload(wait_until='domcontentloaded')
            expect(page.get_by_role('button', name='Launch All')).to_be_visible()
            expect(page.get_by_role('button', name='Launch All')).to_have_class(re.compile(r'.*action-state-idle.*'))
            expect(page.get_by_role('button', name='Stop All')).to_have_class(re.compile(r'.*action-state-danger.*'))
            expect(page.get_by_role('button', name='Start Following')).to_have_class(re.compile(r'.*action-state-warning.*'))
            expect(page.locator('[data-setting="velocity_override"]')).to_have_count(1)
            page.locator('[data-setting="path_index"]').fill('12')
            page.locator('[data-setting="path_index"]').press('Tab')
            page.reload(wait_until='networkidle')
            expect(page.locator('[data-setting="path_index"]')).to_have_value('12')
            assert json.loads(config_path.read_text(encoding='utf-8')) == {'path_index': 12}
            page.set_viewport_size({'width': 1024, 'height': 768})
            expect(page.get_by_alt_text('MATCH')).to_be_visible()
            assert 32 <= page.get_by_alt_text('MATCH').bounding_box()['width'] <= 48
            assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
            page.set_viewport_size({'width': 390, 'height': 844})
            expect(page.get_by_role('button', name='Start Following')).to_be_visible()
            assert 28 <= page.get_by_alt_text('MATCH').bounding_box()['width'] <= 36
            assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth')
            browser.close()
    finally:
        process.terminate()
        process.wait(timeout=10)
