"""Every GUI button must have a registered HTTP action route."""
import re
import asyncio
from types import SimpleNamespace

from am_operator_gui.web_app import app, WEB_ROOT


def test_all_template_actions_have_post_routes():
    actions = set(re.findall(r'data-action="([^"]+)"',
                             (WEB_ROOT / 'templates' / 'index.html').read_text()))
    routes = {route.path for route in app.routes
              if 'POST' in getattr(route, 'methods', set())}
    assert {f'/api/actions/{action}' for action in actions} <= routes


def test_index_pose_preview_post_reaches_service(monkeypatch):
    calls = []
    monkeypatch.setattr(app.state, 'operator', SimpleNamespace(
        action=calls.append, snapshot=lambda: {'ok': True}), raising=False)
    messages = []

    async def receive():
        return {'type': 'http.request', 'body': b'', 'more_body': False}

    async def send(message):
        messages.append(message)

    scope = {'type': 'http', 'asgi': {'version': '3.0'}, 'http_version': '1.1',
             'method': 'POST', 'scheme': 'http', 'path': '/api/actions/index_pose_preview',
             'raw_path': b'/api/actions/index_pose_preview', 'query_string': b'',
             'root_path': '', 'headers': [], 'server': ('test', 80), 'client': ('test', 1)}
    asyncio.run(app(scope, receive, send))
    assert next(m['status'] for m in messages if m['type'] == 'http.response.start') == 200
    assert calls == ['index_pose_preview']
