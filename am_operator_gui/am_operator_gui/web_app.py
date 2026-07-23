"""Local FastAPI application for the AM operator interface."""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel

from .operator_service import OperatorService

WEB_ROOT = Path(__file__).parent / 'web'


def _image_root() -> Path:
    """Find branding assets both from a source checkout and ROS install space."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('am_operator_gui')) / 'img'
        if installed.is_dir():
            return installed
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / 'img'


IMAGE_ROOT = _image_root()


class SettingsPayload(BaseModel):
    values: dict[str, Any]


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.operator = OperatorService()
    app.state.operator.ensure_ros()
    yield
    app.state.operator.close()


app = FastAPI(title='AM Operator Web GUI', docs_url=None, redoc_url=None, lifespan=lifespan)
app.mount('/static', StaticFiles(directory=WEB_ROOT / 'static'), name='static')
# Colcon's --symlink-install links each image back to the source tree.  Permit
# those known package assets so the same server works before and after a build.
app.mount('/img', StaticFiles(directory=IMAGE_ROOT, follow_symlink=True), name='img')
templates = Jinja2Templates(directory=WEB_ROOT / 'templates')


def operator(request: Request) -> OperatorService:
    return request.app.state.operator


@app.get('/', response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(request, 'index.html', {'state': operator(request).snapshot()})


@app.get('/api/state')
async def state(request: Request):
    return operator(request).snapshot()


@app.put('/api/settings')
async def settings(payload: SettingsPayload, request: Request):
    return {'config': operator(request).update_config(payload.values)}


async def _run_action(request: Request, action: str):
    try:
        operator(request).action(action)
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    return operator(request).snapshot()


# Keep every potentially safety-relevant GUI command visible in the HTTP API
# rather than using an unconstrained command endpoint.
ACTION_ENDPOINTS = (
    'launch_all', 'stop_all', 'simulation', 'pose_adapters', 'publish_path',
    'path_index', 'base_follower', 'arm_follower', 'transformations', 'controllers',
    'switch_arm_velocity', 'capture_tool_offset', 'base_accuracy', 'tcp_accuracy',
    'accuracy_report', 'move_base', 'move_arm', 'start_following', 'stop_following',
    'calculate_path_transform', 'check_hardware_topics', 'rviz', 'sync_workspace',
)


def _action_endpoint(action_name: str):
    async def endpoint(request: Request):
        return await _run_action(request, action_name)
    return endpoint


for _action_name in ACTION_ENDPOINTS:
    app.add_api_route(
        f'/api/actions/{_action_name}',
        _action_endpoint(_action_name),
        methods=['POST'],
        name=f'action_{_action_name}',
    )
