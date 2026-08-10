const fields = [...document.querySelectorAll('[data-setting]')];
const dirtyFields = new Set();
const defaults = {follower_type: 'pid', direction_mode: 'goal_direction', accuracy_phase: 'baseline', velocity_override: 100, nozzle_offset_mm: 0, mur_arm: 'none'};
const consoleElements = {
  source: document.querySelector('#console-source'), levels: [...document.querySelectorAll('[data-console-level]')],
  search: document.querySelector('#console-search'), autoScroll: document.querySelector('#console-autoscroll'),
  clear: document.querySelector('#console-clear'), output: document.querySelector('#log'), count: document.querySelector('#log-count'),
};
const consoleStorageKey = 'am-operator-console-preferences';
let clearedAfter = null;
let latestState = null;
const toolOffsetElements = {
  mode: document.querySelector('#tool-offset-mode'),
  xyz: ['x', 'y', 'z'].map(axis => document.querySelector(`#tool-offset-${axis}`)),
  rotation: [0, 1, 2, 3].map(index => document.querySelector(`#tool-offset-r${index}`)),
  labels: [0, 1, 2, 3].map(index => document.querySelector(`#tool-offset-label-${index}`)),
};
const defaultFixedToolOffset = {xyz: [-0.25, 0, 0.015], quaternion_xyzw: [0, -0.7071067812, 0, 0.7071067812]};
let toolOffsetDisplayedMode = 'quaternion';

function normalizeQuaternion(values) {
  const norm = Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
  if (!Number.isFinite(norm) || norm < 1e-12) throw new Error('Quaternion norm must be greater than zero');
  return values.map(value => value / norm);
}
function rpyDegreesToQuaternion(values) {
  const [roll, pitch, yaw] = values.map(value => value * Math.PI / 180);
  const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
  const [cp, sp] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
  const [cy, sy] = [Math.cos(yaw / 2), Math.sin(yaw / 2)];
  return normalizeQuaternion([
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ]);
}
function quaternionToRpyDegrees(values) {
  const [x, y, z, w] = normalizeQuaternion(values);
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw].map(value => value * 180 / Math.PI);
}
function toolOffsetFocused() {
  return [...toolOffsetElements.xyz, ...toolOffsetElements.rotation, toolOffsetElements.mode]
    .includes(document.activeElement);
}
function updateToolOffsetMode() {
  const rpy = toolOffsetElements.mode.value === 'rpy';
  const labels = rpy ? ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)', 'Qw'] : ['Qx', 'Qy', 'Qz', 'Qw'];
  toolOffsetElements.labels.forEach((label, index) => label.textContent = labels[index]);
  toolOffsetElements.rotation.forEach((input, index) => {
    input.disabled = rpy && index === 3;
    input.step = rpy && index < 3 ? '0.1' : '0.000001';
  });
}
function convertToolOffsetMode() {
  const targetMode = toolOffsetElements.mode.value;
  if (targetMode === toolOffsetDisplayedMode) {
    updateToolOffsetMode();
    return;
  }
  try {
    if (targetMode === 'rpy') {
      const quaternion = normalizeQuaternion(toolOffsetElements.rotation.map(input => Number(input.value)));
      const rpy = quaternionToRpyDegrees(quaternion);
      toolOffsetElements.rotation.slice(0, 3).forEach((input, index) => input.value = rpy[index]);
      toolOffsetElements.rotation[3].value = quaternion[3];
    } else {
      const quaternion = rpyDegreesToQuaternion(
        toolOffsetElements.rotation.slice(0, 3).map(input => Number(input.value))
      );
      toolOffsetElements.rotation.forEach((input, index) => input.value = quaternion[index]);
    }
    toolOffsetDisplayedMode = targetMode;
    updateToolOffsetMode();
  } catch (error) {
    showFeedback(`Rotation could not be converted: ${error.message}`, true);
    toolOffsetElements.mode.value = toolOffsetDisplayedMode;
    updateToolOffsetMode();
  }
}
function renderToolOffset(config) {
  const offset = config.fixed_tool_offset || defaultFixedToolOffset;
  let quaternion;
  try { quaternion = normalizeQuaternion((offset.quaternion_xyzw || defaultFixedToolOffset.quaternion_xyzw).map(Number)); }
  catch (_) { quaternion = defaultFixedToolOffset.quaternion_xyzw; }
  toolOffsetElements.xyz.forEach((input, index) => input.value = Number(offset.xyz?.[index] ?? defaultFixedToolOffset.xyz[index]));
  const mode = config.fixed_tool_offset_input_mode === 'rpy' ? 'rpy' : 'quaternion';
  const rotation = mode === 'rpy' ? [...quaternionToRpyDegrees(quaternion), quaternion[3]] : quaternion;
  toolOffsetElements.rotation.forEach((input, index) => input.value = rotation[index]);
  toolOffsetElements.mode.value = mode;
  toolOffsetDisplayedMode = mode;
  updateToolOffsetMode();
}

function setField(field, value) { if (field.type === 'checkbox') field.checked = Boolean(value); else field.value = value ?? defaults[field.dataset.setting] ?? ''; }
function logLevel(item) {
  if (item.level) return item.level;
  const message = (item.message || '').toUpperCase();
  if (message.includes('[ERROR]') || message.includes('[FATAL]')) return 'error';
  if (message.includes('[WARN]') || message.includes('[WARNING]')) return 'warning';
  if (message.includes('[DEBUG]') || message.includes('[TRACE]')) return 'debug';
  return 'info';
}
function loadConsolePreferences() {
  try {
    const saved = JSON.parse(localStorage.getItem(consoleStorageKey) || '{}');
    if (Array.isArray(saved.levels)) consoleElements.levels.forEach(input => input.checked = saved.levels.includes(input.value));
    if (saved.search) consoleElements.search.value = saved.search;
    if (typeof saved.autoScroll === 'boolean') consoleElements.autoScroll.checked = saved.autoScroll;
  } catch (_) { /* Invalid browser storage should not affect the operator UI. */ }
}
function saveConsolePreferences() {
  localStorage.setItem(consoleStorageKey, JSON.stringify({
    source: consoleElements.source.value, levels: selectedLevels(),
    search: consoleElements.search.value, autoScroll: consoleElements.autoScroll.checked,
  }));
}
function selectedLevels() { return consoleElements.levels.filter(input => input.checked).map(input => input.value); }
function updateSourceOptions(logs) {
  const selected = consoleElements.source.value;
  const sources = [...new Set(logs.map(item => item.source).filter(Boolean))].sort();
  consoleElements.source.replaceChildren(new Option('Alle Prozesse', 'all'), ...sources.map(source => new Option(source, source)));
  const stored = (() => { try { return JSON.parse(localStorage.getItem(consoleStorageKey) || '{}').source; } catch (_) { return null; } })();
  consoleElements.source.value = sources.includes(selected) ? selected : (sources.includes(stored) ? stored : 'all');
}
function renderConsole(logs = []) {
  updateSourceOptions(logs);
  const search = consoleElements.search.value.trim().toLowerCase();
  const levels = selectedLevels();
  const visible = logs.filter(item => {
    const timestamp = item.timestamp ? Date.parse(item.timestamp) : Number.POSITIVE_INFINITY;
    return (!clearedAfter || timestamp >= clearedAfter)
      && (consoleElements.source.value === 'all' || item.source === consoleElements.source.value)
      && levels.includes(logLevel(item))
      && (!search || `${item.source} ${item.message}`.toLowerCase().includes(search));
  });
  consoleElements.output.replaceChildren(...visible.map(item => {
    const line = document.createElement('div');
    line.className = `log-line log-${logLevel(item)}`;
    const time = item.timestamp ? new Date(item.timestamp).toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'}) : '';
    line.textContent = `${time ? `${time} ` : ''}[${item.source}] ${item.message}`;
    return line;
  }));
  consoleElements.count.textContent = `${visible.length} von ${logs.length} Ausgaben`;
  if (consoleElements.autoScroll.checked) consoleElements.output.scrollTop = consoleElements.output.scrollHeight;
}
function renderActionButtons(actions = {}) {
  document.querySelectorAll('[data-action]').forEach(button => {
    const action = actions[button.dataset.action];
    if (!action) return;
    button.classList.remove('action-state-idle', 'action-state-running', 'action-state-progress', 'action-state-success', 'action-state-error', 'action-state-warning', 'action-state-ready', 'action-state-danger');
    button.classList.add(`action-state-${action.state}`);
    button.textContent = action.label;
    button.title = action.detail;
    button.setAttribute('aria-label', `${action.label}: ${action.detail}`);
  });
}
function render(state) {
  latestState = state;
  const config = state.config || {};
  const active = document.activeElement;
  fields.forEach(field => { if (active !== field) setField(field, config[field.dataset.setting]); });
  const murArm = fields.find(field => field.dataset.setting === 'mur_arm');
  if (murArm) murArm.disabled = config.platform !== 'mur620_sim';
  if (!toolOffsetFocused()) renderToolOffset(config);
  if (active !== document.querySelector('#advanced-json')) document.querySelector('#advanced-json').value = JSON.stringify(config, null, 2);
  const labels = {path:'Pfad', robot_pose:'Roboterpose', arm_pose:'Deposition pose', jparse_ready:'J-PARSE', controller_ready:'Controller'};
  document.querySelector('#status').innerHTML = Object.entries(labels).map(([key, label]) => `<span class="${state.status[key] ? 'ok' : 'wait'}">${label}: ${state.status[key] ? 'bereit' : 'wartet'}</span>`).join('');
  document.querySelector('#ros-state').textContent = state.ros_error ? `ROS nicht verbunden: ${state.ros_error}` : 'ROS Bridge aktiv';
  renderActionButtons(state.actions);
  renderConsole(state.logs || []);
}
function showFeedback(message, isError = false) {
  const feedback = document.querySelector('#action-feedback');
  feedback.textContent = message;
  feedback.classList.toggle('error', isError);
}
async function jsonResponse(response) {
  if (response.ok) return response.json();
  const error = await response.json().catch(() => ({}));
  throw new Error(error.detail || `HTTP ${response.status}`);
}
async function refresh() {
  try { render(await fetch('/api/state').then(jsonResponse)); }
  catch (error) { showFeedback(`Status konnte nicht aktualisiert werden: ${error.message}`, true); }
}
async function save() {
  if (!dirtyFields.size) return;
  const values = {};
  fields.filter(f => dirtyFields.has(f.dataset.setting)).forEach(f => values[f.dataset.setting] = f.type === 'checkbox' ? f.checked : (f.type === 'number' ? Number(f.value) : f.value));
  await fetch('/api/settings',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({values})}).then(jsonResponse);
  dirtyFields.clear();
}
fields.forEach(field => field.addEventListener('change', async () => { dirtyFields.add(field.dataset.setting); await save(); await refresh(); }));
document.querySelectorAll('[data-action]').forEach(button => button.addEventListener('click', async () => {
  button.classList.add('action-state-progress'); button.disabled = true;
  showFeedback('');
  try {
    await save();
    await fetch(`/api/actions/${button.dataset.action}`, {method:'POST'}).then(jsonResponse);
  } catch (error) {
    showFeedback(`Aktion „${button.textContent}“ fehlgeschlagen: ${error.message}`, true);
  }
  finally { button.disabled = false; await refresh(); }
}));
document.querySelector('#save-advanced').addEventListener('click', async () => {
  const button = document.querySelector('#save-advanced');
  try {
    const values = JSON.parse(document.querySelector('#advanced-json').value);
    await fetch('/api/settings',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({values})}).then(jsonResponse); await refresh();
  } catch (error) { button.textContent = `Ungültiges JSON: ${error.message}`; }
});
toolOffsetElements.mode.addEventListener('change', convertToolOffsetMode);
document.querySelector('#save-tool-offset').addEventListener('click', async () => {
  try {
    const xyz = toolOffsetElements.xyz.map(input => Number(input.value));
    const values = toolOffsetElements.mode.value === 'rpy'
      ? rpyDegreesToQuaternion(toolOffsetElements.rotation.slice(0, 3).map(input => Number(input.value)))
      : normalizeQuaternion(toolOffsetElements.rotation.map(input => Number(input.value)));
    if (![...xyz, ...values].every(Number.isFinite)) throw new Error('All transform values must be numbers');
    await fetch('/api/settings', {method: 'PUT', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({values: {
      fixed_tool_offset: {xyz, quaternion_xyzw: values}, fixed_tool_offset_input_mode: toolOffsetElements.mode.value,
    }})}).then(jsonResponse);
    showFeedback('Flange-to-nozzle transform saved; restart arm controllers and follower to apply.');
    await refresh();
  } catch (error) { showFeedback(`Tool transform could not be saved: ${error.message}`, true); }
});
[consoleElements.source, ...consoleElements.levels, consoleElements.search, consoleElements.autoScroll].forEach(element => element.addEventListener(element === consoleElements.search ? 'input' : 'change', () => { saveConsolePreferences(); if (latestState) renderConsole(latestState.logs || []); }));
consoleElements.clear.addEventListener('click', () => { clearedAfter = Date.now(); if (latestState) renderConsole(latestState.logs || []); });
loadConsolePreferences();
refresh(); setInterval(refresh, 1000);
