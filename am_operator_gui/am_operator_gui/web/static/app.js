const fields = [...document.querySelectorAll('[data-setting]')];
const dirtyFields = new Set();
const defaults = {follower_type: 'pid', direction_mode: 'goal_direction', accuracy_phase: 'baseline', velocity_override: 100, nozzle_offset_mm: 0};
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
const platformSettingsElement = document.querySelector('#platform-settings');
let renderedPlatformSettingsSignature = null;
const platformTuningGroups = [
  {title: 'Maximale Base-Geschwindigkeiten', fields: [
    ['pid_gains', 'base_follower.max_vx', 'Follower X (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'base_follower.max_vy', 'Follower Y (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'base_follower.max_wz', 'Follower Yaw (rad/s)', 'number', 0, 0.001],
    ['pid_gains', 'base_move.max_linear_velocity', 'Move-to-start linear (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'base_move.max_lateral_velocity', 'Move-to-start lateral (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'base_move.max_angular_velocity', 'Move-to-start angular (rad/s)', 'number', 0, 0.001],
  ]},
  {title: 'Base smoothing', fields: [
    ['base_smoothing', 'enabled', 'Geschwindigkeitsglättung aktiv', 'checkbox'],
    ['base_smoothing', 'method', 'Methode', 'select', ['moving_average', 'accel_limit']],
    ['base_smoothing', 'max_accel_x', 'Max. Beschleunigung X (m/s²)', 'number', 0, 0.001],
    ['base_smoothing', 'max_accel_y', 'Max. Beschleunigung Y (m/s²)', 'number', 0, 0.001],
    ['base_smoothing', 'max_accel_wz', 'Max. Winkelbeschleunigung (rad/s²)', 'number', 0, 0.001],
    ['base_smoothing', 'moving_average_window_size', 'Moving-average Fenster', 'number', 1, 1],
    ['base_smoothing', 'external_path_index_stride', 'External path-index stride', 'number', 1, 1],
  ]},
  {title: 'Maximale Arm-Geschwindigkeiten', fields: [
    ['pid_gains', 'arm_direction.max_tracking_linear_velocity', 'Tracking linear (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.max_along_track_correction', 'Along-track Korrektur (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.max_spray_axis_correction', 'Spray-axis Korrektur (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'arm_move.max_linear_velocity', 'Move-to-start linear (m/s)', 'number', 0, 0.001],
    ['pid_gains', 'arm_move.max_angular_velocity', 'Move-to-start angular (rad/s)', 'number', 0, 0.001],
    ['jparse_limits', 'max_joint_velocity', 'J-PARSE Gelenk (rad/s)', 'number', 0.000001, 0.001],
    ['jparse_limits', 'max_cartesian_linear_velocity', 'J-PARSE kartesisch linear (m/s)', 'number', 0.000001, 0.001],
    ['jparse_limits', 'max_cartesian_angular_velocity', 'J-PARSE kartesisch angular (rad/s)', 'number', 0.000001, 0.001],
  ]},
  {title: 'PID gains', fields: [
    ['pid_gains', 'base_follower.kp_x', 'Base follower Kp X', 'number', 0, 0.001],
    ['pid_gains', 'base_follower.kp_y', 'Base follower Kp Y', 'number', 0, 0.001],
    ['pid_gains', 'base_follower.kp_yaw', 'Base follower Kp Yaw', 'number', 0, 0.001],
    ['pid_gains', 'base_move.kp_linear', 'Base move Kp linear', 'number', 0, 0.001],
    ['pid_gains', 'base_move.kp_lateral', 'Base move Kp lateral', 'number', 0, 0.001],
    ['pid_gains', 'base_move.kp_angular_to_point', 'Base move Kp angular-to-point', 'number', 0, 0.001],
    ['pid_gains', 'base_move.kp_angular_reorient', 'Base move Kp angular-reorient', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.kp_z', 'Arm direction Kp Z', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.along_track_kp', 'Arm direction along-track Kp', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.orthogonal_kp', 'Arm direction orthogonal Kp', 'number', 0, 0.001],
    ['pid_gains', 'arm_direction.final_position_tolerance', 'Arm final position tolerance (m)', 'number', 0, 0.001],
    ['pid_gains', 'arm_orientation.kp_orientation', 'Arm orientation Kp', 'number', 0, 0.001],
    ['pid_gains', 'arm_orientation.ki_orientation', 'Arm orientation Ki', 'number', 0, 0.001],
    ['pid_gains', 'arm_orientation.kd_orientation', 'Arm orientation Kd', 'number', 0, 0.001],
    ['pid_gains', 'arm_move.kp_linear', 'Arm move Kp linear', 'number', 0, 0.001],
    ['pid_gains', 'arm_move.kp_angular', 'Arm move Kp angular', 'number', 0, 0.001],
  ]},
  {title: 'Pfadtransformation', fields: [
    ['path_transform', 'x', 'X (m)', 'number', undefined, 0.001],
    ['path_transform', 'y', 'Y (m)', 'number', undefined, 0.001],
    ['path_transform', 'z', 'Z (m)', 'number', undefined, 0.001],
    ['path_transform', 'yaw_deg', 'Yaw (Grad)', 'number', undefined, 0.1],
  ]},
];

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
  const platformOffsets = config.fixed_tool_offsets_by_platform || {};
  const offset = platformOffsets[config.platform] || config.fixed_tool_offset || defaultFixedToolOffset;
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

function platformSettingsFocused() {
  return platformSettingsElement?.contains(document.activeElement);
}
function platformSettingInput(section, key, label, type, minimum, step, value) {
  const wrapper = document.createElement('label');
  wrapper.textContent = label;
  const input = document.createElement(type === 'select' ? 'select' : 'input');
  input.dataset.platformSection = section;
  input.dataset.platformKey = key;
  if (type === 'checkbox') {
    input.type = 'checkbox';
    input.checked = Boolean(value);
    wrapper.prepend(input);
  } else if (type === 'select') {
    minimum.forEach(option => input.add(new Option(option.replace('_', ' '), option, false, option === value)));
    wrapper.append(input);
  } else {
    input.type = 'number';
    if (minimum !== undefined) input.min = String(minimum);
    input.step = String(step ?? 0.001);
    input.value = Number(value);
    wrapper.append(input);
  }
  return wrapper;
}
function renderPlatformSettings(state, config) {
  if (!platformSettingsElement || platformSettingsFocused()) return;
  const platform = config.platform || 'robotnik';
  const settings = state.platform_settings?.[platform];
  const signature = JSON.stringify({platform, settings});
  if (signature === renderedPlatformSettingsSignature) return;
  if (!settings) {
    const message = document.createElement('p');
    message.className = 'platform-settings-unavailable';
    message.textContent = 'Die laufende Web-Serverinstanz liefert noch keine Plattform-Tuning-Daten. Web-GUI neu starten und diese Seite anschließend neu laden.';
    platformSettingsElement.replaceChildren(message);
    const saveButton = document.querySelector('#save-platform-settings');
    saveButton.disabled = true;
    saveButton.title = 'Web-GUI neu starten, damit die Plattform-Tuning-API verfügbar ist';
    renderedPlatformSettingsSignature = signature;
    return;
  }
  const saveButton = document.querySelector('#save-platform-settings');
  saveButton.disabled = false;
  saveButton.title = '';
  const groups = platformTuningGroups.map((group, index) => {
    const details = document.createElement('details');
    details.className = 'platform-tuning-group';
    details.open = index === 0;
    const summary = document.createElement('summary');
    const title = document.createElement('strong');
    title.textContent = group.title;
    summary.append(title);
    const fields = document.createElement('div');
    fields.className = 'fields platform-tuning-fields';
    group.fields.forEach(([section, key, label, type, minimum, step]) => {
      fields.append(platformSettingInput(section, key, label, type, minimum, step, settings[section]?.[key]));
    });
    details.append(summary, fields);
    return details;
  });
  platformSettingsElement.replaceChildren(...groups);
  renderedPlatformSettingsSignature = signature;
}
function collectPlatformSettings() {
  const values = {};
  platformSettingsElement.querySelectorAll('[data-platform-section]').forEach(input => {
    const section = input.dataset.platformSection;
    const key = input.dataset.platformKey;
    values[section] ||= {};
    values[section][key] = input.type === 'checkbox' ? input.checked
      : (input.tagName === 'SELECT' ? input.value : Number(input.value));
  });
  for (const [section, entries] of Object.entries(values)) {
    for (const [key, value] of Object.entries(entries)) {
      if (typeof value === 'number' && !Number.isFinite(value)) {
        throw new Error(`${key} muss eine gültige Zahl sein`);
      }
    }
  }
  return values;
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
function renderHardwareTopicCheck(results = []) {
  const guide = document.querySelector('.hardware-guide details');
  if (!guide) return;
  let table = document.querySelector('#hardware-check-table');
  if (!table) {
    table = document.createElement('table');
    table.id = 'hardware-check-table';
    table.className = 'hardware-check-table';
    const header = table.createTHead().insertRow();
    ['Check', 'Configured topic result'].forEach(label => {
      const cell = document.createElement('th'); cell.textContent = label; header.append(cell);
    });
    const actions = guide.querySelector('.actions');
    actions.insertAdjacentElement('afterend', table);
  }
  const body = table.tBodies[0] || table.createTBody();
  body.replaceChildren();
  const rows = results.length ? results : ['Not run yet'];
  rows.forEach(result => {
    const row = body.insertRow();
    const passed = result.startsWith('OK');
    const failed = result.startsWith('FAIL') || result.includes('skipped') || result.includes('failed');
    row.className = passed ? 'check-ok' : (failed ? 'check-fail' : 'check-neutral');
    const mark = row.insertCell(); mark.textContent = passed ? '✓' : (failed ? '✕' : '—');
    const detail = row.insertCell(); detail.textContent = result;
  });
}
function render(state) {
  latestState = state;
  const config = state.config || {};
  const active = document.activeElement;
  fields.forEach(field => { if (active !== field) setField(field, config[field.dataset.setting]); });
  if (!toolOffsetFocused()) renderToolOffset(config);
  renderPlatformSettings(state, config);
  if (active !== document.querySelector('#advanced-json')) document.querySelector('#advanced-json').value = JSON.stringify(config, null, 2);
  const labels = {path:'Pfad', robot_pose:'Roboterpose', arm_pose:'Deposition pose', jparse_ready:'J-PARSE', controller_ready:'Controller'};
  document.querySelector('#status').innerHTML = Object.entries(labels).map(([key, label]) => `<span class="${state.status[key] ? 'ok' : 'wait'}">${label}: ${state.status[key] ? 'bereit' : 'wartet'}</span>`).join('');
  document.querySelector('#ros-state').textContent = state.ros_error ? `ROS nicht verbunden: ${state.ros_error}` : 'ROS Bridge aktiv';
  renderActionButtons(state.actions);
  renderHardwareTopicCheck(state.hardware_topic_results || []);
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
fields.forEach(field => field.addEventListener('change', async () => {
  dirtyFields.add(field.dataset.setting);
  try { await save(); await refresh(); }
  catch (error) { showFeedback(`Einstellung konnte nicht gespeichert werden: ${error.message}`, true); }
}));
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
  try {
    const values = JSON.parse(document.querySelector('#advanced-json').value);
    if (!values || Array.isArray(values) || typeof values !== 'object') throw new Error('Die oberste Ebene muss ein JSON-Objekt sein');
    await fetch('/api/settings',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({values})}).then(jsonResponse);
    showFeedback('Advanced settings gespeichert.');
    await refresh();
  } catch (error) { showFeedback(`Advanced settings konnten nicht gespeichert werden: ${error.message}`, true); }
});
document.querySelector('#save-platform-settings').addEventListener('click', async () => {
  try {
    const platform = latestState?.config?.platform;
    if (!platform) throw new Error('Keine Plattform ausgewählt');
    const values = collectPlatformSettings();
    await fetch('/api/platform-settings', {method: 'PUT', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({platform, values})}).then(jsonResponse);
    showFeedback(`Plattform-Tuning für ${platform} gespeichert. Follower und Controller bei Bedarf neu starten.`);
    await refresh();
  } catch (error) { showFeedback(`Plattform-Tuning konnte nicht gespeichert werden: ${error.message}`, true); }
});
toolOffsetElements.mode.addEventListener('change', convertToolOffsetMode);
document.querySelector('#save-tool-offset').addEventListener('click', async () => {
  try {
    const xyz = toolOffsetElements.xyz.map(input => Number(input.value));
    const quaternion = toolOffsetElements.mode.value === 'rpy'
      ? rpyDegreesToQuaternion(toolOffsetElements.rotation.slice(0, 3).map(input => Number(input.value)))
      : normalizeQuaternion(toolOffsetElements.rotation.map(input => Number(input.value)));
    if (![...xyz, ...quaternion].every(Number.isFinite)) throw new Error('All transform values must be numbers');
    const config = latestState?.config || {};
    const platformOffsets = {...(config.fixed_tool_offsets_by_platform || {})};
    platformOffsets[config.platform || 'robotnik'] = {xyz, quaternion_xyzw: quaternion};
    await fetch('/api/settings', {method: 'PUT', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({values: {
      fixed_tool_offsets_by_platform: platformOffsets,
      fixed_tool_offset_input_mode: toolOffsetElements.mode.value,
    }})}).then(jsonResponse);
    showFeedback('Flange-to-nozzle transform saved; restart arm controllers and follower to apply.');
    await refresh();
  } catch (error) { showFeedback(`Tool transform could not be saved: ${error.message}`, true); }
});
[consoleElements.source, ...consoleElements.levels, consoleElements.search, consoleElements.autoScroll].forEach(element => element.addEventListener(element === consoleElements.search ? 'input' : 'change', () => { saveConsolePreferences(); if (latestState) renderConsole(latestState.logs || []); }));
consoleElements.clear.addEventListener('click', () => { clearedAfter = Date.now(); if (latestState) renderConsole(latestState.logs || []); });
loadConsolePreferences();
refresh(); setInterval(refresh, 1000);
