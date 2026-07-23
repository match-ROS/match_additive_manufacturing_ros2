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
[consoleElements.source, ...consoleElements.levels, consoleElements.search, consoleElements.autoScroll].forEach(element => element.addEventListener(element === consoleElements.search ? 'input' : 'change', () => { saveConsolePreferences(); if (latestState) renderConsole(latestState.logs || []); }));
consoleElements.clear.addEventListener('click', () => { clearedAfter = Date.now(); if (latestState) renderConsole(latestState.logs || []); });
loadConsolePreferences();
refresh(); setInterval(refresh, 1000);
