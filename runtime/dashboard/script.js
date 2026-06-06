const TOKEN = window.DASHBOARD_CONFIG.TOKEN;
const STREAM = window.DASHBOARD_CONFIG.STREAM;
const STATUS = window.DASHBOARD_CONFIG.STATUS;
const qp = TOKEN ? ("?token=" + encodeURIComponent(TOKEN)) : "";
document.getElementById("stream").src = STREAM + qp;

const steps = [];
const tbody = document.querySelector("#steps tbody");
const preview = document.getElementById("preview");
const runPill = document.getElementById("runPill");
const runDetail = document.getElementById("runDetail");
const progressBar = document.getElementById("progressBar");
const actionInput = document.getElementById("action");
const durationInput = document.getElementById("duration");
const addBtn = document.getElementById("add");
const cancelEditBtn = document.getElementById("cancelEdit");
let currentRunningStep = 0;
let isRunning = false;
let editingIndex = -1;

function resetEditor() {
  editingIndex = -1;
  addBtn.textContent = "+ add step";
  cancelEditBtn.style.display = "none";
}

function loadEditorFromStep(idx) {
  const step = steps[idx];
  if (!step) return;
  editingIndex = idx;
  actionInput.value = step.action;
  durationInput.value = String(step.duration_s);
  addBtn.textContent = "save edit";
  cancelEditBtn.style.display = "";
}

function render() {
  tbody.innerHTML = "";
  steps.forEach((s, i) => {
    const tr = document.createElement("tr");
    tr.className = "step-row";
    tr.dataset.idx = i;
    if (isRunning) {
      if (i + 1 < currentRunningStep) tr.classList.add("done");
      else if (i + 1 === currentRunningStep) tr.classList.add("active");
      else tr.classList.add("pending");
    }
    tr.innerHTML = `<td class="idx">${i+1}</td><td>${s.action}</td><td>${s.duration_s.toFixed(1)} s</td><td>${isRunning ? "" : `<button data-i="${i}" class="edit">edit</button> <button data-i="${i}" class="rm">×</button>`}</td>`;
    tbody.appendChild(tr);
  });
  preview.textContent = JSON.stringify({steps}, null, 2);
}

addBtn.onclick = () => {
  if (isRunning) return;
  const action = actionInput.value;
  const duration_s = parseFloat(durationInput.value || "0");
  if (!isFinite(duration_s) || duration_s < 0) return;
  if (editingIndex >= 0 && editingIndex < steps.length) {
    steps[editingIndex] = {action, duration_s};
    resetEditor();
  } else {
    steps.push({action, duration_s});
  }
  render();
};

cancelEditBtn.onclick = () => {
  if (isRunning) return;
  resetEditor();
};
document.getElementById("clear").onclick = () => {
  if (isRunning) return;
  steps.length = 0;
  resetEditor();
  render();
};
tbody.onclick = (e) => {
  if (isRunning) return;
  const t = e.target;
  const idx = parseInt(t.dataset.i, 10);
  if (!Number.isFinite(idx)) return;

  if (t.classList.contains("edit")) {
    loadEditorFromStep(idx);
    return;
  }

  if (t.classList.contains("rm")) {
    steps.splice(idx, 1);
    if (editingIndex === idx) resetEditor();
    else if (editingIndex > idx) editingIndex -= 1;
    render();
  }
};

document.getElementById("run").onclick = async () => {
  if (editingIndex >= 0) { runDetail.textContent = "save edit first"; return; }
  if (steps.length === 0) { runDetail.textContent = "add steps first"; return; }
  runDetail.textContent = "submitting…";
  const presetSel = document.getElementById("presetSelect");
  const presetInput = document.getElementById("presetName");
  const preset_name = (presetSel && presetSel.value) || (presetInput && presetInput.value.trim()) || null;
  const body = {steps, preset_name, description: null};
  const r = await fetch("/route/script" + qp, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { runDetail.textContent = "error: " + JSON.stringify(j.detail || r.status); return; }
  runDetail.textContent = "";
};
document.getElementById("stop").onclick = async () => {
  await fetch("/route/script/stop" + qp, {method: "POST"});
  runDetail.textContent = "stopping…";
};

async function setLight(on) {
  const sep = qp ? "&" : "?";
  const toggle = document.getElementById("lightToggle");
  const label = document.getElementById("lightLabel");
  try {
    const r = await fetch("/route/relay" + qp + sep + "on=" + (on ? 1 : 0), {method: "POST"});
    if (!r.ok) {
      const j = await r.json().catch(() => ({}));
      label.textContent = "light error: " + (j.detail || j.error || r.status);
      toggle.checked = !on;  // revert switch on failure
      return;
    }
    label.textContent = on ? "light on" : "light off";
  } catch (e) {
    label.textContent = "light error: network";
    toggle.checked = !on;
  }
}
document.getElementById("lightToggle").onchange = (e) => setLight(e.target.checked);

function setPill(klass, text) {
  runPill.className = "pill " + klass;
  runPill.textContent = text;
}

function safeText(v) {
  if (v === null || v === undefined || v === "") return "-";
  return String(v).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
}

function fmtAge(s) {
  if (s == null || !Number.isFinite(Number(s))) return "-";
  const v = Number(s);
  if (v < 1) return Math.round(v * 1000) + " ms";
  if (v < 60) return v.toFixed(1) + " s";
  return Math.floor(v / 60) + "m " + Math.floor(v % 60) + "s";
}

function fmtUptime(seconds) {
  if (seconds == null || !Number.isFinite(Number(seconds))) return "-";
  const total = Math.max(0, Math.floor(Number(seconds)));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return `${String(h).padStart(2,"0")}:${String(m).padStart(2,"0")}:${String(s).padStart(2,"0")}`;
}

function renderStatusBar(rpi) {
  const payload = (rpi && rpi.payload) || {};
  const stale = !rpi || !!rpi.stale;
  const online = !!(rpi && rpi.online);
  const estop = !!payload.estop_active;
  const estopCls = estop ? "text-bad" : "text-ok";
  const estopTxt = estop ? "ACTIVE" : "CLEAR";
  const rpiCls = online ? "text-ok" : "text-bad";
  const rpiTxt = online ? "ONLINE" : "OFFLINE";
  const rpiSub = fmtAge(rpi && rpi.age_s);
  const mqttConnected = payload.mqtt_connected !== false && !stale;
  const mqttTxt = stale ? "stale" : (mqttConnected ? "connected" : "disconnected");
  const mqttCls = mqttConnected ? "text-ok" : "text-bad";
  const pigpio = payload.pigpio_connected;
  const pigpioTxt = stale ? "unknown" : (pigpio ? "ok" : "error");
  const pigpioCls = !stale && pigpio ? "text-ok" : "text-bad";

  document.getElementById("statusBar").innerHTML = `
    <div class="status-grid">
      <div class="status-cell"><div class="label">E-stop</div><div class="value ${estopCls}">${estopTxt}</div></div>
      <div class="status-cell"><div class="label">RPi</div><div class="value ${rpiCls}">${rpiTxt}</div><div class="sub">${rpiSub}</div></div>
      <div class="status-cell"><div class="label">MQTT</div><div class="value ${mqttCls}">${mqttTxt}</div></div>
      <div class="status-cell"><div class="label">pigpio</div><div class="value ${pigpioCls}">${pigpioTxt}</div></div>
    </div>
    <div style="margin-top:4px;font-size:12px;color:#666;text-align:center">
      ${fmtUptime(payload.uptime_s ?? payload.uptime_sec)} · ${safeText(payload.hostname)}
    </div>`;
}

function renderVisionTable(t) {
  const fsm = safeText(t.fsm_state);
  const theta = t.theta != null ? Number(t.theta).toFixed(2) + "°" : "-";
  const servo = t.servo_angle != null ? Number(t.servo_angle).toFixed(1) + "°" : "-";
  const frame = safeText(t.frame_num);
  const route = safeText(t.route_id || t.route_mode || "-");
  document.getElementById("liveVisionTable").innerHTML = `
    <table class="data-table">
      <caption>Live Vision</caption>
      <thead><tr><th>FSM</th><th>Theta</th><th>Servo</th><th>Frame</th><th>Route</th></tr></thead>
      <tbody><tr><td>${fsm}</td><td>${theta}</td><td>${servo}</td><td>${frame}</td><td>${route}</td></tr></tbody>
    </table>`;
}

function renderActuatorTable(p) {
  const base = safeText(p && p.base_state);
  const steer = (p && p.steer_angle != null) ? Number(p.steer_angle).toFixed(1) + "°" : "-";
  const relay = (p && p.relay_on) ? "ON" : "OFF";
  const mode = safeText(p && p.current_route_mode);
  const modeCls = mode === "AUTO" ? "text-ok" : "text-warn";
  document.getElementById("actuatorStatus").innerHTML = `
    <table class="data-table">
      <caption>RPi Actuator</caption>
      <thead><tr><th>Base</th><th>Steer</th><th>Relay</th><th>Mode</th></tr></thead>
      <tbody><tr><td>${base}</td><td>${steer}</td><td>${relay}</td><td class="${modeCls}">${mode}</td></tr></tbody>
    </table>`;
}

const _eventLog = [];
const MAX_EVENTS = 8;
function addEvent(kind, msg) {
  const ts = new Date().toLocaleTimeString();
  _eventLog.unshift({kind, msg, ts});
  if (_eventLog.length > MAX_EVENTS) _eventLog.pop();
  renderEventLog();
}
function renderEventLog() {
  document.getElementById("eventLog").innerHTML = `
    <div style="font-size:12px;color:#888;text-transform:uppercase;letter-spacing:0.5px;margin-bottom:2px">Events</div>
    <div class="event-log">
      ${_eventLog.map(e => `<div class="event-item ${e.kind}">[${e.ts}] ${e.msg}</div>`).join("")}
    </div>`;
}

let lastRouteId;
async function pollStatus() {
  try {
    const r = await fetch("/route/script/status" + qp);
    if (r.ok) {
      const j = await r.json();
      const st = j.status || {};
      const wasRunning = isRunning;
      isRunning = !!st.running;
      currentRunningStep = st.current_step || 0;
      if (st.running) {
        setPill("pill-running", `running ${currentRunningStep}/${st.total}`);
        const cur = st.step ? `${st.step.action} ${st.step.duration_s}s` : "";
        runDetail.textContent = cur;
        progressBar.style.width = (st.total ? (currentRunningStep / st.total * 100) : 0) + "%";
      } else if (st.last_error) {
        setPill("pill-error", "error");
        runDetail.textContent = st.last_error;
        progressBar.style.width = "0%";
      } else {
        setPill("pill-idle", "idle");
        if (wasRunning) runDetail.textContent = "finished";
        progressBar.style.width = "0%";
        currentRunningStep = 0;
      }
      if (wasRunning !== isRunning || st.running) render();
      if (wasRunning && !isRunning) {
        setTimeout(() => { refreshRoutes(); reloadStream(); }, 800);
      }
    }
  } catch (e) {}
  try {
    const r2 = await fetch(STATUS + qp);
    if (r2.ok) {
      const j2 = await r2.json();
      const tel = j2.telemetry || {};
      const rpi = j2.rpi_status || null;
      const rpiPayload = (rpi && rpi.payload) || {};

      renderStatusBar(rpi);
      renderVisionTable(tel);
      renderActuatorTable(rpiPayload);

      const newRid = tel.route_id || null;
      if (lastRouteId !== undefined && lastRouteId !== newRid) {
        setTimeout(() => { refreshRoutes(); reloadStream(); }, 600);
      }
      lastRouteId = newRid;
    }
  } catch (e) {}
}

function reloadStream() {
  const img = document.getElementById("stream");
  if (!img) return;
  const base = STREAM + qp + (qp ? "&" : "?") + "_t=" + Date.now();
  img.src = base;
}
setInterval(pollStatus, 500);
render();
renderStatusBar(null);
renderVisionTable({});
renderActuatorTable(null);
renderEventLog();

const routesTbody = document.querySelector("#routesTable tbody");
function fmtBytes(n) {
  if (n == null) return "-";
  if (n < 1024) return n + " B";
  if (n < 1024*1024) return (n/1024).toFixed(1) + " KB";
  return (n/1024/1024).toFixed(1) + " MB";
}
function fmtElapsed(s) {
  if (s == null) return "-";
  return Number(s).toFixed(1) + " s";
}
function fmtTs(t) {
  if (!t) return "-";
  return t.replace("T", " ").replace(/\.[0-9]+/, "").replace("+00:00", "Z");
}
async function refreshRoutes() {
  try {
    const r = await fetch("/routes/list?limit=50" + (TOKEN ? ("&token=" + encodeURIComponent(TOKEN)) : ""));
    if (!r.ok) return;
    const j = await r.json();
    const list = j.routes || [];
    routesTbody.innerHTML = "";
    list.forEach(r => {
      const tr = document.createElement("tr");
      tr.className = "route-clickable";
      tr.dataset.name = r.route_id;
      const dl = r.has_zip ? `<a class="pill pill-running" style="text-decoration:none;padding:4px 10px;margin-right:6px" href="/routes/download/${encodeURIComponent(r.route_id)}${qp}" onclick="event.stopPropagation()">⬇</a>` : `<span class="muted" style="margin-right:6px">no zip</span>`;
      const del = `<button class="rm route-del" data-name="${r.route_id}" style="padding:4px 10px">🗑</button>`;
      const presetCol = r.preset_name ? `<span class="badge badge-ok">${r.preset_name}</span>` : (r.script_source ? `<span class="muted">${r.script_source}</span>` : `<span class="muted">-</span>`);
      tr.innerHTML = `<td>${r.route_id}</td><td>${r.route_mode||'-'}</td><td>${presetCol}</td><td>${r.status||'-'}${r.accepted===false?' ✗':''}${r.accepted===true?' ✓':''}</td><td>${r.total_frames??'-'}</td><td>${fmtElapsed(r.elapsed_s)}</td><td>${fmtBytes(r.zip_size)}</td><td>${fmtTs(r.end_timestamp_utc)}</td><td>${dl}${del}</td>`;
      routesTbody.appendChild(tr);
    });
  } catch (e) {}
}
document.getElementById("refreshRoutes").onclick = refreshRoutes;
setInterval(refreshRoutes, 5000);
refreshRoutes();

routesTbody.onclick = async (e) => {
  const t = e.target;
  if (t.classList.contains("route-del")) {
    const name = t.dataset.name;
    if (!confirm(`Delete route ${name}? This removes its directory and zip.`)) return;
    const r = await fetch(`/routes/${encodeURIComponent(name)}${qp}`, {method: "DELETE"});
    if (!r.ok) { alert("delete failed"); return; }
    refreshRoutes();
    return;
  }
  const tr = t.closest("tr.route-clickable");
  if (tr && tr.dataset.name) {
    openSummary(tr.dataset.name);
  }
};

async function openSummary(name) {
  const body = document.getElementById("summaryBody");
  const title = document.getElementById("summaryTitle");
  body.innerHTML = `<div class="muted" style="margin-top:14px">loading…</div>`;
  title.textContent = `Route summary · ${name}`;
  document.getElementById("summaryModal").classList.add("show");
  try {
    const r = await fetch(`/routes/${encodeURIComponent(name)}/summary${qp}`);
    if (!r.ok) {
      body.innerHTML = `<div class="muted" style="margin-top:14px;color:#f88">load failed (${r.status})</div>`;
      return;
    }
    const j = await r.json();
    body.innerHTML = renderSummary(j.summary || {});
  } catch (e) {
    body.innerHTML = `<div class="muted" style="margin-top:14px;color:#f88">network error</div>`;
  }
}

function renderSummary(s) {
  const accepted = s.accepted === true ? '<span class="badge badge-ok">accepted ✓</span>' : (s.accepted === false ? '<span class="badge badge-fail">rejected ✗</span>' : '-');
  const extra = s.extra_meta || {};
  const script = extra.script || {};
  const stepsRows = (script.steps || []).map((st, i) => `<tr><td>${i+1}</td><td>${st.action}</td><td>${Number(st.duration_s).toFixed(1)} s</td></tr>`).join("");
  const stepsTable = stepsRows
    ? `<table style="margin-top:6px"><thead><tr><th>#</th><th>Action</th><th>Duration</th></tr></thead><tbody>${stepsRows}</tbody></table>`
    : `<div class="muted">no script steps recorded</div>`;
  const submittedAt = script.submitted_at_unix ? new Date(script.submitted_at_unix * 1000).toISOString() : null;

  return `
    <div class="kv">
      <div class="k">Route ID</div><div class="v">${s.route_id || '-'}</div>
      <div class="k">Mode</div><div class="v">${s.route_mode || '-'}</div>
      <div class="k">Status</div><div class="v">${s.status || '-'} ${accepted}</div>
      <div class="k">Rejection reason</div><div class="v">${s.rejection_reason || '-'}</div>
      <div class="k">Started (UTC)</div><div class="v">${s.start_timestamp_utc || '-'}</div>
      <div class="k">Ended (UTC)</div><div class="v">${s.end_timestamp_utc || '-'}</div>
      <div class="k">Elapsed</div><div class="v">${s.total_elapsed_seconds != null ? Number(s.total_elapsed_seconds).toFixed(2) + ' s' : '-'}</div>
      <div class="k">Total frames</div><div class="v">${s.total_frames ?? '-'}</div>
      <div class="k">Frames with theta</div><div class="v">${s.frames_with_theta ?? '-'}</div>
      <div class="k">Gap ratio</div><div class="v">${s.gap_ratio != null ? Number(s.gap_ratio).toFixed(3) : '-'}</div>
      <div class="k">HW errors</div><div class="v">${s.hardware_error_count ?? '-'}</div>
      <div class="k">Abstract steps</div><div class="v">${s.abstract_steps ?? '-'}</div>
    </div>
    <h3>Script source</h3>
    <div class="kv">
      <div class="k">Source</div><div class="v">${script.source || '-'}</div>
      <div class="k">Preset name</div><div class="v">${script.preset_name || '-'}</div>
      <div class="k">Description</div><div class="v">${script.description || '-'}</div>
      <div class="k">Submitted (UTC)</div><div class="v">${submittedAt || '-'}</div>
      <div class="k">Step count</div><div class="v">${(script.steps || []).length}</div>
    </div>
    <h3>Steps</h3>
    ${stepsTable}
    <h3>Raw JSON</h3>
    <pre>${escapeHtml(JSON.stringify(s, null, 2))}</pre>
  `;
}

function escapeHtml(str) {
  return String(str).replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[c]));
}

document.getElementById("summaryClose").onclick = () => document.getElementById("summaryModal").classList.remove("show");
document.getElementById("summaryModal").addEventListener("click", (e) => {
  if (e.target.id === "summaryModal") e.currentTarget.classList.remove("show");
});

document.getElementById("deleteAllRoutes").onclick = async () => {
  if (!confirm("Delete ALL routes (directories + zips)? This cannot be undone.")) return;
  if (!confirm("Are you really sure? This wipes recorded data.")) return;
  const r = await fetch("/routes/delete_all" + qp, {method: "POST"});
  const j = await r.json().catch(() => ({}));
  alert(`removed=${j.removed||0} errors=${(j.errors||[]).length}`);
  refreshRoutes();
};

// ---------- presets CRUD ----------
const presetSelect = document.getElementById("presetSelect");
const presetName = document.getElementById("presetName");
async function refreshPresets() {
  try {
    const r = await fetch("/presets" + qp);
    if (!r.ok) return;
    const j = await r.json();
    const list = j.presets || [];
    const cur = presetSelect.value;
    presetSelect.innerHTML = "<option value=\"\">— select preset —</option>";
    list.forEach(p => {
      const o = document.createElement("option");
      o.value = p.name;
      o.textContent = `${p.name} (${p.steps_count} steps)`;
      presetSelect.appendChild(o);
    });
    if (cur) presetSelect.value = cur;
  } catch (e) {}
}
document.getElementById("presetLoad").onclick = async () => {
  const name = presetSelect.value;
  if (!name) { alert("select a preset first"); return; }
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`);
  if (!r.ok) { alert("load failed"); return; }
  const j = await r.json();
  const p = j.preset || {};
  steps.length = 0;
  (p.steps || []).forEach(s => steps.push({action: s.action, duration_s: Number(s.duration_s)}));
  presetName.value = p.name || name;
  resetEditor();
  render();
};
document.getElementById("presetSave").onclick = async () => {
  const name = (presetName.value || "").trim();
  if (!name) { alert("enter preset name"); return; }
  if (steps.length === 0) { alert("add steps first"); return; }
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`, {method: "PUT", headers: {"Content-Type": "application/json"}, body: JSON.stringify({steps})});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { alert("save failed: " + JSON.stringify(j.detail || r.status)); return; }
  await refreshPresets();
  presetSelect.value = name;
};
document.getElementById("presetDelete").onclick = async () => {
  const name = presetSelect.value;
  if (!name) { alert("select a preset first"); return; }
  if (!confirm(`Delete preset "${name}"?`)) return;
  const r = await fetch(`/presets/${encodeURIComponent(name)}${qp}`, {method: "DELETE"});
  if (!r.ok) { alert("delete failed"); return; }
  presetSelect.value = "";
  refreshPresets();
};
refreshPresets();
setInterval(refreshPresets, 10000);
