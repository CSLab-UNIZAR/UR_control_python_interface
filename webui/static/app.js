"use strict";

// Keyboard jog map: [plus key, minus key] for axis 0..5 (X,Y,Z,Rx,Ry,Rz or J1..J6).
const JOG_KEYS = [["Q", "A"], ["W", "S"], ["E", "D"], ["R", "F"], ["T", "G"], ["Y", "H"]];
const CART_AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"];
const POSE_FIELDS = [["X", "mm"], ["Y", "mm"], ["Z", "mm"], ["Roll", "°"], ["Pitch", "°"], ["Yaw", "°"]];
const AXIS_CLASS = ["axis-x", "axis-y", "axis-z"];
const FT_WINDOW_S = 10;

const S = {
  cfg: null, state: null, logSince: 0, serverUp: true,
  space: "cartesian", frame: "base", mode: "continuous", tab: "jog",
  posePoint: "tcp", poseRot: "rpy",   // pose card: TCP or flange, RPY or rotation vector
  held: new Map(),            // "axis,dir" -> Set of sources (pointer / keys)
  keepalive: null,
  ft: [],                     // [time, fx, fy, fz, mx, my, mz]
  touched: { joints: false, pose: false },
  planTimers: {}, lastPlan: 0, poses: [],
  wsDirty: false,             // workspace limits edited but not applied yet
};

// ------------------------------------------------------------------ helpers
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v)) ? "—" : v.toFixed(d);
const signed = (v, d = 1) => (v >= 0 ? "+" : "") + fmt(v, d);

function setText(id, text) {
  const el = document.getElementById(id);
  if (el && el.textContent !== text) el.textContent = text;
}

async function api(path, body) {
  const options = body === undefined ? {} : {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  };
  const res = await fetch(path, options);
  return res.json();
}

async function command(path, body) {
  try {
    const r = await api(path, body);
    if (!r.ok) localLog("error", r.error);
    return r;
  } catch {
    localLog("error", "The panel server is not reachable");
    return { ok: false };
  }
}

function localLog(level, text) {
  appendLog([{ time: Date.now() / 1000, level, text }]);
}

function appendLog(entries) {
  const box = $("#log");
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
  for (const e of entries) {
    const row = document.createElement("div");
    row.className = e.level;
    const time = document.createElement("span");
    time.className = "time";
    time.textContent = new Date(e.time * 1000).toLocaleTimeString();
    row.append(time, e.text);
    box.append(row);
  }
  while (box.childElementCount > 400) box.firstElementChild.remove();
  if (atBottom) box.scrollTop = box.scrollHeight;
}

function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(text).then(() => localLog("info", `Copied ${text}`));
  } else {
    window.prompt("Copy:", text);
  }
}

// ------------------------------------------------------------------ building the page
function buildPage() {
  const names = S.cfg.joint_names;
  const jointRows = $("#joint-table tbody");
  const targetRows = $("#joint-targets tbody");
  names.forEach((name, i) => {
    jointRows.insertAdjacentHTML("beforeend",
      `<tr><td>J${i + 1} <span class="muted small">${name}</span></td>
       <td class="num big" id="q-deg-${i}">—</td><td class="num sub" id="q-rad-${i}"></td>
       <td class="num" id="qd-${i}"></td></tr>`);
    targetRows.insertAdjacentHTML("beforeend",
      `<tr><td>J${i + 1} <span class="muted small">${name}</span></td><td class="num" id="jt-cur-${i}">—</td>
       <td class="num"><input type="number" step="0.1" id="jt-${i}"></td>
       <td class="num delta" id="jt-d-${i}"></td></tr>`);
  });
  POSE_FIELDS.forEach(([name, unit], i) => {
    $("#pose-grid").insertAdjacentHTML("beforeend",
      `<div><span class="${AXIS_CLASS[i % 3]}" id="pose-label-${i}">${name} (${unit})</span><b id="pose-${i}">—</b></div>`);
    $("#pose-targets tbody").insertAdjacentHTML("beforeend",
      `<tr><td>${name} <span class="muted small">${unit}</span></td><td class="num" id="pt-cur-${i}">—</td>
       <td class="num"><input type="number" step="${i < 3 ? 1 : 0.5}" id="pt-${i}"></td>
       <td class="num delta" id="pt-d-${i}"></td></tr>`);
  });
  ["Fx (N)", "Fy (N)", "Fz (N)", "Mx (Nm)", "My (Nm)", "Mz (Nm)"].forEach((name, i) => {
    $("#ft-values").insertAdjacentHTML("beforeend",
      `<div><span class="${AXIS_CLASS[i % 3]}">${name}</span> <b id="ft-${i}">—</b></div>`);
  });
  for (let i = 0; i < 6; i++) {
    $("#pad").insertAdjacentHTML("beforeend",
      `<div class="name" id="pad-name-${i}"></div><div class="value" id="pad-value-${i}"></div>
       <button class="needs-robot" data-axis="${i}" data-dir="-1">−<kbd>${JOG_KEYS[i][1]}</kbd></button>
       <button class="needs-robot" data-axis="${i}" data-dir="1">+<kbd>${JOG_KEYS[i][0]}</kbd></button>`);
  }

  const sp = S.cfg.speeds;
  initRange($("#linear-speed"), sp.linear_mm_s, "mm/s");
  initRange($("#angular-speed"), sp.angular_deg_s, "°/s");
  initRange($("#joint-speed"), sp.joint_deg_s, "°/s");
  $$(".move-speed").forEach((el) => initRange(el, sp.move_deg_s, "°/s"));
  fillSelect($("#linear-step"), S.cfg.steps.linear_mm, "mm");
  fillSelect($("#angular-step"), S.cfg.steps.angular_deg, "°");
  fillSelect($("#joint-step"), S.cfg.steps.joint_deg, "°");
  $("#robot-host").value = S.cfg.robot.host;
  $("#robot-port").value = S.cfg.robot.port;
  fillWorkspace(S.cfg.viewer.workspace);
  updateJogMode();
}

// Workspace editor: limits in metres from the server, shown in mm
function fillWorkspace(limits) {
  for (const input of $$(".ws-grid input")) {
    const value = String(Math.round(limits[input.dataset.axis][+input.dataset.end] * 1000));
    if (input.value !== value && document.activeElement !== input) input.value = value;
  }
}

function workspaceInputs() {
  const limits = { x: [0, 0], y: [0, 0], z: [0, 0] };
  for (const input of $$(".ws-grid input")) limits[input.dataset.axis][+input.dataset.end] = +input.value;
  return limits;
}

function initRange(el, [def, min, max], unit) {
  Object.assign(el, { min, max, step: 1, value: def });
  const out = el.nextElementSibling;
  const show = () => { out.textContent = `${el.value} ${unit}`; };
  el.addEventListener("input", () => {
    show();
    if (isJogging()) sendJog();
  });
  show();
}

function fillSelect(el, [choices, def], unit) {
  for (const c of choices) el.add(new Option(`${c} ${unit}`, c, c === def, c === def));
}

// ------------------------------------------------------------------ jogging
function jogAxes() {
  const axes = [0, 0, 0, 0, 0, 0];
  for (const [key, sources] of S.held) {
    if (!sources.size) continue;
    const [axis, dir] = key.split(",").map(Number);
    axes[axis] += dir;               // opposite directions cancel out
  }
  return axes;
}
const isJogging = () => jogAxes().some((v) => v !== 0);

function speeds() {
  return {
    linear_mm_s: +$("#linear-speed").value,
    angular_deg_s: +$("#angular-speed").value,
    joint_deg_s: +$("#joint-speed").value,
  };
}

function press(axis, dir, source) {
  const key = `${axis},${dir}`;
  if (!S.held.has(key)) S.held.set(key, new Set());
  if (S.held.get(key).has(source)) return;
  S.held.get(key).add(source);
  jogChanged();
}

function release(axis, dir, source) {
  const sources = S.held.get(`${axis},${dir}`);
  if (sources && sources.delete(source)) jogChanged();
}

function releaseAll() {
  if (!S.held.size) return;
  S.held.clear();
  jogChanged();
}

// Sends the held axes now, and keeps re-sending them while held: the server
// stops the robot if these keep-alives stop (tab closed, browser frozen...).
function jogChanged() {
  for (const b of $$("#pad button")) {
    b.classList.toggle("held", (S.held.get(`${b.dataset.axis},${b.dataset.dir}`)?.size ?? 0) > 0);
  }
  sendJog();
  const active = isJogging();
  if (active && !S.keepalive) S.keepalive = setInterval(sendJog, S.cfg.keepalive_ms);
  if (!active && S.keepalive) { clearInterval(S.keepalive); S.keepalive = null; }
}

function sendJog() {
  api("/api/jog", { space: S.space, frame: S.frame, axes: jogAxes(), ...speeds() })
    .then((r) => { if (!r.ok) localLog("error", r.error); })
    .catch(() => {});
}

function doStep(axis, dir) {
  let size = +$("#joint-step").value;
  if (S.space === "cartesian") size = axis < 3 ? +$("#linear-step").value : +$("#angular-step").value;
  command("/api/step", { space: S.space, frame: S.frame, axis, direction: dir, size, ...speeds() });
}

function updateJogMode() {
  const tab = $("#tab-jog");
  tab.dataset.space = S.space;
  tab.dataset.mode = S.mode;
  for (let i = 0; i < 6; i++) {
    const el = $(`#pad-name-${i}`);
    el.textContent = S.space === "joint" ? `J${i + 1}` : CART_AXES[i];
    el.className = "name " + (S.space === "joint" ? "" : AXIS_CLASS[i % 3]);
    el.title = S.space === "joint" ? S.cfg.joint_names[i] : `${CART_AXES[i]} in the ${S.frame} frame`;
  }
}

// ------------------------------------------------------------------ moves
function movePayload(kind) {
  const speed = +$(`#tab-${kind} .move-speed`).value;
  const values = [0, 1, 2, 3, 4, 5].map((i) => +$(`#${kind === "joints" ? "jt" : "pt"}-${i}`).value);
  if (kind === "joints") return { kind, q_deg: values, speed_deg_s: speed };
  return { kind, xyz_mm: values.slice(0, 3), rpy_deg: values.slice(3), speed_deg_s: speed };
}

function schedulePlan(kind) {
  clearTimeout(S.planTimers[kind]);
  S.planTimers[kind] = setTimeout(() => plan(kind), 150);
}

async function plan(kind) {
  S.lastPlan = performance.now();
  const box = $(`#preview-${kind}`);
  if (!S.state || S.state.phase !== "ready") {
    box.className = "preview";
    box.textContent = "Connect to the robot to preview moves.";
    return null;
  }
  let r;
  try { r = await api("/api/plan", movePayload(kind)); } catch { return null; }
  if (!r.ok) {
    box.className = "preview error";
    box.textContent = "✗ " + r.error;
    S.viewer?.setGhost(null);
    return null;
  }
  if (S.tab === kind) S.viewer?.setGhost(r.max_motion_deg < 0.01 ? null : r.q_deg.map((v) => v * Math.PI / 180));
  let html = r.max_motion_deg < 0.01 ? "Already at the target."
    : `Largest joint motion: <b>${r.worst_joint} ${fmt(r.max_motion_deg)}°</b> · duration ≈ <b>${fmt(r.duration)} s</b>`;
  html += kind === "pose"
    ? `<br><span class="muted">IK solution: [${r.q_deg.map((v) => fmt(v)).join(", ")}]°</span>`
    : `<br><span class="muted">TCP target: [${r.tcp_mm.map((v) => fmt(v)).join(", ")}] mm</span>`;
  for (const w of r.warnings) html += `<br><span class="warn">⚠ ${w}</span>`;
  box.className = "preview";
  box.innerHTML = html;
  return r;
}

async function move(kind) {
  releaseAll();
  const r = await plan(kind);
  if (!r) return;
  if (r.max_motion_deg < 0.01) return;
  if (r.max_motion_deg > S.cfg.confirm_above_deg || r.warnings.length) {
    const text = `Move ${r.worst_joint} by ${fmt(r.max_motion_deg)}° over about ${fmt(r.duration)} s?`
      + r.warnings.map((w) => `\n\n⚠ ${w}`).join("") + "\n\nKeep the teach pendant within reach.";
    if (!window.confirm(text)) return;
  }
  command("/api/move", movePayload(kind));
}

function fillTargets(kind, values) {
  values.forEach((v, i) => { $(`#${kind === "joints" ? "jt" : "pt"}-${i}`).value = v.toFixed(2); });
  schedulePlan(kind);
}

async function refreshPoses(selectName) {
  const r = await api("/api/poses");
  S.poses = r.poses;
  const sel = $("#pose-select");
  sel.innerHTML = "";
  for (const p of S.poses) sel.add(new Option(p.builtin ? `${p.name} (preset)` : p.name, p.name));
  if (selectName) sel.value = selectName;
  updatePoseButtons();
}

function selectedPose() {
  return S.poses.find((p) => p.name === $("#pose-select").value);
}

function updatePoseButtons() {
  const pose = selectedPose();
  $("#pose-load").disabled = !pose;
  $("#pose-delete").disabled = !pose || pose.builtin;
}

// ------------------------------------------------------------------ rendering
const PHASES = {
  ready: ["ok", (s) => `Connected to ${s.robot.host}:${s.robot.port}`],
  waiting: ["warn", () => "Connected, waiting for /joint_states (is the UR driver running?)"],
  connecting: ["warn", (s) => `Connecting to ${s.robot.host}:${s.robot.port}…`],
  stale: ["warn", () => "No fresh robot data (motion blocked)"],
  lost: ["error", () => "Connection lost, reconnecting…"],
  disconnected: ["error", (s) => s.error || "Not connected"],
};

function render(st) {
  const ready = st.phase === "ready";
  const [level, text] = PHASES[st.phase];
  $("#status").className = "pill " + level;
  setText("status-text", text(st));
  $("#connect-form").querySelectorAll("input, button").forEach((el) => { el.disabled = st.phase !== "disconnected"; });
  setText("rates", st.rates && st.rates.joint_states !== undefined
    ? `joint_states ${fmt(st.rates.joint_states, 0)} Hz · F/T ${fmt(st.rates.ft, 0)} Hz` : "");
  $$(".needs-robot").forEach((el) => { el.disabled = !ready; });
  if (!ready && S.held.size) releaseAll();

  const has = Array.isArray(st.q_deg);
  const view = has ? st.pendant[S.posePoint] : null;
  const rotvec = S.poseRot === "rotvec";
  setText("tcp-point", has ? `TCP (+${fmt(st.pendant.tcp_offset_mm, 0)} mm)` : "TCP");
  for (let i = 0; i < 6; i++) {
    setText(`q-deg-${i}`, has ? fmt(st.q_deg[i], 2) + "°" : "—");
    setText(`q-rad-${i}`, has ? fmt(st.q_rad[i], 4) + " rad" : "");
    setText(`qd-${i}`, has && st.qd_deg_s ? signed(st.qd_deg_s[i], 1) + " °/s" : "");
    const pose = has ? (i < 3 ? st.tcp_mm[i] : st.rpy_deg[i - 3]) : null;   // TCP + RPY (move targets)
    const shown = !has ? null : i < 3 ? view.xyz_mm[i] : (rotvec ? view.rotvec_rad : view.rpy_deg)[i - 3];
    setText(`pose-label-${i}`, i < 3 ? `${POSE_FIELDS[i][0]} (mm)`
      : rotvec ? `R${"XYZ"[i - 3]} (rad)` : `${POSE_FIELDS[i][0]} (°)`);
    setText(`pose-${i}`, fmt(shown, i < 3 ? 1 : rotvec ? 4 : 2));
    setText(`ft-${i}`, has ? signed((i < 3 ? st.force : st.torque)[i % 3], i < 3 ? 2 : 3) : "—");

    let padValue = "";
    if (has && S.space === "joint") padValue = fmt(st.q_deg[i], 2) + "°";
    else if (has && i < 3 && S.frame === "base") padValue = fmt(st.tcp_mm[i], 1) + " mm";
    setText(`pad-value-${i}`, padValue);

    for (const [kind, prefix, current] of [["joints", "jt", has ? st.q_deg[i] : null], ["pose", "pt", pose]]) {
      setText(`${prefix}-cur-${i}`, fmt(current, 2));
      const input = $(`#${prefix}-${i}`);
      if (has && !S.touched[kind] && document.activeElement !== input) input.value = current.toFixed(2);
      const delta = +input.value - current;
      setText(`${prefix}-d-${i}`, has && input.value !== "" ? signed(delta, 2) : "");
    }
  }

  const ws = $("#workspace");
  if (!has) { ws.textContent = ""; ws.className = "badge"; }
  else if (st.workspace.length) { ws.textContent = "⚠ TCP outside workspace: " + st.workspace.join(", "); ws.className = "badge warn"; }
  else { ws.textContent = "✓ TCP inside workspace limits"; ws.className = "badge ok"; }

  setText("gripper-cmd", st.gripper.command === "Close" ? "Closed" : st.gripper.command === "Open" ? "Open" : "—");
  setText("gripper-fb", st.gripper.feedback || "no feedback topic");

  const jog = $("#jog-status");
  setText("jog-status", st.jog.text);
  jog.className = "status-line " + st.jog.level;

  if (has) {
    const now = performance.now() / 1000;
    S.ft.push([now, ...st.force, ...st.torque]);
    while (S.ft.length && S.ft[0][0] < now - FT_WINDOW_S) S.ft.shift();
  }
  drawForceChart();
  if (st.log.length) {
    appendLog(st.log);
    S.logSince = st.log[st.log.length - 1].id;
  }
  if (S.tab !== "jog" && performance.now() - S.lastPlan > 1000) schedulePlan(S.tab);
  $("#viewer-overlay").hidden = has || !S.viewer;
  if (!S.wsDirty) fillWorkspace(st.limits);
  S.viewer?.setWorkspaceLimits(st.limits);
  if (S.viewer && has) {
    S.viewer.update({
      q: st.q_rad, gripperClosed: st.gripper.closed, inside: st.workspace.length === 0,
      jog: { axes: st.jog.axes, space: S.space, frame: S.frame }, tcpMm: st.tcp_mm,
    });
  }
}

function renderServerDown() {
  $("#status").className = "pill error";
  setText("status-text", "Panel server stopped (restart python -m webui)");
  $$(".needs-robot").forEach((el) => { el.disabled = true; });
  releaseAll();
}

function drawForceChart() {
  const canvas = $("#ft-chart");
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const g = canvas.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.documentElement);
  const colors = ["--x", "--y", "--z"].map((v) => css.getPropertyValue(v).trim());
  const grid = css.getPropertyValue("--grid").trim();
  const muted = css.getPropertyValue("--muted").trim();
  const now = performance.now() / 1000;
  const gap = 8, panelH = (h - gap) / 2, left = 4, right = w - 4;
  [[1, "N", 2], [4, "Nm", 0.2]].forEach(([offset, unit, minRange], k) => {
    const top = k * (panelH + gap);
    let range = minRange;
    for (const s of S.ft) for (let j = 0; j < 3; j++) range = Math.max(range, Math.abs(s[offset + j]) * 1.15);
    const y = (v) => top + panelH / 2 - (v / range) * (panelH / 2 - 2);
    const x = (t) => left + (right - left) * (1 - (now - t) / FT_WINDOW_S);
    g.strokeStyle = grid;
    g.lineWidth = 1;
    g.strokeRect(left + 0.5, top + 0.5, right - left - 1, panelH - 1);
    g.beginPath(); g.moveTo(left, y(0)); g.lineTo(right, y(0)); g.stroke();
    g.fillStyle = muted;
    g.font = "10px sans-serif";
    g.fillText(`${unit === "N" ? "Force" : "Torque"} ±${range.toPrecision(2)} ${unit} · last ${FT_WINDOW_S} s`, left + 4, top + 12);
    for (let j = 0; j < 3; j++) {
      g.strokeStyle = colors[j];
      g.lineWidth = 1.5;
      g.beginPath();
      S.ft.forEach((s, i) => { i ? g.lineTo(x(s[0]), y(s[offset + j])) : g.moveTo(x(s[0]), y(s[offset + j])); });
      g.stroke();
    }
  });
}

async function poll() {
  try {
    S.state = await api(`/api/state?since=${S.logSince}`);
    S.serverUp = true;
    render(S.state);
  } catch {
    if (S.serverUp) localLog("error", "Lost contact with the panel server");
    S.serverUp = false;
    renderServerDown();
  }
  setTimeout(poll, 100);
}

// ------------------------------------------------------------------ events
function bindEvents() {
  $("#connect-form").addEventListener("submit", (e) => {
    e.preventDefault();
    command("/api/connect", { host: $("#robot-host").value, port: +$("#robot-port").value });
  });
  $("#stop-btn").addEventListener("click", stop);
  $("#grip-open").addEventListener("click", () => gripper("Open"));
  $("#grip-close").addEventListener("click", () => gripper("Close"));
  $("#zero-ft").addEventListener("click", () => command("/api/zero_ft", {}));
  $("#copy-joints").addEventListener("click", () => S.state?.q_rad && copyText(JSON.stringify(S.state.q_rad.map((v) => +v.toFixed(5)))));
  $("#copy-pose").addEventListener("click", () => S.state?.pose_si && copyText(JSON.stringify(S.state.pose_si.map((v) => +v.toFixed(5)))));

  for (const seg of $$(".seg")) {
    seg.addEventListener("click", (e) => {
      const b = e.target.closest("button");
      if (!b) return;
      releaseAll();
      S[seg.dataset.name] = b.dataset.value;
      for (const x of seg.children) x.classList.toggle("on", x === b);
      updateJogMode();
    });
  }

  for (const b of $$("#pad button")) {
    const axis = +b.dataset.axis, dir = +b.dataset.dir;
    b.addEventListener("pointerdown", (e) => {
      if (e.button !== 0 || b.disabled) return;
      e.preventDefault();
      if (S.mode === "step") { doStep(axis, dir); return; }
      b.setPointerCapture(e.pointerId);
      press(axis, dir, "pointer");
    });
    for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
      b.addEventListener(type, () => release(axis, dir, "pointer"));
    }
    b.addEventListener("contextmenu", (e) => e.preventDefault());
  }

  for (const b of $$(".tabs button")) {
    b.addEventListener("click", () => {
      releaseAll();
      S.tab = b.dataset.tab;
      $$(".tabs button").forEach((x) => x.classList.toggle("active", x === b));
      for (const name of ["jog", "joints", "pose"]) $(`#tab-${name}`).hidden = name !== S.tab;
      if (S.tab !== "jog") schedulePlan(S.tab);
      else S.viewer?.setGhost(null);
    });
  }

  for (const kind of ["joints", "pose"]) {
    $$(`#tab-${kind} .targets input`).forEach((el) => el.addEventListener("input", () => {
      S.touched[kind] = true;
      schedulePlan(kind);
    }));
    $(`#tab-${kind} .move-speed`).addEventListener("input", () => schedulePlan(kind));
    $(`#tab-${kind} .move-btn`).addEventListener("click", () => move(kind));
    $(`#tab-${kind} .copy-current`).addEventListener("click", () => {
      const st = S.state;
      if (!st?.q_deg) return;
      S.touched[kind] = true;
      fillTargets(kind, kind === "joints" ? st.q_deg : [...st.tcp_mm, ...st.rpy_deg]);
    });
  }

  for (const input of $$(".ws-grid input")) {
    input.addEventListener("input", () => {
      S.wsDirty = true;
      $("#ws-apply").disabled = false;
      setText("ws-note", "not applied yet");
    });
  }
  $("#ws-apply").addEventListener("click", async () => {
    const r = await command("/api/workspace", workspaceInputs());
    if (!r.ok) return;
    S.wsDirty = false;
    $("#ws-apply").disabled = true;
    setText("ws-note", "applied and saved in " + (S.cfg.settings_file || "the settings file"));
  });
  $("#ws-reset").addEventListener("click", async () => {
    releaseAll();
    if (!window.confirm("Reset the workspace limits to UR_CONTROL's defaults?")) return;
    const r = await command("/api/workspace/reset", {});
    if (!r.ok) return;
    S.wsDirty = false;
    $("#ws-apply").disabled = true;
    setText("ws-note", "defaults restored and saved in " + (S.cfg.settings_file || "the settings file"));
  });

  $("#pose-select").addEventListener("change", updatePoseButtons);
  $("#pose-load").addEventListener("click", () => {
    const pose = selectedPose();
    if (!pose) return;
    S.touched.joints = true;
    fillTargets("joints", pose.q_deg);
  });
  $("#pose-save").addEventListener("click", async () => {
    releaseAll();
    const name = window.prompt("Name for the current joint pose:");
    if (!name || !S.state?.q_deg) return;
    const r = await command("/api/poses/save", { name, q_deg: S.state.q_deg });
    if (r.ok) refreshPoses(name.trim());
  });
  $("#pose-delete").addEventListener("click", async () => {
    const pose = selectedPose();
    if (!pose || pose.builtin || !window.confirm(`Delete the saved pose "${pose.name}"?`)) return;
    const r = await command("/api/poses/delete", { name: pose.name });
    if (r.ok) refreshPoses();
  });

  // Keyboard: jog keys only on the Jog tab and outside text fields; STOP always.
  const typing = (e) => e.target.matches?.("input[type=text], input[type=number], select, textarea");
  const keymap = {};
  JOG_KEYS.forEach(([plus, minus], axis) => { keymap[plus] = [axis, 1]; keymap[minus] = [axis, -1]; });

  document.addEventListener("keydown", (e) => {
    if (e.ctrlKey || e.altKey || e.metaKey) return;
    if (e.key === "Escape" || (e.key === " " && !typing(e))) {
      e.preventDefault();
      if (!e.repeat) stop();
      return;
    }
    if (typing(e)) return;
    const key = e.key.toUpperCase();
    if (key === "O" || key === "C") {
      e.preventDefault();
      if (!e.repeat) gripper(key === "O" ? "Open" : "Close");
      return;
    }
    const jog = keymap[key];
    if (!jog || S.tab !== "jog" || !$("#keyboard-jog").checked) return;
    e.preventDefault();
    if (e.repeat || S.state?.phase !== "ready") return;
    if (S.mode === "step") doStep(jog[0], jog[1]);
    else press(jog[0], jog[1], "key " + key);
  });
  document.addEventListener("keyup", (e) => {
    const key = e.key.toUpperCase();
    const jog = keymap[key];
    if (jog) release(jog[0], jog[1], "key " + key);
  });
  window.addEventListener("blur", releaseAll);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
}

function stop() {
  releaseAll();
  command("/api/stop", {});
}

function gripper(cmd) {
  command("/api/gripper", { command: cmd });
}

// ------------------------------------------------------------------ start
async function startViewer() {
  const box = $("#viewer");
  try {
    const { RobotViewer } = await import("./viewer.js");
    const viewer = new RobotViewer(box, S.cfg.viewer);
    await viewer.load();
    box.querySelector(".viewer-msg")?.remove();
    for (const input of $$(".chips input")) {
      input.addEventListener("change", () => viewer.setOptions({ [input.dataset.opt]: input.checked }));
    }
    for (const b of $$(".views button")) b.addEventListener("click", () => viewer.setView(b.dataset.view));
    S.viewer = viewer;
  } catch (err) {
    console.error(err);
    box.querySelector(".viewer-msg").textContent = "3D view unavailable: " + err.message;
  }
}

(async () => {
  S.cfg = await api("/api/config");
  buildPage();
  bindEvents();
  await refreshPoses();
  poll();
  startViewer();
})();
