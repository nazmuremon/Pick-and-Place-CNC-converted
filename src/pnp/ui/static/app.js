// The pick and place page. Plain JS, no libraries (works offline on the Pi).
// The machine program sends its status and events over /ws; every button is one POST /api/action/<name>.
"use strict";

const $ = id => document.getElementById(id);
const PAGE_VERSION = (document.querySelector('script[src*="app.js"]').src.split("v=")[1] || "");
let S = null;               // the last status
let MAP = null;             // the last bed map
let events = [];

// ---------------------------------------------------------------- talking to the machine
async function api(name, body = {}) {
  try {
    const r = await fetch(`/api/action/${name}`, {method: "POST", headers: {"Content-Type": "application/json"},
                                                  body: JSON.stringify(body)});
    const j = await r.json();
    if (!j.ok) toast(j.why || "Refused", j.todo || "", "warn");
    return j;
  } catch (e) {
    toast("The machine program does not answer", "Is it running? The page reconnects by itself.", "error");
    return {ok: false};
  }
}

async function stop() {
  try { await fetch("/api/stop", {method: "POST"}); } catch (e) { /* the WS banner shows the link state */ }
}

let ws = null, wsDelay = 500, wsFails = 0, polling = null, lastSeq = 0;
function onStatus(st) { S = st; render(); }
function onEvent(ev) {
  if (ev.seq && ev.seq <= lastSeq) return;
  lastSeq = ev.seq || lastSeq;
  events.push(ev); if (events.length > 200) events.shift();
  renderLog();
  if (ev.level === "warn" || ev.level === "error") toast(ev.text, "", ev.level);
}
function connectWS() {
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onopen = () => { const was = !$("linkLost").hidden; wsDelay = 500; wsFails = 0; stopPolling(); $("linkLost").hidden = true; if (was) restartVideo(); };
  ws.onmessage = m => {
    const d = JSON.parse(m.data);
    if (d.type === "hello") {
      events = d.events || [];
      lastSeq = events.length ? events[events.length - 1].seq : 0;
      renderLog();
      $("updated").hidden = !(PAGE_VERSION && d.version !== PAGE_VERSION);
    } else if (d.type === "status") onStatus(d.status);
    else if (d.type === "event") onEvent(d.event);
  };
  ws.onclose = () => {
    wsFails += 1;
    if (wsFails >= 2) startPolling();          // no WebSocket (server without the library, a proxy): poll instead
    setTimeout(connectWS, wsDelay);
    wsDelay = Math.min(wsDelay * 2, 5000);
  };
}
function startPolling() {
  if (polling) return;
  polling = setInterval(async () => {
    try {
      onStatus(await (await fetch("/api/status")).json());
      for (const ev of await (await fetch("/api/events?n=50")).json()) onEvent(ev);
      if ($("linkLost").hidden === false) restartVideo();
      $("linkLost").hidden = true;
    } catch (e) { $("linkLost").hidden = false; }
  }, 400);
}
function stopPolling() { if (polling) { clearInterval(polling); polling = null; } }

// ---------------------------------------------------------------- toasts
function toast(text, sub = "", level = "info") {
  const t = document.createElement("div");
  t.className = `toast ${level}`;
  t.textContent = text;
  if (sub) { const s = document.createElement("small"); s.textContent = sub; t.appendChild(s); }
  $("toasts").appendChild(t);
  setTimeout(() => t.remove(), level === "info" ? 3500 : 7000);
}

// ---------------------------------------------------------------- render
const fmt = v => (v === null || v === undefined) ? "–" : (Math.abs(v) < 0.0005 ? 0 : v).toFixed(2);

function cap(role) { return (S.capabilities || []).find(c => c.role === role) || {ok: false, why: "unknown"}; }

function render() {
  if (!S) return;
  $("profileName").textContent = S.profile.name;
  $("simBadge").hidden = !S.sim;
  $("lgSim").hidden = !S.sim;
  $("simCard").hidden = !S.sim;
  if (S.sim_truth) {
    const h = S.sim_truth.head, l = S.sim_truth.lost_steps;
    $("simTruth").textContent = `Real head: X ${fmt(h.X)} Y ${fmt(h.Y)} Z ${fmt(h.Z)} (from HOME) · lost steps X ${fmt(l.X)} Y ${fmt(l.Y)} Z ${fmt(l.Z)} mm`
      + ` · servo at ${S.sim_truth.servo_angle.toFixed(1)}°`;
  }
  const b = S.banner;
  const ban = $("banner");
  ban.className = `banner ${b.level}`;
  ban.href = b.link || "#machine";
  $("bTitle").textContent = b.title + (b.detail ? ` (${b.detail})` : "");
  $("bTodo").textContent = (S.busy ? `Busy: ${S.busy}. ` : "") + (b.todo || "") +
    (S.profile.reconnect_needed ? " The profile changed: press Connect to use it." : "");

  const p = S.position || {};
  $("pX").textContent = fmt(p.X); $("pY").textContent = fmt(p.Y); $("pZ").textContent = fmt(p.Z);
  const mode = S.safety.mode;
  $("posNote").textContent = mode === "homed" ? "X / Y in mm from HOME" + (p.Z === null ? "; Z: no reference yet" : "; Z above the reference")
    : mode === "not_homed" ? "Not homed: X / Y are not known (slow jogs only)" : "";
  const c = S.controller;
  $("rawLine").textContent = c ? `Controller: ${c.state} · counted X ${fmt(c.raw.X)} Y ${fmt(c.raw.Y)} Z ${fmt(c.raw.Z)}` : "";
  $("ctlLine").textContent = c ? `${c.version || "?"}${c.firmware_ok ? "" : " (NOT the profile's firmware)"} · session token T${c.token ?? "–"}`
    + (Object.keys(c.mismatched || {}).length ? ` · settings differ: ${Object.entries(c.mismatched).map(([k, v]) => `${k}=${v[1]} (want ${v[0]})`).join(", ")}` : "")
    : "Not connected";

  // what each button may do now (the machine checks again: this only greys them out)
  const connected = S.safety.connected, locked = mode === "locked", busy = !!S.busy;
  setDis("[data-jog]", !connected || locked || busy);
  setDis("[data-jz='-1']", !connected || locked || busy);
  setDis("[data-jz='1']", !connected || busy);
  $("gotoBtn").disabled = !connected || mode !== "homed" || busy || !S.limits.travel.X;
  $("homeBtn").disabled = !connected || locked || busy || !cap("homing").ok;
  $("homeNote").textContent = cap("homing").ok ? `Method: ${cap("homing").kind}` : cap("homing").why;
  $("setZBtn").disabled = !connected || locked || busy || !cap("z_reference").ok;
  $("zNote").textContent = cap("z_reference").ok ? `Method: ${cap("z_reference").kind}` : cap("z_reference").why;
  $("recoverRow").hidden = !locked;
  const code = S.safety.lock && S.safety.lock.code;
  $("unlockBtn").hidden = !(code === "alarm" || code === "estop");
  $("clearBtn").hidden = !(locked && !["alarm", "estop", "link_lost"].includes(code));
  const rot = cap("rotation");
  setDis("#turnChips button", !connected || locked || busy || !rot.ok);
  $("turnBtn").disabled = !connected || locked || busy || !rot.ok;
  $("limpBtn").disabled = !connected || busy || rot.kind === "none";
  const r = S.rotation;
  $("rotNote").textContent = rot.ok ? `${r.kind}: ${r.angle === null ? "angle not set" : `at ${r.angle.toFixed(1)}°`}`
    + (r.range ? ` · range ${r.range[0].toFixed(0)}° … ${r.range[1].toFixed(0)}°` : "")
    + (r.accuracy ? ` · repeats only to ±${r.accuracy}°` : "") : `Rotation: ${rot.why}`;
  $("pumpOn").disabled = $("pumpOff").disabled = !connected || busy || !cap("vacuum").ok;
  $("pumpState").textContent = S.pump ? "ON" : "off";
  $("pumpState").style.color = S.pump ? "var(--ok)" : "";
  $("connectBtn").disabled = busy;
  $("disconnectBtn").disabled = busy || !connected;
  renderLights();
  renderMode();
  renderOverlay();
  renderVideoNote();
  if (current === "setup") renderCuPos();
  if (current === "board") renderRegistration();
  if (current === "job") renderJob();
}

function setDis(sel, v) { document.querySelectorAll(sel).forEach(b => b.disabled = v); }

function renderLights() {
  const row = $("lightsRow");
  const lights = (S.capabilities || []).filter(c => c.role.startsWith("light_") && c.kind !== "none");
  const key = lights.map(l => l.role + l.ok).join();
  if (row.dataset.key === key) return;
  row.dataset.key = key;
  row.innerHTML = "";
  if (!lights.length) { row.hidden = true; return; }
  row.hidden = false;
  for (const l of lights) {
    const lbl = document.createElement("span"); lbl.className = "lbl"; lbl.textContent = l.role.replace("light_", "Light ");
    row.appendChild(lbl);
    for (const on of [true, false]) {
      const btn = document.createElement("button"); btn.textContent = on ? "On" : "Off"; btn.disabled = !l.ok;
      btn.title = l.ok ? "" : l.why;
      btn.onclick = () => api("light", {role: l.role, on}); row.appendChild(btn);
    }
  }
}

const V = () => (S && S.vision) || {};
function canClick() { return !!(V().model && S.safety.connected && S.safety.mode !== "locked" && !S.busy); }
function renderMode() {
  // what a click on the picture does right now (docs/13 §4.9: always say it)
  const v = V();
  let t;
  if (!v.model) t = "Click on the picture: nothing (the camera is not calibrated: Setup, step 3).";
  else if (!S.safety.connected || S.safety.mode === "locked") t = "Click on the picture: nothing now (not connected, or locked).";
  else if (clickMode === "pointer") t = "Click on the picture: the POINTER. Click the exact spot the tip touched (set-up step 5).";
  else t = "Click on the picture: the head goes there (that spot comes to the red cross)"
    + (S.safety.homed ? "." : ": a slow jog, not homed yet.");
  $("modeLine").textContent = t + " The red cross is the picture centre: the head's reference point."
    + (v.model ? ` Camera: ${v.model.tilt_deg}° off vertical, ${v.model.px_per_mm} px/mm on the bed.` : "");
  $("camwrap").classList.toggle("clickable", canClick());
}

function renderOverlay() {
  // the cross on the picture centre (the head's reference point); the nozzle landing spot comes with Phase D
  const svg = $("overlay");
  if (svg.dataset.done) return;
  svg.dataset.done = "1";
  svg.setAttribute("viewBox", "0 0 160 100");
  svg.innerHTML = `<g id="ovBoard" fill="none" vector-effect="non-scaling-stroke"></g>
    <g stroke="#ff3b30" stroke-width="1.5" vector-effect="non-scaling-stroke" fill="none">
    <line x1="80" y1="40" x2="80" y2="47"/><line x1="80" y1="53" x2="80" y2="60"/>
    <line x1="68" y1="50" x2="77" y2="50"/><line x1="83" y1="50" x2="92" y2="50"/>
    <circle cx="80" cy="50" r="1.2"/></g>`;
}

// the registered board drawn on the live picture: pads, holes, outline (fractions of the picture)
async function refreshBoardOverlay() {
  const g = $("ovBoard");
  if (!g) return;
  if (!["machine", "setup"].includes(current) || !V().model) { g.innerHTML = ""; return; }
  let ov;
  try { ov = await (await fetch("/api/overlay")).json(); } catch (e) { return; }
  TIP = ov.tip || null;
  const ring = TIP ? `<circle cx="${(TIP[0] * 160).toFixed(2)}" cy="${(TIP[1] * 100).toFixed(2)}" r="2.2" stroke="#5ac8fa" stroke-width="2" vector-effect="non-scaling-stroke"/>` : "";
  if (!$("ovShow").checked || !V().registration) { g.innerHTML = ring; return; }
  const pts = poly => poly.map(([x, y]) => `${(x * 160).toFixed(2)},${(y * 100).toFixed(2)}`).join(" ");
  const line = (list, colour, width) => list.map(p => `<polyline points="${pts(p)}" stroke="${colour}" stroke-width="${width}" vector-effect="non-scaling-stroke"/>`).join("");
  g.innerHTML = line(ov.outline || [], "#ffd60a", 1.5) + line((ov.pads || []).map(p => p.concat([p[0]])), "#34c759", 1)
    + line((ov.holes || []).map(p => p.concat([p[0]])), "#5ac8fa", 1.5) + ring;
}

function onPictureClick(e) {
  if (!canClick()) return;
  const r = $("video").getBoundingClientRect();
  // the picture fills the 16:10 box (object-fit: contain, same shape)
  const u = (e.clientX - r.left) / r.width, v = (e.clientY - r.top) / r.height;
  if (clickMode === "pointer") { clickMode = "go"; renderMode(); setup("pointer_click", {u, v}); return; }
  api("click", {u, v});
}

function renderRegistration() {
  const r = V().registration;
  $("rBtn").disabled = !!S.busy || !S.safety.homed || !V().model;
  $("rLine").textContent = r ? `${r.board}, side ${r.side}: ${r.kind} fit, ${r.rms} mm RMS (worst ${r.max} mm), turned ${r.angle}°.`
    : (!V().model ? "The camera is not calibrated yet (Setup, step 3)." : !S.safety.homed ? "Home first (Machine tab)." : "Not registered yet.");
}

// ---------------------------------------------------------------- bed map
async function refreshMap() {
  if (current !== "machine") return;
  try { MAP = await (await fetch("/api/map")).json(); drawMap(); } catch (e) { /* next time */ }
}

function drawMap() {
  if (!MAP) return;
  const svg = $("map");
  const pts = [MAP.home];
  Object.values(MAP.slots).forEach(s => { pts.push(s); pts.push([s[0] + MAP.slot_direction[0] * 77, s[1] + MAP.slot_direction[1] * 77]); });
  if (MAP.pcb_area) { pts.push(MAP.pcb_area.xy); pts.push([MAP.pcb_area.xy[0] + MAP.pcb_area.size[0], MAP.pcb_area.xy[1] + MAP.pcb_area.size[1]]); }
  const t = MAP.travel;
  if (t.X && t.Y) { pts.push([t.X[0], t.Y[0]]); pts.push([t.X[1], t.Y[1]]); }
  const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
  const x0 = Math.min(...xs) - 20, x1 = Math.max(...xs) + 20, y0 = Math.min(...ys) - 20, y1 = Math.max(...ys) + 20;
  svg.setAttribute("viewBox", `${x0} ${-y1} ${x1 - x0} ${y1 - y0}`);   // svg y = −Y: the back is at the top
  const parts = [];
  // T-slot bars (rough: under every slot and HOME)
  for (const x of [MAP.home[0], ...Object.values(MAP.slots).map(s => s[0])])
    parts.push(`<rect x="${x - 10}" y="${-y1}" width="20" height="${y1 - y0}" fill="var(--map-bar)"/>`);
  if (MAP.pcb_area) {
    const [ax, ay] = MAP.pcb_area.xy, [w, h] = MAP.pcb_area.size;
    parts.push(`<rect x="${ax}" y="${-(ay + h)}" width="${w}" height="${h}" fill="#2e7d4f" opacity="0.85" rx="2"/>`);
  }
  for (const [n, s] of Object.entries(MAP.slots)) {
    const d = MAP.slot_direction, L = 87, W = 20;
    const cx = s[0] + d[0] * (L / 2 - 10), cy = s[1] + d[1] * (L / 2 - 10);
    const w = Math.abs(d[0]) > 0.5 ? L : W, h = Math.abs(d[0]) > 0.5 ? W : L;
    parts.push(`<rect x="${cx - w / 2}" y="${-(cy + h / 2)}" width="${w}" height="${h}" fill="var(--label)" rx="1"/>`);
    parts.push(`<rect x="${s[0] - 5}" y="${-s[1] - 5}" width="10" height="10" fill="#111"/>`);
    parts.push(`<text x="${s[0]}" y="${-s[1] + 16}" font-size="9" text-anchor="middle" fill="#222">${n}</text>`);
  }
  parts.push(`<rect x="${MAP.home[0] - 7.5}" y="${-MAP.home[1] - 7.5}" width="15" height="15" fill="var(--label)"/>`);
  parts.push(`<rect x="${MAP.home[0] - 5}" y="${-MAP.home[1] - 5}" width="10" height="10" fill="#111"/>`);
  parts.push(`<text x="${MAP.home[0]}" y="${-MAP.home[1] - 10}" font-size="8" text-anchor="middle" fill="var(--label)">HOME</text>`);
  for (const c of MAP.pcb_corners || [])
    parts.push(`<rect x="${c[0] - 5}" y="${-c[1] - 5}" width="10" height="10" fill="#111" stroke="var(--label)" stroke-width="2"/>`);
  if (t.X && t.Y)
    parts.push(`<rect x="${t.X[0]}" y="${-t.Y[1]}" width="${t.X[1] - t.X[0]}" height="${t.Y[1] - t.Y[0]}" fill="none" stroke="#8fd18f" stroke-width="1.2" stroke-dasharray="4 3"/>`);
  if (MAP.sim_head) parts.push(`<circle cx="${MAP.sim_head[0]}" cy="${-MAP.sim_head[1]}" r="6" fill="#c0c0c0" stroke="#fff" stroke-width="1" opacity="0.9"/>`);
  if (MAP.head) parts.push(`<circle cx="${MAP.head[0]}" cy="${-MAP.head[1]}" r="3.5" fill="#ff5a4f" stroke="#fff" stroke-width="1"/>`);
  svg.innerHTML = parts.join("");
}

// ---------------------------------------------------------------- video
function restartVideo() {
  // an MJPEG <img> fires no load event per frame: the status says whether the camera has frames
  const img = $("video");
  img.onerror = () => { setTimeout(restartVideo, 3000); };
  img.src = `/video?t=${Date.now()}`;
}
function renderVideoNote() {
  const cam = (S.capabilities || []).find(c => c.role === "camera_head");
  $("noVideo").hidden = !!(cam && cam.ok);
  $("noVideo").textContent = cam ? (cam.ok ? "" : `No camera picture: ${cam.why}`) : "No head camera in this profile";
}

// ---------------------------------------------------------------- jog
const stepOf = id => parseFloat(document.querySelector(`#${id} .on`).dataset.v);
function chips(id) {
  document.querySelectorAll(`#${id} button`).forEach(b => b.onclick = () => {
    document.querySelectorAll(`#${id} button`).forEach(x => x.classList.remove("on"));
    b.classList.add("on");
  });
}
function bumpStep(id, dir) {
  const bs = [...document.querySelectorAll(`#${id} button`)];
  const i = bs.findIndex(b => b.classList.contains("on"));
  const j = Math.max(0, Math.min(bs.length - 1, i + dir));
  bs[i].classList.remove("on"); bs[j].classList.add("on");
  toast(`${id === "stepXY" ? "X / Y" : "Z"} step ${bs[j].dataset.v} mm`);
}
function jogXY(ux, uy) { const s = stepOf("stepXY"); return api("jog", {dx: ux * s, dy: uy * s}); }
function jogZ(u) { return api("jog", {dz: u * stepOf("stepZ")}); }

// ---------------------------------------------------------------- tabs
let current = "machine";
function showTab(name) {
  if (!$(`tab-${name}`)) return;
  current = name;
  document.querySelectorAll(".tab").forEach(t => t.classList.toggle("on", t.id === `tab-${name}`));
  document.querySelectorAll("#tabs [data-tab]").forEach(b => b.classList.toggle("on", b.dataset.tab === name));
  if (location.hash !== `#${name}`) history.replaceState(null, "", `#${name}`);
  if (name === "doctor") loadDoctor();
  if (name === "settings") loadProfile();
  if (name === "setup") loadWizard();
  if (name === "feeders") loadFeeders();
  if (name === "hand" && window.onHandTab) window.onHandTab();
  if (name === "job") { loadPlan(); loadJobs(); }
  if (name === "logs") renderLog();
  if (name === "machine") refreshMap();
  if (name === "board") loadBoard();
}

// ---------------------------------------------------------------- doctor, settings, setup, logs
function checkRows(rows) {
  return rows.map(c => {
    const cls = c.ok === true ? "ok" : c.ok === false ? "bad" : "note";
    const word = c.word || (c.ok === true ? "OK" : c.ok === false ? "FIX" : "note");
    return `<tr><td class="mark ${cls}">${word}</td><td><b>${esc(c.name)}</b><br>${esc(c.detail || "")}</td>
      <td class="fix">${esc(c.ok === true ? "" : (c.fix || ""))}</td></tr>`;
  }).join("");
}
async function loadDoctor() {
  $("checks").innerHTML = "<tr><td>Checking…</td></tr>";
  try { $("checks").innerHTML = checkRows(await (await fetch("/api/doctor")).json()); }
  catch (e) { $("checks").innerHTML = "<tr><td>The machine program does not answer</td></tr>"; }
}
async function loadProfile() {
  const j = await (await fetch("/api/profile")).json();
  $("profFile").textContent = j.file;
  $("profErrors").innerHTML = j.status.errors.length
    ? `<div class="errors"><b>The file has errors (the running profile stays):</b><br>${j.status.errors.map(esc).join("<br>")}</div>` : "";
  $("tbd").innerHTML = j.status.missing.length ? checkRows(j.status.missing.map(m => ({name: m.field, ok: null, detail: m.effect, fix: m.fix})))
    : "<tr><td>Nothing: every field is known.</td></tr>";
  $("caps").innerHTML = checkRows(j.capabilities.map(c => ({name: `${c.role}: ${c.kind}`,
    ok: c.state === "absent" ? null : (c.state === "later" ? null : c.ok),
    word: c.state === "absent" ? "none" : (c.state === "later" ? "later" : null),
    detail: c.why, fix: c.state ? "" : c.todo})));
  $("profText").textContent = j.text;
}
// ---------------------------------------------------------------- the set-up wizard (docs/13 §4.6)
let WIZ = [], clickMode = "go", TIP = null;
const wizVals = {sheet_mm: "", block_mm: "", surface_z: "0", slot: "3", package: "0805", ref: "", dx: "0", dy: "0",
                 dtheta: "0", end_dir: "X+"};
async function loadWizard() {
  if (current !== "setup") return;
  const el = document.activeElement;
  if (el && $("wizard").contains(el) && el.tagName === "INPUT") return;      // don't redraw under your fingers
  try { WIZ = (await getJSON("/api/wizard")).steps; } catch (e) { return; }
  renderWizard();
}
const setup = (action, extra = {}) => api("setup", Object.assign({action}, extra));
function inp(name, label, unit = "", attrs = 'type="number" step="0.01" inputmode="decimal"') {
  return `<span class="lbl">${esc(label)}</span><label><input data-w="${name}" ${attrs} value="${esc(wizVals[name] ?? "")}"> ${unit}</label>`;
}
function btn(id, label, opts = {}) {
  return `<button data-wa="${id}" ${opts.extra || ""} class="${opts.danger ? "danger" : ""}" ${opts.dis ? "disabled" : ""}>${esc(label)}</button>`;
}
function stepBody(st, dis) {
  const i = st.info || {};
  switch (st.id) {
    case "connect":
      return `<div class="row">${btn("connect", "Connect")}${btn("outputs_test", "Test the outputs", {dis})}</div>`
        + (i.tbd && i.tbd.length ? `<div class="winfo">Still TBD in the profile: ${esc(i.tbd.join(", "))}</div>` : "");
    case "travel":
      if (st.title.startsWith("Homing switches"))
        return `<div class="row">${btn("switch_test", "Test the switches", {dis})}</div>`;
      return `<div class="winfo">Travel now (${esc(i.source || "")}): X ${fmtR(i.x)} · Y ${fmtR(i.y)} mm` +
        (Object.keys(i.found || {}).length ? ` · found in this session: ${esc(Object.entries(i.found).map(([k, v]) => `${k} ${v}`).join(", "))}` : "") + `</div>
        <div class="row">${["X-", "X+", "Y-", "Y+"].map(d => btn("creep", `Creep ${d}`, {dis, extra: `data-dir="${d}"`})).join("")}</div>
        <div class="row"><select data-w="end_dir">${["X-", "X+", "Y-", "Y+"].map(d => `<option ${wizVals.end_dir === d ? "selected" : ""}>${d}</option>`).join("")}</select>
        ${btn("end_here", "End here", {dis})}${btn("save_travel", "Save the travel", {dis})}</div>`;
    case "camera":
      return `<div class="row"><a href="/charuco.pdf" target="_blank" rel="noopener">Print the calibration sheet (PDF)</a></div>
        <div class="row">${btn("auto_exposure", "Find and lock the exposure", {dis})}${btn("calibrate_bed", "Sheet on the bed", {dis})}</div>
        <div class="row">${inp("block_mm", "Block", "mm")}${btn("calibrate_block", "Sheet on the block", {dis})}${btn("calibrate_solve", "Solve and save", {dis})}</div>
        <div class="sub">${esc(camLine())}</div>`;
    case "home_z":
      return `<div class="row">${btn("home", "Home", {dis})}${inp("surface_z", "The tip touches", "mm above the labels")}${btn("set_z", "Set Z here", {dis})}</div>`;
    case "nozzle_offset":
      return `<div class="row">${inp("sheet_mm", "Sheet height", "mm")}${btn("nozzle_marks", "Make marks", {dis, danger: true})}</div>
        <div class="row"><span class="lbl">By eye</span>${btn("pointer_tip", "The tip is on it", {dis})}${btn("pointer_mode", clickMode === "pointer" ? "Now click the spot…" : "Then click the spot", {dis})}</div>`
        + (i.pending ? `<div class="winfo">${esc(i.pending.text)}: ${i.pending.change_mm} mm from the saved one. ${btn("accept", "Accept it")}</div>` : "");
    case "rotation":
      return st.state === "absent" || st.state === "later" ? "" :
        `<div class="row">${inp("sheet_mm", "Sheet height", "mm")}${btn("rotation_marks", (st.actions[0] || {}).label || "Measure", {dis, danger: true})}</div>`
        + (i.accuracy_deg != null ? `<div class="winfo">The nozzle turns to ±${i.accuracy_deg}°</div>` : "");
    case "heights":
      return i.moving != null ? `<div class="winfo">Press ${i.press} mm · clearance ${i.clearance} mm · tape floor ${i.tape_floor} mm ·
        board top ${i.board_top ?? "–"} mm · tallest part ${i.tallest_part} mm<br>For a 1.35 mm part (0805): pick at Z ${i.pick} ·
        place at Z ${i.place ?? "–"} · moving at Z ${i.moving}</div>` : "";
    case "bottom":
      return `<div class="row">${btn("bottom_calibrate", "Measure", {dis, danger: true})}</div>`;
    case "survey":
      return `<div class="row">${btn("survey", "Survey", {dis})}</div>`;
    case "vacuum":
      return st.state === "later" ? "" : `<div class="row">${btn("vacuum_open", "Open", {dis})}${btn("vacuum_closed", "Closed", {dis})}</div>`;
    case "board":
      return `<div class="row"><a href="#board">Board tab: import, side, Register</a></div>`
        + (i.board ? `<div class="winfo">${esc(i.board)}, side ${i.side}: ${i.rms} mm RMS, worst ${i.max} mm</div>` : "");
    case "first_part": {
      let h = `<div class="row">${inp("slot", "Slot", "", 'type="number" step="1" min="1" max="99"')}${inp("package", "Package", "", 'type="text" size="8"')}${btn("look_at_slot", "Look at the slot", {dis})}</div>`;
      if (i.candidate) h += `<div class="winfo">Slot ${i.candidate.slot}: is this a part?<br><img src="${i.candidate.image}?t=${Date.now()}" alt="The first part found in the slot" style="max-width:100%">
        <div class="row">${btn("confirm_yes", "Yes, a part")}${btn("confirm_no", "No")}</div></div>`;
      h += `<div class="row">${inp("ref", "Board part", "", 'type="text" size="8" placeholder="R41"')}${btn("first_part", "Place it", {dis, danger: true})}</div>`;
      const L = i.last;
      if (L) {
        h += `<div class="winfo">${esc(L.ref)} from slot ${L.slot}: ` + (L.error ? `measured ${Math.hypot(...L.error).toFixed(2)} mm off (X ${L.error[0]}, Y ${L.error[1]}), angle ${L.angle_error}° (the camera reads angles to about ±3° on small parts)` : esc(L.why))
          + `<div class="wphotos">${Object.entries(L.photos || {}).filter(([k]) => k.startsWith("place")).map(([k, p]) => `<img src="/api/photo?path=${encodeURIComponent(p)}" alt="${k}" title="${k}">`).join("")}</div>
          <div class="row">${btn("fp_ok", "OK")}${inp("dx", "dx", "mm")}${inp("dy", "dy", "mm")}${inp("dtheta", "dθ", "°")}${btn("fp_move", "Move it by this")}</div></div>`;
      }
      return h;
    }
  }
  return "";
}
const fmtR = r => r ? `${r[0].toFixed(1)} … ${r[1].toFixed(1)}` : "TBD";
function camLine() {
  const v = V(), views = v.views || {}, m = v.model;
  return `Pictures: ${views[0] || 0} on the bed, ${views[1] || 0} on the block. ` + (m ? `Saved: ${m.tilt_deg}° off vertical, ${m.height_mm} mm high, ${m.px_per_mm} px/mm, fit ${m.rms_px} px.` : `Not calibrated${v.model_why ? `: ${v.model_why}` : ""}.`);
}
function renderWizard() {
  if (!WIZ.length) return;
  const firstOpen = (WIZ.find(s => !["valid", "absent", "later"].includes(s.state) && !s.blocked_by.length) || {}).id;
  const done = WIZ.filter(s => s.state === "valid").length, all = WIZ.filter(s => !["absent", "later"].includes(s.state)).length;
  $("wizSummary").textContent = `${done} of ${all} steps valid.` + (firstOpen ? ` Next: step ${WIZ.find(s => s.id === firstOpen).n}.` : " Ready for a job.");
  const busy = S && !!S.busy, connected = S && S.safety.connected;
  $("wizard").innerHTML = WIZ.map(st => {
    const blocked = st.blocked_by.length > 0;
    const dis = busy || !connected || blocked;
    const names = st.blocked_by.map(b => (WIZ.find(x => x.id === b) || {}).n).join(", ");
    return `<div class="card wstep ${st.state} ${blocked ? "blocked" : ""}" data-step="${st.id}">
      <h3><span class="n">${st.n}.</span> ${esc(st.title)} <span class="st ${st.state}">${st.state}</span>
        ${st.date ? `<small>${esc(st.date)}</small>` : ""}
        ${st.undo ? `<button class="small" data-undo="${st.undo}" ${busy || st.state === "missing" ? "disabled" : ""}>Undo</button>` : ""}</h3>
      ${st.why ? `<div class="sub">${esc(st.why)}</div>` : ""}
      ${blocked ? `<div class="sub">Do step ${names} first.</div>` : ""}
      ${st.you_do ? `<details ${st.id === firstOpen ? "open" : ""}><summary>How</summary><div class="wtexts">
        <div><b>You:</b> ${esc(st.you_do)}</div><div><b>The machine:</b> ${esc(st.machine_does)}</div>
        <div><b>Done when:</b> ${esc(st.done_when)}</div></div></details>` : ""}
      ${stepBody(st, dis)}
    </div>`;
  }).join("");
}
function wizardClick(e) {
  const u = e.target.closest("[data-undo]");
  if (u) { if (confirm("Put the value before the last one back?")) api("undo", {key: u.dataset.undo}).then(loadWizard); return; }
  const b = e.target.closest("[data-wa]");
  if (!b) return;
  const a = b.dataset.wa, W = wizVals;
  const after = p => Promise.resolve(p).then(() => setTimeout(loadWizard, 300));
  const map = {
    connect: () => api("connect"), outputs_test: () => setup("outputs_test"),
    creep: () => setup("creep", {dir: b.dataset.dir}), end_here: () => setup("end_here", {dir: W.end_dir}),
    save_travel: () => setup("save_travel"),
    auto_exposure: () => api("auto_exposure"), calibrate_bed: () => api("calibrate", {step: "bed"}),
    calibrate_block: () => api("calibrate", {step: "block", block_mm: W.block_mm}),
    calibrate_solve: () => api("calibrate", {step: "solve"}),
    home: () => api("home"), set_z: () => api("set_z", {surface_z: W.surface_z || 0}),
    nozzle_marks: () => setup("nozzle_marks", {sheet_mm: W.sheet_mm}),
    rotation_marks: () => setup("rotation_marks", {sheet_mm: W.sheet_mm}),
    pointer_tip: () => setup("pointer_tip"),
    pointer_mode: () => { clickMode = "pointer"; renderMode(); renderWizard(); toast("Click the feature's exact spot on the live picture (Machine tab or the close-up)"); },
    accept: () => setup("accept", {key: "nozzle_offset"}),
    survey: () => setup("survey"),
    bottom_calibrate: () => setup("bottom_calibrate"),
    switch_test: () => setup("switch_test"),
    vacuum_open: () => setup("vacuum_open"), vacuum_closed: () => setup("vacuum_closed"),
    look_at_slot: () => setup("look_at_slot", {slot: W.slot, package: W.package}),
    confirm_yes: () => setup("confirm", {yes: true}), confirm_no: () => setup("confirm", {yes: false}),
    first_part: () => setup("first_part", {slot: W.slot, ref: W.ref.trim()}),
    fp_ok: () => setup("first_part_answer", {ok: true}),
    fp_move: () => setup("first_part_answer", {ok: false, dx: W.dx, dy: W.dy, dtheta: W.dtheta}),
  };
  if (map[a]) after(map[a]());
}

// ---------------------------------------------------------------- feeders (docs/13 §4.4.1)
let PLAN = null;
async function loadFeeders() {
  if (current !== "feeders") return;
  let f, pv;
  try { f = await getRaw("/api/feeders"); pv = await getRaw("/api/job/plan"); } catch (e) { return; }
  $("fDict").textContent = `markers: ${f.dictionary}`;
  $("fTrays").innerHTML = `<tr><th>Tray</th><th>Marker</th><th>Kind</th><th>Reach</th><th>Line</th><th>Parts used</th><th></th></tr>` +
    f.trays.map(t => `<tr><td>${t.number}</td><td>${t.marker}</td><td>${t.type === "holder" ? "holder" : "label"} ${t.tape_w} mm ${t.kind}</td>
      <td>${esc(t.reach)}${t.measured ? "" : " (not surveyed)"}</td>
      <td>${t.line ? esc(t.line.replace("|", " ")) + (t.confirmed ? "" : " (first part not confirmed)") : "–"}</td>
      <td>${t.line ? `${t.used} of ~${t.capacity}${t.empty ? " · EMPTY" : ""}` : ""}</td>
      <td><button class="small" data-rmtray="${t.number}">Remove</button></td></tr>`).join("");
  if (!pv.ok) { $("fLoads").textContent = `${pv.why}. ${pv.todo || ""}`; return; }
  PLAN = pv;
  $("fLoadNote").textContent = `${pv.summary.lines} lines in ${pv.summary.loads} loads`;
  $("fLoads").innerHTML = pv.loads.map(ld => `<div class="winfo"><b>Load ${ld.index} of ${pv.loads.length}</b>: ` +
    Object.entries(ld.trays).map(([t, line]) => `tray ${t}: ${esc(line.replace("|", " "))}`).join(" · ") + `</div>`).join("") || "No reachable tray.";
  const sel = $("lLine"), keep = sel.value;
  sel.innerHTML = pv.lines.map(ln => `<option value="${esc(ln.key)}">${esc(ln.value)} ${esc(ln.package)} (${ln.count})</option>`).join("");
  if (keep) sel.value = keep;
}
async function feederAction(name, body) { const r = await api(name, body); setTimeout(loadFeeders, 300); return r; }

// ---------------------------------------------------------------- the job (docs/13 §4.7)
async function loadPlan() {
  let pv;
  try { pv = await getRaw("/api/job/plan"); } catch (e) { return; }
  if (!pv.ok) { $("jPlan").textContent = `${pv.why}. ${pv.todo || ""}`; return; }
  const s = pv.summary;
  $("jPlan").innerHTML = `${esc(s.board)}, side ${s.side}: <b>${s.parts}</b> parts by the machine in ${s.loads} loads (${s.lines} lines),
    about ${s.estimate_min} min; ${s.skipped} not by the machine.`;
  $("jSkipped").innerHTML = `<tr><th>Ref</th><th>Value</th><th>Package</th><th>Why</th></tr>` +
    pv.skipped.map(x => `<tr><td>${esc(x.ref)}</td><td>${esc(x.value || "")}</td><td>${esc(x.package || "")}</td><td>${esc(x.reason)}</td></tr>`).join("");
}
function renderJob() {
  const j = S && S.job;
  $("jStatus").textContent = j ? `${j.id}: ${j.mode}, ${j.status}` : "no job yet";
  const running = j && j.status === "running", paused = j && j.status === "paused";
  $("jStart").disabled = !!(running || paused || (S && S.busy));
  $("jResume").disabled = !paused || (j.pause && j.pause.kind === "held");
  $("jAbort").disabled = !paused || (j.pause && j.pause.kind === "held");
  $("jPause").hidden = !paused;
  $("jPicksL").hidden = $("jMode").value !== "pick_test";
  if (paused) { $("jPauseWhy").textContent = j.pause.why || ""; $("jPauseTodo").textContent = j.pause.todo || "";
    $("jHeld").hidden = j.pause.kind !== "held"; $("jNozzle").hidden = j.pause.kind !== "nozzle";
    $("jNozzleBtn").dataset.nozzle = j.pause.nozzle || ""; }
  if (!j) { $("jCounts").textContent = ""; $("jParts").innerHTML = ""; $("jReport").hidden = true; return; }
  $("jCounts").textContent = `Load ${j.load} of ${j.loads} · ` + Object.entries(j.counts).map(([k, v]) => `${v} ${k}`).join(", ")
    + (j.picks ? ` · picks ${j.picks.picked} of ${j.picks.done} (${j.picks.rate == null ? "–" : (100 * j.picks.rate).toFixed(1) + " %"})` : "")
    + (j.overrides.length ? ` · started with stale: ${j.overrides.join(", ")}` : "");
  const rel = j.folder.split("/jobs/")[1];
  $("jReport").hidden = !rel; if (rel) $("jReport").href = `/jobs/${rel}/report.html`;
  $("jParts").innerHTML = `<tr><th>Ref</th><th>Load</th><th>Tray</th><th>State</th><th>Why</th></tr>` +
    (j.parts_list || []).map(p => `<tr class="${p.state}"><td>${esc(p.ref)}</td><td>${p.load}</td><td>${p.tray ?? ""}</td><td>${p.state}</td><td>${esc(p.reason)}</td></tr>`).join("");
}
async function loadJobs() {
  if (current !== "job") return;
  try {
    const d = await getRaw("/api/jobs");
    $("jJobs").innerHTML = d.jobs.map(j => { const rel = j.report.split("/jobs/")[1];
      return `<li>${esc(j.created)} · ${esc(j.board)} · ${esc(j.mode)} · ${esc(j.status)} · ${Object.entries(j.counts).map(([k, v]) => `${v} ${k}`).join(", ")}
        ${rel ? `· <a href="/jobs/${rel}" target="_blank" rel="noopener">report</a>` : ""}</li>`; }).join("") || "<li>none</li>";
  } catch (e) { /* next time */ }
}

// the close-up of the live picture around the tip, for the touch-off (docs/13 §4.6 step 4)
function drawCloseup() {
  if (current !== "setup") return;
  const img = $("video"), c = $("closeup");
  if (!img.naturalWidth) return;
  const ctx = c.getContext("2d");
  const t = TIP || [0.5, 0.5];
  const zw = img.naturalWidth / 6, zh = img.naturalHeight / 6;
  const sx = Math.max(0, Math.min(img.naturalWidth - zw, t[0] * img.naturalWidth - zw / 2));
  const sy = Math.max(0, Math.min(img.naturalHeight - zh, t[1] * img.naturalHeight - zh / 2));
  ctx.imageSmoothingEnabled = false;
  try { ctx.drawImage(img, sx, sy, zw, zh, 0, 0, c.width, c.height); } catch (e) { return; }
  const cx = (t[0] * img.naturalWidth - sx) / zw * c.width, cy = (t[1] * img.naturalHeight - sy) / zh * c.height;
  ctx.strokeStyle = "#5ac8fa"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, 14, 0, 2 * Math.PI); ctx.stroke();
}
function renderCuPos() {
  const p = (S && S.position) || {};
  $("cuPos").textContent = `Tip Z ${p.Z == null ? "– (no Z reference)" : p.Z.toFixed(2) + " mm"} · X ${fmt(p.X)} · Y ${fmt(p.Y)}`;
}
function renderLog() {
  if (current !== "logs") return;
  const dbg = $("showDebug").checked;
  $("log").innerHTML = events.slice().reverse().filter(e => dbg || e.level !== "debug").map(e =>
    `<li class="${e.level}"><time>${new Date(e.t * 1000).toLocaleTimeString()}</time><span>${esc(e.text)}</span></li>`).join("");
}
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));

// ---------------------------------------------------------------- wiring
function wire() {
  $("stopBtn").onclick = stop;
  $("reloadBtn").onclick = () => location.reload();
  document.querySelectorAll("#tabs [data-tab]").forEach(b => b.onclick = () => showTab(b.dataset.tab));
  chips("stepXY"); chips("stepZ");
  document.querySelectorAll("[data-jog]").forEach(b => b.onclick = () => { const [x, y] = b.dataset.jog.split(",").map(Number); jogXY(x, y); });
  document.querySelectorAll("[data-jz]").forEach(b => b.onclick = () => jogZ(Number(b.dataset.jz)));
  $("gotoBtn").onclick = () => api("goto", {x: $("gx").value, y: $("gy").value});
  $("homeBtn").onclick = () => api("home");
  $("setZBtn").onclick = () => api("set_z", {surface_z: $("zSurface").value || 0});
  $("unlockBtn").onclick = () => api("unlock_alarm");
  $("clearBtn").onclick = () => api("confirm_clear");
  document.querySelectorAll("#turnChips button").forEach(b => b.onclick = () => { $("turnDeg").value = b.dataset.deg; api("turn", {deg: b.dataset.deg}); });
  $("turnBtn").onclick = () => api("turn", {deg: $("turnDeg").value});
  $("limpBtn").onclick = () => api("servo_limp");
  $("pumpOn").onclick = () => api("pump", {on: true});
  $("pumpOff").onclick = () => api("pump", {on: false});
  $("connectBtn").onclick = () => api("connect");
  $("disconnectBtn").onclick = () => api("disconnect");
  $("resumeBtn").onclick = () => api("resume");
  $("expoBtn").onclick = () => api("exposure", {value: $("expo").value});
  $("expoAuto").onclick = () => api("exposure", {value: null});
  $("expoLock").onclick = () => api("auto_exposure");
  $("camwrap").onclick = onPictureClick;
  $("ovShow").onchange = refreshBoardOverlay;
  $("wizard").addEventListener("click", wizardClick);
  $("wizard").addEventListener("input", e => { const n = e.target.dataset.w; if (n) wizVals[n] = e.target.value; });
  $("wizard").addEventListener("change", e => { const n = e.target.dataset.w; if (n) wizVals[n] = e.target.value; });
  chips("cuStep");
  $("cuUp").onclick = () => api("jog", {dz: stepOf("cuStep")});
  $("cuDown").onclick = () => api("jog", {dz: -stepOf("cuStep")});
  $("closeup").onclick = e => {
    if (clickMode !== "pointer" || !TIP) return;
    const r = $("closeup").getBoundingClientRect(), img = $("video");
    const zw = 1 / 6, zh = 1 / 6;
    const sx = Math.max(0, Math.min(1 - zw, TIP[0] - zw / 2)), sy = Math.max(0, Math.min(1 - zh, TIP[1] - zh / 2));
    clickMode = "go"; renderMode();
    setup("pointer_click", {u: sx + (e.clientX - r.left) / r.width * zw, v: sy + (e.clientY - r.top) / r.height * zh});
  };
  setInterval(loadWizard, 1500);
  setInterval(loadFeeders, 3000);
  setInterval(loadJobs, 5000);
  $("fRefresh").onclick = loadFeeders;
  $("fTrays").addEventListener("click", e => { const b = e.target.closest("[data-rmtray]");
    if (b && confirm(`Remove tray ${b.dataset.rmtray}? Its marker becomes free.`)) feederAction("remove_tray", {number: b.dataset.rmtray}); });
  $("lLoad").onclick = async () => {
    const t = $("lTray").value, line = $("lLine").value;
    const r = await feederAction("load_tray", {number: t, line});
    if (r.ok) { await setup("look_at_slot", {slot: t, package: line.split("|")[1]}); pollCandidate(); }
  };
  $("lRefilled").onclick = () => feederAction("refilled", {number: $("lTray").value});
  $("lYes").onclick = () => { setup("confirm", {yes: true}).then(() => { $("lCand").hidden = true; loadFeeders(); }); };
  $("lNo").onclick = () => { setup("confirm", {yes: false}).then(() => { $("lCand").hidden = true; }); };
  $("aType").onchange = () => { $("aGrid").hidden = !["matrix", "stick"].includes($("aType").value); };
  $("aAdd").onclick = async () => {
    const r = await feederAction("add_tray", {number: $("aNum").value, tape_w: $("aTape").value, kind: $("aKind").value,
      type: $("aType").value, zone_mm: $("aZone").value, x: $("aX").value, y: $("aY").value,
      rows: $("aRows").value, cols: $("aCols").value, pitch_u: $("aPu").value, pitch_v: $("aPv").value,
      first_u: $("aFu").value, tape_floor_mm: $("aFloor").value, top_mm: $("aTop").value});
    if (r.ok) $("aOut").innerHTML = `Tray ${r.tray}: marker ${r.marker}. ` + Object.entries(r.files || {}).map(([k, p]) =>
      `<a href="/api/trays/file/${encodeURIComponent(p.split("/").pop())}">download the ${k.toUpperCase()}</a>`).join(" ");
  };
  $("jPlanBtn").onclick = loadPlan;
  $("bundleBtn").onclick = async () => {
    const f = $("bundleFile").files[0];
    if (!f) { toast("Choose the bundle's zip first"); return; }
    if (!confirm(`Import ${f.name}? What is here now is kept in a zip first.`)) return;
    const r = await fetch("/api/bundle", {method: "POST", body: await f.arrayBuffer()}).then(x => x.json());
    toast(r.ok ? `Imported ${r.files} files: restart the program` : r.why, "", r.ok ? "info" : "warn");
  };
  $("jMode").onchange = renderJob;
  $("jStart").onclick = () => api("job_start", {mode: $("jMode").value, n_picks: $("jPicks").value, override: $("jOverride").checked});
  $("jResume").onclick = () => api("job_resume");
  $("jAbort").onclick = () => { if (confirm("Abort this job? Its report stays.")) api("job_abort"); };
  $("jNozzleBtn").onclick = () => api("nozzle_changed", {nozzle: $("jNozzleBtn").dataset.nozzle});
  document.querySelectorAll("[data-held]").forEach(b => b.onclick = () => api("job_answer", {choice: b.dataset.held}));
  setInterval(drawCloseup, 150);
  $("rBtn").onclick = () => api("register", {side: $("rSide").value, top_edge: $("rEdge").value, copy: $("rCopy").value,
    x: $("rCopy").value === "1" ? "" : $("rX").value, y: $("rCopy").value === "1" ? "" : $("rY").value});
  setInterval(refreshBoardOverlay, 700);
  $("doctorBtn").onclick = loadDoctor;
  document.querySelectorAll("[data-sim]").forEach(b => b.onclick = () => api("sim", {event: b.dataset.sim}));
  $("showDebug").onchange = renderLog;
  $("helpBtn").onclick = () => $("help").hidden = false;
  $("helpClose").onclick = () => $("help").hidden = true;
  $("help").onclick = e => { if (e.target === $("help")) $("help").hidden = true; };
  document.addEventListener("keydown", onKey);
  window.addEventListener("hashchange", () => showTab(location.hash.slice(1) || "machine"));
}

function onKey(e) {
  if (window.scanWedge && window.scanWedge(e)) return;              // a USB / Bluetooth barcode scanner (hand.js)
  if (e.target.matches("input, textarea, select")) return;
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key;
  // the Hand tab has its own keys; Space there toggles "placed" only while the machine is idle (else it is STOP)
  if (current === "hand" && window.handKey && window.handKey(e)) return;
  const map = {ArrowLeft: () => jogXY(-1, 0), ArrowRight: () => jogXY(1, 0), ArrowUp: () => jogXY(0, 1),
    ArrowDown: () => jogXY(0, -1), PageUp: () => jogZ(1), PageDown: () => jogZ(-1),
    "[": () => bumpStep("stepXY", -1), "]": () => bumpStep("stepXY", 1), "{": () => bumpStep("stepZ", -1),
    "}": () => bumpStep("stepZ", 1), h: () => api("home"), p: () => api("pump", {on: !(S && S.pump)}),
    "?": () => { $("help").hidden = !$("help").hidden; }, Escape: () => { $("help").hidden = true; }};
  if (k === " ") { e.preventDefault(); stop(); return; }
  if (/^[1-9]$/.test(k)) { showTab(["machine", "setup", "feeders", "board", "job", "hand", "doctor", "settings", "logs"][+k - 1]); return; }
  if (map[k]) {
    e.preventDefault();
    if (e.repeat && !k.startsWith("Arrow") && !k.startsWith("Page")) return;
    if (S && S.busy && (k.startsWith("Arrow") || k.startsWith("Page"))) return;   // one jog at a time
    map[k]();
  }
}


// ================================================================ Board tab (Phase B)
const BD = {side: "F", pv: null, importId: null, meta: null, sel: null, vb: null, drag: null};
const CLASS_ORDER = ["machine", "bottom_camera", "by_hand", "tht", "pad_only", "mechanical", "unknown", "dnp"];
const CLASS_WORD = {machine: "machine", bottom_camera: "bottom camera", by_hand: "by hand", tht: "through-hole",
  pad_only: "pad only", mechanical: "mounting hole", unknown: "unknown", dnp: "DNP"};

async function getRaw(url) {                       // polled views: an {ok: false} answer is shown, not a pop-up
  const r = await fetch(url);
  return r.json();
}

async function getJSON(url, opts) {
  const r = await fetch(url, opts);
  const j = await r.json();
  if (j && j.ok === false) { toast(j.why || "Refused", j.todo || "", "warn"); return null; }
  return j;
}

async function loadBoard() {
  let pv = null;
  if (BD.importId) {
    pv = await getJSON(`/api/board/import/${BD.importId}/preview?side=${BD.side}`);
    $("bShowing").textContent = "the import (not saved yet)";
  } else {
    const j = await getJSON(`/api/board?side=${BD.side}`);
    if (!j) return;
    pv = j.board;
    $("bShowing").textContent = pv ? `the current board: ${pv.name}` : "no board yet";
    $("bNames").innerHTML = (j.names || []).map(n => `<option${n === j.current ? " selected" : ""}>${esc(n)}</option>`).join("");
    renderCurrent(pv);
  }
  BD.pv = pv;
  if (!pv) { $("bsvg").innerHTML = ""; $("bParts").innerHTML = ""; $("bIssues").innerHTML = ""; return; }
  drawBoard(); renderParts(); renderIssues();
}

function renderCurrent(pv) {
  if (!pv) { $("bCurrent").textContent = "No board yet: import one."; return; }
  const s = pv.summary, c = pv.classes;
  $("bCurrent").innerHTML = `<b>${esc(pv.name)}</b> (${esc(pv.source)}) · ${s.size[0]} × ${s.size[1]} mm · ${s.components} parts:
    ${CLASS_ORDER.filter(k => c[k]).map(k => `${c[k]} ${CLASS_WORD[k]}`).join(", ")} · ${s.holes} holes, ${s.fiducials} fiducials`;
  $("bThick").value = pv.thickness;
  $("bThickFrom").textContent = pv.thickness_from === "default" ? "not in the files: check it" : `from: ${pv.thickness_from}`;
}

function drawBoard(keepView = true) {
  const pv = BD.pv, svg = $("bsvg");
  const W = pv.width || 100, H = pv.height || 100;
  if (!keepView || !BD.vb) BD.vb = [-5, -5, W + 10, H + 10];
  svg.setAttribute("viewBox", BD.vb.join(" "));
  const P = pts => pts.map(p => `${p[0]},${H - p[1]}`).join(" ");
  const parts = [];
  pv.outline.forEach((lp, i) => parts.push(`<polygon class="${i ? "cut" : "outline"}" points="${P(lp)}"/>`));
  pv.holes.forEach(h => parts.push(`<circle class="hole" cx="${h.xy[0]}" cy="${H - h.xy[1]}" r="${h.d / 2}"><title>hole ${esc(h.ref)} Ø${h.d}</title></circle>`));
  const issues = pv.issue_refs || {};
  for (const c of pv.components) {
    if (c.side !== pv.side) { parts.push(`<circle class="other" cx="${c.xy[0]}" cy="${H - c.xy[1]}" r="0.4"/>`); continue; }
    const a = c.rot * Math.PI / 180, L = 0.9;
    const ex = c.xy[0] + L * Math.cos(a), ey = c.xy[1] + L * Math.sin(a);
    parts.push(`<g class="part c-${c.cls}${issues[c.ref] ? " issue" : ""}${BD.sel === c.ref ? " sel" : ""}" data-ref="${esc(c.ref)}">`
      + (c.pads || []).map(pp => `<polygon class="pad" points="${P(pp)}"/>`).join("")
      + `<line class="mark" x1="${c.xy[0]}" y1="${H - c.xy[1]}" x2="${ex}" y2="${H - ey}"/>`
      + `<circle class="mark" cx="${c.xy[0]}" cy="${H - c.xy[1]}" r="0.3"/>`
      + (c.pin1 ? `<circle class="pin1" cx="${c.pin1[0]}" cy="${H - c.pin1[1]}" r="0.22"/>` : "") + `</g>`);
  }
  svg.innerHTML = parts.join("");
  const c = pv.classes;
  $("bLegend").innerHTML = CLASS_ORDER.filter(k => c[k]).map(k => `<span class="lg ${k}"></span>${CLASS_WORD[k]} (${c[k]})`).join(" ")
    + ` <span class="lg" style="background:#ff3b30;border-radius:50%"></span>pin 1 · the white line = the part's 0° direction`;
}

function partInfo(ref) {
  const c = BD.pv.components.find(x => x.ref === ref);
  if (!c) return "";
  const iss = (BD.pv.issue_refs || {})[ref] || [];
  return `${c.ref} · ${c.value} · ${c.package || c.footprint} · ${c.side === "F" ? "top" : "bottom"} · ${c.rot}° — ${c.why}`
    + (iss.length ? ` — ⚠ ${iss.join("; ")}` : "");
}

function renderParts() {
  const f = $("bFilter").value, pv = BD.pv;
  const cnt = pv.classes;
  const cur = $("bFilter").value;
  $("bFilter").innerHTML = `<option value="">all (${pv.components.length})</option>` + CLASS_ORDER.filter(k => cnt[k])
    .map(k => `<option value="${k}"${k === cur ? " selected" : ""}>${CLASS_WORD[k]} (${cnt[k]})</option>`).join("");
  const rows = pv.components.filter(c => !f || c.cls === f).sort((a, b) =>
    (CLASS_ORDER.indexOf(a.cls) - CLASS_ORDER.indexOf(b.cls)) || a.ref.localeCompare(b.ref, undefined, {numeric: true}));
  $("bParts").innerHTML = `<tr><th>Ref</th><th>Value</th><th>Package</th><th>Side</th><th>°</th><th>What happens</th></tr>` +
    rows.map(c => `<tr data-ref="${esc(c.ref)}"${BD.sel === c.ref ? ' class="sel"' : ""}><td>${esc(c.ref)}</td><td>${esc(c.value)}</td>
      <td>${esc(c.package || c.footprint)}</td><td>${c.side === "F" ? "top" : "bottom"}</td><td>${c.rot}</td><td>${esc(CLASS_WORD[c.cls] || c.cls)}</td></tr>`).join("");
}

function renderIssues() {
  const pv = BD.pv;
  const items = [...pv.notes.map(n => ({level: "info", text: n})), ...pv.issues];
  if (pv.conventions && pv.conventions.bottom_rotation) items.unshift({level: "info", text: `Bottom angles: ${pv.conventions.bottom_rotation}`});
  $("bIssues").innerHTML = items.length ? items.map(i => `<li class="${i.level}">${esc(i.text)}</li>`).join("")
    : "<li class='info'>No problems found.</li>";
}

function selectPart(ref) {
  BD.sel = ref;
  $("bHover").textContent = partInfo(ref);
  document.querySelectorAll("#bsvg g.part").forEach(g => g.classList.toggle("sel", g.dataset.ref === ref));
  document.querySelectorAll("#bParts tr[data-ref]").forEach(tr => tr.classList.toggle("sel", tr.dataset.ref === ref));
  const c = BD.pv && BD.pv.components.find(x => x.ref === ref);
  if (c && c.side !== BD.side) { BD.side = c.side; setSideChips(); loadBoard(); }
}

function setSideChips() { document.querySelectorAll("#bSide button").forEach(b => b.classList.toggle("on", b.dataset.side === BD.side)); }

async function uploadBoard() {
  const files = [...$("bFiles").files];
  if (!files.length) { toast("Choose the board files first"); return; }
  let iid = "";
  for (const f of files) {
    const j = await getJSON(`/api/board/upload?name=${encodeURIComponent(f.name)}&import_id=${iid}`, {method: "POST", body: f});
    if (!j) return;
    iid = j.import_id;
  }
  BD.importId = iid;
  await runImport({});
}

async function runImport(extra) {
  const j = await getJSON("/api/board/import", {method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({import_id: BD.importId, ...extra})});
  if (!j) return;
  BD.meta = j;
  const lines = [`<p><b>${esc(j.kind)}</b>: ${j.board.components} parts, ${j.board.smd} SMD (${j.board.smd_top} top, ${j.board.smd_bottom} bottom), ${j.board.holes} holes · ${j.board.size[0]} × ${j.board.size[1]} mm</p>`];
  (j.fit ? j.fit.report : []).forEach(t => lines.push(`<p>${esc(t)}</p>`));
  (j.sniff ? j.sniff.unsure : []).forEach(t => lines.push(`<p class="note">⚠ ${esc(t)}</p>`));
  $("bResult").innerHTML = `<div class="report">${lines.join("")}</div>`;
  $("bSaveRow").hidden = false;
  $("bName").value = $("bName").value || (j.files[0] || "board").split(".")[0];
  $("bMapper").hidden = j.kind !== "centroid";
  if (j.kind === "centroid") fillMapper(j);
  BD.vb = null;
  await loadBoard();
}

function fillMapper(j) {
  const sp = j.spec, hdr = j.sniff.header.length ? j.sniff.header : (j.sniff.sample[0] || []).map((_, i) => `column ${i + 1}`);
  $("bTool").textContent = j.sniff.tool ? `(looks like ${j.sniff.tool})` : "";
  const roles = {ref: "Reference", x: "X", y: "Y", rotation: "Angle", side: "Side / layer", value: "Value", package: "Package"};
  $("bCols").innerHTML = Object.entries(roles).map(([r, label]) => `<label>${label}<select data-role="${r}"><option value="">—</option>`
    + hdr.map((h, i) => `<option value="${i}"${sp.columns[r] === i ? " selected" : ""}>${esc(h)}</option>`).join("") + `</select></label>`).join("");
  $("bUnits").value = sp.units; $("bYup").checked = sp.y_up; $("bNegX").checked = sp.bottom_x_negated;
  $("bRule").innerHTML = Object.entries(j.rules).map(([k, v]) => `<option value="${k}"${k === sp.bottom_rule ? " selected" : ""}>model = ${esc(v.formula)}: ${esc(v.about)}</option>`).join("");
  $("bTopOff").value = String(sp.top_offset);
  $("bOx").value = sp.origin[0]; $("bOy").value = sp.origin[1];
  BD.spec = sp;
}

function mapperSpec() {
  const cols = {};
  document.querySelectorAll("#bCols select").forEach(s => { if (s.value !== "") cols[s.dataset.role] = +s.value; });
  return {...BD.spec, columns: cols, units: $("bUnits").value, y_up: $("bYup").checked, bottom_x_negated: $("bNegX").checked,
    bottom_rule: $("bRule").value, top_offset: +$("bTopOff").value, origin: [+$("bOx").value || 0, +$("bOy").value || 0]};
}

function wireBoard() {
  $("bUpload").onclick = uploadBoard;
  $("bApply").onclick = () => runImport({spec: mapperSpec(), fit: false});
  $("bRefit").onclick = () => runImport({spec: mapperSpec(), fit: true});
  $("bSave").onclick = async () => {
    const j = await getJSON("/api/board/save", {method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({import_id: BD.importId, name: $("bName").value})});
    if (!j) return;
    toast(`Board ${j.name} is now the current board`);
    BD.importId = null; $("bSaveRow").hidden = true; $("bMapper").hidden = true; $("bResult").innerHTML = ""; BD.vb = null;
    loadBoard();
  };
  $("bDiscard").onclick = () => { BD.importId = null; $("bSaveRow").hidden = true; $("bMapper").hidden = true; $("bResult").innerHTML = ""; BD.vb = null; loadBoard(); };
  $("bThickSet").onclick = async () => {
    if (await getJSON("/api/board/thickness", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({mm: $("bThick").value})})) loadBoard();
  };
  $("bUse").onclick = async () => {
    if (await getJSON("/api/board/use", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({name: $("bNames").value})})) { BD.vb = null; loadBoard(); }
  };
  document.querySelectorAll("#bSide button").forEach(b => b.onclick = () => { BD.side = b.dataset.side; setSideChips(); loadBoard(); });
  $("bFit").onclick = () => { BD.vb = null; if (BD.pv) drawBoard(false); };
  $("bFilter").onchange = renderParts;
  $("bParts").onclick = e => { const tr = e.target.closest("tr[data-ref]"); if (tr) selectPart(tr.dataset.ref); };
  const svg = $("bsvg");
  svg.addEventListener("mouseover", e => { const g = e.target.closest("g.part"); if (g) $("bHover").textContent = partInfo(g.dataset.ref); });
  svg.addEventListener("click", e => { const g = e.target.closest("g.part"); if (g && !BD.moved) selectPart(g.dataset.ref); });
  svg.addEventListener("wheel", e => {
    if (!BD.vb) return;
    e.preventDefault();
    const r = svg.getBoundingClientRect(), k = e.deltaY > 0 ? 1.2 : 1 / 1.2;
    const [x, y, w, h] = BD.vb, fx = (e.clientX - r.left) / r.width, fy = (e.clientY - r.top) / r.height;
    BD.vb = [x + fx * w * (1 - k), y + fy * h * (1 - k), w * k, h * k];
    svg.setAttribute("viewBox", BD.vb.join(" "));
  }, {passive: false});
  svg.addEventListener("pointerdown", e => { BD.drag = [e.clientX, e.clientY, ...BD.vb]; BD.moved = false; });
  svg.addEventListener("pointermove", e => {
    if (!BD.drag) return;
    const r = svg.getBoundingClientRect(), [sx, sy, x, y, w, h] = BD.drag;
    const dx = (e.clientX - sx) / r.width * w, dy = (e.clientY - sy) / r.height * h;
    if (Math.abs(e.clientX - sx) + Math.abs(e.clientY - sy) > 3) BD.moved = true;
    BD.vb = [x - dx, y - dy, w, h];
    svg.setAttribute("viewBox", BD.vb.join(" "));
  });
  window.addEventListener("pointerup", () => { BD.drag = null; });
}

wire();
wireBoard();
restartVideo();
connectWS();
showTab(location.hash.slice(1) || "machine");
setInterval(refreshMap, 700);

// the first-part picture of a tray being loaded (the same flow as Setup step 10)
async function pollCandidate(n = 0) {
  try {
    const w = (await getJSON("/api/wizard")).steps.find(s => s.id === "first_part");
    const c = w && w.info && w.info.candidate;
    if (c) { $("lCand").hidden = false; $("lCandText").textContent = `Tray ${c.slot}: is this a part?`;
             $("lCandImg").src = `${c.image}?t=${Date.now()}`; return; }
  } catch (e) { /* again */ }
  if (n < 40) setTimeout(() => pollCandidate(n + 1), 500);
}
