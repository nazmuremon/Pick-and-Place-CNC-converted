// The Hand tab: Solder Helper merged into the program (docs/13 §4.15). The same Board model and placed state as the
// machine (/api/store/hand), the drawers and bag links on the server (shared by every browser), labels through the
// profile's label printer. Plain JS, no libraries except the shipped QR decoders (/vendor, offline).
"use strict";

const H = {pass: "F", view: "F", data: null, pv: null, sel: null, q: "", drawers: null};

async function onHandTab() { await loadHand(); }
window.onHandTab = onHandTab;

async function loadHand() {
  let d;
  try { d = await getRaw("/api/store/hand"); } catch (e) { return; }
  if (!d.ok) { $("hList").innerHTML = `<li>${esc(d.why)}. ${esc(d.todo || "")}</li>`; return; }
  H.data = d;
  if (!H.pv || H.pv.side !== H.view) {
    const j = await getRaw(`/api/board?side=${H.view}`);
    H.pv = j && j.board;
  }
  renderHand();
}

function groups() {
  const p = H.data.passes[H.pass];
  const q = H.q.trim().toLowerCase();
  if (!q) return p.groups;
  const dm = /^d(\d+)$/.exec(q);
  return p.groups.filter(g => dm ? g.drawer === +dm[1] :
    [g.value, g.footprint, g.feeder_key, ...g.refs.map(r => r.ref)].join(" ").toLowerCase().includes(q));
}

function renderHand() {
  const p = H.data.passes[H.pass];
  $("hBar").style.width = `${p.total ? (100 * p.done / p.total).toFixed(1) : 0}%`;
  $("hProg").textContent = `${p.name}: ${p.done} of ${p.total} placed`;
  document.querySelectorAll("#hPass button").forEach(b => b.classList.toggle("on", b.dataset.pass === H.pass));
  $("hFlip").textContent = `View: ${H.view === "F" ? "top" : "bottom"}`;
  const gs = groups();
  if (H.sel && !gs.some(g => g.key === H.sel)) H.sel = null;
  if (!H.sel) H.sel = (gs.find(g => g.refs.some(r => !r.placed)) || gs[0] || {}).key || null;
  $("hList").innerHTML = gs.map(g => {
    const done = g.refs.filter(r => r.placed).length;
    return `<li class="hitem${g.key === H.sel ? " sel" : ""}${done === g.refs.length ? " done" : ""}" data-key="${esc(g.key)}">
      <b>${esc(g.value)}</b> <small>${esc(g.footprint)}</small> <span class="st">drawer ${g.drawer ?? "–"}</span>
      <span class="sub">${done}/${g.refs.length} · ${g.refs.map(r => esc(r.ref)).join(" ")}</span></li>`;
  }).join("") || "<li>Nothing in this pass.</li>";
  const li = document.querySelector("#hList .sel");
  if (li) li.scrollIntoView({block: "nearest"});
  drawHand();
  renderHandCard();
}

function renderHandCard() {
  const g = H.data && groups().find(x => x.key === H.sel);
  $("hCard").hidden = !g;
  if (!g) return;
  $("hCard").innerHTML = `<b>${esc(g.value)}</b> · ${esc(g.footprint)} · drawer <b>${g.drawer ?? "–"}</b>
    <div class="sub">${g.refs.map(r => `${esc(r.ref)}${r.placed ? ` ✓ (${esc(r.by || "")})` : ""}${r.why ? ` — ${esc(r.why)}` : ""}`).join("<br>")}</div>
    <div class="row"><button id="hDone">Placed (Enter)</button><button id="hUndo">Not placed</button></div>`;
  $("hDone").onclick = () => markSel(true, true);
  $("hUndo").onclick = () => markSel(false, false);
}

function drawHand() {
  const pv = H.pv, svg = $("hsvg");
  if (!pv) { svg.innerHTML = ""; return; }
  const W = pv.width || 100, Hh = pv.height || 100;
  svg.setAttribute("viewBox", `-5 -5 ${W + 10} ${Hh + 10}`);
  const P = pts => pts.map(p => `${p[0]},${Hh - p[1]}`).join(" ");
  const g = H.data && groups().find(x => x.key === H.sel);
  const lit = new Set(g ? g.refs.map(r => r.ref) : []);
  const placed = new Set();
  for (const ps of Object.values(H.data.passes)) for (const gg of ps.groups) for (const r of gg.refs) if (r.placed) placed.add(r.ref);
  const out = pv.outline.map((lp, i) => `<polygon class="${i ? "cut" : "outline"}" points="${P(lp)}"/>`);
  for (const c of pv.components) {
    if (!c.pads) continue;
    const cls = lit.has(c.ref) ? "part hl" : placed.has(c.ref) ? "part placed" : "part";
    out.push(`<g class="${cls}" data-ref="${esc(c.ref)}">` + c.pads.map(pp => `<polygon class="pad" points="${P(pp)}"/>`).join("")
      + (c.pin1 && lit.has(c.ref) ? `<circle class="pin1" cx="${c.pin1[0]}" cy="${Hh - c.pin1[1]}" r="0.3"/>` : "") + `</g>`);
  }
  svg.innerHTML = out.join("");
}

async function markSel(on, next) {
  const g = groups().find(x => x.key === H.sel);
  if (!g) return;
  await api("hand_mark", {refs: g.refs.map(r => r.ref), placed: on});
  await loadHand();
  if (next) {
    const gs = groups(), i = gs.findIndex(x => x.key === g.key);
    const nxt = gs.slice(i + 1).find(x => x.refs.some(r => !r.placed)) || gs.find(x => x.refs.some(r => !r.placed));
    if (nxt) { H.sel = nxt.key; renderHand(); }
  }
}

function moveSel(d) {
  const gs = groups(), i = gs.findIndex(x => x.key === H.sel);
  const n = gs[Math.max(0, Math.min(gs.length - 1, i + d))];
  if (n) { H.sel = n.key; renderHand(); }
}

function handKey(e) {
  const k = e.key;
  const idle = S && !S.busy && !(S.job && S.job.status === "running");
  if (k === " " && idle) { e.preventDefault(); const g = groups().find(x => x.key === H.sel);
    if (g) markSel(!g.refs.every(r => r.placed), false); return true; }
  if (k === "Enter") { e.preventDefault(); markSel(true, true); return true; }
  if (k === "ArrowDown") { e.preventDefault(); moveSel(1); return true; }
  if (k === "ArrowUp") { e.preventDefault(); moveSel(-1); return true; }
  if (k === "1" || k === "2" || k === "3") { H.pass = {1: "F", 2: "B", 3: "T"}[k]; H.view = H.pass === "T" ? H.view : H.pass;
    H.sel = null; loadHand(); return true; }
  if (k === "f" || k === "F") { flip(); return true; }
  if (k === "s" || k === "S") { openScan(); return true; }
  if (k === "d" || k === "D") { openDrawers(); return true; }
  return false;
}
window.handKey = handKey;

function flip() { H.view = H.view === "F" ? "B" : "F"; H.pv = null; loadHand(); }

// ---------------------------------------------------------------- scanning (typed, USB scanner, camera QR)
function openScan() { $("hScan").hidden = false; setTimeout(() => $("hScanIn").focus(), 30); }
async function doScan(text) {
  const r = await fetch("/api/action/scan", {method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({text})}).then(x => x.json()).catch(() => ({ok: false, why: "no answer"}));
  if (current === "hand") $("hScan").hidden = false;
  const out = $("hScanOut");
  if (!r.ok) {
    out.innerHTML = `${esc(r.why)}${r.code && H.sel ? ` <button id="hLink">Link it to the selected line</button>` : ""}`;
    const b = $("hLink"); if (b) b.onclick = () => api("link", {text, key: H.sel}).then(() => doScan(text));
    return;
  }
  const ln = r.line;
  out.innerHTML = `<b>${esc(r.where)}</b> · ${esc(ln.value)} ${esc(ln.footprint)} · need ${r.need} for this board`
    + (r.code && r.code.qty ? ` · the bag has ${r.code.qty}` : "");
  H.q = ""; $("hSearch").value = ""; H.sel = ln.key;
  for (const [ps, p] of Object.entries(H.data ? H.data.passes : {})) if (p.groups.some(g => g.key === ln.key)) { H.pass = ps; break; }
  if (H.data) renderHand();
}
window.doScan = doScan;

// a USB / Bluetooth barcode scanner types fast and ends with Enter: catch it anywhere on the Hand and Feeders tabs
const wedge = {buf: "", t: 0};
window.scanWedge = function (e) {
  if (!["hand", "feeders"].includes(current) || e.target.matches("input, textarea, select")) return false;
  const now = performance.now();
  if (now - wedge.t > 60) wedge.buf = "";
  wedge.t = now;
  if (e.key === "Enter" && wedge.buf.length >= 4) {
    const text = wedge.buf; wedge.buf = ""; e.preventDefault();
    if (current === "feeders" && $("lTray").value) api("scan", {text, tray: $("lTray").value}).then(r => { if (r.ok) toast(`${r.where}: ${r.line.value} ${r.line.footprint}`); });
    else doScan(text);
    return true;
  }
  if (e.key.length === 1) { wedge.buf += e.key; if (wedge.buf.length > 1) { e.preventDefault(); return true; } }
  return false;
};

let cam = null, zx = null;
function zxing() {
  if (zx !== null) return zx;
  if (!window.ZXingWASM) return (zx = false);
  try {
    ZXingWASM.setZXingModuleOverrides({locateFile: (p, prefix) => p.endsWith(".wasm") ? "/vendor/" + p : prefix + p});
    zx = ZXingWASM.prepareZXingModule({fireImmediately: true}).then(() => true, () => (zx = false));
  } catch (e) { zx = false; }
  return zx;
}
async function decode(img) {
  const z = zxing();
  if (z && (await z)) {
    try { const r = await ZXingWASM.readBarcodes(img, {formats: ["QRCode"], tryHarder: true}); if (r[0] && r[0].text) return r[0].text; } catch (e) { /* jsQR */ }
  }
  return window.jsQR ? ((jsQR(img.data, img.width, img.height, {inversionAttempts: "attemptBoth"}) || {}).data || null) : null;
}
async function startCam() {
  if (cam) { stopCam(); return; }
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) { toast("The camera needs http://localhost or https"); return; }
  try {
    const id = $("hCamSel").value;
    const stream = await navigator.mediaDevices.getUserMedia({video: id ? {deviceId: {exact: id}} : {facingMode: "environment"}, audio: false});
    const v = $("hVid"); v.hidden = false; v.srcObject = stream; await v.play();
    const devs = (await navigator.mediaDevices.enumerateDevices()).filter(d => d.kind === "videoinput");
    $("hCamSel").innerHTML = devs.map((d, i) => `<option value="${esc(d.deviceId)}">${esc(d.label || "Camera " + (i + 1))}</option>`).join("");
    cam = {stream, cv: document.createElement("canvas"), last: "", lastT: 0, busy: false};
    $("hCam").textContent = "Stop camera";
    tick();
  } catch (e) { toast("Camera: " + e.message, "", "warn"); }
}
function stopCam() { if (cam) { cam.stream.getTracks().forEach(t => t.stop()); cancelAnimationFrame(cam.raf); cam = null; }
  $("hVid").hidden = true; $("hCam").textContent = "Camera"; }
async function tick() {
  if (!cam) return;
  const v = $("hVid");
  if (!cam.busy && v.readyState >= 2 && v.videoWidth) {
    cam.busy = true;
    const c = cam.cv, k = Math.min(1, 1280 / v.videoWidth);
    c.width = Math.round(v.videoWidth * k); c.height = Math.round(v.videoHeight * k);
    const g = c.getContext("2d", {willReadFrequently: true});
    g.drawImage(v, 0, 0, c.width, c.height);
    try {
      const text = await decode(g.getImageData(0, 0, c.width, c.height));
      if (text && !(text === cam.last && Date.now() - cam.lastT < 3000)) { cam.last = text; cam.lastT = Date.now(); doScan(text); }
    } catch (e) { /* next frame */ }
    if (cam) cam.busy = false;
  }
  if (cam) cam.raf = requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- drawers and labels
async function openDrawers() {
  $("hDrawers").hidden = false;
  const d = await getRaw("/api/store/drawers");
  if (!d.ok) { $("hDList").textContent = d.why; return; }
  $("hDCount").value = d.count;
  const opts = n => Array.from({length: d.count}, (_, i) => `<option${i + 1 === n ? " selected" : ""}>${i + 1}</option>`).join("");
  $("hDList").innerHTML = d.drawers.map(dr => `<div class="winfo"><b>Drawer ${dr.n}</b><br>` + dr.lines.map(l =>
    `${esc(l.value)} <small>${esc(l.footprint)}</small> (${l.count}${l.qty ? `, ${l.qty} in the bag` : ""})
     <select data-move="${esc(l.key)}">${opts(dr.n)}</select>`).join("<br>") + `</div>`).join("");
}
async function printLabels(which) {
  const r = await api("print_labels", {which, mode: $("hPrintMode").value});
  if (r.ok) $("hDOut").innerHTML = `Printed ${r.printed} labels` + (r.file ? ` · <a href="/api/labels/file/${encodeURIComponent(r.file)}" target="_blank">the PDF</a>` : "");
}

// ---------------------------------------------------------------- the old Solder Helper progress (localStorage), once
async function importOld() {
  const name = H.data ? H.data.board : "";
  const keys = Object.keys(localStorage).filter(k => k.startsWith("solder-helper:") && !k.includes(":", 14));
  const key = keys.find(k => k.endsWith(name)) || keys[0];
  if (!key) { toast("No Solder Helper progress in this browser (open this page in the browser you used it in)"); return; }
  const get = k => { try { return JSON.parse(localStorage.getItem(k) || "null"); } catch (e) { return null; } };
  const data = {placed: get(key) || [], drawers: get(key + ":drawers") || {}, links: get(key + ":lcsc-links") || {}};
  const r = await api("import_solder_helper", {data});
  if (r.ok) { toast(`Imported ${r.placed} placed marks`); loadHand(); }
}

document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll("#hPass button").forEach(b => b.onclick = () => { H.pass = b.dataset.pass; if (H.pass !== "T") H.view = H.pass; H.pv = null; H.sel = null; loadHand(); });
  $("hSearch").oninput = e => { H.q = e.target.value; renderHand(); };
  $("hList").onclick = e => { const li = e.target.closest("[data-key]"); if (li) { H.sel = li.dataset.key; renderHand(); } };
  $("hsvg").onclick = e => { const g = e.target.closest("[data-ref]"); if (!g || !H.data) return;
    for (const [ps, p] of Object.entries(H.data.passes)) { const gg = p.groups.find(x => x.refs.some(r => r.ref === g.dataset.ref));
      if (gg) { H.pass = ps; H.sel = gg.key; renderHand(); return; } } };
  $("hFlip").onclick = flip;
  $("hScanBtn").onclick = openScan;
  $("hScanClose").onclick = () => { $("hScan").hidden = true; stopCam(); };
  $("hScanIn").onkeydown = e => { if (e.key === "Enter") { doScan(e.target.value); e.target.select(); } };
  $("hCam").onclick = startCam;
  $("hDrawersBtn").onclick = openDrawers;
  $("hDrawersClose").onclick = () => { $("hDrawers").hidden = true; };
  $("hDCountSet").onclick = () => api("drawer_count", {count: $("hDCount").value}).then(() => { openDrawers(); loadHand(); });
  $("hDList").onchange = e => { const s = e.target.closest("[data-move]"); if (s) api("drawer_move", {key: s.dataset.move, drawer: s.value}).then(() => { openDrawers(); loadHand(); }); };
  $("hPrintD").onclick = () => printLabels("drawers");
  $("hPrintP").onclick = () => printLabels("parts");
  $("hImport").onclick = importOld;
  setInterval(() => { if (current === "hand") loadHand(); }, 4000);    // the machine marks parts too
});
