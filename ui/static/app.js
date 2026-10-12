// Pick and place page: polls /api/status, sends the buttons to /api/action/<name>.
"use strict";
const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));
let S = null;            // last status
let linesLoaded = false;

async function post(url, body) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" },
                                 body: JSON.stringify(body || {}) });
    const j = await r.json();
    if (j && j.ok === false && j.why) say(j.why);
    if (!r.ok && j && j.detail) say(String(j.detail));
    return j;
  } catch (err) { say("No answer from the machine program: " + err); return null; }
}
const act = (name, body) => post("/api/action/" + name, body);
function say(text) { $("#msg").textContent = text; }

// ---- tabs
$$("#tabs button").forEach((b) => b.onclick = () => {
  $$("#tabs button").forEach((x) => x.classList.toggle("on", x === b));
  $$(".tab").forEach((t) => t.classList.add("hidden"));
  $("#tab-" + b.dataset.tab).classList.remove("hidden");
  if (b.dataset.tab === "feeders") loadLines();
  if (b.dataset.tab === "job") loadPictures();
});

// ---- buttons
$$("[data-act]").forEach((b) => b.onclick = () => act(b.dataset.act));
$$("[data-jog]").forEach((b) => b.onclick = () => {
  const [x, y] = b.dataset.jog.split(",").map(Number), s = Number($("#step").value);
  act("jog", { dx: x * s, dy: y * s });
});
$$("[data-z]").forEach((b) => b.onclick = () => act("jog_z", { dz: Number(b.dataset.z) }));
$$("[data-height]").forEach((b) => b.onclick = () => act("height", { name: b.dataset.height }));
$$("[data-turn]").forEach((b) => b.onclick = () => act("turn", { delta: Number(b.dataset.turn) }));
$("#turn0").onclick = () => act("turn_to", { deg: 0 });
$$("[data-land]").forEach((b) => b.onclick = () => act("land", { surface: b.dataset.land }));
$$("[data-spot]").forEach((b) => b.onclick = () => act("spot", { name: b.dataset.spot }));
$$("[data-nudge]").forEach((b) => b.onclick = () => {
  const [x, y] = b.dataset.nudge.split(",").map(Number), st = Number($("#nudge").value);
  act("pointer_nudge", { dx: x * st, dy: y * st });
});
$$("[data-corner-go]").forEach((b) => b.onclick = () => act("corner_go", { hole: b.dataset.cornerGo }));
$$("[data-corner-set]").forEach((b) => b.onclick = () => act("corner_set", { hole: b.dataset.cornerSet }));
$("#stop").onclick = () => post("/api/stop");
$("#quit").onclick = () => {
  if (confirm("Quit? The nozzle goes up first, then the program closes.")) post("/api/quit").then(() => say("Closed."));
};
$("#unlock").onclick = () => {
  if (confirm("Is the nozzle tip clear of the board, the parts and the labels?\n\nThen the machine lifts Z a " +
              "little more, finds HOME and brings the camera back to its height.")) act("home");
};
$("#camheight").onclick = () => {
  if (confirm("Is the nozzle tip about 5 mm ABOVE the parts and the board?\n\nThis height becomes the camera " +
              "height. Heights and offsets set before are not used any more.")) act("camera_height");
};
$("#sidebtn").onclick = () => post("/api/side", { side: $("#side").value, top_edge: $("#top").value })
  .then(() => loadLines());
$("#runbtn").onclick = () => {
  const mode = document.querySelector("input[name=mode]:checked").value;
  if (mode === "real" && !confirm("REAL pick and place: the nozzle goes down and the pump runs.\nWatch the " +
                                  "machine, keep a hand near STOP. Start?")) return;
  act("run", { mode });
};
$("#onebtn").onclick = () => {
  if (confirm("REAL: pick and place ONE part, then stop. Watch the machine, hand near STOP. Start?")) act("run_one");
};
$("#pausebtn").onclick = () => post("/api/pause");
$("#resumebtn").onclick = () => post("/api/resume");
$("#newboard").onclick = () => {
  if (confirm("A NEW board: forget which parts are already placed, and register the board again?")) act("new_board");
};

// click in the picture: that spot goes to the red cross
$("#video").addEventListener("click", (ev) => {
  const img = ev.currentTarget, r = img.getBoundingClientRect();
  const u = (ev.clientX - r.left) * img.naturalWidth / r.width, v = (ev.clientY - r.top) * img.naturalHeight / r.height;
  act("click", { u, v });
});

// keys (not while typing in a field)
document.addEventListener("keydown", (ev) => {
  if (["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName)) return;
  const s = Number($("#step").value);
  const jog = { ArrowLeft: [-s, 0], ArrowRight: [s, 0], ArrowUp: [0, s], ArrowDown: [0, -s] }[ev.key];
  if (jog) { act("jog", { dx: jog[0], dy: jog[1] }); ev.preventDefault(); return; }
  const z = { "[": -0.1, "]": 0.1, "{": -1, "}": 1 }[ev.key];
  if (z !== undefined) { act("jog_z", { dz: z }); ev.preventDefault(); return; }
  if (ev.key === " ") { post("/api/stop"); ev.preventDefault(); return; }
  if (ev.key === ",") { act("turn", { delta: -10 }); return; }
  if (ev.key === ".") { act("turn", { delta: 10 }); return; }
  if (ev.key === "h") act("home");
});

// ---- feeders
async function loadLines() {
  const r = await fetch("/api/lines"); const L = await r.json();
  $("#f-side").textContent = L.side + " side up";
  $("#side").value = L.side; $("#top").value = L.top_edge;
  const ok = L.lines.filter((g) => g.supported);
  const rows = L.slots.map((slot) => {
    const a = L.assigned[String(slot)];
    const opts = ['<option value="">(empty)</option>'].concat(ok.map((g) =>
      `<option value="${g.key}" ${a && a.line === g.key ? "selected" : ""}>${g.label}</option>`));
    return `<tr><td><b>Slot ${slot}</b></td><td><select data-slot="${slot}">${opts.join("")}</select></td>` +
           `<td>${a ? (a.side !== L.side ? "(other side)" : "") : ""}</td></tr>`;
  });
  $("#slots").innerHTML = rows.join("");
  $$("#slots select").forEach((sel) => sel.onchange = async () => {
    const j = await post("/api/feeder", { slot: Number(sel.dataset.slot), key: sel.value || null });
    if (j && j.ok) say(`Slot ${sel.dataset.slot}: ${sel.value ? sel.options[sel.selectedIndex].text : "empty"}`);
    loadLines();
  });
  $("#manual").innerHTML = L.lines.filter((g) => !g.supported)
    .map((g) => `<li>${g.label} (${g.values.join(", ")}): ${g.why}. ${g.refs.join(" ")}</li>`).join("");
  linesLoaded = true;
}

async function loadPictures() {
  const r = await fetch("/api/pictures"); const names = await r.json();
  $("#pics").innerHTML = names.map((n) => `<a href="/pictures/${n}" target="_blank"><img src="/pictures/${n}" title="${n}"></a>`).join("");
}

// ---- status
function chip(text, cls) { return `<span class="chip ${cls || ""}">${text}</span>`; }
const mm = (v) => (v === null || v === undefined ? "-" : (v >= 0 ? "+" : "") + v.toFixed(2));

function show(s) {
  S = s;
  const ch = [];
  ch.push(chip(s.homed ? "HOME ✔" : "HOME ✗", s.homed ? "ok" : "bad"));
  ch.push(chip(s.camera_height.checked ? "camera height ✔" : (s.camera_height.set ? "camera height: press Home" : "camera height: not set"),
               s.camera_height.checked ? "ok" : "warn"));
  ch.push(chip(s.reach.stale ? "reach: OLD" : "reach ✔", s.reach.stale ? "warn" : "ok"));
  ch.push(chip(s.board.registered ? `board ${s.board.side} ✔` : `board ${s.board.side}: not registered`, s.board.registered ? "ok" : "warn"));
  ch.push(chip(s.pump ? "PUMP ON" : "pump off", s.pump ? "warn" : ""));
  ch.push(chip(s.busy ? (s.moving ? "MOVING" : "BUSY") : "ready", s.busy ? "warn" : "ok"));
  if (s.run.state !== "idle") ch.push(chip("JOB: " + s.run.state, "warn"));
  $("#chips").innerHTML = ch.join("");
  $("#lockbox").classList.toggle("hidden", !s.locked);
  if (s.locked) $("#locked").textContent = "LOCKED: " + s.locked + ". Look at the nozzle: if it is low, press Z up " +
    "(Machine tab) until it is clear of the board and the parts. Then press the button: ";
  if (s.msg) $("#msg").textContent = s.msg;
  $("#events").innerHTML = s.events.slice().reverse().map(([tm, t]) => `<li>${tm} <b>${escapeHtml(t)}</b></li>`).join("");
  $("#px").textContent = mm(s.pos[0]); $("#py").textContent = mm(s.pos[1]); $("#pz").textContent = mm(s.z);
  $("#pumpbtn").textContent = s.pump ? "Pump OFF" : "Pump ON";
  $("#turn").textContent = (s.turn_deg >= 0 ? "+" : "") + s.turn_deg;
  document.body.classList.toggle("landmode", s.click_mode === "land");
  $("#v-land").textContent = s.click_mode === "land"
    ? `Landed (${s.land_surface === "part" ? "pick" : "place"} height): now CLICK the mark / the part in the picture` : "";
  drawMap();

  // setup cards
  const done = (id, ok, attn) => { const c = $(id); c.classList.toggle("done", !!ok); c.classList.toggle("attn", !ok && !!attn); };
  done("#c-reach", !s.reach.stale && !s.reach.measuring, s.reach.stale);
  $("#v-reach").textContent = s.reach.measuring
    ? `MEASURING. Driven so far: X ${s.reach.extremes ? s.reach.extremes[0] + " … " + s.reach.extremes[1] : "-"}, ` +
      `Y ${s.reach.extremes ? s.reach.extremes[2] + " … " + s.reach.extremes[3] : "-"} mm from HOME`
    : (s.reach.x ? `X ${s.reach.x[0]} … ${s.reach.x[1]}, Y ${s.reach.y[0]} … ${s.reach.y[1]} mm from HOME` +
       (s.reach.stale ? "  (OLD: measured before the camera moved)" : "") : "");
  done("#c-cam", s.camera_height.checked, !s.camera_height.set);
  $("#v-cam").textContent = s.camera_height.checked ? "Set and checked: Z0 = the camera height" :
                            (s.camera_height.set ? "Remembered: press Home to bring Z back to it" : "Not set yet");
  const H = s.heights;
  ["part", "place", "move", "paper"].forEach((n) => $("#h-" + n).textContent = H[n] === null ? "-" : mm(H[n]));
  done("#c-heights", H.part !== null && H.place !== null && H.move !== null, s.camera_height.checked);
  const op = s.offsets.offset_pick, ol = s.offsets.offset_place;
  done("#c-offset", op && op.ok && ol && ol.ok);
  $("#offsetbtn").textContent = s.offset_step === 2
    ? `2. The NOZZLE tip is exactly over the ${s.offset_kind === "hole" ? "hole" : (s.offset_kind || "target")}`
    : "1. Mark what is at the red cross (a part, or a board hole)";
  const fo = (o) => (o ? `X ${mm(o.xy[0])}  Y ${mm(o.xy[1])} mm${o.ok ? "" : " (other camera height: do it again)"}` : "not set");
  $("#v-offset").textContent = `Pick offset (on a part): ${fo(op)}\nPlace offset (on a hole): ${fo(ol)}`;
  done("#c-board", s.board.registered);
  ["H1", "H2", "H3", "H4"].forEach((h) => $("#cs-" + h).textContent = s.board.corners.includes(h) ? "✔ set" : "");
  const bi = s.board.info || {};
  $("#v-board").textContent = s.board.registered
    ? `Registered (${bi.by || ""}): angle ${bi.angle_deg} deg, worst hole ${bi.residuals_mm ? Math.max(...bi.residuals_mm).toFixed(2) : "-"} mm`
    : "Not registered";

  // job
  const r = s.run, tot = r.total || 0, dn = r.done || 0;
  $("#bar").style.width = (tot ? Math.round(100 * dn / tot) : 0) + "%";
  $("#v-job").textContent = (r.state === "idle" && !tot) ? `Parts already on this board: ${s.placed}` :
    `${r.state.toUpperCase()}  ${r.mode || ""}  ${dn} of ${tot} placed` +
    (r.slot ? `\nNow: slot ${r.slot}, ${r.line}` : "") + (r.skipped ? `\nSkipped: ${r.skipped}` : "") +
    `\nParts on this board: ${s.placed}`;
}

function escapeHtml(t) { return String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c])); }

async function poll() {
  try { const r = await fetch("/api/status"); if (r.ok) show(await r.json()); }
  catch (err) { say("The machine program does not answer (closed?)"); }
  setTimeout(poll, 400);
}
poll();
setInterval(() => { if (!$("#tab-job").classList.contains("hidden")) loadPictures(); }, 5000);

// ---- bed map (mm from HOME, +X right, +Y back = up)
let MAP = null, view = null;
async function loadMap() {
  try { const r = await fetch("/api/map"); if (r.ok) MAP = await r.json(); } catch (err) { /* keep the old one */ }
  drawMap();
}
setInterval(loadMap, 3000); loadMap();

function mapBounds() {
  const pts = [];
  if (MAP.reach) { pts.push([MAP.reach.x[0], MAP.reach.y[0]], [MAP.reach.x[1], MAP.reach.y[1]]); }
  Object.values(MAP.labels).forEach(([x, y]) => pts.push([x - 12, y - 12], [x + 12, y + 12]));
  if (MAP.pcb_area) pts.push([MAP.pcb_area[0], MAP.pcb_area[1]], [MAP.pcb_area[2], MAP.pcb_area[3]]);
  if (MAP.board) MAP.board.forEach((p) => pts.push(p));
  if (S && S.homed) pts.push(S.pos);
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  return [Math.min(...xs) - 8, Math.min(...ys) - 8, Math.max(...xs) + 8, Math.max(...ys) + 8];
}

function drawMap() {
  const cv = $("#map"), g = cv.getContext("2d");
  g.fillStyle = "#101316"; g.fillRect(0, 0, cv.width, cv.height);
  if (!MAP) return;
  const [x0, y0, x1, y1] = mapBounds();
  const k = Math.min(cv.width / (x1 - x0), cv.height / (y1 - y0));
  const ox = (cv.width - k * (x1 - x0)) / 2, oy = (cv.height - k * (y1 - y0)) / 2;
  view = { x0, y1, k, ox, oy };
  const P = (x, y) => [ox + (x - x0) * k, oy + (y1 - y) * k];
  g.lineWidth = 1;
  if (MAP.reach) {                                   // where the camera centre can go
    g.setLineDash([6, 4]); g.strokeStyle = "#6b7785";
    const [a, b] = P(MAP.reach.x[0], MAP.reach.y[1]), [c, d] = P(MAP.reach.x[1], MAP.reach.y[0]);
    g.strokeRect(a, b, c - a, d - b); g.setLineDash([]);
    g.fillStyle = "#6b7785"; g.font = "12px sans-serif"; g.fillText("reach", a + 4, b + 14);
  }
  if (MAP.pcb_area) {
    const [a, b] = P(MAP.pcb_area[0], MAP.pcb_area[3]), [c, d] = P(MAP.pcb_area[2], MAP.pcb_area[1]);
    g.fillStyle = "#1b2a22"; g.fillRect(a, b, c - a, d - b);
  }
  if (MAP.board) {
    g.strokeStyle = "#39c06b"; g.lineWidth = 2; g.beginPath();
    MAP.board.forEach(([x, y], i) => { const [a, b] = P(x, y); i ? g.lineTo(a, b) : g.moveTo(a, b); });
    g.closePath(); g.stroke(); g.lineWidth = 1;
  }
  MAP.parts.forEach(([x, y, ref, st]) => {
    const [a, b] = P(x, y);
    g.fillStyle = st === 2 ? "#39c06b" : (st === 1 ? "#f0b429" : "#4a5562");
    g.fillRect(a - 2, b - 2, 4, 4);
  });
  Object.entries(MAP.holes).forEach(([h, [x, y]]) => {
    const [a, b] = P(x, y), set = S && S.board.corners.includes(h);
    g.strokeStyle = set ? "#39c06b" : "#22a3ff"; g.beginPath(); g.arc(a, b, Math.max(4, 1.6 * k), 0, 7); g.stroke();
    g.fillStyle = g.strokeStyle; g.font = "12px sans-serif"; g.fillText(h, a + 6, b - 6);
  });
  Object.entries(MAP.labels).forEach(([n, [x, y]]) => {
    const [a, b] = P(x - MAP.marker_mm / 2, y + MAP.marker_mm / 2), s = MAP.marker_mm * k;
    g.fillStyle = n === "home" ? "#2f6f4a" : "#3b4652"; g.fillRect(a, b, s, s);
    g.fillStyle = "#dfe6ec"; g.font = "13px sans-serif";
    g.fillText(n === "home" ? "HOME" : n.replace("slot_", "S"), a, b - 4);
  });
  if (S && S.homed) {
    if (S.nozzle) {                                   // where the nozzle comes down
      const [a, b] = P(S.nozzle[0], S.nozzle[1]);
      g.strokeStyle = "#22d3ee"; g.lineWidth = 2; g.beginPath(); g.arc(a, b, 6, 0, 7); g.stroke();
    }
    const [a, b] = P(S.pos[0], S.pos[1]);            // the camera centre (red cross)
    g.strokeStyle = "#ff4d4d"; g.lineWidth = 2; g.beginPath();
    g.moveTo(a - 9, b); g.lineTo(a + 9, b); g.moveTo(a, b - 9); g.lineTo(a, b + 9); g.stroke(); g.lineWidth = 1;
  } else {
    g.fillStyle = "#f0b429"; g.font = "16px sans-serif"; g.fillText("Press Home: the map is in mm from HOME", 14, 24);
  }
}

function mapMM(ev) {
  const cv = $("#map"), r = cv.getBoundingClientRect();
  const px = (ev.clientX - r.left) * cv.width / r.width, py = (ev.clientY - r.top) * cv.height / r.height;
  return [view.x0 + (px - view.ox) / view.k, view.y1 - (py - view.oy) / view.k];
}
$("#map").addEventListener("mousemove", (ev) => {
  if (!view) return;
  const [x, y] = mapMM(ev);
  $("#mapxy").textContent = `X ${x.toFixed(1)}  Y ${y.toFixed(1)} mm from HOME`;
});
$("#map").addEventListener("click", (ev) => {
  if (!view) return;
  const [x, y] = mapMM(ev);
  act("goto", { x, y });
});
