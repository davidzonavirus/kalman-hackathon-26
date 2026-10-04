"use strict";
// Ground speed dashboard UI. No dependencies; talks to gsdash.server via SSE + JSON.
// Accepts v1 and v2 frames (v2 adds ax, ay, gz, flow_vx, flow_vy, net_forward).

const BIT = { IMU_OK: 1, FLOW_OK: 2, GNSS_OK: 4, LIDAR_OK: 8, RECORDING: 16, ZUPT: 32,
  FLOW_GATED: 64, GNSS_GATED: 128, TORCH_ON: 256, FILTER_INIT: 512, CALIBRATING: 1024 };
const NO_SIGNAL_S = 1.0;
const MAX_PTS = 45000;            // 15 min at 50 Hz kept in the browser
const FT = 0.3048;
const LAMPS = ["L-IMU", "L-FLOW", "L-GNSS", "L-LIDAR", "L-ZUPT", "L-REC"];

const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { const v = localStorage.getItem(k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} },
};

// ------------------------------------------------------------------ theme + units
let COL = {};
function readColors() {
  const cs = getComputedStyle(document.documentElement);
  const v = (n) => cs.getPropertyValue(n).trim();
  COL = { ink: v("--ink"), ink2: v("--ink-2"), ink3: v("--ink-3"), rule: v("--rule"), accent: v("--accent"),
    alert: v("--alert"), ochre: v("--ochre"), band: v("--band"), bandLost: v("--band-lost"),
    lostCol: v("--lost-col"), runCol: v("--run-col"), grid: v("--grid"), paper: v("--paper"),
    mono: v("--mono"), serif: v("--serif") };
  dirty = true;
}
let units = store.get("gs-units", "m");
const lenOut = (m) => units === "ft" ? m / FT : m;
const lenIn = (x) => units === "ft" ? x * FT : x;
let targetM = store.get("gs-target-m", 10);

// ------------------------------------------------------------------ telemetry state
const pts = [];             // {t, vx, vy, sx, sy, sp, fvx, fok, ax, ay, D, N, q, h, st}
let last = null, lastPerf = 0, lastStats = null, sseUp = false;
let acc = null;             // cumulative, reset-robust distance / net accumulators
let zeroD = 0, zeroN = 0, zeroWall = null, zeroPendingUntil = 0;
let prevRec = false;
let dirty = true;

function accumulate(f) {
  const dist = f.distance == null ? (acc ? acc.prevDist : 0) : f.distance;
  if (!acc) {
    acc = { prevDist: dist, prevNet: f.net_forward, prevT: f.t, D: 0, N: 0 };
    zeroD = -dist; zeroN = -(f.net_forward == null ? 0 : f.net_forward);   // show the phone's own value
    return;
  }
  const pending = performance.now() < zeroPendingUntil;
  // phone zeroed / started a run: distance restarts near 0. Smaller dips (filter settling)
  // must not count as a reset, or the whole distance gets added again.
  const reset = dist < acc.prevDist - 0.005 && dist < Math.min(0.1, acc.prevDist * 0.5);
  const netReset = reset || (pending && f.net_forward != null && acc.prevNet != null &&
    Math.abs(f.net_forward) < 0.02 && Math.abs(acc.prevNet - f.net_forward) > 0.02);
  if (reset || netReset) zeroPendingUntil = 0;
  acc.D += reset ? Math.max(dist, 0) : Math.max(dist - acc.prevDist, 0);
  if (f.net_forward != null && acc.prevNet != null) acc.N += netReset ? f.net_forward : f.net_forward - acc.prevNet;
  else acc.N += (f.v_x || 0) * Math.min(Math.max(f.t - acc.prevT, 0), 0.2);
  acc.prevDist = dist; acc.prevNet = f.net_forward; acc.prevT = f.t;
}

function ingest(f) {
  if (f.t == null) return;
  const lp = pts.length ? pts[pts.length - 1] : null;
  if (lp && f.t < lp.t - 1) {                               // phone restarted: new clock
    if (activeRun) stopRun(false);
    pts.length = 0;
    if (acc) acc.prevT = f.t;
  } else if (lp && f.t <= lp.t) return;                     // late or duplicate frame
  accumulate(f);
  const sp = (f.v_x == null || f.v_y == null) ? null : Math.hypot(f.v_x, f.v_y);
  pts.push({ t: f.t, vx: f.v_x, vy: f.v_y, sx: f.sigma_vx, sy: f.sigma_vy, sp, fvx: f.flow_vx,
    fok: !!(f.status & BIT.FLOW_OK), ax: f.ax, ay: f.ay, D: acc.D, N: acc.N, q: f.flow_quality,
    h: f.h == null ? null : f.h * 100, st: f.status });
  if (pts.length > MAX_PTS + 1000) pts.splice(0, 1000);
  last = f; lastPerf = performance.now();

  // runs follow the telemetry: update the open run, mirror phone-side RECORDING edges
  const rec = !!(f.status & BIT.RECORDING);
  if (activeRun) {
    if (activeRun.pending) beginRun(activeRun);
    updateRun(activeRun, sp);
    if (rec) activeRun.seenRec = true;
    if (!rec && prevRec && activeRun.seenRec) stopRun(false);
  } else if (rec && !prevRec) {
    startRun("phone", false);
  }
  prevRec = rec;
  dirty = true;
}

// ------------------------------------------------------------------ SSE
function connect() {
  const es = new EventSource("/events?history=60");
  es.onopen = () => { sseUp = true; };
  es.onerror = () => { sseUp = false; };
  es.addEventListener("history", (ev) => {
    const h = JSON.parse(ev.data);
    for (const f of h.frames) ingest(f);
    if (last && h.frames.length) lastPerf = performance.now() - (h.now - last.recv_time) * 1000;
  });
  es.addEventListener("frames", (ev) => { for (const f of JSON.parse(ev.data)) ingest(f); });
  es.addEventListener("stats", (ev) => { lastStats = JSON.parse(ev.data); renderStats(); });
}

// ------------------------------------------------------------------ runs
const saved = store.get("gs-runs", { nextN: 1, runs: [] });
let runs = saved.runs;             // finished runs, oldest first (summaries only)
let nextN = saved.nextN;
let activeRun = null;
let heldRun = null;                // finished run whose distance the big readout holds until Zero/Start
const runPts = new Map();          // run id -> frozen points (this session only)
let selectedId = null;

function persistRuns() { store.set("gs-runs", { nextN, runs: runs.map(summary) }); }
function summary(r) {
  return { id: r.id, n: r.n, label: r.label, startWall: r.startWall, dur: r.dur, dist: r.dist,
    net: r.net, avg: r.avg, maxSp: r.maxSp, mode: r.mode, target_m: r.target_m, source: r.source };
}
// Mode speed (cruise-control speed): most populated 0.1 m/s bin of the run's speed samples,
// refined to the mean of that bin and its neighbours. Speeds below 0.2 m/s (stopped) are ignored.
const MODE_BIN = 0.1, MODE_MIN = 0.2;
function addModeSample(r, sp) {
  if (sp == null || !(sp >= MODE_MIN)) return;
  r.hist = r.hist || {};
  const k = Math.round(sp / MODE_BIN);
  const b = r.hist[k] || (r.hist[k] = { n: 0, s: 0 });
  b.n++; b.s += sp;
}
function modeSpeed(r) {
  if (!r.hist) return null;
  let best = null;
  for (const k in r.hist) if (best == null || r.hist[k].n > r.hist[best].n) best = k;
  if (best == null) return null;
  let n = 0, s = 0;
  for (let k = +best - 1; k <= +best + 1; k++) if (r.hist[k]) { n += r.hist[k].n; s += r.hist[k].s; }
  return s / n;
}
const fmtMode = (r) => r.mode == null ? "–" : r.mode.toFixed(2);
function beginRun(r) {
  r.pending = false; r.t0 = last.t; r.D0 = acc.D; r.N0 = acc.N;
}
function updateRun(r, sp) {
  r.t1 = last.t;
  r.dur = Math.max(0, r.t1 - r.t0);
  r.dist = acc.D - r.D0;
  r.net = acc.N - r.N0;
  r.avg = r.dur > 0 ? r.dist / r.dur : 0;
  if (sp != null && sp > r.maxSp) r.maxSp = sp;
  addModeSample(r, sp);
  r.mode = modeSpeed(r);
}
function startRun(source, sendCmd) {
  if (activeRun) return;
  heldRun = null;
  const label = $("label").value.trim().replace(/\s+/g, "_");
  const r = { id: Date.now().toString(36) + Math.random().toString(36).slice(2, 6), n: nextN++,
    label: label || (source === "phone" ? "phone" : ""), startWall: Date.now(), source,
    dur: 0, dist: 0, net: 0, avg: 0, maxSp: 0, target_m: targetM, seenRec: false, pending: true };
  if (last) { beginRun(r); updateRun(r, null); }
  activeRun = r;
  view.frozenId = null; selectedId = null;
  persistRuns();
  renderRuns();
  if (sendCmd) send(label ? { cmd: "start_run", label } : { cmd: "start_run" }, $("btn-track"));
}
function stopRun(sendCmd) {
  const r = activeRun;
  if (!r) return;
  activeRun = null;
  if (!r.pending) {
    r.target_m = targetM;
    runs.push(r);
    heldRun = r;
    const i0 = lowerBound(pts, r.t0 - 1), i1 = lowerBound(pts, r.t1 + 1);
    runPts.set(r.id, pts.slice(i0, i1));
    selectedId = r.id;
    view.frozenId = r.id; view.zoom = "run"; syncSegs();
  }
  persistRuns();
  renderRuns();
  dirty = true;
  if (sendCmd) send({ cmd: "stop_run" }, $("btn-track"));
}
const errPct = (r) => r.target_m > 0 ? (r.dist - r.target_m) / r.target_m * 100 : null;
const fmtDur = (s) => s < 60 ? s.toFixed(1) + " s" : `${Math.floor(s / 60)}:${(s % 60).toFixed(0).padStart(2, "0")}`;
const fmtErr = (e) => e == null ? "–" : (e >= 0 ? "+" : "−") + Math.abs(e).toFixed(1) + "%";

function runRow(r, active) {
  const tr = document.createElement("tr");
  tr.dataset.id = r.id;
  if (active) tr.className = "active";
  if (r.id === selectedId) tr.classList.add("sel");
  const e = active ? (targetM > 0 ? (r.dist - targetM) / targetM * 100 : null) : errPct(r);
  const cells = [r.n, r.label || "–", fmtDur(r.dur || 0), lenOut(r.dist).toFixed(2), lenOut(r.net).toFixed(2),
    (r.avg || 0).toFixed(2), (r.maxSp || 0).toFixed(2), fmtMode(r), active ? "…" : fmtErr(e)];
  cells.forEach((c, i) => {
    const td = document.createElement("td");
    td.textContent = c;
    if (i === 1) td.className = "l";
    if (i === 8 && !active && e != null) td.className = Math.abs(e) <= 2 ? "good" : Math.abs(e) > 5 ? "bad" : "";
    tr.appendChild(td);
  });
  const td = document.createElement("td");
  td.className = "del";
  if (!active) {
    const b = document.createElement("button");
    b.textContent = "×"; b.title = "Delete run";
    b.onclick = (ev) => { ev.stopPropagation(); deleteRun(r.id); };
    td.appendChild(b);
  }
  tr.appendChild(td);
  return tr;
}
function renderRuns() {
  const body = $("runs-body");
  body.textContent = "";
  if (activeRun && !activeRun.pending) body.appendChild(runRow(activeRun, true));
  for (let i = runs.length - 1; i >= 0; i--) body.appendChild(runRow(runs[i], false));
  $("runs-empty").hidden = runs.length > 0 || !!activeRun;
  renderDetail();
}
function renderDetail() {
  const el = $("rundetail");
  const r = runs.find((x) => x.id === selectedId);
  if (!r) { el.hidden = true; return; }
  el.hidden = false;
  const e = errPct(r);
  const when = new Date(r.startWall).toLocaleTimeString([], { hour12: false });
  el.innerHTML = "";
  const line1 = document.createElement("div");
  line1.innerHTML = `Run <b></b> · started <b></b> · <b></b> · avg <b></b> m/s · max <b></b> m/s · mode <b></b> m/s`;
  const bs = line1.querySelectorAll("b");
  bs[0].textContent = `${r.n}${r.label ? " " + r.label : ""}`; bs[1].textContent = when; bs[2].textContent = fmtDur(r.dur);
  bs[3].textContent = r.avg.toFixed(3); bs[4].textContent = r.maxSp.toFixed(3);
  bs[5].textContent = r.mode == null ? "–" : `${r.mode.toFixed(3)} (${(r.mode * 2.236936).toFixed(1)} mph)`;
  const line2 = document.createElement("div");
  line2.innerHTML = `travelled <b></b> ${units} · net <b></b> ${units} · target <input type="number" step="0.01" min="0"> ${units} · error <b></b>`;
  const b2 = line2.querySelectorAll("b");
  b2[0].textContent = lenOut(r.dist).toFixed(3); b2[1].textContent = lenOut(r.net).toFixed(3); b2[2].textContent = fmtErr(e);
  const inp = line2.querySelector("input");
  inp.value = lenOut(r.target_m).toFixed(2);
  inp.onchange = () => { const v = parseFloat(inp.value); if (v > 0) { r.target_m = lenIn(v); persistRuns(); renderRuns(); } };
  el.append(line1, line2);
  if (!runPts.has(r.id)) {
    const n = document.createElement("div");
    n.className = "hint"; n.textContent = "charts for this run are not kept after a page reload";
    el.append(n);
  }
}
function deleteRun(id) {
  runs = runs.filter((r) => r.id !== id);
  runPts.delete(id);
  if (selectedId === id) selectedId = null;
  if (view.frozenId === id) { view.frozenId = null; dirty = true; }
  persistRuns(); renderRuns();
}
$("runs-body").addEventListener("click", (e) => {
  const tr = e.target.closest("tr");
  if (!tr || tr.classList.contains("active")) return;
  selectedId = tr.dataset.id;
  if (runPts.has(selectedId)) { view.frozenId = selectedId; view.zoom = "run"; syncSegs(); }
  dirty = true;
  renderRuns();
});
$("btn-clear").onclick = () => {
  if (!runs.length || !confirm(`Delete all ${runs.length} runs from this browser?`)) return;
  runs = []; runPts.clear(); selectedId = null; view.frozenId = null; nextN = 1;
  persistRuns(); renderRuns(); dirty = true;
};
$("btn-export").onclick = () => {
  const head = ["n", "label", "start_time", "duration_s", "distance_m", "net_forward_m", "avg_speed_mps",
    "max_speed_mps", "mode_speed_mps", "target_m", "error_pct", "distance_ft", "target_ft"];
  const q = (s) => /[",\n]/.test(s) ? `"${String(s).replace(/"/g, '""')}"` : s;
  const rows = runs.map((r) => [r.n, q(r.label || ""), new Date(r.startWall).toISOString(), r.dur.toFixed(3),
    r.dist.toFixed(4), r.net.toFixed(4), r.avg.toFixed(4), r.maxSp.toFixed(4), r.mode == null ? "" : r.mode.toFixed(4), r.target_m.toFixed(4),
    errPct(r) == null ? "" : errPct(r).toFixed(3), (r.dist / FT).toFixed(4), (r.target_m / FT).toFixed(4)].join(","));
  const blob = new Blob([head.join(",") + "\n" + rows.join("\n") + "\n"], { type: "text/csv" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `groundspeed_runs_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-")}.csv`;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};

// ------------------------------------------------------------------ readouts
const fmt = (v, d = 2) => (v == null || !isFinite(v)) ? "–" : (v < 0 ? "−" : "") + Math.abs(v).toFixed(d);
const fmtS = (v, d = 2) => {
  if (v == null || !isFinite(v)) return "–";
  const s = Math.abs(v).toFixed(d);
  return (Number(s) === 0 ? " " : v < 0 ? "−" : "+") + s;   // no sign flicker around zero
};
const fmtAge = (s) => s < 60 ? s.toFixed(1) + " s" : s < 3600 ? (s / 60).toFixed(0) + " min" : (s / 3600).toFixed(1) + " h";
const fmtClock = (s) => `${String(Math.floor(s / 60)).padStart(2, "0")}:${(s % 60).toFixed(1).padStart(4, "0")}`;
const setText = (id, txt) => { const el = $(id); if (el.textContent !== txt) el.textContent = txt; };
function setLamp(id, state) { const el = $(id); if (el.dataset.state !== state) el.dataset.state = state; }

let lastDom = 0, lastRunsDom = 0;
function renderFrame(now) {
  if (now - lastDom < 33) return;
  lastDom = now;
  const age = last ? (now - lastPerf) / 1000 : Infinity;
  const live = age < NO_SIGNAL_S && sseUp;
  document.body.classList.toggle("nosignal", !live);
  if (!live) {
    const L = lastStats && lastStats.link;
    setText("nosig-sub", !sseUp ? "dashboard server not reachable — is gsdash running?"
      : (L && L.last_seen_age != null) ? `last frame ${fmtAge(L.last_seen_age)} ago · ${L.source_ip}`
      : `waiting for telemetry on UDP :${lastStats ? lastStats.udp_port : 9000}`);
    for (const id of LAMPS) setLamp(id, "off");
    document.body.classList.remove("flowlost");
    setText("gated", ""); setText("sflag", "");
  }
  const rs = $("runstate");
  rs.classList.toggle("on", !!activeRun);
  setText("runtext", activeRun ? `Tracking run ${activeRun.n}` : live ? "Standby" : "Offline");
  setText("runclock", activeRun ? `${fmtClock(activeRun.dur || 0)} · ${lenOut(activeRun.dist || 0).toFixed(2)} ${units}` : "");
  setText("zero-age", heldRun ? `· run ${heldRun.n} final (Zero for live)`
    : zeroWall ? `· zeroed ${fmtAge((Date.now() - zeroWall) / 1000)} ago` : "");
  if (activeRun && now - lastRunsDom > 250) { lastRunsDom = now; renderRuns(); }

  if (!last || !acc) return;
  const f = last, st = f.status;
  setText("dist", lenOut(heldRun ? heldRun.dist : acc.D - zeroD).toFixed(2));
  setText("net", fmtS(lenOut(heldRun ? heldRun.net : acc.N - zeroN), 2));
  const sp = (f.v_x == null || f.v_y == null) ? null : Math.hypot(f.v_x, f.v_y);
  setText("speed", fmt(sp, 2));
  setText("kmh", sp == null ? "–" : (sp * 3.6).toFixed(1));
  setText("mph", sp == null ? "–" : (sp * 2.236936).toFixed(1));
  setText("vx", fmtS(f.v_x, 2)); setText("svx", fmt(2 * f.sigma_vx, 2));
  setText("vy", fmtS(f.v_y, 2)); setText("svy", fmt(2 * f.sigma_vy, 2));
  $("svx").parentElement.classList.toggle("wide", 2 * f.sigma_vx >= 0.15);
  $("svy").parentElement.classList.toggle("wide", 2 * f.sigma_vy >= 0.15);
  const torch = (st & BIT.TORCH_ON || f.torch > 0) ? f.torch + "%" : "off";
  const aux = [`flow q ${fmt(f.flow_quality, 1)}`, `h ${f.h == null ? "–" : (f.h * 100).toFixed(1) + " cm"}`];
  if (f.gz != null) aux.push(`yaw ${fmtS(f.gz, 2)} rad/s`);
  aux.push(`battery ${f.battery === 255 ? "?" : f.battery + "%"}`, `torch ${torch}`, `seq ${f.seq}`,
    `v${f.version || 1} ${f.fmt === "json" ? "json" : "bin"}`);
  setText("aux", aux.join("  ·  "));
  if (document.activeElement !== slider && now - sliderTouched > 3000 && +slider.value !== f.torch) {
    slider.value = f.torch; $("torch-val").textContent = f.torch + "%";
  }
  if (live) {
    const flowOk = !!(st & BIT.FLOW_OK);
    document.body.classList.toggle("flowlost", !flowOk && !!(st & BIT.FILTER_INIT));
    setLamp("L-IMU", st & BIT.IMU_OK ? "ok" : "bad");
    setLamp("L-FLOW", flowOk ? "ok" : "bad");
    setText("gated", st & BIT.FLOW_GATED ? "gated" : "");
    setLamp("L-GNSS", st & BIT.GNSS_OK ? "ok" : "off");
    setLamp("L-LIDAR", st & BIT.LIDAR_OK ? "ok" : "off");
    setLamp("L-ZUPT", st & BIT.ZUPT ? "on" : "off");
    setLamp("L-REC", st & BIT.RECORDING ? "on" : "off");
    const flag = !(st & BIT.FILTER_INIT) ? "filter not initialised"
      : st & BIT.CALIBRATING ? "calibrating…" : st & BIT.GNSS_GATED ? "gnss gated" : "";
    setText("sflag", flag);
    $("sflag").classList.toggle("bad", !(st & BIT.FILTER_INIT));
  }
}

function renderStats() {
  const s = lastStats; if (!s) return;
  const L = s.link, ph = s.phone;
  const age = L.last_seen_age;
  let cls = "";
  if (age != null) cls = age < NO_SIGNAL_S ? (L.rate_hz >= 40 ? "ok" : "warn") : "bad";
  $("ldot").className = "ldot " + cls;
  const parts = [`${L.rate_hz.toFixed(1)} Hz`, `${L.lost} gaps`, `${L.crc_errors} crc`];
  if (L.out_of_order) parts.push(`${L.out_of_order} ooo`);
  if (L.decode_errors) parts.push(`${L.decode_errors} bad`);
  parts.push(age == null ? "never" : age < 1 ? (age * 1000).toFixed(0) + " ms" : fmtAge(age));
  parts.push(ph.target_ip || "phone ?");
  setText("linktext", parts.join("  ·  "));
  const cs = $("cmd-state");
  cs.textContent = ph.connected ? `command link ✓ ${ph.target_ip}:${ph.target_port} (${ph.target_source})`
    : ph.target_ip ? `command link ✗ ${ph.last_error || "connecting…"}` : "command link — phone IP unknown";
  cs.classList.toggle("ok", !!ph.connected);
  if (document.activeElement !== $("phone-input"))
    $("phone-input").placeholder = ph.explicit_ip ? ph.explicit_ip : "auto" + (ph.target_ip ? ` (${ph.target_ip})` : "");
}

// ------------------------------------------------------------------ charts
const canvas = $("chart");
const ctx = canvas.getContext("2d");
const view = { zoom: store.get("gs-zoom", 20), frozenId: null };
let hoverX = null, hoverY = null;
const yr = [null, null, null, null, null];   // eased y ranges per panel (4 = h axis)

function lowerBound(a, t) {
  let lo = 0, hi = a.length;
  while (lo < hi) { const m = (lo + hi) >> 1; if (a[m].t < t) lo = m + 1; else hi = m; }
  return lo;
}
function niceStep(range, n) {
  const raw = range / Math.max(n, 1), p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p;
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
}

function currentView(now) {
  const run = view.frozenId ? runs.find((r) => r.id === view.frozenId) : null;
  if (run && runPts.has(run.id)) {
    const data = runPts.get(run.id);
    let tA = run.t0, tB = Math.max(run.t1, run.t0 + 1);
    if (view.zoom !== "run") tA = tB - view.zoom;
    return { data, tA, tB, rel: "start", t0: run.t0, base: { D: run.D0, N: run.N0 }, frozen: true, run };
  }
  const tNow = last ? last.t + Math.min((now - lastPerf) / 1000, 0.25) : 0;
  if (view.zoom === "run") {
    if (activeRun && !activeRun.pending)
      return { data: pts, tA: activeRun.t0, tB: Math.max(tNow, activeRun.t0 + 1), rel: "start", t0: activeRun.t0,
        base: { D: activeRun.D0, N: activeRun.N0 }, frozen: false, run: activeRun };
    const tA = pts.length ? pts[0].t : tNow - 20;
    return { data: pts, tA, tB: Math.max(tNow, tA + 1), rel: "start", t0: tA, base: { D: zeroD, N: zeroN }, frozen: false };
  }
  return { data: pts, tA: tNow - view.zoom, tB: tNow, rel: "end", t0: tNow, base: { D: zeroD, N: zeroN }, frozen: false };
}

// min/max-per-pixel-column polyline; breaks at nulls and time gaps when not decimated
function trace(data, i0, i1, X, Y, get, pw) {
  ctx.beginPath();
  if (i1 - i0 <= pw * 1.5) {
    let pen = false, pt = null;
    for (let i = i0; i < i1; i++) {
      const p = data[i], v = get(p);
      if (v == null || !isFinite(v) || (pt != null && p.t - pt > 0.3)) { pen = false; pt = p.t; if (v == null) continue; }
      const x = X(p.t), y = Y(v);
      pen ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      pen = true; pt = p.t;
    }
  } else {
    let col = null, mn = 0, mx = 0, started = false;
    const flush = () => { if (col == null) return; const a = Y(mn), b = Y(mx); started ? ctx.lineTo(col, a) : ctx.moveTo(col, a); ctx.lineTo(col, b); started = true; };
    for (let i = i0; i < i1; i++) {
      const p = data[i], v = get(p);
      if (v == null || !isFinite(v)) continue;
      const c = Math.round(X(p.t));
      if (c !== col) { flush(); col = c; mn = mx = v; } else { if (v < mn) mn = v; if (v > mx) mx = v; }
    }
    flush();
  }
}

// per-column buckets for the v_x band and flow-lost spans
function bandBuckets(data, i0, i1, X, pw) {
  const out = [];
  const per = i1 - i0 > pw * 1.5;
  let cur = null;
  for (let i = i0; i < i1; i++) {
    const p = data[i];
    if (p.vx == null || p.sx == null) continue;
    const up = p.vx + 2 * p.sx, lo = p.vx - 2 * p.sx, dead = !p.fok;
    const x = per ? Math.round(X(p.t)) : X(p.t);
    if (per && cur && cur.x === x) { cur.up = Math.max(cur.up, up); cur.lo = Math.min(cur.lo, lo); cur.dead = cur.dead || dead; continue; }
    cur = { x, up, lo, dead, t: p.t };
    out.push(cur);
  }
  return out;
}

function ease(k, lo, hi, live) {
  if (!yr[k] || !live) { yr[k] = { lo, hi }; return yr[k]; }
  yr[k].lo += (lo - yr[k].lo) * 0.08; yr[k].hi += (hi - yr[k].hi) * 0.08;
  return yr[k];
}
function rangeOf(vals, minLo, minHi, padFrac = 0.08) {
  let lo = minLo, hi = minHi;
  for (const v of vals) { if (v == null || !isFinite(v)) continue; if (v < lo) lo = v; if (v > hi) hi = v; }
  const pad = (hi - lo) * padFrac;
  return [lo - pad, hi + pad];
}

function drawAxisY(p, r, fs, right, fmtv) {
  const step = niceStep(r.hi - r.lo, Math.max(2, p.h / (fs * 2.6)));
  ctx.textBaseline = "middle"; ctx.textAlign = right ? "left" : "right";
  ctx.font = `${fs * 0.78}px ${COL.mono}`;
  for (let v = Math.ceil(r.lo / step) * step; v <= r.hi + 1e-9; v += step) {
    const y = Math.round(p.y + (r.hi - v) / (r.hi - r.lo) * p.h) + 0.5;
    if (!right) {
      ctx.strokeStyle = Math.abs(v) < 1e-9 ? COL.ink3 : COL.grid; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(p.x, y); ctx.lineTo(p.x + p.w, y); ctx.stroke();
    }
    ctx.fillStyle = right ? COL.ochre : COL.ink3;
    ctx.fillText(fmtv ? fmtv(v, step) : v.toFixed(step < 0.1 ? 2 : step < 1 ? 1 : 0), right ? p.x + p.w + fs * 0.45 : p.x - fs * 0.45, y);
  }
}

function panelTitle(p, fs, title, legend) {
  ctx.textBaseline = "top"; ctx.textAlign = "left";
  ctx.font = `italic ${fs * 0.85}px ${COL.serif}`;
  ctx.fillStyle = COL.ink2;
  ctx.fillText(title, p.x + fs * 0.4, p.y + fs * 0.25);
  ctx.font = `${fs * 0.75}px ${COL.mono}`;
  ctx.textAlign = "right";
  let x = p.x + p.w - fs * 0.3;
  for (let i = legend.length - 1; i >= 0; i--) {
    const [name, col, kind] = legend[i];
    ctx.fillStyle = COL.ink2;
    ctx.fillText(name, x, p.y + fs * 0.3);
    x -= ctx.measureText(name).width + fs * 0.4;
    ctx.fillStyle = col; ctx.strokeStyle = col;
    if (kind === "band") { ctx.globalAlpha = 1; ctx.fillStyle = COL.band; ctx.fillRect(x - fs * 1.2, p.y + fs * 0.25, fs * 1.2, fs * 0.7); ctx.fillStyle = col; ctx.fillRect(x - fs * 1.2, p.y + fs * 0.55, fs * 1.2, 2); }
    else if (kind === "dot") { ctx.fillRect(x - fs * 0.7, p.y + fs * 0.5, 3, 3); }
    else { ctx.fillRect(x - fs * 1.2, p.y + fs * 0.6, fs * 1.2, kind === "thick" ? 2 : 1.2); }
    x -= fs * 2;
  }
}

let tipLast = "";
function drawCharts(now) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const fs = Math.max(10, Math.min(16, W / 85));
  const padL = fs * 3.3, padR = fs * 3.0, padB = fs * 1.6, gap = fs * 0.9;
  const pw = W - padL - padR;
  const avail = H - padB - gap * 3;
  if (pw < 40 || avail < 80) return;
  const fr = [0.40, 0.20, 0.22, 0.18];
  const P = []; let y = 0;
  for (const f of fr) { P.push({ x: padL, y, w: pw, h: avail * f }); y += avail * f + gap; }

  const v = currentView(now);
  const { data, tA, tB } = v;
  const X = (t) => padL + (t - tA) / (tB - tA) * pw;
  const i0 = Math.max(0, lowerBound(data, tA) - 1), i1 = Math.min(data.length, lowerBound(data, tB) + 1);
  const live = !v.frozen;
  const uS = units === "ft" ? 1 / FT : 1;
  const dOf = (p) => (p.D - v.base.D) * uS, nOf = (p) => (p.N - v.base.N) * uS;

  // y ranges from a strided pass
  const stride = Math.max(1, Math.floor((i1 - i0) / 3000));
  const A = [], B = [], Cc = [], Q = [], Hh = [];
  for (let i = i0; i < i1; i += stride) {
    const p = data[i];
    if (p.vx != null) { const b = Math.min(2 * (p.sx || 0), 1.5); A.push(p.vx + b, p.vx - b); }
    A.push(p.vy, p.sp); if (p.fok && p.fvx != null) A.push(p.fvx);
    B.push(p.ax, p.ay); Cc.push(dOf(p), nOf(p)); Q.push(p.q); Hh.push(p.h);
  }
  const rA = ease(0, ...rangeOf(A, -0.2, 1.0), live);
  const rB = ease(1, ...rangeOf(B, -0.3, 0.3), live);
  const rC = ease(2, ...rangeOf(Cc, 0, units === "ft" ? 3 : 1), live);
  const rQ = ease(3, ...rangeOf(Q, 0, 10), live);
  let [hl, hh] = rangeOf(Hh, Infinity, -Infinity, 0.15);
  if (!isFinite(hl)) { hl = 0; hh = 50; }
  if (hh - hl < 2) { const m = (hh + hl) / 2; hl = m - 1; hh = m + 1; }
  const rH = ease(4, hl, hh, live);
  const Yf = (p, r) => (val) => p.y + (r.hi - val) / (r.hi - r.lo) * p.h;
  const YA = Yf(P[0], rA), YB = Yf(P[1], rB), YC = Yf(P[2], rC), YQ = Yf(P[3], rQ), YH = Yf(P[3], rH);

  // time grid + labels (shared axis)
  const span = tB - tA;
  const tstep = niceStep(span, Math.max(3, pw / (fs * 6)));
  ctx.strokeStyle = COL.grid; ctx.lineWidth = 1;
  ctx.font = `${fs * 0.78}px ${COL.mono}`; ctx.fillStyle = COL.ink3; ctx.textBaseline = "top"; ctx.textAlign = "center";
  const ref = v.rel === "end" ? tB : v.t0;
  const kStart = Math.ceil((tA - ref) / tstep), kEnd = Math.floor((tB - ref) / tstep);
  for (let k = kStart; k <= kEnd; k++) {
    const t = ref + k * tstep, x = Math.round(X(t)) + 0.5;
    for (const p of P) { ctx.beginPath(); ctx.moveTo(x, p.y); ctx.lineTo(x, p.y + p.h); ctx.stroke(); }
    const rel = k * tstep;
    const lbl = v.rel === "end" ? (Math.abs(rel) < 1e-9 ? "now" : `−${(-rel).toFixed(tstep < 1 ? 1 : 0)} s`) : `${rel.toFixed(tstep < 1 ? 1 : 0)} s`;
    ctx.fillText(lbl, Math.min(Math.max(x, padL + fs), padL + pw - fs), H - padB + fs * 0.4);
  }
  // panel frames
  ctx.strokeStyle = COL.rule;
  for (const p of P) { ctx.beginPath(); ctx.moveTo(p.x, Math.round(p.y + p.h) + 0.5); ctx.lineTo(p.x + p.w, Math.round(p.y + p.h) + 0.5); ctx.stroke(); }

  drawAxisY(P[0], rA, fs); drawAxisY(P[1], rB, fs); drawAxisY(P[2], rC, fs); drawAxisY(P[3], rQ, fs);
  drawAxisY(P[3], rH, fs, true);

  ctx.save();
  ctx.beginPath(); ctx.rect(padL, 0, pw, H - padB); ctx.clip();

  // runs in view (faint teal span), flow-lost spans across all panels
  for (const r of activeRun && !activeRun.pending ? runs.concat([activeRun]) : runs) {
    if (r.t0 == null || r.t1 == null || !runPts.has(r.id) && r !== activeRun) continue;
    if (r.t1 < tA || r.t0 > tB || (v.frozen && r.id === v.run.id && view.zoom === "run")) continue;
    ctx.fillStyle = COL.runCol;
    ctx.fillRect(X(r.t0), 0, X(r.t1) - X(r.t0), H - padB);
    ctx.fillStyle = COL.accent; ctx.font = `${fs * 0.72}px ${COL.mono}`; ctx.textAlign = "left"; ctx.textBaseline = "bottom";
    ctx.fillText(`#${r.n}`, X(r.t0) + 3, P[1].y - 2);
  }
  const bk = bandBuckets(data, i0, i1, X, pw);
  ctx.fillStyle = COL.lostCol;
  for (let i = 0; i < bk.length; i++) {
    if (!bk[i].dead) continue;
    let j = i; while (j + 1 < bk.length && bk[j + 1].dead) j++;
    const x0 = bk[i].x, x1 = j + 1 < bk.length ? bk[j + 1].x : bk[j].x;
    ctx.fillRect(x0, 0, Math.max(1, x1 - x0), H - padB);
    i = j;
  }

  // (a) velocity: band, flow dots, v_y, |v|, v_x
  ctx.save(); ctx.beginPath(); ctx.rect(padL, P[0].y, pw, P[0].h); ctx.clip();
  for (let i = 0; i < bk.length - 1;) {
    const dead = bk[i].dead; let j = i;
    while (j < bk.length - 1 && bk[j + 1].dead === dead) j++;
    const seg = bk.slice(i, Math.min(j + 2, bk.length));
    ctx.beginPath();
    seg.forEach((b, k) => k ? ctx.lineTo(b.x, YA(b.up)) : ctx.moveTo(b.x, YA(b.up)));
    for (let k = seg.length - 1; k >= 0; k--) ctx.lineTo(seg[k].x, YA(seg[k].lo));
    ctx.closePath();
    ctx.fillStyle = dead ? COL.bandLost : COL.band; ctx.fill();
    if (dead) { ctx.strokeStyle = COL.alert; ctx.globalAlpha = .5; ctx.lineWidth = 1; ctx.stroke(); ctx.globalAlpha = 1; }
    i = j + 1;
  }
  ctx.fillStyle = COL.ink3;
  const dotStride = Math.max(1, Math.floor((i1 - i0) / (pw / 2)));
  for (let i = i0; i < i1; i += dotStride) {
    const p = data[i];
    if (p.fok && p.fvx != null) ctx.fillRect(X(p.t) - 1, YA(p.fvx) - 1, 2.2, 2.2);
  }
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.strokeStyle = COL.ochre; ctx.lineWidth = 1.1; trace(data, i0, i1, X, YA, (p) => p.vy, pw); ctx.stroke();
  ctx.strokeStyle = COL.accent; ctx.lineWidth = 1.3; trace(data, i0, i1, X, YA, (p) => p.sp, pw); ctx.stroke();
  ctx.strokeStyle = COL.ink; ctx.lineWidth = Math.max(1.8, fs / 8); trace(data, i0, i1, X, YA, (p) => p.vx, pw); ctx.stroke();
  ctx.restore();
  // (b) acceleration
  ctx.save(); ctx.beginPath(); ctx.rect(padL, P[1].y, pw, P[1].h); ctx.clip();
  ctx.strokeStyle = COL.ochre; ctx.lineWidth = 1; trace(data, i0, i1, X, YB, (p) => p.ay, pw); ctx.stroke();
  ctx.strokeStyle = COL.ink; ctx.lineWidth = 1.3; trace(data, i0, i1, X, YB, (p) => p.ax, pw); ctx.stroke();
  ctx.restore();
  // (c) distance travelled and net forward
  ctx.save(); ctx.beginPath(); ctx.rect(padL, P[2].y, pw, P[2].h); ctx.clip();
  ctx.strokeStyle = COL.accent; ctx.lineWidth = 1.4; trace(data, i0, i1, X, YC, nOf, pw); ctx.stroke();
  ctx.strokeStyle = COL.ink; ctx.lineWidth = 1.9; trace(data, i0, i1, X, YC, dOf, pw); ctx.stroke();
  ctx.restore();
  // (d) flow quality and h
  ctx.save(); ctx.beginPath(); ctx.rect(padL, P[3].y, pw, P[3].h); ctx.clip();
  ctx.strokeStyle = COL.ochre; ctx.lineWidth = 1; trace(data, i0, i1, X, YH, (p) => p.h, pw); ctx.stroke();
  ctx.strokeStyle = COL.ink2; ctx.lineWidth = 1.1; trace(data, i0, i1, X, YQ, (p) => p.q, pw); ctx.stroke();
  ctx.restore();
  ctx.restore();

  panelTitle(P[0], fs, "velocity  m/s", [["v_x ±2σ", COL.ink, "band"], ["v_y", COL.ochre], ["|v|", COL.accent], ["flow v_x", COL.ink3, "dot"]]);
  panelTitle(P[1], fs, "acceleration  m/s²", [["a_x", COL.ink, "thick"], ["a_y", COL.ochre]]);
  panelTitle(P[2], fs, `distance  ${units}`, [["travelled", COL.ink, "thick"], ["net forward", COL.accent]]);
  panelTitle(P[3], fs, "flow quality · h", [["PSR", COL.ink2], ["h cm", COL.ochre]]);

  // crosshair
  const tip = $("tip");
  if (hoverX != null && hoverX >= padL && hoverX <= padL + pw && i1 > i0) {
    const th = tA + (hoverX - padL) / pw * (tB - tA);
    let k = lowerBound(data, th);
    if (k >= data.length) k = data.length - 1;
    if (k > 0 && Math.abs(data[k - 1].t - th) < Math.abs(data[k].t - th)) k--;
    const p = data[k];
    if (p && p.t >= tA - 0.5 && p.t <= tB + 0.5) {
      const x = Math.round(X(p.t)) + 0.5;
      ctx.strokeStyle = COL.ink2; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H - padB); ctx.stroke(); ctx.setLineDash([]);
      const dot = (yy, col) => { if (yy == null || !isFinite(yy)) return; ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, yy, 3, 0, 7); ctx.fill(); };
      if (p.vx != null) dot(YA(p.vx), COL.ink);
      if (p.sp != null) dot(YA(p.sp), COL.accent);
      if (p.ax != null) dot(YB(p.ax), COL.ink);
      dot(YC(dOf(p)), COL.ink); dot(YC(nOf(p)), COL.accent);
      if (p.q != null) dot(YQ(p.q), COL.ink2);
      const rel = v.rel === "end" ? p.t - tB : p.t - v.t0;
      const n2 = (x, d = 3) => x == null || !isFinite(x) ? "–" : (x < 0 ? "−" : " ") + Math.abs(x).toFixed(d);
      const lines = [
        `t      ${n2(rel, 2)} s${v.rel === "start" ? " from start" : ""}`,
        `v_x    ${n2(p.vx)}  ±${p.sx == null ? "–" : (2 * p.sx).toFixed(3)} m/s`,
        `v_y    ${n2(p.vy)}  ±${p.sy == null ? "–" : (2 * p.sy).toFixed(3)} m/s`,
        `|v|    ${n2(p.sp)} m/s  ${p.sp == null ? "" : (p.sp * 3.6).toFixed(2) + " km/h"}`,
        `flow   ${p.fok ? n2(p.fvx) + " m/s" : "lost"}`,
        `a_x    ${n2(p.ax)}   a_y ${n2(p.ay)} m/s²`,
        `dist   ${n2(dOf(p))} ${units}   net ${n2(nOf(p))} ${units}`,
        `PSR    ${n2(p.q, 1)}   h ${p.h == null ? "–" : p.h.toFixed(1)} cm`,
      ];
      const txt = lines.join("\n");
      if (txt !== tipLast) { tip.textContent = txt; tipLast = txt; }
      tip.hidden = false;
      const tw = tip.offsetWidth, thh = tip.offsetHeight;
      let lx = hoverX + 14; if (lx + tw > W) lx = hoverX - tw - 14;
      let ly = (hoverY == null ? 10 : hoverY) - thh / 2; ly = Math.max(0, Math.min(ly, H - thh));
      tip.style.left = lx + "px"; tip.style.top = ly + "px";
    } else tip.hidden = true;
  } else tip.hidden = true;

  // header state of the chart
  const vs = $("viewstate");
  if (v.frozen) { vs.textContent = `run ${v.run.n}${v.run.label ? " · " + v.run.label : ""} · frozen for inspection`; vs.className = "viewstate frozen"; }
  else { vs.textContent = view.zoom === "run" ? (activeRun ? `live · run ${activeRun.n} so far` : "live · whole buffer") : "live"; vs.className = "viewstate"; }
  $("btn-live").hidden = !v.frozen;
  document.body.classList.toggle("live-view", !v.frozen);
}

canvas.addEventListener("mousemove", (e) => { const r = canvas.getBoundingClientRect(); hoverX = e.clientX - r.left; hoverY = e.clientY - r.top; dirty = true; });
canvas.addEventListener("mouseleave", () => { hoverX = hoverY = null; dirty = true; });
window.addEventListener("resize", () => { dirty = true; });

function syncSegs() {
  for (const b of document.querySelectorAll("[data-zoom]")) b.classList.toggle("on", String(view.zoom) === b.dataset.zoom);
  for (const b of document.querySelectorAll("[data-unit]")) b.classList.toggle("on", units === b.dataset.unit);
  for (const el of document.querySelectorAll('[data-u="len"]')) el.textContent = units;
  $("dist-unit").textContent = units;
  if (document.activeElement !== $("target")) $("target").value = lenOut(targetM).toFixed(2);
  $("target-alt").textContent = units === "ft" ? `= ${targetM.toFixed(2)} m` : `= ${(targetM / FT).toFixed(2)} ft`;
  dirty = true;
}
for (const b of document.querySelectorAll("[data-zoom]")) b.onclick = () => {
  view.zoom = b.dataset.zoom === "run" ? "run" : +b.dataset.zoom; store.set("gs-zoom", view.zoom); syncSegs();
};
for (const b of document.querySelectorAll("[data-unit]")) b.onclick = () => {
  units = b.dataset.unit; store.set("gs-units", units); syncSegs(); renderRuns();
};
$("btn-live").onclick = () => { view.frozenId = null; selectedId = null; renderRuns(); dirty = true; };
$("target").addEventListener("change", () => {
  const v = parseFloat($("target").value);
  if (v > 0) { targetM = lenIn(v); store.set("gs-target-m", targetM); }
  syncSegs(); renderRuns();
});

function loop(now) {
  const v = view.frozenId && runPts.has(view.frozenId);
  if (!v || dirty) { drawCharts(now); dirty = false; }
  renderFrame(now);
  requestAnimationFrame(loop);
}

// ------------------------------------------------------------------ commands
const logEl = $("cmdlog");
let logN = 0;
function logCmd(req, rep, ms) {
  const li = document.createElement("li");
  const ok = rep && rep.ok;
  const extra = Object.assign({}, rep); delete extra.ok; delete extra.cmd; delete extra.local;
  const reqx = Object.assign({}, req); delete reqx.cmd;
  li.innerHTML = `<span class="ts"></span><span class="${ok ? "ok" : "err"}"></span><span class="rep"></span>`;
  li.children[0].textContent = new Date().toLocaleTimeString([], { hour12: false });
  li.children[1].textContent = (ok ? "✓ " : "✗ ") + req.cmd + (Object.keys(reqx).length ? " " + Object.values(reqx).join(" ") : "");
  li.children[2].textContent = (rep && rep.error ? rep.error : Object.keys(extra).length ? JSON.stringify(extra) : "ok") + `  ${ms.toFixed(0)} ms`;
  li.title = JSON.stringify(rep);
  logEl.prepend(li);
  while (logEl.children.length > 80) logEl.lastChild.remove();
  logN++;
  $("logcount").textContent = `${logN}${ok ? "" : " · last failed"}`;
  $("logcount").style.color = ok ? "" : "var(--alert)";
}
async function send(req, btn) {
  if (btn) btn.classList.add("busy");
  const t = performance.now();
  let rep;
  try {
    const r = await fetch("/api/cmd", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(req) });
    rep = await r.json();
  } catch (e) {
    rep = { ok: false, cmd: req.cmd, error: "dashboard server unreachable" };
  }
  if (btn) btn.classList.remove("busy");
  logCmd(req, rep, performance.now() - t);
  return rep;
}

function clientZero() {
  // client-side baseline first, so the screen reads 0 immediately; the phone's own reset
  // (distance/net drop in later frames) is absorbed by the accumulators
  if (acc) { zeroD = acc.D; zeroN = acc.N; }
  heldRun = null;
  zeroWall = Date.now();
  zeroPendingUntil = performance.now() + 5000;
  dirty = true;
}
$("btn-zero").onclick = (e) => {
  clientZero();
  const b = e.currentTarget; b.classList.add("flash"); setTimeout(() => b.classList.remove("flash"), 250);
  send({ cmd: "zero" }, b);
};
function syncTrack() {
  const b = $("btn-track");
  b.classList.toggle("on", !!activeRun);
  b.textContent = activeRun ? "Stop tracking" : "Start tracking";
}
$("btn-track").onclick = () => {
  if (!activeRun && last && !(last.status & BIT.FLOW_OK)
      && !confirm("Flow is not OK (camera not tracking the floor). Distance will be IMU-only garbage.\n\nStart anyway?")) return;
  activeRun ? stopRun(true) : startRun("dashboard", true);
  syncTrack();
};
setInterval(syncTrack, 250);
$("btn-mark").onclick = (e) => send({ cmd: "mark" }, e.currentTarget);
$("btn-cal").onclick = (e) => send({ cmd: "calibrate" }, e.currentTarget);
$("btn-height").onclick = async (e) => {
  const b = e.currentTarget;
  b.textContent = "Measuring…";
  clientZero();   // the phone zeroes distance and re-estimates IMU bias during the measurement
  const rep = await send({ cmd: "measure_height" }, b);
  b.textContent = rep.ok && rep.h != null ? `h ${(rep.h * 100).toFixed(1)} cm` : "h failed";
  b.title = rep.message || rep.error || "";
  setTimeout(() => { b.textContent = "Measure h"; }, 8000);
};
$("btn-reset").onclick = (e) => send({ cmd: "reset_distance" }, e.currentTarget);
$("btn-ping").onclick = (e) => send({ cmd: "ping" }, e.currentTarget);
const slider = $("torch-slider");
let sliderTouched = 0;
slider.oninput = () => { sliderTouched = performance.now(); $("torch-val").textContent = slider.value + "%"; };
slider.onchange = () => { sliderTouched = performance.now(); slider.blur(); send({ cmd: "set_torch", level: Math.round(slider.value) / 100 }); };
$("btn-phone").onclick = async () => {
  const ip = $("phone-input").value.trim();
  await fetch("/api/phone", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ ip }) });
  $("phone-input").value = ""; $("phone-input").blur();
};
$("phone-input").addEventListener("keydown", (e) => { if (e.key === "Enter") $("btn-phone").click(); });

function applyTheme(t) {
  document.documentElement.dataset.theme = t;
  try { localStorage.setItem("gs-theme", t); } catch (e) {}
  $("btn-theme").textContent = t === "dark" ? "Light" : "Dark";
  readColors();
}
$("btn-theme").onclick = () => applyTheme(document.documentElement.dataset.theme === "dark" ? "light" : "dark");
$("btn-fs").onclick = () => document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen();
document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.metaKey || e.ctrlKey || e.altKey) return;
  if (e.key === "m" || e.key === "M") $("btn-mark").click();
  if (e.key === "f" || e.key === "F") $("btn-fs").click();
});

applyTheme(document.documentElement.dataset.theme === "dark" ? "dark" : "light");
syncSegs(); renderRuns(); syncTrack();
connect();
requestAnimationFrame(loop);
