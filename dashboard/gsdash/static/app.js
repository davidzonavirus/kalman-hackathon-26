"use strict";
// Ground speed dashboard UI. No dependencies; talks to gsdash.server via SSE + JSON.

const BIT = { IMU_OK: 1, FLOW_OK: 2, GNSS_OK: 4, LIDAR_OK: 8, RECORDING: 16, ZUPT: 32,
  FLOW_GATED: 64, GNSS_GATED: 128, TORCH_ON: 256, FILTER_INIT: 512, CALIBRATING: 1024 };
const WINDOW_S = 20;
const NO_SIGNAL_S = 1.0;
const LAMPS = ["L-IMU", "L-FLOW", "L-GNSS", "L-LIDAR", "L-ZUPT", "L-REC"];

const $ = (id) => document.getElementById(id);

// theme colours (re-read when the theme changes)
let COL = {};
function readColors() {
  const cs = getComputedStyle(document.documentElement);
  const v = (n) => cs.getPropertyValue(n).trim();
  COL = { ink: v("--ink"), ink2: v("--ink-2"), ink3: v("--ink-3"), band: v("--band"), bandLost: v("--band-lost"),
    lostCol: v("--lost-col"), grid: v("--grid"), alert: v("--alert"), mono: v("--mono") };
}

// ------------------------------------------------------------------ state
const buf = [];            // {t, vx, vy, sx, sy, st}
let last = null;           // last frame
let lastPerf = 0;          // performance.now() at last frame arrival
let lastStats = null;
let sseUp = false;
let wasRecording = false;
let recSincePerf = null;

// ------------------------------------------------------------------ SSE
function connect() {
  const es = new EventSource("/events");
  es.onopen = () => { sseUp = true; };
  es.onerror = () => { sseUp = false; };
  es.addEventListener("history", (ev) => {
    const h = JSON.parse(ev.data);
    buf.length = 0;
    for (const f of h.frames) ingest(f);
    if (last) lastPerf = performance.now() - (h.now - last.recv_time) * 1000;
  });
  es.addEventListener("frames", (ev) => { for (const f of JSON.parse(ev.data)) ingest(f); });
  es.addEventListener("stats", (ev) => { lastStats = JSON.parse(ev.data); renderStats(); });
}

function ingest(f) {
  if (f.t == null) return;
  // phone restarted / clock jumped back: start the trace over
  if (buf.length && f.t < buf[buf.length - 1].t - 1) buf.length = 0;
  if (!buf.length || f.t > buf[buf.length - 1].t)
    buf.push({ t: f.t, vx: f.v_x, vy: f.v_y, sx: f.sigma_vx, sy: f.sigma_vy, st: f.status });
  while (buf.length && buf[0].t < f.t - WINDOW_S - 2) buf.shift();
  last = f;
  lastPerf = performance.now();
  const rec = !!(f.status & BIT.RECORDING);
  if (rec && !wasRecording) { setResult(null); recSincePerf = performance.now(); }
  if (!rec) recSincePerf = null;
  wasRecording = rec;
}

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

function setLamp(id, state) {
  const el = $(id);
  if (el.dataset.state !== state) el.dataset.state = state;
}

let lastDom = 0;
function renderFrame(now) {
  if (now - lastDom < 33) return;   // ~30 Hz DOM updates is plenty
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
  const rec = live && last && (last.status & BIT.RECORDING);
  $("runstate").classList.toggle("rec", !!rec);
  setText("runtext", rec ? "Recording" : live ? "Standby" : "Offline");
  setText("runclock", rec && recSincePerf != null ? fmtClock((now - recSincePerf) / 1000) : "");

  if (!last) return;
  const f = last, st = f.status;
  setText("dist", fmt(f.distance, 2));
  setText("vx", fmtS(f.v_x, 2));
  setText("svx", fmt(2 * f.sigma_vx, 2));
  setText("vy", fmtS(f.v_y, 2));
  setText("svy", fmt(2 * f.sigma_vy, 2));
  $("svx").parentElement.classList.toggle("wide", 2 * f.sigma_vx >= 0.15);
  $("svy").parentElement.classList.toggle("wide", 2 * f.sigma_vy >= 0.15);
  const torch = (st & BIT.TORCH_ON || f.torch > 0) ? f.torch + "%" : "off";
  setText("aux", [
    `flow q ${fmt(f.flow_quality, 1)}`,
    `h ${f.h == null ? "–" : (f.h * 100).toFixed(1) + " cm"}`,
    `battery ${f.battery === 255 ? "?" : f.battery + "%"}`,
    `torch ${torch}`,
    `seq ${f.seq}`,
    f.fmt === "json" ? "json" : "bin",
  ].join("  ·  "));
  const target = parseFloat($("target").value);
  const frac = target > 0 && f.distance != null ? f.distance / target : 0;
  $("gauge-fill").style.width = Math.max(0, Math.min(frac, 1)) * 100 + "%";
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
  let cls = "bad";
  if (age != null && age < NO_SIGNAL_S) cls = L.rate_hz >= 40 ? "ok" : "warn";
  if (age == null) cls = "";
  $("ldot").className = "ldot " + cls;
  const ageTxt = age == null ? "never" : age < 1 ? (age * 1000).toFixed(0) + " ms" : fmtAge(age);
  const parts = [`${L.rate_hz.toFixed(1)} Hz`, `${L.lost} gaps`, `${L.crc_errors} crc`];
  if (L.out_of_order) parts.push(`${L.out_of_order} ooo`);
  if (L.decode_errors) parts.push(`${L.decode_errors} bad`);
  parts.push(ageTxt);
  parts.push(ph.target_ip || "phone ?");
  setText("linktext", parts.join("  ·  "));
  const cs = $("cmd-state");
  cs.textContent = ph.connected ? `command link ✓ ${ph.target_ip}:${ph.target_port} (${ph.target_source})`
    : ph.target_ip ? `command link ✗ ${ph.last_error || "connecting…"}` : "command link — phone IP unknown";
  cs.classList.toggle("ok", !!ph.connected);
  if (document.activeElement !== $("phone-input"))
    $("phone-input").placeholder = ph.explicit_ip ? ph.explicit_ip : "auto" + (ph.target_ip ? ` (${ph.target_ip})` : "");
}

// ------------------------------------------------------------------ plot
const canvas = $("plot");
const ctx = canvas.getContext("2d");
let yLo = -0.5, yHi = 1.5;

function niceStep(range, n) {
  const raw = range / n, p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p;
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
}

function drawPlot(now) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const fs = Math.max(11, Math.min(17, W / 95));
  const padL = fs * 3.2, padR = fs * 0.4, padT = fs * 0.9, padB = fs * 1.9;
  const pw = W - padL - padR, plotH = H - padT - padB;
  if (pw < 20 || plotH < 20) return;

  // phone-time "now": extrapolate from the last frame so the trace scrolls smoothly
  const tNow = last ? last.t + Math.min((now - lastPerf) / 1000, 0.25) : 0;
  const t0 = tNow - WINDOW_S;
  const X = (t) => padL + (t - t0) / WINDOW_S * pw;

  // autoscale (eased); the band can widen the range but is capped
  let lo = -0.3, hi = 1.2;
  for (const p of buf) {
    if (p.t < t0) continue;
    const b = Math.min(2 * (p.sx || 0), 1.5);
    lo = Math.min(lo, p.vx - b, p.vy); hi = Math.max(hi, p.vx + b, p.vy);
  }
  lo = Math.max(lo, -3); hi = Math.min(hi, 4);
  const pad = (hi - lo) * 0.06; lo -= pad; hi += pad;
  yLo += (lo - yLo) * 0.06; yHi += (hi - yHi) * 0.06;
  const Y = (v) => padT + (yHi - v) / (yHi - yLo) * plotH;

  // hairline grid
  ctx.font = `${fs * 0.82}px ${COL.mono}`;
  ctx.lineWidth = 1;
  ctx.strokeStyle = COL.grid;
  for (let i = 0; i <= 10; i++) {
    const x = Math.round(padL + i / 10 * pw) + 0.5;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
  }
  const step = niceStep(yHi - yLo, Math.max(3, plotH / (fs * 3.6)));
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (let v = Math.ceil(yLo / step) * step; v <= yHi; v += step) {
    const y = Math.round(Y(v)) + 0.5;
    const zero = Math.abs(v) < 1e-9;
    ctx.strokeStyle = zero ? COL.ink3 : COL.grid;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();
    ctx.fillStyle = COL.ink3;
    ctx.fillText(v.toFixed(step < 0.1 ? 2 : 1), padL - fs * 0.6, y);
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top"; ctx.fillStyle = COL.ink3;
  for (let s = 0; s <= WINDOW_S; s += 4) {
    const x = padL + (1 - s / WINDOW_S) * pw;
    ctx.textAlign = s === 0 ? "right" : "center";
    ctx.fillText(s === 0 ? "now" : `−${s} s`, x, padT + plotH + fs * 0.6);
  }
  ctx.textAlign = "left"; ctx.textBaseline = "top";
  ctx.fillText("m/s", 0, padT - fs * 0.7);

  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT, pw, plotH); ctx.clip();

  // visible segments, split at telemetry gaps > 0.25 s
  const segs = []; let cur = [];
  for (const p of buf) {
    if (p.t < t0 - 0.5) continue;
    if (cur.length && p.t - cur[cur.length - 1].t > 0.25) { segs.push(cur); cur = []; }
    cur.push(p);
  }
  if (cur.length) segs.push(cur);

  // faint columns where flow was lost
  ctx.fillStyle = COL.lostCol;
  for (const s of segs) {
    let start = null;
    for (let i = 0; i < s.length; i++) {
      const dead = !(s[i].st & BIT.FLOW_OK);
      if (dead && start === null) start = s[i].t;
      if ((!dead || i === s.length - 1) && start !== null) {
        ctx.fillRect(X(start), padT, Math.max(1, X(s[i].t) - X(start)), plotH);
        start = null;
      }
    }
  }

  // v_x ±2σ band: warm grey, faint vermilion while flow is lost
  for (const s of segs) {
    let i = 0;
    while (i < s.length - 1) {
      const dead = !(s[i].st & BIT.FLOW_OK);
      let j = i;
      while (j < s.length - 1 && !(s[j + 1].st & BIT.FLOW_OK) === dead) j++;
      const run = s.slice(i, Math.min(j + 2, s.length));   // overlap one sample so runs join
      ctx.beginPath();
      run.forEach((p, k) => { const x = X(p.t), y = Y(p.vx + 2 * p.sx); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      for (let k = run.length - 1; k >= 0; k--) ctx.lineTo(X(run[k].t), Y(run[k].vx - 2 * run[k].sx));
      ctx.closePath();
      ctx.fillStyle = dead ? COL.bandLost : COL.band; ctx.fill();
      if (dead) { ctx.strokeStyle = COL.alert; ctx.globalAlpha = 0.55; ctx.lineWidth = 1; ctx.stroke(); ctx.globalAlpha = 1; }
      i = j + 1;
    }
  }

  // traces: v_y thin, v_x graphite
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  for (const [key, col, w] of [["vy", COL.ink2, 1.1], ["vx", COL.ink, Math.max(2, fs / 7)]]) {
    ctx.strokeStyle = col; ctx.lineWidth = w;
    for (const s of segs) {
      ctx.beginPath();
      s.forEach((p, i) => { const x = X(p.t), y = Y(p[key]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
    }
  }
  if (buf.length) {
    const p = buf[buf.length - 1];
    ctx.fillStyle = COL.ink;
    ctx.beginPath(); ctx.arc(X(p.t), Y(p.vx), Math.max(3, fs / 4), 0, 2 * Math.PI); ctx.fill();
  }
  ctx.restore();
}

function loop(now) {
  drawPlot(now);
  renderFrame(now);
  requestAnimationFrame(loop);
}

// ------------------------------------------------------------------ commands
const logEl = $("cmdlog");
let logN = 0;
function logCmd(req, rep, ms) {
  const li = document.createElement("li");
  const ts = new Date().toLocaleTimeString([], { hour12: false });
  const ok = rep && rep.ok;
  const extra = Object.assign({}, rep); delete extra.ok; delete extra.cmd; delete extra.local;
  const reqx = Object.assign({}, req); delete reqx.cmd;
  li.innerHTML = `<span class="ts"></span><span class="${ok ? "ok" : "err"}"></span><span class="rep"></span>`;
  li.children[0].textContent = ts;
  li.children[1].textContent = (ok ? "✓ " : "✗ ") + req.cmd + (Object.keys(reqx).length ? " " + Object.values(reqx).join(" ") : "");
  li.children[2].textContent = (rep && rep.error ? rep.error : Object.keys(extra).length ? JSON.stringify(extra) : "ok") + `  ${ms.toFixed(0)} ms`;
  li.title = JSON.stringify(rep);
  logEl.prepend(li);
  while (logEl.children.length > 60) logEl.lastChild.remove();
  logN++;
  const suffix = ok ? "" : " · last failed";
  $("logcount").textContent = `${logN}${suffix}`;
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

let lastResult = null;
function setResult(dist) {
  lastResult = dist;
  const el = $("result");
  if (dist == null) { el.textContent = ""; el.className = "result num"; return; }
  const target = parseFloat($("target").value);
  if (!(target > 0)) { el.textContent = `run ${dist.toFixed(2)} m`; el.className = "result num"; return; }
  const err = (dist - target) / target * 100;
  el.textContent = `run ${dist.toFixed(2)} m  ·  error ${err >= 0 ? "+" : "−"}${Math.abs(err).toFixed(1)} %`;
  el.className = "result num " + (Math.abs(err) <= 2 ? "good" : Math.abs(err) <= 5 ? "meh" : "bad");
}

$("btn-start").onclick = (e) => {
  const label = $("label").value.trim().replace(/\s+/g, "_");
  setResult(null);
  send(label ? { cmd: "start_run", label } : { cmd: "start_run" }, e.currentTarget);
};
$("btn-stop").onclick = async (e) => {
  const distAtStop = last ? last.distance : null;
  const rep = await send({ cmd: "stop_run" }, e.currentTarget);
  if (rep.ok) setResult(typeof rep.distance === "number" ? rep.distance : distAtStop);
};
$("btn-mark").onclick = (e) => send({ cmd: "mark" }, e.currentTarget);
$("btn-cal").onclick = (e) => send({ cmd: "calibrate" }, e.currentTarget);
$("btn-reset").onclick = (e) => send({ cmd: "reset_distance" }, e.currentTarget);
$("btn-ping").onclick = (e) => send({ cmd: "ping" }, e.currentTarget);
const slider = $("torch-slider");
let sliderTouched = 0;
slider.oninput = () => { sliderTouched = performance.now(); $("torch-val").textContent = slider.value + "%"; };
slider.onchange = () => { sliderTouched = performance.now(); slider.blur(); send({ cmd: "set_torch", level: Math.round(slider.value) / 100 }); };
$("btn-phone").onclick = async () => {
  const ip = $("phone-input").value.trim();
  await fetch("/api/phone", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ ip }) });
  $("phone-input").value = "";
  $("phone-input").blur();
};
$("phone-input").addEventListener("keydown", (e) => { if (e.key === "Enter") $("btn-phone").click(); });
$("target").addEventListener("change", () => { if (lastResult != null) setResult(lastResult); });

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
connect();
requestAnimationFrame(loop);
