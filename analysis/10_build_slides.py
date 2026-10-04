"""Phase 10: builds the slideshow analysis/out/GroundSpeed_Forensics.pptx from final_metrics.json and
the figures. Every number on a slide is read from final_metrics.json (source named in the footer)."""
import json
import os

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

from common import FIG, OUT

W, H = Inches(13.333), Inches(7.5)
INK = RGBColor(0x1F, 0x23, 0x2B); MUTED = RGBColor(0x5F, 0x66, 0x70); BLUE = RGBColor(0x00, 0x72, 0xB2)
ORANGE = RGBColor(0xE6, 0x9F, 0x00); RED = RGBColor(0xD5, 0x5E, 0x00); GREEN = RGBColor(0x00, 0x9E, 0x73)
LIGHT = RGBColor(0xF2, 0xF5, 0xF8); WHITE = RGBColor(0xFF, 0xFF, 0xFF); LINE = RGBColor(0xD0, 0xD6, 0xDD)
TAG = {"CONFIRMED": GREEN, "SUPPORTED": BLUE, "HYPOTHESIS": ORANGE, "ASSUMPTION": MUTED, "FAILED": RED}

M = json.load(open(os.path.join(OUT, "final_metrics.json"), encoding="utf-8"))


def f(x, d=2, sign=False):
    return (f"{x:+.{d}f}" if sign else f"{x:.{d}f}").replace("-", "−")


prs = Presentation()
prs.slide_width, prs.slide_height = W, H
BLANK = prs.slide_layouts[6]
_n = [0]


def text(slide, x, y, w, h, runs, size=14, color=INK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, spacing=1.05):
    """runs: str or list of paragraphs; a paragraph is str or list of (text, {bold,color,size}) tuples."""
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.04); tf.margin_top = tf.margin_bottom = Inches(0.02)
    paras = runs if isinstance(runs, list) else [runs]
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = align; para.line_spacing = spacing
        parts = p if isinstance(p, list) else [(p, {})]
        for t, st in parts:
            r = para.add_run(); r.text = t
            r.font.size = Pt(st.get("size", size)); r.font.bold = st.get("bold", bold)
            r.font.color.rgb = st.get("color", color); r.font.name = "Calibri"
        para.space_after = Pt(st.get("after", 4) if parts else 4)
    return tb


def bullets(slide, x, y, w, h, items, size=14):
    paras = []
    for it in items:
        if isinstance(it, tuple):  # (bold lead, rest)
            paras.append([("• " + it[0], {"bold": True}), (" " + it[1], {})])
        else:
            paras.append([("• " + it, {})])
    return text(slide, x, y, w, h, paras, size=size)


def rect(slide, x, y, w, h, fill=LIGHT, line=None, shape=MSO_SHAPE.RECTANGLE):
    s = slide.shapes.add_shape(shape, x, y, w, h)
    s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line; s.line.width = Pt(1)
    s.shadow.inherit = False
    return s


def figure(slide, name, x, y, w, h):
    p = os.path.join(FIG, name)
    iw, ih = Image.open(p).size
    sc = min(w / iw, h / ih)
    pw, ph = int(iw * sc), int(ih * sc)
    return slide.shapes.add_picture(p, x + (w - pw) // 2, y + (h - ph) // 2, pw, ph)


def card(slide, x, y, w, h, value, label, color=BLUE):
    rect(slide, x, y, w, h, LIGHT)
    rect(slide, x, y, Inches(0.07), h, color)
    text(slide, x + Inches(0.15), y + Inches(0.05), w - Inches(0.2), Inches(0.55), value, size=22, bold=True, color=color)
    text(slide, x + Inches(0.15), y + Inches(0.6), w - Inches(0.2), h - Inches(0.62), label, size=10.5, color=MUTED)


def tag(slide, x, y, kind):
    s = rect(slide, x, y, Inches(1.25), Inches(0.28), TAG[kind], shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    tf = s.text_frame; tf.margin_top = tf.margin_bottom = 0
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
    r = p.add_run(); r.text = kind; r.font.size = Pt(10); r.font.bold = True; r.font.color.rgb = WHITE


def new_slide(title, kicker=None, source=None, notes=None):
    s = prs.slides.add_slide(BLANK)
    _n[0] += 1
    rect(s, 0, 0, W, Inches(0.09), BLUE)
    if kicker:
        text(s, Inches(0.55), Inches(0.25), Inches(12), Inches(0.35), kicker.upper(), size=11, bold=True, color=BLUE)
    text(s, Inches(0.55), Inches(0.52), Inches(12.3), Inches(0.9), title, size=27, bold=True)
    foot = f"{_n[0]}"
    text(s, Inches(12.3), Inches(7.08), Inches(0.6), Inches(0.3), foot, size=10, color=MUTED, align=PP_ALIGN.RIGHT)
    if source:
        text(s, Inches(0.55), Inches(7.08), Inches(11.5), Inches(0.3), "Source: " + source, size=9, color=MUTED)
    if notes:
        s.notes_slide.notes_text_frame.text = notes
    return s


def table(slide, x, y, w, rows, col_w, size=11, row_h=0.32, header_fill=BLUE):
    nr, nc = len(rows), len(rows[0])
    shp = slide.shapes.add_table(nr, nc, x, y, w, Inches(row_h * nr))
    tb = shp.table
    for j, cw in enumerate(col_w):
        tb.columns[j].width = Inches(cw)
    for i, row in enumerate(rows):
        tb.rows[i].height = Inches(row_h)
        for j, v in enumerate(row):
            c = tb.cell(i, j); c.text = ""
            c.margin_left = c.margin_right = Inches(0.06); c.margin_top = c.margin_bottom = Inches(0.02)
            p = c.text_frame.paragraphs[0]
            r = p.add_run(); r.text = str(v); r.font.size = Pt(size); r.font.name = "Calibri"
            r.font.bold = (i == 0); r.font.color.rgb = WHITE if i == 0 else INK
            c.fill.solid(); c.fill.fore_color.rgb = header_fill if i == 0 else (LIGHT if i % 2 else WHITE)
    return shp


def flow_diagram(slide, x, y, boxes, bw=1.95, bh=1.05, gap=0.28, fills=None):
    for i, (head, body) in enumerate(boxes):
        bx = x + Inches(i * (bw + gap))
        b = rect(slide, bx, y, Inches(bw), Inches(bh), (fills[i] if fills else LIGHT), line=LINE, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
        text(slide, bx + Inches(0.06), y + Inches(0.05), Inches(bw - 0.12), Inches(0.35), head, size=12.5, bold=True, color=BLUE)
        text(slide, bx + Inches(0.06), y + Inches(0.38), Inches(bw - 0.12), Inches(bh - 0.4), body, size=10, color=INK)
        if i < len(boxes) - 1:
            c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, bx + Inches(bw), y + Inches(bh / 2), bx + Inches(bw + gap), y + Inches(bh / 2))
            c.line.color.rgb = MUTED; c.line.width = Pt(1.75)
            c.line._get_or_add_ln().append(_arrow())


def _arrow():
    from pptx.oxml.ns import qn
    from lxml import etree
    e = etree.SubElement(etree.Element("dummy"), qn("a:tailEnd")); e.set("type", "triangle"); e.set("w", "med"); e.set("len", "med")
    return e


# ------------------------------------------------------------------ data shortcuts
dr, vr, mcd, mcv = M["distance_real"], M["velocity_real"], M["mc_distance"], M["mc_velocity"]
ins, loo = dr["final_in_sample"], dr["final_leave_one_out"]
env, st = vr["envelope_stats"], M["story"]
S2, S1 = mcd["new_run"], mcd["repeat_run"]
VM = mcv["main"]
cal = M["calibration_refit"]
FIN = "2026-10-04_01-59-42"

# ================================================================== 1 Title
s = prs.slides.add_slide(BLANK); _n[0] += 1
rect(s, 0, 0, W, H, RGBColor(0x0E, 0x1A, 0x2B))
rect(s, Inches(0.7), Inches(2.0), Inches(0.12), Inches(2.6), ORANGE)
text(s, Inches(1.0), Inches(1.9), Inches(11), Inches(1.0), "Open Ground Speed Sensor", size=44, bold=True, color=WHITE)
text(s, Inches(1.0), Inches(2.85), Inches(11.5), Inches(0.8), "A forensic reconstruction of one night of testing: what broke, what we fixed, and how accurate it really is",
     size=21, color=RGBColor(0xC9, 0xD6, 0xE3))
text(s, Inches(1.0), Inches(3.95), Inches(11.5), Inches(0.9),
     "Objective: turn an iPhone (camera + torch + LiDAR + IMU + GPS) on a cart or car into an optical ground-speed and distance sensor, "
     "like a commercial Correvit, with a Kalman filter running on the phone.", size=15, color=RGBColor(0xE8, 0xEE, 0xF4))
text(s, Inches(1.0), Inches(5.6), Inches(11.5), Inches(1.0),
     ["MHacks 2026 · team repository davidzonavirus/kalman-hackathon-26 (app: David · Kalman filter: Joseph · dashboard: Sean)",
      "Independent re-analysis of all 58 recorded runs, rebuilt from code, logs and git history · every number traceable to analysis/out/final_metrics.json"],
     size=12, color=RGBColor(0x9F, 0xB3, 0xC8))
s.notes_slide.notes_text_frame.text = "Team roles from README.md. Analysis code: analysis/ (run_all.py)."

# ================================================================== 2 What we built
s = new_slide("An iPhone looking at the floor, fused by a 4-state Kalman filter", "What we built",
              "README.md, docs/HACKATHON_WRITEUP.md §2, docs/KALMAN_INTEGRATION.md, SensorFusionEngine.swift, FlowConverter.swift",
              "Pipeline as found in the code, not assumed. Velocity is primary; distance is integrated from the filtered velocity.")
flow_diagram(s, Inches(0.55), Inches(1.65), [
    ("Camera", "1280×720 @ 240 fps, torch, exposure ≤ 1/1000 s; 256² centre crop → 128²"),
    ("Phase correlation", "image shift (px/frame) + quality PSR (peak-to-sidelobe ratio)"),
    ("Shift → velocity", "v = shift·2·h / (f·Δt); h from LiDAR, f from lens FOV; gyro de-rotation"),
    ("Kalman filter", "x = [vₓ, v_y, bₓ, b_y]; IMU 100 Hz predict; flow, GPS speed, zero-velocity updates"),
    ("Distance", "∫|v̄|dt: 0.05 m/s deadband, 0.5 s low-pass, lead term"),
])
text(s, Inches(0.55), Inches(3.0), Inches(12.3), Inches(0.4), "Where error can enter the measurement", size=15, bold=True)
bullets(s, Inches(0.55), Inches(3.4), Inches(6.1), Inches(3.4), [
    ("Scale (multiplies everything):", "camera height h and focal length f. A 1 mm error in h at 19 cm = 0.5 % in speed and distance."),
    ("Image noise:", "per-frame shift jitter, weighted in the filter by 1/PSR²; frames with PSR < 8 are dropped."),
    ("Tracking failures:", "blur, window wrap, lock onto the sensor's fixed pattern → speed reads low or zero."),
    ("Integration:", "deadband and smoothing change how speed becomes distance."),
], size=13)
rect(s, Inches(7.0), Inches(3.45), Inches(5.8), Inches(3.3), LIGHT)
text(s, Inches(7.2), Inches(3.55), Inches(5.5), Inches(3.1), [
    [("What counts as truth in this archive", {"bold": True, "size": 14})],
    [("Cart: ", {"bold": True}), ("taped 16 ft (4.877 m) and 20 ft (6.096 m) courses → distance truth only, no speed-vs-time truth.", {})],
    [("Car: ", {"bold": True}), ("phone GPS Doppler speed at 1 Hz → an independent but noisy speed reference, no distance truth.", {})],
    [("Neither ", {"bold": True}), ("is a calibrated reference (no wheel encoder, no RTK). Every accuracy number in this deck inherits that limitation.", {})],
], size=12.5)

# ================================================================== 3 The data
h_ = M["data_health"]
s = new_slide(f"{h_['n_runs']} runs, 4 sensor streams, and clean timestamps", "The data",
              "analysis/out/inventory_runs.csv, data_health.json, figures/story_raw_push.png")
table(s, Inches(0.55), Inches(1.55), Inches(5.6), [
    ["Stream (file)", "Rate", "Content"],
    ["flow.csv", "240 Hz (60–120 early)", "camera vₓ, v_y (m/s), PSR, h"],
    ["imu.csv", "100 Hz", "accel (gravity removed), gyro"],
    ["gnss.csv", "~1 Hz", "speed, speed accuracy, course"],
    ["est.csv", "100 Hz", "filter v, σ, distance, status"],
    ["events.csv", "—", "start/stop, ZUPT, calibrate…"],
    ["lidar/*.csv", "~30 Hz, 1.5 s", "32 height measurements"],
], [1.35, 1.75, 2.5], size=11)
bullets(s, Inches(0.55), Inches(4.0), Inches(5.6), Inches(3.0), [
    ("Archive:", f"{h_['n_runs']} phone runs, {h_['n_lidar']} LiDAR logs, {h_['n_dash']} dashboard sessions, 21 iOS crash/resource reports."),
    ("Health:", f"{h_['total_nan']} NaN, {h_['total_inf']} infinite values, 0 backwards timestamps; 1 duplicate flow and 3 duplicate GPS timestamps (dropped)."),
    ("Replay check:", "our Python port of the filter reproduces the phone's own est.csv on all 53 runs from 20:06 on (median max |Δv| 2e-6 m/s, worst 0.015 m/s), so we analyse exactly what ran."),
    ("Gaps:", "camera pauses of 2.1 s during in-run height measurements; a 0.84 s camera interruption in the final car run."),
], size=12.5)
figure(s, "story_raw_push.png", Inches(6.35), Inches(1.5), Inches(6.6), Inches(5.4))

# ================================================================== 4 Timeline
s = new_slide("Git has one code commit for the whole night, so the run metadata tells the story", "Chronology",
              "meta.json (focal_px, mount_height_m), flow.csv rates, ios_diagnostics/ file times; git log (4 commits)",
              "Git history: a9e3712 initial, 379dcfd 19:41 (whole project), aa0d32a 02:12 (all fixes), ef48eaa 02:25 (data + write-up). Intermediate builds are only visible through what they logged.")
figure(s, "story_timeline.png", Inches(0.45), Inches(1.45), Inches(8.6), Inches(5.5))
bullets(s, Inches(9.2), Inches(1.6), Inches(3.8), Inches(5.3), [
    ("19:34–20:31", "Debug build: 60–120 frames/s processed; h stuck at the 0.30 m default."),
    ("20:26–21:43", f"{st['camera_crash_reports']['n']} camera-daemon crash reports, all after LiDAR height measurements."),
    ("22:22–22:25", "16 ft pushes read +7 % → focal length ×1.073 at 22:37, then ×1.027 plus a −7.9 mm LiDAR offset from 22:54."),
    ("22:59–00:01", "79 cm mount and varying-ride-height experiments (both failed, see slide 9)."),
    ("01:14–01:59", "three car drives; the image-expansion height tracker drifts in drives 1–2 and is switched off for drive 3."),
], size=12)

# ================================================================== 5 Problem/Fix 1: camera crash
s = new_slide("Problem 1: the camera died after every height measurement, and the IMU kept 'driving'", "Problem → fix #1",
              "story_metrics.json; runs 2026-10-03_21-37-55, 21-34-31, 21-46-55, 21-48-05; ios_diagnostics/cameracaptured-*.ips; writeup §3.6")
figure(s, "story_imu_runaway.png", Inches(0.45), Inches(1.45), Inches(8.0), Inches(2.9))
r1 = st["imu_only_2026-10-03_21-37-55"]; r2 = st["imu_only_2026-10-03_21-34-31"]
card(s, Inches(8.75), Inches(1.55), Inches(2.0), Inches(1.25), f"{r1['distance_m']:.1f} m", f"in {r1['dur_s']:.0f} s with the phone at rest (21:37)", RED)
card(s, Inches(10.9), Inches(1.55), Inches(2.0), Inches(1.25), f"{st['camera_crash_reports']['n']}", "camera-daemon crash reports, 20:26–21:43", RED)
tag(s, Inches(8.75), Inches(3.0), "CONFIRMED")
text(s, Inches(10.1), Inches(2.98), Inches(2.9), Inches(0.35), "crash logs + zero flow frames", size=10, color=MUTED)
bullets(s, Inches(0.55), Inches(4.5), Inches(6.2), Inches(2.5), [
    ("Symptom:", f"no flow frames at all; IMU-only velocity integrates accelerometer bias: {r2['distance_m']:.1f} m in {r2['dur_s']:.0f} s, {r1['distance_m']:.1f} m in {r1['dur_s']:.0f} s."),
    ("Root cause (from crash logs):", "LiDAR leaves the camera in a 30 fps mode; asking for 240 fps straight afterwards aborted iOS's camera service."),
], size=12.5)
bullets(s, Inches(6.9), Inches(4.5), Inches(6.0), Inches(2.5), [
    ("Fix:", "restart at 30 fps and step up to 240; watchdog restarts the camera after 1.5 s without flow; after 2 s without flow any still IMU moment forces v = 0."),
    ("Verified:", "the next two in-run height measurements (21:46, 21:48) show a 2.1 s camera gap, then flow resumes; those runs end at 0.07 m and 0.00 m at rest. No crash reports in the archive after 21:43."),
], size=12.5)

# ================================================================== 6 Problem 2: LiDAR tilt
lt = st["lidar_tilt_example"]
s = new_slide(f"Problem 2: camera tilt was counted twice, inflating speed by {lt['speed_error_pct_if_bug']:.0f} % at {lt['tilt_deg_median']:.0f}°", "Problem → fix #2",
              "git diff 379dcfd..aa0d32a DepthSource.swift; data/lidar/2026-10-03_20-26-19.csv; story_metrics.json",
              "The 19:41 code stored the LiDAR depth along the optical axis as the height; FlowConverter then divided by cos θ again. Error factor 1/cos²θ.")
figure(s, "story_lidar_tilt.png", Inches(0.45), Inches(1.45), Inches(6.6), Inches(4.1))
tag(s, Inches(7.4), Inches(1.6), "CONFIRMED")
text(s, Inches(8.75), Inches(1.58), Inches(4.2), Inches(0.35), "code diff + logged LiDAR frames", size=10, color=MUTED)
bullets(s, Inches(7.4), Inches(2.05), Inches(5.5), Inches(4.6), [
    ("Evidence in code:", "commit 379dcfd uses the centre depth directly as h (no cos θ); FlowConverter divides h by cos θ for tilt. Commit aa0d32a multiplies depth by cos θ once."),
    ("Evidence in data:", f"the 20:26 attempt logs depth {lt['depth_median_m']:.3f} m at {lt['tilt_deg_median']:.0f}° tilt → vertical h {lt['vertical_h_m']:.3f} m. Under the bug, speed reads 1/cos²θ = +{lt['speed_error_pct_if_bug']:.0f} % high."),
    ("Fix:", "convert depth to vertical height once; reject readings with > 8 % frame spread, non-metric depth, or h outside 5 cm–2 m; log every frame."),
    ("Limit:", "no recorded run used a buggy LiDAR height (all runs before 20:36 used the 0.30 m default), so there is no before/after distance for this one. The size of the bug comes from geometry plus the logged frames."),
], size=12.5)

# ================================================================== 7 Problem/Fix 3: +7 % scale
pre = dr["before_any_fix_pct"]; first = dr["after_first_fix_pct"]
s = new_slide("Problem 3: every push read 7 % long; a 2-parameter calibration fixed it", "Problem → fix #3",
              "distance_calibration.json, distance_runs.csv (replay of each run with the final constants), meta.json focal_px",
              "Calibration refit: least squares over the six 16 ft runs; the scale for each run is applied by replaying its logged sensors through the filter.")
figure(s, "dist_calibration.png", Inches(0.45), Inches(1.45), Inches(6.4), Inches(3.9))
card(s, Inches(7.2), Inches(1.55), Inches(1.85), Inches(1.25), f"+{min(pre):.1f}…+{max(pre):.1f} %", "runs 5–7, before any fix", RED)
card(s, Inches(9.2), Inches(1.55), Inches(1.85), Inches(1.25), f"{f(min(first),1)}…{f(max(first),1)} %", "runs 9–11, focal ×1.073", ORANGE)
card(s, Inches(11.2), Inches(1.55), Inches(1.8), Inches(1.25), f"±{ins['max_abs_pct']:.2f} %", "all six, final (in-sample)", GREEN)
bullets(s, Inches(7.2), Inches(3.0), Inches(5.8), Inches(4.0), [
    ("Cause (supported):", "at 240 fps iOS gives no per-frame intrinsics, so f came from the field of view; barrel distortion magnifies the image centre where the flow patch sits. Path length, net displacement and raw flow all agreed, so it was a scale error, not filtering."),
    ("Fix:", f"focal ×1.027 and LiDAR height −7.9 mm. Our independent refit gives ×{cal['focal_correction']:.4f} ± {cal['focal_correction_se']:.3f} and {cal['height_offset_m']*1000:.1f} ± {cal['height_offset_se_m']*1000:.1f} mm (correlation {cal['param_corr']:.2f})."),
    ("Caveat:", f"2 parameters fitted to 2 heights force each height's mean error to 0, so ±{ins['max_abs_pct']:.2f} % shows repeatability. Leave-one-out prediction is the honest figure: max ±{loo['max_abs_pct']:.2f} %."),
], size=12)
bullets(s, Inches(0.55), Inches(5.45), Inches(6.3), Inches(1.5), [
    "Focal-only fit (no height offset) leaves ±0.8 % and an error that grows with height; the offset term removes that trend.",
], size=12)

# ================================================================== 8 Car problems
br = vr["car_runs_band_ratio"]
s = new_slide("Problem 4: in the car, three failures in a row, each found in the logs", "Problem → fix #4",
              "velocity_summary.json (filter replayed without GPS vs GPS speed), figures/vel_car_runs_timeseries.png, vel_height_tracker.png; writeup §3.13–3.16")
figure(s, "vel_car_runs_timeseries.png", Inches(0.4), Inches(1.4), Inches(7.4), Inches(5.6))
c1 = br["2026-10-04_01-19-31"]; c2 = br["2026-10-04_01-45-16"]
seg0 = vr["segments"][0]
table(s, Inches(7.95), Inches(1.5), Inches(5.0), [
    ["Drive", "Main failure (evidence)", "speed ÷ GPS"],
    ["1 (01:19)", "height tracker collapsed 0.17 → 0.06 m; window wrap ≈ 6 m/s", f"{c1['4.5–6.7 (10–15 mph)']:.2f} (median, 10–15 mph)"],
    ["2 (01:45)", "tracker drifted to 0.11 m; zero-lock at t = 163 s", f"{c2['4.5–6.7 (10–15 mph)']:.2f} (median, 10–15 mph)"],
    ["3 (01:59)", "height fixed by LiDAR; motion prediction", f"{seg0['ratio']:.3f} (steady 4.5–5.3 m/s)"],
], [0.9, 2.65, 1.45], size=10, row_h=0.5)
bullets(s, Inches(7.95), Inches(4.2), Inches(5.0), Inches(2.8), [
    ("Fixes:", "motion-predicted correlation window (bench range 3 → 9 m/s); height tracking switched off; coast 24 frames before re-acquiring; weak results that jump far are rejected."),
    ("Still failing in drive 3:", f"above ~25 mph blur drops PSR to ~11; the tracker then locked onto zero and read 0 from t = {vr['zero_lock']['first_t']:.0f} to {vr['zero_lock']['last_t']:.0f} s while GPS said 4–6.6 m/s."),
    ("Not verified:", "the 'fake-stop' rule meant to prevent that lock (commit ef48eaa) was never driven: the only runs recorded after it are three short indoor pushes (02:26–02:27) with no GPS and no course length."),
], size=11.5)

# ================================================================== 9 What didn't work
s = new_slide("Failures we kept: high mounts and changing ride height are outside the envelope", "What did not work",
              "distance_runs.csv (as recorded; these runs already used the final calibration), writeup §3.8, §3.10")
figure(s, "dist_all_runs.png", Inches(0.4), Inches(1.4), Inches(8.3), Inches(4.4))
tag(s, Inches(8.95), Inches(1.5), "FAILED")
bullets(s, Inches(8.95), Inches(1.9), Inches(4.05), Inches(5.0), [
    ("79 cm mount:", "−6.6, −16.7, −15.0 %. Correction: the write-up says runs 15–17 read '15–17 % low', but run 15 (a faster push) was −6.6 %."),
    ("Static-pattern subtraction:", "−8.4 → −0.4 % in simulation, −31.3 % on the real cart. Reverted."),
    ("Ride height changed mid-run:", "−6.2, −13.7, −24.6 % with fixed h; image-expansion tracking gave −3.4 % once and −34 % the next run."),
    ("Hypothesis (untested):", "at 79 cm the floor moves ~1 px/frame and static image content pulls the shift toward zero."),
], size=12)
text(s, Inches(0.55), Inches(5.95), Inches(8.2), Inches(1.0),
     "Lesson: the simulator (flowbench) was right about window wrap and blur, and wrong twice about real optics. Only fully logged runs caught that.",
     size=13, color=MUTED)

# ================================================================== 10 Final pipeline
s = new_slide("The final configuration (what the accuracy numbers below describe)", "Final estimation pipeline",
              "CameraFlowSource.swift, DepthSource.swift, GroundSpeedFilter.swift (FilterConfig), SensorFusionEngine.swift; meta.json of final runs")
flow_diagram(s, Inches(0.55), Inches(1.55), [
    ("LiDAR h (once)", "depth·cos θ − 7.9 mm; cart still; also zeroes IMU bias"),
    ("Flow 240 Hz", "f = FOV focal ×1.027; motion-predicted window; PSR ≥ 8"),
    ("De-rotate", "v −= h·ω (gyro); wobble lowers PSR weight"),
    ("ReferenceKF4", "q_accel 0.1, q_bias 1e-5, R = 1.6e-3·(20/PSR)², gate 9"),
    ("Odometer", "∫|v̄|, deadband 0.05, τ 0.5 s, lead term"),
], fills=[LIGHT] * 5)
table(s, Inches(0.55), Inches(2.95), Inches(12.2), [
    ["Change", "When (evidence)", "What it fixed", "Status"],
    ["Release build, 240 fps", "20:36 (flow rate 69–118 → 232–240 Hz)", "fewer px per frame at a given speed → higher PSR", "kept"],
    ["Camera restart / watchdog / ZUPT after flow loss", "after 21:43 (no later crash reports)", "IMU-only runaway after Measure h", "kept"],
    ["LiDAR tilt: depth × cos θ once", "aa0d32a code diff", "+61 % speed at 38° tilt", "kept"],
    ["Distance low-pass τ = 0.5 s + lead", "21:20 (inferred from est.csv)", "wobble counted as distance", "kept"],
    ["Focal ×1.027, LiDAR −7.9 mm", "22:54 (focal_px 883.89)", "+7 % scale error", "kept"],
    ["Image-expansion height tracking", "00:00–01:45", "variable ride height", "OFF (failed in car)"],
    ["Motion prediction, coast, relock PSR 30", "01:45 and 01:59 drives", "wrap at 6 m/s; zero-lock", "kept"],
    ["Fake-stop rule; prediction only > 32 px/frame", "ef48eaa, 5295780 (02:37)", "zero-lock after overspeed; keeps cart path calibrated", "NOT ROAD-TESTED"],
], [3.6, 3.3, 3.6, 1.7], size=11, row_h=0.37)

# ================================================================== 11 Distance accuracy
s = new_slide(f"Distance: about ±{loo['max_abs_pct']:.1f} % on a 4.9 m course, within a narrow envelope", "Distance accuracy (real data)",
              "distance_calibration.json, distance_runs.csv",
              "Error = replayed final-calibration distance − tape length. In-sample: calibration fitted on the same 6 runs. Leave-one-out: each run predicted with constants fitted on the other 5. "
              "t-based 95% CI on the mean and 95% prediction interval for a new run, n = 6.")
figure(s, "dist_calibration.png", Inches(0.4), Inches(1.5), Inches(6.2), Inches(3.8))
table(s, Inches(6.85), Inches(1.55), Inches(6.1), [
    ["Metric (6 runs, 4.877 m)", "In-sample", "Leave-one-out"],
    ["Bias (mean error)", f"{f(ins['bias_pct'],2,True)} %", f"{f(loo['bias_pct'],2,True)} %"],
    ["MAE", f"{ins['mae_pct']:.2f} %  ({ins['mae_pct']*48.77:.0f} mm)", f"{loo['mae_pct']:.2f} %"],
    ["RMSE", f"{ins['rmse_pct']:.2f} %  ({ins['rmse_m']*1000:.0f} mm)", f"{loo['rmse_pct']:.2f} %"],
    ["SD", f"{ins['sd_pct']:.2f} %", f"{loo['sd_pct']:.2f} %"],
    ["Max |error| (observed)", f"{ins['max_abs_pct']:.2f} %  ({ins['max_abs_m']*1000:.0f} mm)", f"{loo['max_abs_pct']:.2f} %"],
    ["95 % CI of the mean", f"{f(ins['ci95_mean_pct'][0])}…{f(ins['ci95_mean_pct'][1])} %", f"{f(loo['ci95_mean_pct'][0])}…{f(loo['ci95_mean_pct'][1])} %"],
    ["95 % prediction interval, new run", f"{f(ins['pi95_new_run_pct'][0])}…{f(ins['pi95_new_run_pct'][1])} %", f"{f(loo['pi95_new_run_pct'][0])}…{f(loo['pi95_new_run_pct'][1])} %"],
], [2.6, 1.8, 1.7], size=11, row_h=0.36)
bullets(s, Inches(0.55), Inches(5.45), Inches(12.4), Inches(1.6), [
    ("Valid for:", "indoor floor, mount 0.186–0.235 m, walking pace, straight 16 ft course. Six runs is a small sample: the SD itself has a 95 % CI of "
     f"{ins['sd_ci95_pct'][0]:.2f}–{ins['sd_ci95_pct'][1]:.2f} % (in-sample)."),
    ("Not valid for:", "79 cm mounts (−7 to −17 %), mid-run height changes (−6 to −34 %), or the car (no distance truth there; integrated GPS agrees within "
     f"{f(vr['distance_vs_gnss']['diff_pct'],1,True)} % over {vr['distance_vs_gnss']['gnss_m']:.0f} m, but GPS is not a reference)."),
], size=12.5)

# ================================================================== 12 Velocity accuracy
seg = vr["segments"]
s = new_slide(f"Velocity: RMSE {env['rmse']:.2f} m/s against GPS in the final drive, worst during acceleration", "Velocity accuracy (real data)",
              f"velocity_summary.json → final_envelope; velocity_samples_{FIN}.csv; figures/vel_final_timeseries.png",
              "Target = GPS Doppler speed (iOS CLLocation.speed), 1 Hz, shifted by the fitted 0.65 s lag. Estimate = filter replayed WITHOUT GPS updates, so the comparison is independent. "
              "Envelope = GPS 2–11.2 m/s before the first >25 mph excursion. Errors are autocorrelated: CIs use a moving-block bootstrap.")
figure(s, "vel_final_timeseries.png", Inches(0.35), Inches(1.4), Inches(7.6), Inches(4.5))
table(s, Inches(8.1), Inches(1.5), Inches(4.9), [
    ["est − target (m/s)", f"envelope, n = {env['n']}"],
    ["Bias", f"{f(env['bias'],2,True)}  [95 % CI {f(vr['envelope_bootstrap']['mean_ci95'][0],2)}…{f(vr['envelope_bootstrap']['mean_ci95'][1],2)}]"],
    ["RMSE", f"{env['rmse']:.2f}  [95 % CI {vr['envelope_bootstrap']['rmse_ci95'][0]:.2f}…{vr['envelope_bootstrap']['rmse_ci95'][1]:.2f}]"],
    ["MAE / SD", f"{env['mae']:.2f} / {env['sd']:.2f}"],
    ["|e| p50 / p90 / p95", f"{env['p50']:.2f} / {env['p90']:.2f} / {env['p95']:.2f}"],
    ["Max |e| (observed)", f"{env['max']:.2f}"],
    ["Steady (|a| < 0.3 m/s²)", f"RMSE {vr['steady']['rmse']:.2f}, n = {vr['steady']['n']}"],
    ["Accel / braking", f"RMSE {vr['transient']['rmse']:.2f}, n = {vr['transient']['n']}"],
    ["PSR < 15 (near blur limit)", f"bias {f(vr['low_psr_lt15']['bias'],2,True)}, n = {vr['low_psr_lt15']['n']}"],
], [2.2, 2.7], size=10.5, row_h=0.34)
bullets(s, Inches(0.55), Inches(6.0), Inches(12.4), Inches(1.0), [
    ("Segments:", f"cruise 4.5–5.3 m/s ratio {seg[0]['ratio']:.3f} (RMSE {seg[0]['rmse']:.2f}); 2.6 m/s {seg[1]['ratio']:.2f}; accelerate to 8.7 m/s and brake {seg[2]['ratio']:.2f}; just before overspeed {seg[3]['ratio']:.2f}. "
     f"Only ~{vr['n_effective']:.0f} effectively independent samples. Including the zero-lock failure, RMSE is {vr['whole_run_including_failures']['rmse']:.1f} m/s."),
], size=12)

# ================================================================== 13 Target velocity / MC method
pv = mcv["params"]
s = new_slide("Monte Carlo: the real filter, real timestamps, measured noise", "Monte Carlo methodology",
              "06_mc_distance.py, 07_mc_velocity.py, noise_params.json, mc_*_summary.json",
              "Vectorised ReferenceKF4 was checked against the scalar port on real runs (max difference 9e-16 m/s). Each trial uses one recorded run's timestamps, PSR sequence and ZUPT times; only the measurements are synthetic.")
table(s, Inches(0.55), Inches(1.5), Inches(12.25), [
    ["Input", "Value", "Derived from", "Type"],
    ["Flow frame noise (per PSR band)", f"robust SD {M['noise']['flow_cart_robust_sd']*1000:.0f} mm/s cart, {M['noise']['flow_car_robust_sd']*1000:.0f} mm/s car; excess kurtosis 15–25",
     "residual vs 0.1 s median, 12.5k cart / 17.9k car frames; resampled empirically", "empirical"],
    ["Camera height h", f"SD {mcd['variance_decomposition']['height_sd_m_incl_rounding']*1000:.2f} mm", "repeated Measure-h at an unchanged mount + 1 mm app rounding", "empirical"],
    ["Focal correction c, LiDAR offset δ", f"{cal['focal_correction']:.4f} ± {cal['focal_correction_se']:.4f}, {cal['height_offset_m']*1000:.1f} ± {cal['height_offset_se_m']*1000:.1f} mm, r = {cal['param_corr']:.2f}",
     "our least-squares refit, 6 runs (covariance × residual variance)", "empirical"],
    ["Unexplained per-run term (distance)", f"{mcd['variance_decomposition']['unexplained_sd_pct']:.2f} %", "variance matching: LiDAR + flow already explain the observed 0.33 % scatter", "derived"],
    ["Slow speed error (velocity)", f"SD {pv['sd_slow_abs']:.2f} m/s, τ = {pv['tau_slow_s']:.1f} s (Ornstein–Uhlenbeck)", "GPS residuals at PSR ≥ 15 minus GPS white noise; 1 s autocorrelation", "empirical, weakly constrained"],
    ["GPS reference", f"white SD {pv['sd_gnss']:.2f} m/s, lag {abs(pv['lag_s']):.2f} s", "2nd difference of the 1 Hz series; lag by cross-correlation", "empirical"],
    ["IMU", f"SD {pv['sd_imu']:.3f} (car), 0.046 (cart) m/s²; bias SD 0.02", "ZUPT (at rest) samples; bias SD assumed", "empirical + assumption"],
    ["Trials", f"{mcd['N']:,} distance · {mcv['N']:,} velocity", f"seeds {mcd['seed']}, {mcv['seed']}", "—"],
], [3.0, 3.6, 4.35, 1.3], size=10.5, row_h=0.42)
text(s, Inches(0.55), Inches(5.55), Inches(12.3), Inches(1.3), [
    [("Kept distinct: ", {"bold": True}), ("TARGET (true speed or length) · MEASURED (camera flow) · ESTIMATED (Kalman output) · MONTE CARLO (distribution of estimated − target across trials). "
      "Velocity error is propagated through the filter dynamics and is time-correlated, not an independent Gaussian. Distance error comes from integrating the simulated velocity through the real odometer.", {})],
], size=12.5)

# ================================================================== 14 MC results
s = new_slide("Monte Carlo bounds: ±0.8–1.2 % on distance, ±1.0 m/s on speed (95 %)", "Monte Carlo results",
              "mc_distance_summary.json, mc_velocity_summary.json; figures/mc_dist.png, mc_vel.png")
figure(s, "mc_dist.png", Inches(0.35), Inches(1.4), Inches(6.5), Inches(2.6))
figure(s, "mc_vel.png", Inches(0.35), Inches(4.1), Inches(6.5), Inches(2.4))
table(s, Inches(7.0), Inches(1.5), Inches(5.95), [
    ["Distance on 4.877 m", "Repeat run (cal fixed)", "New run (cal uncertain)"],
    ["Mean error", f"{f(S1['mean_pct'],2,True)} %", f"{f(S2['mean_pct'],2,True)} %"],
    ["SD (1σ)", f"{S1['sd_pct']:.2f} %", f"{S2['sd_pct']:.2f} %"],
    ["95 % interval", f"{f(S1['int95_pct'][0])}…{f(S1['int95_pct'][1])} %", f"{f(S2['int95_pct'][0])}…{f(S2['int95_pct'][1])} %"],
    ["99 % interval", f"{f(S1['int99_pct'][0])}…{f(S1['int99_pct'][1])} %", f"{f(S2['int99_pct'][0])}…{f(S2['int99_pct'][1])} %"],
    ["|e| 95th pct", f"{S1['abs_p95_pct']:.2f} % ({S1['abs_p95_m']*100:.1f} cm)", f"{S2['abs_p95_pct']:.2f} % ({S2['abs_p95_m']*100:.1f} cm)"],
    ["P(|e| > 1 %)", f"{100*S1['p_abs_gt_1pct']:.1f} %", f"{100*S2['p_abs_gt_1pct']:.1f} %"],
], [1.9, 2.0, 2.05], size=10.5, row_h=0.33)
table(s, Inches(7.0), Inches(4.05), Inches(5.95), [
    ["Speed, car 2–9 m/s", "Instantaneous", "10 s average"],
    ["Bias / SD (1σ)", f"{f(VM['bias'],2,True)} / {VM['sd']:.2f} m/s", f"SD {VM['mean_over_10s']['sd']:.2f} m/s"],
    ["95 % interval", f"{f(VM['int95'][0])}…{f(VM['int95'][1])} m/s", f"|e|₉₅ {VM['mean_over_10s']['abs_p95']:.2f} m/s"],
    ["99 % interval", f"{f(VM['int99'][0])}…{f(VM['int99'][1])} m/s", f"|e|₉₉ {VM['mean_over_10s']['abs_p99']:.2f} m/s"],
    ["P(|e| > 0.5 / 1 m/s)", f"{100*VM['p_abs_gt_0p5']:.0f} % / {100*VM['p_abs_gt_1']:.0f} %", "—"],
], [1.9, 2.15, 1.9], size=10.5, row_h=0.33)
text(s, Inches(7.0), Inches(5.85), Inches(5.95), Inches(1.1),
     "These are prediction (uncertainty) intervals for one new measurement, i.e. percentiles of simulated error. They are not confidence intervals of a mean. "
     "Outside the envelope (high mount, ride-height change, above 25 mph, zero-lock) they do not apply.", size=11, color=MUTED)

# ================================================================== 15 Reality vs simulation
ed, ev_ = mcd["empirical_vs_mc"], mcv["empirical_vs_mc"]
s = new_slide("Reality vs simulation: agreement where we can test it, and what dominates", "Validating the Monte Carlo",
              "mc_distance_summary.json (convergence, seeds, sensitivity, analytic), mc_velocity_summary.json; figures/mc_dist_convergence.png")
figure(s, "mc_dist_convergence.png", Inches(0.4), Inches(1.45), Inches(4.2), Inches(2.75))
table(s, Inches(4.8), Inches(1.5), Inches(8.15), [
    ["Check", "Distance", "Velocity"],
    ["Real vs MC spread", f"LOO SD {ed['loo_vs_S2']['emp_sd']:.2f} % vs MC {ed['loo_vs_S2']['mc_sd']:.2f} %",
     f"real SD {ev_['real_sd']:.2f} vs MC (incl. GPS noise) {ev_['mc_obs_sd']:.2f} m/s"],
    ["Tails", f"worst real LOO run at MC pct {100*ed['loo_vs_S2']['mc_quantile_of_emp_max']:.0f}", f"|e|₉₅ real {ev_['real_abs_p95']:.2f} vs MC {ev_['mc_obs_abs_p95']:.2f} m/s"],
    ["Analytic", f"independent-term SD {mcd['analytic_sd_pct']:.2f} % vs MC {S2['sd_pct']:.2f} %", f"filter's own σ (white noise only) {mcv['analytic']['sd_post']*1000:.0f} mm/s ≪ real error"],
    ["Convergence / seeds", "|e|₉₉ stable to ±0.02 % across 5 seeds", "|e|₉₉ stable across 5 seeds"],
], [1.8, 3.15, 3.2], size=10.5, row_h=0.42)
bullets(s, Inches(0.55), Inches(4.4), Inches(12.4), Inches(2.6), [
    ("Distance is dominated by scale:", f"LiDAR height repeatability alone ({mcd['variance_decomposition']['height_rel_sd_pct']:.2f} %) explains the observed run-to-run scatter. "
     f"Flow noise integrates to {mcd['pipeline_pilot']['analytic_white_noise_sd_pct']:.2f} % and push-profile effects to about 0.17 %. The calibration constants dominate when extrapolating: at 0.50 m the 95 % interval widens to "
     f"{f(mcd['sensitivity']['mount_h_0p50_extrapolated']['int95_pct'][0],1)}…{f(mcd['sensitivity']['mount_h_0p50_extrapolated']['int95_pct'][1],1)} %."),
    ("Velocity is dominated by a slow error the data barely constrain:", "frame noise is ~1 % and averaged away by the filter; the ±0.5 m/s, ~3 s drift (pitch/ride height, reference lag) sets the bound. "
     "A model where that error scales with speed over-predicted the real spread (0.79 vs 0.53 m/s), so the additive model is used. Frame-noise distribution choice (empirical, Gaussian, Student-t) changes nothing."),
    ("Circularity, stated plainly:", "the slow-error size is fitted to the same drive it is compared with. The match in spread is built in; the matching tail (p95) and time structure are the real test."),
], size=12)

# ================================================================== 16 Conclusion
s = new_slide("What we learned, what the sensor can claim, and what to do next", "Conclusion",
              "final_metrics.json")
card(s, Inches(0.55), Inches(1.5), Inches(3.0), Inches(1.45), f"±{S2['abs_p95_pct']:.1f} % (95 %)", f"distance, new run, 0.19–0.24 m mount, indoor, ~5 m; observed max {loo['max_abs_pct']:.2f} % (LOO)", GREEN)
card(s, Inches(3.75), Inches(1.5), Inches(3.0), Inches(1.45), f"±{VM['abs_p95']:.1f} m/s (95 %)", f"instantaneous car speed 2–9 m/s; real RMSE vs GPS {env['rmse']:.2f} m/s", BLUE)
card(s, Inches(6.95), Inches(1.5), Inches(3.0), Inches(1.45), f"{seg[0]['ratio']:.3f} × GPS", "best steady cruise (4.5–5.3 m/s); accel/brake segments 0.87–1.24×", BLUE)
card(s, Inches(10.15), Inches(1.5), Inches(2.8), Inches(1.45), "≈ 25 mph", "tracking limit at night (blur); 31 s zero-lock after overspeed; fix untested", RED)
text(s, Inches(0.55), Inches(3.15), Inches(4.0), Inches(0.4), "Lessons", size=16, bold=True)
bullets(s, Inches(0.55), Inches(3.55), Inches(4.0), Inches(3.4), [
    "Scale errors (h, f) mattered far more than filter tuning: 7 % and 61 %, versus ~0.1 % from frame noise.",
    "Logging every sensor was the decisive choice: every fix here was found and checked in recorded data.",
    "The simulator was right about window wrap and blur and wrong about real optics (twice).",
], size=12)
text(s, Inches(4.75), Inches(3.15), Inches(4.0), Inches(0.4), "Remaining limits", size=16, bold=True)
bullets(s, Inches(4.75), Inches(3.55), Inches(4.0), Inches(3.4), [
    "Only 6 truth runs, 2 heights, 1 floor: bounds are wide and local.",
    "No independent speed truth: GPS has 0.65 s lag and reports 1σ ≈ 2 m/s.",
    "Ride height is measured once; pitch/heave is unmeasured.",
    "79 cm mounts read 7–17 % low; cause unproven. Last pushes at 0.32 and 0.51 m have no recorded course length.",
], size=12)
text(s, Inches(8.95), Inches(3.15), Inches(4.0), Inches(0.4), "Next", size=16, bold=True)
bullets(s, Inches(8.95), Inches(3.55), Inches(4.0), Inches(3.4), [
    "Drive the fake-stop build; repeat the 30 mph pass.",
    "Ground truth: wheel encoder or RTK GPS, plus 20+ taped runs at 3+ heights.",
    "Second rangefinder (or LiDAR at intervals) for live ride height.",
    "Per-device lens calibration (checkerboard) instead of a FOV-based focal length.",
    "Daylight and higher mount to push the blur limit.",
], size=12)

dst = os.path.join(OUT, "GroundSpeed_Forensics.pptx")
prs.save(dst)
print("saved", dst, "slides:", len(prs.slides))
