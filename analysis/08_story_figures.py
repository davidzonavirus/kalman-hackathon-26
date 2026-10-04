"""Phase 3 evidence figures: chronology of the night, raw data, and before/after evidence for each
problem the data can quantify. Writes figures/story_*.png and out/story_metrics.json."""
import os
import re
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from common import FIG, LIDAR, OUT, ROOT, load_run, run_ids, save_json, speed
from kf import DistanceIntegrator
from plotstyle import C, style


def tod(run):
    return datetime.strptime(run[:19], "%Y-%m-%d_%H-%M-%S")


def main():
    style()
    inv = pd.read_csv(os.path.join(OUT, "inventory_runs.csv"))
    inv = inv[inv.est_n > 0].copy()
    inv["when"] = inv.run.map(tod)
    metrics = {}

    # ---------- A. Timeline ----------
    diag = os.listdir(os.path.join(ROOT, "data", "ios_diagnostics"))
    crashes = [datetime.strptime(re.search(r"(\d{4}-\d\d-\d\d-\d{6})", f).group(1), "%Y-%m-%d-%H%M%S")
               for f in diag if f.startswith("cameracaptured")]
    fig, axs = plt.subplots(3, 1, figsize=(12, 6.4), sharex=True, gridspec_kw=dict(height_ratios=[1, 1, 1]))
    ax = axs[0]
    ax.plot(inv.when, inv.flow_rate_hz, "o", color=C["blue"], ms=5)
    ax.set_ylabel("flow frames\nprocessed (Hz)"); ax.set_ylim(0, 270)
    ax.axvline(datetime(2026, 10, 3, 20, 34), color=C["grey"], ls=":")
    ax.text(datetime(2026, 10, 3, 19, 35), 200, "Debug build\n69–118 Hz", fontsize=8)
    ax.text(datetime(2026, 10, 3, 20, 40), 30, "Release + 240 fps: 232–240 Hz", fontsize=8)
    for c in crashes:
        ax.plot(c, 255, "v", color=C["red"], ms=6)
    ax.text(datetime(2026, 10, 3, 20, 10), 258, "▼ camera-daemon crash reports", color=C["red"], fontsize=8, va="bottom")
    ax.set_title("What the logs say happened, run by run (Oct 3 19:34 → Oct 4 02:27)")
    ax = axs[1]
    ax.plot(inv.when, inv.focal_px, "s", color=C["orange"], ms=5)
    ax.set_ylabel("focal length\nused (px)")
    ax.text(datetime(2026, 10, 3, 19, 35), 868, "pinhole from FOV: 860.7", fontsize=8)
    ax.text(datetime(2026, 10, 3, 22, 30), 915, "×1.073 (first fix)", fontsize=8)
    ax.text(datetime(2026, 10, 3, 23, 0), 888, "×1.027 + LiDAR −7.9 mm (final)", fontsize=8)
    ax = axs[2]
    ax.plot(inv.when, inv.flow_h_med, "^", color=C["green"], ms=5, label="median h")
    ax.vlines(inv.when, inv.flow_h_min, inv.flow_h_max, color=C["green"], alpha=0.5)
    ax.set_ylabel("camera height\nused h (m)"); ax.set_ylim(0, 0.85)
    ax.text(datetime(2026, 10, 3, 19, 35), 0.34, "0.30 m default (no LiDAR)", fontsize=8)
    ax.text(datetime(2026, 10, 4, 0, 5), 0.5, "tracked h drifts\n(car 1–2)", fontsize=8)
    ax.text(datetime(2026, 10, 3, 22, 59), 0.72, "79 cm", fontsize=8)
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    ax.set_xlabel("local time (EDT)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "story_timeline.png"), dpi=170); plt.close(fig)
    metrics["camera_crash_reports"] = dict(n=len(crashes), first=str(min(crashes)), last=str(max(crashes)))

    # ---------- B. Flow quality vs per-frame image shift, Debug vs Release ----------
    rows = []
    for run in run_ids():
        r = load_run(run)
        f = r["flow"]
        if len(f) < 500 or not r["meta"]:
            continue
        # Debug era = every early run; Release = only the clean 16 ft calibration pushes (the 21:xx
        # jostle runs had 66-77 % of frames below psr_min and would confound the comparison).
        clean = {"2026-10-03_22-18-56", "2026-10-03_22-20-43", "2026-10-03_22-22-18", "2026-10-03_22-23-45",
                 "2026-10-03_22-25-09", "2026-10-03_22-37-35", "2026-10-03_22-38-37", "2026-10-03_22-39-51"}
        if not (run < "2026-10-03_20-36" or run in clean):
            continue
        s = speed(f, "vx_cam", "vy_cam")
        dt = np.r_[np.diff(f.t.to_numpy()), np.nan]
        shift = s * dt * r["meta"]["focal_px"] / f.h.to_numpy()   # full-res px per frame actually measured
        era = "Debug build (60–120 fps, h unknown)" if run < "2026-10-03_20-36" else "Release, 240 fps (16 ft pushes)"
        ok = np.isfinite(shift) & (s > 0.05)
        rows.append(pd.DataFrame(dict(era=era, shift=shift[ok], q=f.quality.to_numpy()[ok], hz=1 / np.nanmedian(dt))))
    d = pd.concat(rows)
    edges = np.array([1, 2, 4, 6, 8, 12, 16, 24, 32, 48])
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.9))
    ax = axs[0]
    med = {}
    for era, col in (("Debug build (60–120 fps, h unknown)", C["red"]), ("Release, 240 fps (16 ft pushes)", C["blue"])):
        x = d[d.era == era]
        b = np.digitize(x["shift"], edges)
        m = [x.q[b == i].median() if (b == i).sum() > 30 else np.nan for i in range(1, len(edges))]
        mid = np.sqrt(edges[:-1] * edges[1:])
        ax.plot(mid, m, "o-", color=col, label=era)
        med[era] = dict(zip(mid.round(1).tolist(), m))
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("image shift between frames (full-res px)"); ax.set_ylabel("flow quality (PSR, median)")
    ax.set_title("Within one setup, PSR falls steeply with shift/frame")
    ax.legend(fontsize=8)
    ax = axs[1]
    v = np.linspace(0.05, 3, 200)
    for hz, col, lab in ((90, C["red"], "90 Hz (Debug)"), (240, C["blue"], "240 Hz (Release)")):
        ax.plot(v, v / hz * 884 / 0.186, color=col, label=lab)
    ax.set_xlabel("cart speed at h = 0.186 m (m/s)"); ax.set_ylabel("shift per frame (px)")
    ax.set_title("Same speed → 2.7× less shift at 240 fps"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "story_framerate.png"), dpi=170); plt.close(fig)
    metrics["psr_vs_shift"] = med

    # ---------- C. Camera crash -> IMU-only runaway ----------
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6))
    for ax, run in zip(axs, ("2026-10-03_21-37-55", "2026-10-03_21-34-31")):
        e = load_run(run)["est"]
        t = e.t - e.t.iloc[0]
        ax.plot(t, speed(e), color=C["red"], label="filter speed (no camera frames at all)")
        ax2 = ax.twinx(); ax2.plot(t, e.distance, color=C["dark"], ls="--", label="distance"); ax2.set_ylabel("distance (m)")
        ax.set_xlabel("time (s)"); ax.set_ylabel("speed (m/s)")
        ax.set_title(f"{run[11:]}: phone at rest, {e.distance.iloc[-1]:.1f} m in {t.iloc[-1]:.0f} s", fontsize=10)
        metrics[f"imu_only_{run}"] = dict(dur_s=float(t.iloc[-1]), distance_m=float(e.distance.iloc[-1]), max_speed=float(speed(e).max()))
    axs[0].legend(fontsize=8, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "story_imu_runaway.png"), dpi=170); plt.close(fig)

    # ---------- D. LiDAR tilt counted twice ----------
    lid = pd.read_csv(os.path.join(LIDAR, "2026-10-03_20-26-19.csv"), comment="#").dropna(subset=["depth_median"])
    cos = lid.cos_tilt.to_numpy(); depth = lid.depth_median.to_numpy()
    th = np.degrees(np.arccos(np.clip(cos, 0, 1)))
    ok = (th > 30) & (th < 45)
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    tt = np.linspace(0, 45, 100)
    ax.plot(tt, 100 * (1 / np.cos(np.radians(tt)) ** 2 - 1), color=C["red"], label="bug: slant depth stored as h, then ÷cos θ again")
    ax.plot(tt, 100 * (1 / np.cos(np.radians(tt)) - 1), color=C["grey"], ls="--", label="if tilt were ignored entirely")
    ax.axhline(0, color=C["blue"], label="fixed: h = depth·cos θ, ÷cos θ once")
    ax.plot(th[ok], 100 * (1 / cos[ok] ** 2 - 1), "o", color=C["red"], ms=4, alpha=0.6, label="logged frames, 20:26 attempt")
    ax.set_xlabel("phone tilt from vertical θ (deg)"); ax.set_ylabel("speed / distance error (%)")
    ax.set_title("LiDAR tilt counted twice"); ax.legend(fontsize=7.5)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "story_lidar_tilt.png"), dpi=170); plt.close(fig)
    metrics["lidar_tilt_example"] = dict(tilt_deg_median=float(np.median(th[ok])), depth_median_m=float(np.median(depth[ok])),
                                         vertical_h_m=float(np.median(depth[ok] * cos[ok])),
                                         speed_error_pct_if_bug=float(100 * (1 / np.median(cos[ok]) ** 2 - 1)))

    # ---------- E. Raw data of one calibration push ----------
    r = load_run("2026-10-03_22-37-35")
    f, e, imu = r["flow"], r["est"], r["imu"]
    t0 = e.t.iloc[0]
    fig, axs = plt.subplots(3, 1, figsize=(11, 6), sharex=True, gridspec_kw=dict(height_ratios=[1.6, 1, 1]))
    ax = axs[0]
    sc = ax.scatter(f.t - t0, speed(f, "vx_cam", "vy_cam"), c=np.log10(np.maximum(f.quality, 1)), s=2, cmap="viridis", vmin=0.7, vmax=3)
    ax.plot(e.t - t0, speed(e), color=C["red"], lw=1, label="Kalman filter output")
    cb = fig.colorbar(sc, ax=ax, pad=0.01); cb.set_label("log10 PSR")
    ax.set_ylabel("speed (m/s)"); ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Raw data, one 16 ft push (run 9, h = 0.236 m): 240 Hz camera flow, 100 Hz IMU, 100 Hz filter")
    ax = axs[1]
    ax.semilogy(f.t - t0, f.quality, color=C["green"], lw=0.5); ax.axhline(8, color=C["red"], ls="--", lw=0.8)
    ax.text(0.3, 10, "psr_min = 8 (below: frame skipped)", fontsize=7, color=C["red"]); ax.set_ylabel("PSR")
    ax = axs[2]
    ax.plot(imu.t - t0, imu.ax, color=C["purple"], lw=0.6, label="a_x"); ax.plot(imu.t - t0, imu.ay, color=C["sky"], lw=0.6, label="a_y")
    z = r["events"].loc[r["events"].event == "zupt", "t"] - t0
    ax.plot(z, np.full(len(z), -0.9), "|", color=C["grey"], ms=4, label="ZUPT applied")
    ax.set_ylabel("accel (m/s²)"); ax.set_xlabel("time (s)"); ax.legend(fontsize=7, ncol=3, loc="upper right")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "story_raw_push.png"), dpi=170); plt.close(fig)

    # ---------- F. Jostle runs: integrator before/after on real logs ----------
    jr = {}
    for run in ("2026-10-03_21-20-30", "2026-10-03_21-21-06"):
        e = load_run(run)["est"]
        out = {}
        for name, (db, tau, lead) in {"∫|v| (19:34 build)": (0, 0, False), "+0.05 m/s deadband (20:06)": (0.05, 0, False),
                                      "+0.5 s smoothing (21:20→final)": (0.05, 0.5, True)}.items():
            dI = DistanceIntegrator(db, tau, "path")
            for a, b, c in zip(e.t, e.v_x, e.v_y):
                dI.add(a, b, c)
            out[name] = dI.lead_compensated if lead else dI.distance
        out["net forward |∫v_x|"] = float(abs(np.trapezoid(e.v_x, e.t)))
        jr[run] = out
    metrics["jostle_integrators"] = jr

    # ---------- G. Dashboard +4 m jump: phone distance dip at stop ----------
    e = load_run("2026-10-04_00-01-22")["est"]
    dd = np.diff(e.distance.to_numpy())
    i = int(np.argmin(dd))
    metrics["distance_dip_00_01_22"] = dict(max_drop_m=float(-dd[i]), at_s=float(e.t.iloc[i + 1] - e.t.iloc[0]),
                                            peak_m=float(e.distance.max()), final_m=float(e.distance.iloc[-1]))
    save_json(metrics, "story_metrics.json")
    import json
    print(json.dumps(metrics, indent=1, default=str)[:4000])


if __name__ == "__main__":
    main()
