"""Phase 4: velocity error against the best available reference.

TARGET velocity (reference): GNSS Doppler ground speed (gnss.csv `speed`, 1 Hz), the only
independent, time-resolved speed measurement in the archive (the team used it the same way,
writeup 3.13-3.16). Its own error is quantified here, not assumed:
  * reported 1-sigma `speed_acc` from iOS,
  * empirical white-noise level from the second difference of the 1 Hz series,
  * time lag vs the camera estimate (cross-correlation).
ESTIMATED velocity:
  * `est`        = phone's filter output as logged (est.csv; GNSS IS FUSED -> not independent)
  * `est_nognss` = the same filter replayed WITHOUT GNSS updates (independent of the reference)
  * `flow`       = raw camera velocity (flow.csv, q >= psr_min), 1 s median at each GNSS epoch
velocity_error = estimated - target.

Writes out/velocity_samples_<run>.csv, out/velocity_summary.json, figures/vel_*.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from common import CAR_RUNS, FIG, OUT, load_run, save_json, speed
from kf import replay
from plotstyle import C, style

MPH = 0.44704
FINAL = "2026-10-04_01-59-42"
# Documented overspeed/zero-lock episode in the final car run (data/README.md: t ~ 264-272 s);
# located precisely from the data below.
BINS = [(1.0, 2.0), (2.0, 4.47), (4.47, 6.71), (6.71, 8.94), (8.94, 11.18), (11.18, 15.0)]
BIN_LABELS = ["1–2 m/s", "2–4.5 (≤10 mph)", "4.5–6.7 (10–15 mph)", "6.7–8.9 (15–20 mph)",
              "8.9–11.2 (20–25 mph)", ">11.2 (>25 mph)"]


def stats(e):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if len(e) == 0:
        return dict(n=0)
    a = np.abs(e)
    return dict(n=int(len(e)), bias=float(e.mean()), sd=float(e.std(ddof=1)) if len(e) > 1 else np.nan,
                rmse=float(np.sqrt((e ** 2).mean())), mae=float(a.mean()),
                p50=float(np.percentile(a, 50)), p90=float(np.percentile(a, 90)), p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), max=float(a.max()),
                bias_ci95=[float(e.mean() - 1.96 * e.std(ddof=1) / np.sqrt(len(e))),
                           float(e.mean() + 1.96 * e.std(ddof=1) / np.sqrt(len(e)))] if len(e) > 1 else None)


def block_bootstrap_ci(e, block=10, n=4000, seed=0):
    """95% CI of mean and RMSE with a moving-block bootstrap (errors are autocorrelated)."""
    e = np.asarray(e, float)
    rng = np.random.default_rng(seed)
    nb = int(np.ceil(len(e) / block))
    starts = rng.integers(0, len(e) - block + 1, size=(n, nb))
    idx = (starts[:, :, None] + np.arange(block)).reshape(n, -1)[:, :len(e)]
    s = e[idx]
    m = s.mean(1); r = np.sqrt((s ** 2).mean(1))
    return dict(mean_ci95=np.percentile(m, [2.5, 97.5]).tolist(), rmse_ci95=np.percentile(r, [2.5, 97.5]).tolist())


def flow_median_at(flow, tq, win=0.5, psr_min=8):
    t = flow["t"].to_numpy(); s = speed(flow, "vx_cam", "vy_cam"); q = flow["quality"].to_numpy()
    ok = q >= psr_min
    out = np.full(len(tq), np.nan)
    lo = np.searchsorted(t, tq - win); hi = np.searchsorted(t, tq + win)
    for i, (a, b) in enumerate(zip(lo, hi)):
        m = ok[a:b]
        if m.sum() >= 10:
            out[i] = np.median(s[a:b][m])
    return out


def best_lag(t_ref, v_ref, t_est, v_est, mask):
    lags = np.arange(-2.0, 2.0001, 0.05)
    rm = []
    for L in lags:
        ve = np.interp(t_ref[mask] + L, t_est, v_est)
        rm.append(np.sqrt(np.mean((ve - v_ref[mask]) ** 2)))
    rm = np.array(rm)
    return float(lags[rm.argmin()]), lags, rm


def analyse(run):
    r = load_run(run)
    est, flow, gnss = r["est"], r["flow"], r["gnss"]
    t0 = est["t"].iloc[0]
    rep = replay(r, use_gnss=False)
    te = est["t"].to_numpy()
    v_est = speed(est)
    v_ng = np.hypot(rep["v_x"], rep["v_y"])
    tg = gnss["t"].to_numpy(); vg = gnss["speed"].to_numpy(); acc = gnss["speed_acc"].to_numpy()
    # Drop duplicate GNSS timestamps (3 in the archive)
    keep = np.r_[True, np.diff(tg) > 0]
    tg, vg, acc = tg[keep], vg[keep], acc[keep]
    moving = vg > 2.0
    # Lag fitted only where the camera is tracking (estimate within 30 % of GNSS at zero lag),
    # otherwise the 30 s zero-lock / wrap failures dominate the fit and say nothing about timing.
    v0 = np.interp(tg, rep["t"], v_ng)
    lag_mask = moving & (vg < 11.2) & (np.abs(v0 / np.maximum(vg, 1e-9) - 1) < 0.3)
    lag, lags, rm = best_lag(tg, vg, rep["t"], v_ng, lag_mask)
    ve = np.interp(tg + lag, te, v_est)
    vn = np.interp(tg + lag, rep["t"], v_ng)
    vf = flow_median_at(flow, tg + lag)
    q = flow["quality"].to_numpy(); tf = flow["t"].to_numpy()
    lo = np.searchsorted(tf, tg + lag - 0.5); hi = np.searchsorted(tf, tg + lag + 0.5)
    qmed = np.array([np.median(q[a:b]) if b > a else 0 for a, b in zip(lo, hi)])
    hmed = np.interp(tg, tf, flow["h"].to_numpy())
    # Empirical GNSS white noise: var(second difference)/6 for white noise on a smooth signal
    d2 = vg[2:] - 2 * vg[1:-1] + vg[:-2]
    m2 = moving[1:-1]
    gnss_noise_sd = float(np.sqrt(np.median(d2[m2] ** 2) / 0.4549 / 6))  # robust (median of chi2_1 = 0.4549)
    df = pd.DataFrame(dict(t=tg - t0, gps=vg, gps_acc=acc, est=ve, est_nognss=vn, flow=vf, q_med=qmed, h=hmed))
    df["err_est"] = df.est - df.gps
    df["err_nognss"] = df.est_nognss - df.gps
    df["err_flow"] = df.flow - df.gps
    df.to_csv(os.path.join(OUT, f"velocity_samples_{run}.csv"), index=False)
    return dict(run=run, df=df, lag=lag, lag_curve=(lags, rm), t0=t0, est_t=te - t0, v_est=v_est,
                rep_t=rep["t"] - t0, v_ng=v_ng, gnss_noise_sd=gnss_noise_sd,
                gnss_acc_med=float(np.median(acc[moving])), gnss_influence_max=float(np.max(np.abs(np.interp(rep["t"], te, v_est) - v_ng))),
                gnss_influence_p95=float(np.percentile(np.abs(np.interp(rep["t"], te, v_est) - v_ng), 95)),
                flow=flow.assign(t=flow.t - t0), est=est.assign(t=est.t - t0))


def find_zero_lock(df):
    """GNSS epochs where the reference says moving (> 3 m/s) but the estimate reads < 0.5 m/s."""
    z = (df.gps > 3) & (df.est_nognss < 0.5)
    return df.t[z].to_numpy()


def main():
    style()
    res = {run: analyse(run) for run in CAR_RUNS}
    out = {}
    for run, a in res.items():
        df = a["df"]
        zl = find_zero_lock(df)
        o = dict(label=CAR_RUNS[run], lag_s=a["lag"], gnss_noise_sd_empirical=a["gnss_noise_sd"],
                 gnss_speed_acc_median_moving=a["gnss_acc_med"], gnss_influence_on_est_max=a["gnss_influence_max"],
                 gnss_influence_on_est_p95=a["gnss_influence_p95"],
                 zero_lock_epochs=int(len(zl)), zero_lock_t_first=float(zl.min()) if len(zl) else None,
                 zero_lock_t_last=float(zl.max()) if len(zl) else None)
        mv = df[df.gps > 2.0]
        o["all_moving"] = {k: stats(mv[c]) for k, c in (("est", "err_est"), ("est_nognss", "err_nognss"), ("flow", "err_flow"))}
        o["flow_coverage_moving"] = float(np.isfinite(mv.flow).mean())
        o["bins"] = {}
        for (lo, hi), lab in zip(BINS, BIN_LABELS):
            b = df[(df.gps >= lo) & (df.gps < hi)]
            if len(b) < 3:
                continue
            ratio = (b.est_nognss / b.gps)
            o["bins"][lab] = dict(n=len(b), ratio_mean=float(ratio.mean()), ratio_sd=float(ratio.std(ddof=1)),
                                  ratio_median=float(ratio.median()),
                                  err_nognss=stats(b.err_nognss), err_est=stats(b.err_est),
                                  rel_err_pct=stats(100 * b.err_nognss / b.gps))
        out[run] = o

    # ---- Final run: envelope definitions ----
    a = res[FINAL]; df = a["df"]
    zl = find_zero_lock(df)
    t_fail = float(zl.min()) if len(zl) else np.inf
    over = df.t[(df.gps > 11.2)]
    t_over = float(over.min()) if len(over) else np.inf
    env = df[(df.gps > 2.0) & (df.gps <= 11.2) & (df.t < min(t_fail, t_over))]
    e = env.err_nognss.to_numpy()
    envelope = dict(definition="GNSS speed 2-11.2 m/s (4.5-25 mph), before the first >25 mph excursion "
                               f"(t={t_over:.1f} s) and before first zero-lock epoch (t={t_fail:.1f} s)",
                    t_first_overspeed=t_over, t_first_zero_lock=t_fail,
                    err_nognss=stats(e), err_est=stats(env.err_est), rel_pct=stats(100 * e / env.gps.to_numpy()),
                    bootstrap=block_bootstrap_ci(e),
                    gnss_noise_sd_empirical=a["gnss_noise_sd"],
                    implied_sensor_sd=float(np.sqrt(max(np.var(e, ddof=1) - a["gnss_noise_sd"] ** 2, 0))),
                    ratio=dict(mean=float((env.est_nognss / env.gps).mean()), sd=float((env.est_nognss / env.gps).std(ddof=1))),
                    lag2_autocorr=float(pd.Series(e).autocorr(1)))
    full = df[df.gps > 2.0]
    envelope["whole_run_including_failures"] = stats(full.err_nognss)
    # Reference quality: classify by the REFERENCE's own acceleration (never by the error itself).
    acc_g = np.gradient(df.gps.to_numpy(), df.t.to_numpy())
    df["gps_accel"] = acc_g
    env = env.assign(gps_accel=df.loc[env.index, "gps_accel"])
    steady = env[env.gps_accel.abs() < 0.3]; trans = env[env.gps_accel.abs() >= 0.3]
    envelope["steady_abs_accel_lt_0p3"] = stats(steady.err_nognss)
    envelope["transient_abs_accel_ge_0p3"] = stats(trans.err_nognss)
    envelope["corr_err_vs_gps_accel"] = float(np.corrcoef(env.err_nognss, env.gps_accel)[0, 1])
    envelope["corr_err_vs_psr"] = float(np.corrcoef(env.err_nognss, env.q_med)[0, 1])
    envelope["low_psr_lt15"] = stats(env[env.q_med < 15].err_nognss)
    envelope["psr_ge15"] = stats(env[env.q_med >= 15].err_nognss)
    # Effective sample size under AR(1) autocorrelation
    r1 = envelope["lag2_autocorr"]
    envelope["n_effective_ar1"] = float(len(e) * (1 - r1) / (1 + r1))
    # Segments (contiguous moving stretches)
    segs = []
    tt = env.t.to_numpy()
    cut = np.where(np.diff(tt) > 3)[0]
    for idx in np.split(np.arange(len(env)), cut + 1):
        s_ = env.iloc[idx]
        segs.append(dict(t_from=float(s_.t.min()), t_to=float(s_.t.max()), n=len(s_), gps_mean=float(s_.gps.mean()),
                         gps_max=float(s_.gps.max()), bias=float(s_.err_nognss.mean()),
                         rmse=float(np.sqrt((s_.err_nognss ** 2).mean())),
                         ratio=float((s_.est_nognss / s_.gps).mean()), psr_med=float(s_.q_med.median())))
    envelope["segments"] = segs
    # Lag sensitivity: envelope RMSE if the GNSS lag were 0 or +/-0.5 s from the fitted one
    lags, rm = a["lag_curve"]
    envelope["lag_rmse_curve_tracking_epochs"] = dict(zip([round(float(x), 2) for x in lags[::5]], [float(x) for x in rm[::5]]))
    out["final_envelope"] = envelope
    # Distance cross-check vs integrated GNSS, moving epochs only (GNSS speed is a magnitude, so
    # it reads > 0 when parked: median 0.09 m/s; integrating it while parked adds fake metres).
    seg = df[(df.t >= env.t.min()) & (df.t <= env.t.max())]
    mv_ = ((seg.gps > 1.0) | (seg.est_nognss > 1.0)).to_numpy()
    g_ = np.where(mv_, seg.gps, 0.0); n_ = np.where(mv_, seg.est_nognss, 0.0)
    dg = np.trapezoid(g_, seg.t); dn = np.trapezoid(n_, seg.t)
    out["final_distance_vs_gnss"] = dict(t_from=float(seg.t.min()), t_to=float(seg.t.max()), gnss_m=float(dg),
                                         est_nognss_m=float(dn), diff_pct=float(100 * (dn / dg - 1)),
                                         note="1 Hz samples, moving epochs only (GNSS>1 or est>1 m/s)")
    # Steady-cruise segment distance (first stretch), the cleanest comparison available
    s0 = seg[(seg.t >= segs[0]["t_from"]) & (seg.t <= segs[0]["t_to"])]
    out["final_distance_vs_gnss"]["first_segment_diff_pct"] = float(100 * (np.trapezoid(s0.est_nognss, s0.t) / np.trapezoid(s0.gps, s0.t) - 1))
    save_json(out, "velocity_summary.json")

    # ---------------- figures ----------------
    fig, axs = plt.subplots(3, 1, figsize=(11, 8.2), sharex=False)
    for ax, (run, a) in zip(axs, res.items()):
        df = a["df"]
        ax.plot(a["rep_t"], a["v_ng"], color=C["blue"], lw=0.7, label="Camera+IMU filter (GNSS removed)")
        ax.plot(df.t, df.gps, ".", color=C["orange"], ms=3.5, label="GNSS Doppler speed (reference)")
        ax.set_ylabel("speed (m/s)")
        ax.set_title(CAR_RUNS[run] + f"   [{run}]", fontsize=10, loc="left")
        ax.set_ylim(-0.5, 15)
        for mph in (10, 20, 30):
            ax.axhline(mph * MPH, color=C["grey"], lw=0.5, ls=":")
            ax.text(ax.get_xlim()[1] if False else 0, mph * MPH + 0.15, f"{mph} mph", fontsize=7, color=C["grey"])
    hh = res["2026-10-04_01-19-31"]["flow"]
    axs[0].legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("time since run start (s)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "vel_car_runs_timeseries.png"), dpi=170); plt.close(fig)

    # Height used in car runs 1-2 (tracker collapse) vs 3 (fixed)
    fig, ax = plt.subplots(figsize=(8, 3.3))
    for (run, a), col in zip(res.items(), (C["red"], C["orange"], C["green"])):
        f = a["flow"].iloc[::24]
        ax.plot(f.t, f.h, color=col, lw=1, label=CAR_RUNS[run].split("(")[0].strip())
    ax.set_ylabel("camera height used, h (m)"); ax.set_xlabel("time since run start (s)")
    ax.set_title("Image-expansion height tracker drifted in car tests 1–2; fixed LiDAR h in test 3")
    ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(os.path.join(FIG, "vel_height_tracker.png"), dpi=170); plt.close(fig)

    # Final run: error time series and distribution
    a = res[FINAL]; df = a["df"]
    fig, axs = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True, gridspec_kw=dict(height_ratios=[1.3, 1]))
    ax = axs[0]
    ax.plot(a["rep_t"], a["v_ng"], color=C["blue"], lw=0.7, label="Estimated (filter, GNSS removed)")
    ax.plot(df.t, df.gps, ".", color=C["orange"], ms=4, label="Target: GNSS Doppler speed")
    ax.fill_between(df.t, df.gps - df.gps_acc, df.gps + df.gps_acc, color=C["orange"], alpha=0.15, lw=0, label="GNSS reported ±1σ speed_acc")
    ax.axvspan(env.t.min(), env.t.max(), color=C["green"], alpha=0.06)
    if np.isfinite(t_over):
        ax.axvline(t_over, color=C["red"], lw=1, ls="--"); ax.text(t_over + 1, 13.5, ">25 mph: blur,\nflow lost", fontsize=8, color=C["red"])
    if np.isfinite(t_fail) and t_fail != t_over:
        ax.axvline(t_fail, color=C["red"], lw=1, ls=":")
    ax.set_ylabel("speed (m/s)"); ax.set_ylim(-0.5, 15); ax.legend(fontsize=8, loc="upper left")
    ax.set_title("Final car test: estimated vs target speed (green = analysed envelope)")
    ax = axs[1]
    ax.axhline(0, color="k", lw=0.7)
    ax.plot(df.t, df.err_nognss, ".", color=C["blue"], ms=4)
    ax.set_ylim(-4, 4); ax.set_ylabel("est − target (m/s)"); ax.set_xlabel("time since run start (s)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "vel_final_timeseries.png"), dpi=170); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    ax = axs[0]
    ax.hist(env.err_nognss, bins=30, color=C["blue"], alpha=0.85)
    ax.axvline(0, color="k", lw=0.8)
    s = envelope["err_nognss"]
    ax.set_title(f"Velocity error in envelope (n={s['n']} GNSS epochs)")
    ax.set_xlabel("estimated − GNSS speed (m/s)")
    ax.text(0.02, 0.95, f"bias {s['bias']:+.3f} m/s\nRMSE {s['rmse']:.3f} m/s\n|e| p95 {s['p95']:.3f} m/s",
            transform=ax.transAxes, va="top", fontsize=9)
    ax = axs[1]
    labs, rat, sd = [], [], []
    for lab, b in out[FINAL]["bins"].items():
        labs.append(lab.replace(" (", "\n(")); rat.append(b["ratio_mean"]); sd.append(b["ratio_sd"])
    ax.errorbar(range(len(rat)), rat, yerr=sd, fmt="o", color=C["blue"], capsize=4)
    ax.axhline(1, color="k", lw=0.8); ax.set_xticks(range(len(labs)), labs, fontsize=7)
    ax.set_ylabel("estimated / GNSS speed (mean ± SD)"); ax.set_title("Speed ratio by GNSS speed band (whole final run)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "vel_final_error_dist.png"), dpi=170); plt.close(fig)

    import json
    print(json.dumps(out, indent=1, default=float)[:9000])


if __name__ == "__main__":
    main()
