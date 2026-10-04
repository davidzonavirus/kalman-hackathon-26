"""Phase 3/5: distance error on the taped course.

1. Raw (as-recorded) distance error for every taped run, with the calibration state at record time.
2. Re-derives the team's 2-parameter calibration (focal correction c, LiDAR height offset delta)
   from the six 16 ft runs, checks it against the shipped constants (1.027, -7.9 mm), and
   re-applies the FINAL calibration to each run by replaying the logged sensors through the
   ported filter with the flow rescaled (exact pipeline, not a linear approximation).
3. Leave-one-out cross-validation: each run is predicted with c, delta fitted on the other five
   (the in-sample residuals are optimistic because the same 6 runs fitted 2 parameters).

Writes out/distance_runs.csv, out/distance_calibration.json, figures/dist_*.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from common import FIG, OUT, TAPE_RUNS, FOCAL_CORRECTION, LIDAR_H_OFFSET, load_run, save_json
from kf import replay
from plotstyle import style, C

F0 = 860.6556862016167  # pinhole focal from videoFieldOfView (meta.json focal_px before any correction)
CAL6 = ["2026-10-03_22-22-18", "2026-10-03_22-23-45", "2026-10-03_22-25-09",
        "2026-10-03_22-37-35", "2026-10-03_22-38-37", "2026-10-03_22-39-51"]
WRITEUP_CAL_PCT = [0.05, 0.30, -0.33, -0.02, 0.36, -0.36]  # docs/HACKATHON_WRITEUP.md section 1


def recorded_cal(meta, h_used):
    """(focal correction, height offset) that were active when the run was recorded."""
    c_rec = meta["focal_px"] / F0
    # Offset only exists in builds with focal 883.89 (LiDAR result lines include it; see lidar/*.csv).
    d_rec = LIDAR_H_OFFSET if abs(c_rec - FOCAL_CORRECTION) < 1e-3 else 0.0
    return c_rec, d_rec


def scale_to(c_rec, d_rec, h_used, c_new, d_new):
    """Factor that converts flow recorded with (c_rec, d_rec) to calibration (c_new, d_new)."""
    h_raw = h_used - d_rec
    return (c_rec / c_new) * ((h_raw + d_new) / h_used)


def main():
    style()
    rows = []
    cache = {}
    for run, (L, group, note) in TAPE_RUNS.items():
        r = load_run(run)
        cache[run] = r
        meta = r["meta"]
        h_used = float(r["flow"]["h"].median())
        c_rec, d_rec = recorded_cal(meta, h_used)
        D_rec = float(r["est"]["distance"].iloc[-1])
        rows.append(dict(run=run, group=group, note=note, course_m=L, h_used=h_used, h_raw=h_used - d_rec,
                         focal_px=meta["focal_px"], c_rec=c_rec, d_rec=d_rec, D_recorded=D_rec,
                         err_recorded_pct=100 * (D_rec / L - 1), dur_s=float(r["est"]["t"].iloc[-1] - r["est"]["t"].iloc[0])))
    df = pd.DataFrame(rows)

    # ---- 2-parameter fit on the six 16 ft runs (linearised: distance scales with flow) ----
    cal = df[df.run.isin(CAL6)].reset_index(drop=True)

    def resid(p, d=cal):
        c, dl = p
        k = scale_to(d.c_rec, d.d_rec, d.h_used, c, dl)
        return d.D_recorded * k / d.course_m - 1

    fit = least_squares(resid, [1.03, -0.005])
    J = fit.jac
    dof = len(cal) - 2
    s2 = float((fit.fun ** 2).sum() / dof)
    cov = np.linalg.inv(J.T @ J) * s2
    c_hat, d_hat = fit.x
    # Focal-only fit (delta = 0) for comparison (writeup 3.7: "lens-only would drift").
    fit1 = least_squares(lambda p: resid([p[0], 0.0]), [1.03])

    # LOO
    loo = []
    for i in range(len(cal)):
        tr = cal.drop(index=i)
        fi = least_squares(lambda p: resid(p, tr), [1.03, -0.005])
        k = scale_to(cal.c_rec[i], cal.d_rec[i], cal.h_used[i], *fi.x)
        loo.append(100 * (cal.D_recorded[i] * k / cal.course_m[i] - 1))
    cal["err_loo_pct_linear"] = loo

    # ---- Exact re-application of the FINAL shipped calibration by full replay ----
    exact = {}
    for run in df.run:
        row = df[df.run == run].iloc[0]
        k = scale_to(row.c_rec, row.d_rec, row.h_used, FOCAL_CORRECTION, LIDAR_H_OFFSET)
        rep = replay(cache[run], flow_scale=k)
        rep0 = replay(cache[run], flow_scale=1.0)
        # Correct for the tiny integrator-version difference (lead term) by using the ratio.
        exact[run] = (k, row.D_recorded * rep["final_distance"] / rep0["final_distance"])
    df["k_final"] = [exact[r][0] for r in df.run]
    df["D_final_cal"] = [exact[r][1] for r in df.run]
    df["err_final_cal_pct"] = 100 * (df.D_final_cal / df.course_m - 1)
    df["err_final_cal_m"] = df.D_final_cal - df.course_m
    df = df.merge(cal[["run", "err_loo_pct_linear"]], on="run", how="left")
    df.to_csv(os.path.join(OUT, "distance_runs.csv"), index=False)

    c6 = df[df.run.isin(CAL6)]
    e = c6.err_final_cal_pct.to_numpy()
    eloo = c6.err_loo_pct_linear.to_numpy()
    summary = dict(
        fit_focal_correction=c_hat, fit_height_offset_m=d_hat,
        fit_se_focal=float(np.sqrt(cov[0, 0])), fit_se_offset_m=float(np.sqrt(cov[1, 1])),
        fit_corr=float(cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])),
        fit_residual_sd_pct=100 * np.sqrt(s2), fit_dof=dof,
        focal_only_fit=float(fit1.x[0]), focal_only_resid_pct=(100 * resid([fit1.x[0], 0.0])).tolist(),
        shipped_focal_correction=FOCAL_CORRECTION, shipped_offset_m=LIDAR_H_OFFSET,
        final_cal_err_pct=dict(zip(c6.run, e)), writeup_claimed_pct=dict(zip(CAL6, WRITEUP_CAL_PCT)),
        in_sample=dict(n=len(e), bias_pct=e.mean(), sd_pct=e.std(ddof=1), rmse_pct=np.sqrt((e ** 2).mean()),
                       mae_pct=np.abs(e).mean(), max_abs_pct=np.abs(e).max(),
                       bias_m=c6.err_final_cal_m.mean(), rmse_m=np.sqrt((c6.err_final_cal_m ** 2).mean()),
                       max_abs_m=c6.err_final_cal_m.abs().max()),
        loo=dict(n=len(eloo), bias_pct=eloo.mean(), sd_pct=eloo.std(ddof=1), rmse_pct=np.sqrt((eloo ** 2).mean()),
                 mae_pct=np.abs(eloo).mean(), max_abs_pct=np.abs(eloo).max(), values=dict(zip(c6.run, eloo))),
        pre_cal_runs_5_7_err_pct=df[df.group == "16ft h=0.186 pre-cal"].err_recorded_pct.tolist(),
        first_cal_runs_9_11_err_pct=df[df.group == "16ft h=0.235 cal-1"].err_recorded_pct.tolist(),
        first_cal_focal_factor=float(923.483551 / F0),
    )
    # t-based 95% CI on the mean error (n=6) and a prediction interval for a NEW run.
    from scipy import stats
    for key, arr in (("in_sample", e), ("loo", eloo)):
        n = len(arr); tq = stats.t.ppf(0.975, n - 1); sd = arr.std(ddof=1)
        summary[key]["ci95_mean_pct"] = [arr.mean() - tq * sd / np.sqrt(n), arr.mean() + tq * sd / np.sqrt(n)]
        summary[key]["pi95_new_run_pct"] = [arr.mean() - tq * sd * np.sqrt(1 + 1 / n), arr.mean() + tq * sd * np.sqrt(1 + 1 / n)]
        # chi-square 95% CI on the SD itself (small n => wide)
        summary[key]["sd_ci95_pct"] = [sd * np.sqrt((n - 1) / stats.chi2.ppf(0.975, n - 1)),
                                       sd * np.sqrt((n - 1) / stats.chi2.ppf(0.025, n - 1))]
    save_json(summary, "distance_calibration.json")

    # ---------------- figures ----------------
    order = list(TAPE_RUNS)
    d = df.set_index("run").loc[order].reset_index()
    fig, ax = plt.subplots(figsize=(11, 4.6))
    x = np.arange(len(d))
    ax.axhspan(-1, 1, color=C["band"], zorder=0)
    ax.bar(x - 0.2, d.err_recorded_pct, 0.4, color=C["grey"], label="As recorded on the phone")
    ax.bar(x + 0.2, d.err_final_cal_pct, 0.4, color=C["blue"], label="Replayed with final calibration")
    ax.axhline(0, color="k", lw=0.8)
    labels = [f"{g.split()[0]}\n{g.split()[1].replace('h=', 'h ')}\n{n.split(',')[0]}" for g, n in zip(d.group, d.note)]
    ax.set_xticks(x, labels, fontsize=7)
    ax.set_ylabel("Distance error vs tape (%)")
    ax.set_title("Taped-course distance error, every run with a known course length")
    for xi, v in zip(x, d.err_final_cal_pct):
        ax.text(xi + 0.2, v + (0.6 if v >= 0 else -0.6), f"{v:+.1f}", ha="center", va="bottom" if v >= 0 else "top", fontsize=7, color=C["blue"])
    ax.axvspan(5.5, 14.5, color=C["warn"], alpha=0.08, zorder=0)
    ax.set_ylim(ax.get_ylim()[0], 12)
    ax.text(10, 6, "outside the validated envelope\n(79 cm mount, or ride height changed mid-run)", ha="center", fontsize=8.5, color=C["warn"])
    ax.legend(loc="lower left", fontsize=8, frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "dist_all_runs.png"), dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    c6o = d[d.run.isin(CAL6)]
    xx = np.arange(6)
    ax.axhspan(-1, 1, color=C["band"], zorder=0)
    ax.plot(xx, c6o.err_recorded_pct, "o", color=C["grey"], ms=7, label="As recorded")
    ax.plot(xx, c6o.err_final_cal_pct, "s", color=C["blue"], ms=7, label="Final calibration (in-sample)")
    ax.plot(xx, c6o.err_loo_pct_linear, "^", color=C["orange"], ms=7, label="Leave-one-out prediction")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xticks(xx, ["run 5\n18.6 cm", "run 6", "run 7\n~1 m/s", "run 9\n23.5 cm", "run 10", "run 11\nother light"], fontsize=8)
    ax.set_ylabel("Distance error vs 4.877 m tape (%)")
    ax.set_title("Calibration runs: before vs after, and cross-validated")
    ax.legend(fontsize=8, frameon=False, loc="center right")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "dist_calibration.png"), dpi=180); plt.close(fig)

    print(df[["run", "group", "h_used", "c_rec", "d_rec", "D_recorded", "err_recorded_pct", "k_final", "D_final_cal",
              "err_final_cal_pct", "err_loo_pct_linear"]].to_string())
    import json
    print(json.dumps({k: v for k, v in summary.items() if k not in ("final_cal_err_pct", "writeup_claimed_pct")}, indent=1, default=float))


if __name__ == "__main__":
    main()
