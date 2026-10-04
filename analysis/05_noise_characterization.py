"""Phase 6 inputs: empirical noise statistics that parameterise the Monte Carlo.

* Flow (camera) velocity noise: high-frequency residual = flow - centred rolling median
  (25 samples ~ 0.1 s at 240 Hz), on moving samples with PSR >= 8, binned by PSR.
  Includes real lens vibration (it is noise relative to vehicle speed). Lag-k autocorrelation
  and tail shape (kurtosis, normal-vs-empirical quantiles) recorded.
* IMU accel noise at rest (ZUPT periods) on the cart and in the car.
* LiDAR height repeatability: repeated "Measure h" results at an unchanged mount.
* App height quantisation (h stored to 1 mm).
Writes out/noise_params.json, figures/noise_*.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

from common import FIG, LIDAR, OUT, load_run, save_json, speed
from plotstyle import C, style

CART = ["2026-10-03_22-22-18", "2026-10-03_22-23-45", "2026-10-03_22-25-09",
        "2026-10-03_22-37-35", "2026-10-03_22-38-37", "2026-10-03_22-39-51"]
CAR = "2026-10-04_01-59-42"
CAR_ENVELOPE_T = 262.0  # s after start; before the >25 mph excursion (04_velocity_analysis)
PSR_EDGES = [8, 15, 25, 40, 70, 150, 1e9]


def flow_residuals(flow, t_max=None, min_speed=0.3):
    f = flow.copy()
    if t_max is not None:
        f = f[f.t - f.t.iloc[0] < t_max]
    f = f[f.quality >= 8].reset_index(drop=True)
    s = speed(f, "vx_cam", "vy_cam")
    med = pd.Series(s).rolling(25, center=True, min_periods=13).median().to_numpy()
    res = s - med
    ok = np.isfinite(res) & (med > min_speed)
    return res[ok], f.quality.to_numpy()[ok], med[ok], s


def acf(x, lags):
    x = x - x.mean()
    v = (x * x).mean()
    return [float((x[:-k] * x[k:]).mean() / v) for k in lags]


def main():
    style()
    out = {}
    # ---------- flow noise ----------
    for name, runs, tmax in (("cart", CART, None), ("car", [CAR], CAR_ENVELOPE_T)):
        R, Q, M = [], [], []
        acfs = []
        for run in runs:
            r = load_run(run)
            res, q, med, _ = flow_residuals(r["flow"], tmax)
            R.append(res); Q.append(q); M.append(med)
            acfs.append(acf(res, [1, 2, 3, 5, 10, 24]))
        R = np.concatenate(R); Q = np.concatenate(Q); M = np.concatenate(M)
        bins = []
        for lo, hi in zip(PSR_EDGES[:-1], PSR_EDGES[1:]):
            m = (Q >= lo) & (Q < hi)
            if m.sum() < 50:
                continue
            x = R[m]
            mad = 1.4826 * np.median(np.abs(x - np.median(x)))
            bins.append(dict(psr_lo=lo, psr_hi=hi, n=int(m.sum()), sd=float(x.std()), robust_sd=float(mad),
                             rel_sd=float((x / M[m]).std()), kurtosis_excess=float(stats.kurtosis(x)),
                             p99_abs=float(np.percentile(np.abs(x), 99))))
        mad_all = 1.4826 * np.median(np.abs(R - np.median(R)))
        out[f"flow_{name}"] = dict(n=int(len(R)), sd=float(R.std()), robust_sd=float(mad_all),
                                   kurtosis_excess=float(stats.kurtosis(R)), acf_lags=[1, 2, 3, 5, 10, 24],
                                   acf_mean=np.mean(acfs, 0).tolist(), psr_bins=bins,
                                   frac_beyond_4robust_sd=float((np.abs(R) > 4 * mad_all).mean()),
                                   gaussian_expect_beyond_4sd=float(2 * stats.norm.sf(4)))
        out[f"_flow_{name}_res"] = R  # kept in memory for plots only
        out[f"_flow_{name}_q"] = Q

    # Fit sd(PSR) = a / PSR + b for the MC (R in the filter already assumes sd ~ 1/PSR)
    for name in ("cart", "car"):
        b = pd.DataFrame(out[f"flow_{name}"]["psr_bins"])
        mid = np.sqrt(b.psr_lo * np.minimum(b.psr_hi, 400))
        A = np.c_[1 / mid, np.ones(len(mid))]
        coef, *_ = np.linalg.lstsq(A, b.robust_sd, rcond=None)
        out[f"flow_{name}"]["sd_model_a_over_psr_plus_b"] = coef.tolist()

    # ---------- IMU at rest ----------
    for name, runs in (("cart", CART), ("car", [CAR])):
        ax_all = []
        for run in runs:
            r = load_run(run)
            z = r["events"].loc[r["events"].event == "zupt", "t"].to_numpy()
            imu = r["imu"]
            still = np.isin(np.round(imu.t.to_numpy(), 6), np.round(z, 6))
            ax_all.append(imu.loc[still, ["ax", "ay"]].to_numpy())
        a = np.concatenate(ax_all)
        out[f"imu_{name}_rest"] = dict(n=int(len(a)), sd_ax=float(a[:, 0].std()), sd_ay=float(a[:, 1].std()),
                                       mean_ax=float(a[:, 0].mean()), mean_ay=float(a[:, 1].mean()))

    # ---------- LiDAR repeatability ----------
    res = {}
    for f in sorted(os.listdir(LIDAR)):
        with open(os.path.join(LIDAR, f), encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("# result") and "none" not in line.split(":")[0]:
                    res[f[:-4]] = float(line.split()[2].rstrip(":"))
    # Groups at an unchanged mount (data/README.md tables; consecutive measurements)
    groups = {
        "0.186 (runs 3-7)": ["2026-10-03_22-18-50", "2026-10-03_22-20-36", "2026-10-03_22-22-12", "2026-10-03_22-23-38", "2026-10-03_22-25-02"],
        "0.235 (runs 9-11)": ["2026-10-03_22-37-30", "2026-10-03_22-38-32", "2026-10-03_22-39-45"],
        "0.252 (runs 12-14)": ["2026-10-03_22-55-15", "2026-10-03_22-55-49", "2026-10-03_22-57-20"],
        "0.79 (runs 16-17)": ["2026-10-03_23-01-45", "2026-10-03_23-03-14"],
        "0.182 (21:41-21:43)": ["2026-10-03_21-41-39", "2026-10-03_21-43-37"],
    }
    ss, dof, gstats = 0.0, 0, {}
    for g, ids in groups.items():
        v = np.array([res[i] for i in ids])
        gstats[g] = dict(values=v.tolist(), sd=float(v.std(ddof=1)), range=float(v.max() - v.min()))
        ss += ((v - v.mean()) ** 2).sum(); dof += len(v) - 1
    out["lidar_repeatability"] = dict(groups=gstats, pooled_sd_m=float(np.sqrt(ss / dof)), dof=dof,
                                      note="pooled within-mount SD of repeated Measure-h results; includes any real mount movement between pushes")
    out["lidar_vs_tape_79cm"] = dict(tape_m=0.787, lidar_results=[res[i] for i in groups["0.79 (runs 16-17)"]] + [res["2026-10-03_22-59-41"]],
                                     note="writeup 3.8: tape 0.787 m; LiDAR results include the -7.9 mm offset")
    out["h_quantisation_sd_m"] = 0.001 / np.sqrt(12)

    # ---------- figures ----------
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.9))
    ax = axs[0]
    for name, col in (("cart", C["blue"]), ("car", C["orange"])):
        b = pd.DataFrame(out[f"flow_{name}"]["psr_bins"])
        mid = np.sqrt(b.psr_lo * np.minimum(b.psr_hi, 400))
        ax.plot(mid, b.robust_sd * 1000, "o-", color=col, label=f"{name} (robust SD)")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("flow quality (PSR)"); ax.set_ylabel("per-frame speed noise (mm/s)")
    ax.set_title("Flow noise falls with quality"); ax.legend(fontsize=8)
    ax = axs[1]
    R = out["_flow_car_res"]; mad = out["flow_car"]["robust_sd"]
    z = R / mad
    ax.hist(z, bins=np.linspace(-10, 10, 161), density=True, color=C["orange"], alpha=0.7, label="car residual / robust SD")
    xx = np.linspace(-10, 10, 400); ax.plot(xx, stats.norm.pdf(xx), color="k", lw=1, label="Gaussian")
    ax.set_yscale("log"); ax.set_ylim(1e-5, 1); ax.set_xlabel("standardised residual"); ax.set_title("Tails are heavier than Gaussian")
    ax.legend(fontsize=8)
    ax = axs[2]
    for name, col in (("cart", C["blue"]), ("car", C["orange"])):
        ax.plot([0] + out[f"flow_{name}"]["acf_lags"], [1] + out[f"flow_{name}"]["acf_mean"], "o-", color=col, label=name)
    ax.axhline(0, color="k", lw=0.6); ax.set_xlabel("lag (frames at 240 Hz)"); ax.set_ylabel("autocorrelation")
    ax.set_title("Residual autocorrelation"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "noise_flow.png"), dpi=170); plt.close(fig)

    clean = {k: v for k, v in out.items() if not k.startswith("_")}
    save_json(clean, "noise_params.json")
    import json
    print(json.dumps(clean, indent=1, default=float))


if __name__ == "__main__":
    main()
