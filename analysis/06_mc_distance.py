"""Phase 6/8: Monte Carlo of the DISTANCE measurement on the 16 ft (4.877 m) taped course.

Every trial runs the real pipeline (vectorised ReferenceKF4 -> DistanceIntegrator with
deadband 0.05 m/s, tau 0.5 s, lead compensation) on the timestamps, PSR sequence and ZUPT
times of one of the six real calibration pushes. Only the measurements are synthetic:

  true velocity  v(t): the push's own replayed velocity profile, rescaled so the true path
                       length is exactly 4.8768 m
  flow           v(t) * s + n(t)
     n(t)        per-frame noise drawn from the EMPIRICAL residual pool of the same PSR bin
                 (05_noise_characterization; heavy-tailed), or Gaussian / Student-t(3) for
                 sensitivity
     s           per-run scale = s_cal * s_h * s_u
       s_cal     focal correction c and LiDAR offset delta drawn from the fitted bivariate
                 normal (03_distance_analysis: estimate + covariance, corr 0.99)  [scenario S2]
       s_h       LiDAR height error: within-mount repeatability (pooled SD of repeated
                 Measure-h results in the calibration groups) + 1 mm rounding in the app
       s_u       unexplained per-run variation (push, course alignment, floor): SD chosen so
                 the simulated in-sample scatter equals the empirical fit residual SD
                 (variance matching; derived below, not hand-picked)
  IMU            a = dv/dt + N(0, sigma_rest) + bias ~ N(0, 0.02)   (sigma_rest measured at rest;
                 bias SD is an ASSUMPTION; flow dominates so it barely matters)

Scenarios
  S1 "repeat run" : calibration fixed at the fitted values (comparable to in-sample residuals)
  S2 "new run"    : calibration parameters uncertain (comparable to leave-one-out residuals)
Writes out/mc_distance_summary.json, out/mc_distance_samples.npz, figures/mc_dist_*.png
"""
import importlib
import json
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

from common import COURSE_16FT, FIG, OUT, SEED, load_run, save_json
from kf import parse_filter_state, replay
from mc_kf import build_events, run_vectorised
from plotstyle import C, style

noise_mod = importlib.import_module("05_noise_characterization")
CAL6 = noise_mod.CART
PSR_EDGES = np.array(noise_mod.PSR_EDGES, float)
L = COURSE_16FT


def load_inputs():
    cal = json.load(open(os.path.join(OUT, "distance_calibration.json")))
    runs = pd.read_csv(os.path.join(OUT, "distance_runs.csv")).set_index("run")
    noise = json.load(open(os.path.join(OUT, "noise_params.json")))
    # Residual pools per PSR bin (cart)
    pools = [[] for _ in range(len(PSR_EDGES) - 1)]
    for run in CAL6:
        res, q, _, _ = noise_mod.flow_residuals(load_run(run)["flow"])
        b = np.clip(np.searchsorted(PSR_EDGES, q, side="right") - 1, 0, len(pools) - 1)
        for i in range(len(pools)):
            pools[i].append(res[b == i])
    pools = [np.concatenate(p) for p in pools]
    # Empty/small bins borrow the nearest populated bin with LOWER quality (conservative)
    for i in range(len(pools)):
        if len(pools[i]) < 50:
            j = min((j for j in range(len(pools)) if len(pools[j]) >= 50), key=lambda j: abs(j - i))
            pools[i] = pools[j]
    g = noise["lidar_repeatability"]["groups"]
    v1, v2 = np.array(g["0.186 (runs 3-7)"]["values"]), np.array(g["0.235 (runs 9-11)"]["values"])
    sd_lidar_cal = float(np.sqrt((((v1 - v1.mean()) ** 2).sum() + ((v2 - v2.mean()) ** 2).sum()) / (len(v1) + len(v2) - 2)))
    sd_h = float(np.hypot(sd_lidar_cal, noise["h_quantisation_sd_m"]))
    profiles = []
    for run in CAL6:
        r = load_run(run)
        k = float(runs.loc[run, "k_final"])
        rep = replay(r, flow_scale=k)
        t = rep["t"]
        # Truth = the push's own velocity, smoothed over 0.5 s (removes sensor noise, which would
        # otherwise inflate the "true" path), rescaled so the forward displacement equals the
        # straight taped course. (These pushes went along the phone's -x axis; sign is irrelevant
        # to the path-length odometer.)
        vx = pd.Series(rep["v_x"]).rolling(51, center=True, min_periods=1).mean().to_numpy()
        vy = pd.Series(rep["v_y"]).rolling(51, center=True, min_periods=1).mean().to_numpy()
        fwd = abs(np.trapezoid(vx, t))
        vx, vy = vx * L / fwd, vy * L / fwd
        ev = r["events"]
        zt = ev.loc[ev.event == "zupt", "t"].to_numpy(float)
        fl = r["flow"]
        events = build_events(r["imu"].t.to_numpy(), fl.t.to_numpy(), fl.quality.to_numpy(), None, zt)
        fs = parse_filter_state(ev)
        tf = fl.t.to_numpy()
        profiles.append(dict(run=run, h=float(runs.loc[run, "h_raw"]), events=events, t_imu=t,
                             ax=np.gradient(vx, t), ay=np.gradient(vy, t),
                             vfx=np.interp(tf, t, vx), vfy=np.interp(tf, t, vy),
                             fbin=np.clip(np.searchsorted(PSR_EDGES, fl.quality.to_numpy(), side="right") - 1, 0, len(pools) - 1),
                             x0=[fs[1]["vx"], fs[1]["vy"], fs[1]["bx"], fs[1]["by"]], pdiag=fs[2]))
    return cal, noise, pools, sd_h, sd_lidar_cal, profiles


def simulate(profiles, pools, N, rng, model="empirical", cal=None, cal_uncertain=True, sd_h=0.0, sd_u=0.0,
             sd_imu=0.046, sd_imu_bias=0.02, flow_noise_mult=1.0, h_override=None, chunk=25000, norm=1.0):
    """Returns distance errors (m) for N trials split evenly over the profiles."""
    sds = np.array([p.std() for p in pools])
    out = []
    per = int(np.ceil(N / len(profiles)))
    c_hat, d_hat = cal["fit_focal_correction"], cal["fit_height_offset_m"]
    cov = np.array([[cal["fit_se_focal"] ** 2, cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"]],
                    [cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"], cal["fit_se_offset_m"] ** 2]])
    for prof in profiles:
        left = per
        while left > 0:
            n = min(chunk, left); left -= n
            h = h_override if h_override is not None else prof["h"]
            if cal_uncertain:
                cd = rng.multivariate_normal([c_hat, d_hat], cov, size=n)
                s_cal = (c_hat / cd[:, 0]) * ((h + cd[:, 1]) / (h + d_hat))
            else:
                s_cal = np.ones(n)
            s_h = 1 + rng.normal(0, sd_h, n) / (h + d_hat)
            s_u = 1 + rng.normal(0, sd_u, n)
            s = s_cal * s_h * s_u * norm
            bx = rng.normal(0, sd_imu_bias, n); by = rng.normal(0, sd_imu_bias, n)

            def noise(i, size):
                b = prof["fbin"][i]
                if model == "empirical":
                    return pools[b][rng.integers(0, len(pools[b]), size)] * flow_noise_mult
                if model == "gaussian":
                    return rng.normal(0, sds[b], size) * flow_noise_mult
                if model == "t3":
                    return rng.standard_t(3, size) * sds[b] / np.sqrt(3) * flow_noise_mult
                raise ValueError(model)

            def flow_fn(i):
                return prof["vfx"][i] * s + noise(i, n), prof["vfy"][i] * s + noise(i, n)

            def imu_fn(i):
                return (prof["ax"][i] + bx + rng.normal(0, sd_imu, n), prof["ay"][i] + by + rng.normal(0, sd_imu, n), 0.0)

            kf, dist = run_vectorised(prof["events"], n, flow_fn, imu_fn, x0=prof["x0"], pdiag=prof["pdiag"])
            out.append(dist.lead - L)
    return np.concatenate(out)[:N]


def summarise(e_m):
    e = 100 * e_m / L
    a = np.abs(e)
    q = lambda p: float(np.percentile(e, p))
    return dict(n=int(len(e)), mean_pct=float(e.mean()), sd_pct=float(e.std(ddof=1)), mean_m=float(e_m.mean()), sd_m=float(e_m.std(ddof=1)),
                int68_pct=[q(16), q(84)], int90_pct=[q(5), q(95)], int95_pct=[q(2.5), q(97.5)], int99_pct=[q(0.5), q(99.5)],
                abs_p50_pct=float(np.percentile(a, 50)), abs_p90_pct=float(np.percentile(a, 90)), abs_p95_pct=float(np.percentile(a, 95)),
                abs_p99_pct=float(np.percentile(a, 99)), abs_p95_m=float(np.percentile(np.abs(e_m), 95)), abs_p99_m=float(np.percentile(np.abs(e_m), 99)),
                p_abs_gt_0p5pct=float((a > 0.5).mean()), p_abs_gt_1pct=float((a > 1).mean()), p_abs_gt_2pct=float((a > 2).mean()),
                p_abs_gt_5cm=float((np.abs(e_m) > 0.05).mean()))


def main():
    style()
    t0 = time.time()
    rng = np.random.default_rng(SEED)
    cal, noise, pools, sd_h, sd_lidar_cal, profiles = load_inputs()
    log = {}

    # ---- Pilot 1: pipeline-only (flow+IMU noise, no scale terms) -> integration noise ----
    e_flow = simulate(profiles, pools, 6000, rng, cal=cal, cal_uncertain=False)
    # The real calibration (c, delta) was fitted on the real pipeline output, so it already absorbs
    # the pipeline's mean bias for this kind of push (deadband 0.05 m/s drops the slow start/end,
    # lateral wobble adds path). Normalise that mean out once, like the calibration did; the
    # push-to-push spread of that bias stays in the simulation as a real effect.
    per_prof = [float(100 * e_flow[i * 1000:(i + 1) * 1000].mean() / L) for i in range(len(profiles))]
    b0 = float(e_flow.mean() / L)
    norm = 1.0 / (1.0 + b0)
    e_flow = simulate(profiles, pools, 6000, rng, cal=cal, cal_uncertain=False, norm=norm)
    sd_flow_pct = float(100 * e_flow.std() / L)
    # Analytic check: integrated white flow noise, sigma_D ~ sqrt(sum sigma_i^2) * dt_frame (KF passes low frequencies with gain 1)
    sds = np.array([p.std() for p in pools])
    an = []
    for p in profiles:
        dtf = np.median(np.diff(p["events"][0][p["events"][1] == 1]))
        an.append(np.sqrt((sds[p["fbin"]] ** 2).sum()) * dtf)
    analytic_flow_pct = float(100 * np.mean(an) / L)
    # mean (bias) of pipeline-only: path length of noisy 2-D velocity is biased up; deadband biases down
    log["pilot_pipeline_only"] = dict(n=6000, raw_pipeline_bias_pct=100 * b0, per_profile_bias_pct=per_prof,
                                      norm_factor=norm, mean_pct_after_norm=float(100 * e_flow.mean() / L), sd_pct=sd_flow_pct,
                                      analytic_white_noise_sd_pct=analytic_flow_pct)

    # ---- Variance matching for the unexplained per-run term ----
    sd_emp = cal["fit_residual_sd_pct"] / 100     # dof-corrected residual SD of the 2-parameter fit
    h_mean = np.mean([p["h"] + cal["fit_height_offset_m"] for p in profiles])
    sd_h_rel = sd_h / h_mean
    var_u = sd_emp ** 2 - sd_h_rel ** 2 - (sd_flow_pct / 100) ** 2
    sd_u = float(np.sqrt(max(var_u, 0.0)))
    log["variance_decomposition"] = dict(empirical_residual_sd_pct=100 * sd_emp, lidar_cal_groups_sd_m=sd_lidar_cal,
                                         height_sd_m_incl_rounding=sd_h, height_rel_sd_pct=100 * sd_h_rel,
                                         flow_integration_sd_pct=sd_flow_pct, unexplained_sd_pct=100 * sd_u,
                                         share_of_variance=dict(height=sd_h_rel ** 2 / sd_emp ** 2,
                                                                flow=(sd_flow_pct / 100) ** 2 / sd_emp ** 2,
                                                                unexplained=sd_u ** 2 / sd_emp ** 2))
    print(json.dumps(log, indent=1), flush=True)

    base = dict(cal=cal, sd_h=sd_h, sd_u=sd_u, norm=norm)
    FAST = os.environ.get("GSK_MC_FAST") == "1"   # run_all.py --fast: quick smoke run, not for reporting
    N = 3000 if FAST else 100_000
    res = {}
    e_S1 = simulate(profiles, pools, N, rng, cal_uncertain=False, **base); res["S1_repeat_run"] = summarise(e_S1)
    print("S1", time.time() - t0, res["S1_repeat_run"], flush=True)
    e_S2 = simulate(profiles, pools, N, rng, cal_uncertain=True, **base); res["S2_new_run"] = summarise(e_S2)
    print("S2", time.time() - t0, res["S2_new_run"], flush=True)

    # ---- Sensitivity (N = 20k each; convergence below shows 20k is ample for these stats) ----
    Ns = 1200 if FAST else 20_000
    sens = {
        "gaussian_flow_noise": dict(model="gaussian"),
        "student_t3_flow_noise": dict(model="t3"),
        "flow_noise_x3": dict(flow_noise_mult=3.0),
        "height_sd_all_groups_0p85mm": dict(sd_h=float(np.hypot(noise["lidar_repeatability"]["pooled_sd_m"], noise["h_quantisation_sd_m"]))),
        "unexplained_sd_at_chi2_upper95": dict(sd_u=float(np.sqrt(max((cal["in_sample"]["sd_ci95_pct"][1] / 100) ** 2 - sd_h_rel ** 2, 0)))),
        "no_unexplained_term": dict(sd_u=0.0),
        "mount_h_0p171_car": dict(h_override=0.171 + 0.0079),
        "mount_h_0p30_extrapolated": dict(h_override=0.30 + 0.0079),
        "mount_h_0p50_extrapolated": dict(h_override=0.50 + 0.0079),
    }
    for name, kw in sens.items():
        args = dict(base, cal_uncertain=True); args.update(kw)
        e = simulate(profiles, pools, Ns, rng, **args)
        res["sens_" + name] = summarise(e)
        print(name, round(time.time() - t0), {k: res["sens_" + name][k] for k in ("mean_pct", "sd_pct", "int95_pct", "abs_p99_pct")}, flush=True)

    # ---- Convergence (nested subsets of the S2 sample) and seed stability ----
    conv = {}
    # Trials are generated profile by profile, so subsets must be RANDOM (a prefix would contain
    # only the first push profiles). Dedicated seed so this step is reproducible on its own.
    e_perm = np.random.default_rng(SEED + 99).permutation(e_S2)
    for n in [n for n in (1000, 3000, 10000, 30000, 100000) if n <= N]:
        s = summarise(e_perm[:n]); conv[n] = dict(sd_pct=s["sd_pct"], abs_p95_pct=s["abs_p95_pct"], abs_p99_pct=s["abs_p99_pct"], int95_pct=s["int95_pct"])
    seeds = {}
    for sd in range(5):
        r2 = np.random.default_rng(SEED + 1000 + sd)
        s = summarise(simulate(profiles, pools, 1200 if FAST else 20000, r2, cal_uncertain=True, **base))  # noqa
        seeds[sd] = dict(sd_pct=s["sd_pct"], abs_p95_pct=s["abs_p95_pct"], abs_p99_pct=s["abs_p99_pct"])
    res["convergence_S2"] = conv
    per = int(np.ceil(N / len(profiles)))   # simulate() fills trials profile by profile
    res["per_profile_S2"] = {p["run"]: summarise(e_S2[i * per:(i + 1) * per]) for i, p in enumerate(profiles)}
    res["seeds_S2_N20000"] = seeds
    # ---- Analytic cross-check of total S2 variance (independent terms add) ----
    c_hat, d_hat = cal["fit_focal_correction"], cal["fit_height_offset_m"]
    cov = np.array([[cal["fit_se_focal"] ** 2, cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"]],
                    [cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"], cal["fit_se_offset_m"] ** 2]])
    var_cal = []
    for p in profiles:
        h = p["h"]
        g = np.array([-1 / c_hat, 1 / (h + d_hat)])       # d ln s / d(c, delta)
        var_cal.append(g @ cov @ g)
    an_sd = 100 * np.sqrt(np.mean(var_cal) + sd_h_rel ** 2 + sd_u ** 2 + (sd_flow_pct / 100) ** 2)
    res["analytic_S2_sd_pct"] = float(an_sd)
    res["analytic_cal_param_sd_pct"] = float(100 * np.sqrt(np.mean(var_cal)))
    # ---- Compare with empirical residuals ----
    emp_in = np.array(list(cal["final_cal_err_pct"].values()))
    emp_loo = np.array(list(cal["loo"]["values"].values()))
    res["empirical_vs_mc"] = dict(
        in_sample_vs_S1=dict(emp_sd=float(emp_in.std(ddof=1)), mc_sd=res["S1_repeat_run"]["sd_pct"],
                             ks_p=float(stats.kstest(emp_in, lambda x: np.searchsorted(np.sort(100 * e_S1 / L), x) / len(e_S1)).pvalue)),
        loo_vs_S2=dict(emp_sd=float(emp_loo.std(ddof=1)), mc_sd=res["S2_new_run"]["sd_pct"],
                       ks_p=float(stats.kstest(emp_loo, lambda x: np.searchsorted(np.sort(100 * e_S2 / L), x) / len(e_S2)).pvalue),
                       emp_max_abs=float(np.abs(emp_loo).max()),
                       mc_quantile_of_emp_max=float((np.abs(100 * e_S2 / L) < np.abs(emp_loo).max()).mean())),
        note="n=6 empirical runs: KS has little power; agreement is necessary, not sufficient")
    res["inputs"] = dict(log, N_main=N, seed=SEED, sd_h_m=sd_h, sd_u_pct=100 * sd_u, imu_sd=0.046, imu_bias_sd_assumed=0.02,
                         course_m=L, profiles=[p["run"] for p in profiles])
    res["runtime_s"] = time.time() - t0
    save_json(res, "mc_distance_summary.json")
    np.savez_compressed(os.path.join(OUT, "mc_distance_samples.npz"), S1=e_S1.astype(np.float32), S2=e_S2.astype(np.float32))

    # ---------------- figures ----------------
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.3))
    ax = axs[0]
    bins = np.linspace(-2, 2, 161)
    ax.hist(100 * e_S2 / L, bins=bins, density=True, color=C["blue"], alpha=0.55, label="MC S2: new run (N=100,000)")
    ax.hist(100 * e_S1 / L, bins=bins, density=True, histtype="step", color=C["dark"], lw=1.2, label="MC S1: repeat run, cal fixed")
    for v in emp_loo:
        ax.axvline(v, color=C["orange"], lw=1.5, ymax=0.25)
    for v in emp_in:
        ax.axvline(v, color=C["green"], lw=1.5, ymax=0.15)
    ax.plot([], [], color=C["orange"], lw=1.5, label="real runs, leave-one-out (n=6)")
    ax.plot([], [], color=C["green"], lw=1.5, label="real runs, in-sample (n=6)")
    s2 = res["S2_new_run"]
    for lo, hi in [s2["int95_pct"], s2["int99_pct"]]:
        ax.axvline(lo, color=C["blue"], ls="--", lw=0.8); ax.axvline(hi, color=C["blue"], ls="--", lw=0.8)
    ax.set_xlabel("distance error on 4.877 m course (%)"); ax.set_ylabel("density")
    ax.set_title("Distance Monte Carlo vs real runs"); ax.legend(fontsize=7.5, loc="upper left")
    ax = axs[1]
    names = ["S2_new_run"] + [k for k in res if k.startswith("sens_")]
    lab = {"S2_new_run": "BASELINE (S2)", "sens_gaussian_flow_noise": "Gaussian flow noise", "sens_student_t3_flow_noise": "Student-t(3) flow noise",
           "sens_flow_noise_x3": "flow noise ×3", "sens_height_sd_all_groups_0p85mm": "LiDAR SD 0.85 mm",
           "sens_unexplained_sd_at_chi2_upper95": "unexplained SD at 95% upper CI", "sens_no_unexplained_term": "no unexplained term",
           "sens_mount_h_0p171_car": "mount h 0.171 m (car)", "sens_mount_h_0p30_extrapolated": "mount h 0.30 m (extrapolated)",
           "sens_mount_h_0p50_extrapolated": "mount h 0.50 m (extrapolated)"}
    y = np.arange(len(names))[::-1]
    for yi, nm in zip(y, names):
        r_ = res[nm]
        ax.plot(r_["int95_pct"], [yi, yi], color=C["blue"], lw=4, solid_capstyle="butt")
        ax.plot(r_["int99_pct"], [yi, yi], color=C["blue"], lw=1.2, alpha=0.6)
        ax.plot(r_["mean_pct"], yi, "|", color="k", ms=10)
    ax.set_yticks(y, [lab[n] for n in names], fontsize=8); ax.axvline(0, color="k", lw=0.6)
    ax.set_xlabel("distance error (%): thick = 95% interval, thin = 99%"); ax.set_title("Sensitivity of the distance bounds")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "mc_dist.png"), dpi=170); plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.5, 3.6))
    ns = list(conv); ax.semilogx(ns, [conv[n]["abs_p95_pct"] for n in ns], "o-", color=C["blue"], label="|e| p95")
    ax.semilogx(ns, [conv[n]["abs_p99_pct"] for n in ns], "s-", color=C["orange"], label="|e| p99")
    for sd, v in seeds.items():
        ax.plot(20000, v["abs_p95_pct"], ".", color=C["blue"], alpha=0.6); ax.plot(20000, v["abs_p99_pct"], ".", color=C["orange"], alpha=0.6)
    ax.set_xlabel("trials N"); ax.set_ylabel("distance error (%)"); ax.set_title("Convergence and 5 extra seeds (dots)")
    ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(os.path.join(FIG, "mc_dist_convergence.png"), dpi=170); plt.close(fig)
    print(json.dumps({k: res[k] for k in ("convergence_S2", "seeds_S2_N20000", "analytic_S2_sd_pct", "analytic_cal_param_sd_pct", "empirical_vs_mc")}, indent=1, default=float))
    print("runtime", time.time() - t0)


if __name__ == "__main__":
    main()
