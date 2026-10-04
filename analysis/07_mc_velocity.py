"""Phase 6/8: Monte Carlo of the VELOCITY measurement in the car (final build, run 01-59-42).

Keeps four velocities distinct:
  TARGET     v_true(t): the car's true speed. Simulated as the final run's own (GNSS-free) filter
             speed for t = 0-95 s, smoothed over 0.5 s (steady cruise, acceleration to 8.7 m/s, braking)
  MEASURED   flow(t) = v_true(t) * s_static * (1 + eps_slow(t)) + n(t), at the real 240 Hz frame
             times with the real PSR sequence
  ESTIMATED  output of the vectorised ReferenceKF4 (IMU predict + flow + GNSS + ZUPT, as on the phone)
  MONTE CARLO = the distribution of ESTIMATED - TARGET over trials (and of ESTIMATED - simulated GNSS,
             which is what the real-data comparison can observe)

Error terms and where each number comes from
  n(t)        per-frame flow noise: EMPIRICAL car residual pool for the frame's PSR bin, AR(1) with
              the measured lag-1 autocorrelation (05_noise_characterization)
  eps_slow    slowly varying relative scale error (ride-height change under pitch/heave, texture,
              residual reference effects): zero-mean Ornstein-Uhlenbeck process. Its SD and time
              constant are DERIVED from the real envelope residuals (PSR >= 15) after subtracting the
              GNSS white-noise variance; this term is the one the data cannot pin down well (n_eff ~ 7)
  s_static    calibration (c, delta) uncertainty at the car mount h = 0.171 m (outside the 0.186-0.235 m
              calibration span: mild extrapolation) + LiDAR repeatability
  IMU         a = dv/dt (+ yaw coupling, real gyro z) + N(0, sd at rest in the car) + bias N(0, 0.02) (assumed)
  GNSS        |v_true(t - 0.65 s)| + N(0, empirical white SD), fused with the logged speed_acc as on the phone
Writes out/mc_velocity_summary.json, out/mc_velocity_samples.npz, figures/mc_vel_*.png
"""
import json
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

import importlib
from common import FIG, OUT, SEED, load_run, save_json
from kf import parse_filter_state, replay
from mc_kf import build_events, run_vectorised
from plotstyle import C, style

noise_mod = importlib.import_module("05_noise_characterization")
PSR_EDGES = np.array(noise_mod.PSR_EDGES, float)
RUN = "2026-10-04_01-59-42"
T_END = 95.0


def derive_inputs():
    vel = json.load(open(os.path.join(OUT, "velocity_summary.json")))
    noise = json.load(open(os.path.join(OUT, "noise_params.json")))
    cal = json.load(open(os.path.join(OUT, "distance_calibration.json")))
    lag = vel[RUN]["lag_s"]
    sd_gnss = vel["final_envelope"]["gnss_noise_sd_empirical"]
    s = pd.read_csv(os.path.join(OUT, f"velocity_samples_{RUN}.csv"))
    env = s[(s.gps > 2) & (s.gps <= 11.2) & (s.t < vel["final_envelope"]["t_first_overspeed"])]
    good = env[env.q_med >= 15]
    rel = (good.err_nognss / good.gps).to_numpy()
    gnss_rel_var = np.mean((sd_gnss / good.gps.to_numpy()) ** 2)
    sd_slow = float(np.sqrt(max(np.mean(rel ** 2) - gnss_rel_var, 0)))
    ab = good.err_nognss.to_numpy()
    sd_slow_abs = float(np.sqrt(max(np.mean(ab ** 2) - sd_gnss ** 2, 0)))
    # Lag-1 autocorrelation within contiguous 1 Hz stretches
    tt = good.t.to_numpy(); pairs = np.where(np.abs(np.diff(tt) - 1.0) < 0.2)[0]
    rc = rel - rel.mean()
    rho1 = float(np.sum(rc[pairs] * rc[pairs + 1]) / np.sqrt(np.sum(rc[pairs] ** 2) * np.sum(rc[pairs + 1] ** 2)))
    tau = float(-1.0 / np.log(max(min(rho1, 0.99), 0.05)))
    # Empirical residual pools (car, envelope) and AR(1)
    r = load_run(RUN)
    res, q, _, _ = noise_mod.flow_residuals(r["flow"], 262.0)
    b = np.clip(np.searchsorted(PSR_EDGES, q, side="right") - 1, 0, len(PSR_EDGES) - 2)
    pools = [res[b == i] for i in range(len(PSR_EDGES) - 1)]
    for i in range(len(pools)):
        if len(pools[i]) < 50:
            j = min((j for j in range(len(pools)) if len(pools[j]) >= 50), key=lambda j: abs(j - i))
            pools[i] = pools[j]
    rho_white = noise["flow_car"]["acf_mean"][0]
    g = noise["lidar_repeatability"]["groups"]
    v1, v2 = np.array(g["0.186 (runs 3-7)"]["values"]), np.array(g["0.235 (runs 9-11)"]["values"])
    sd_lidar = float(np.sqrt((((v1 - v1.mean()) ** 2).sum() + ((v2 - v2.mean()) ** 2).sum()) / (len(v1) + len(v2) - 2)))
    sd_h = float(np.hypot(sd_lidar, noise["h_quantisation_sd_m"]))
    params = dict(lag_s=lag, sd_gnss=sd_gnss, sd_slow_rel=sd_slow, sd_slow_abs=sd_slow_abs, slow_rho1_at_1s=rho1, tau_slow_s=tau,
                  n_epochs_used=int(len(good)), rel_err_mean=float(rel.mean()), rel_err_rms=float(np.sqrt(np.mean(rel ** 2))),
                  rho_white_240hz=rho_white, sd_h_m=sd_h, sd_imu=noise["imu_car_rest"]["sd_ax"], sd_imu_bias_assumed=0.02,
                  h_car=0.171)
    return params, pools, cal, good


def build_profile():
    r = load_run(RUN)
    rep = replay(r, use_gnss=False)
    t0 = r["est"].t.iloc[0]
    m = rep["t"] - t0 <= T_END
    t = rep["t"][m]
    vx = pd.Series(rep["v_x"][m]).rolling(51, center=True, min_periods=1).mean().to_numpy()
    vy = pd.Series(rep["v_y"][m]).rolling(51, center=True, min_periods=1).mean().to_numpy()
    imu = r["imu"][r["imu"].t - t0 <= T_END]
    fl = r["flow"][r["flow"].t - t0 <= T_END]
    gn = r["gnss"][r["gnss"].t - t0 <= T_END]
    gn = gn[np.r_[True, np.diff(gn.t.to_numpy()) > 0]]
    ev = r["events"]
    zt = ev.loc[(ev.event == "zupt") & (ev.t - t0 <= T_END), "t"].to_numpy(float)
    gz = np.interp(t, imu.t.to_numpy(), imu.gz.to_numpy())
    ax = np.gradient(vx, t) - gz * vy
    ay = np.gradient(vy, t) + gz * vx
    events = build_events(t, fl.t.to_numpy(), fl.quality.to_numpy(), gn.t.to_numpy(), zt)
    fs = parse_filter_state(ev)
    tf = fl.t.to_numpy()
    return dict(t0=t0, t=t, vx=vx, vy=vy, ax=ax, ay=ay, gz=gz, events=events, tf=tf,
                vfx=np.interp(tf, t, vx), vfy=np.interp(tf, t, vy),
                fbin=np.clip(np.searchsorted(PSR_EDGES, fl.quality.to_numpy(), side="right") - 1, 0, len(PSR_EDGES) - 2),
                tg=gn.t.to_numpy(), acc=gn.speed_acc.to_numpy(),
                x0=[fs[1]["vx"], fs[1]["vy"], fs[1]["bx"], fs[1]["by"]], pdiag=fs[2])


def simulate(prof, pools, prm, cal, N, rng, model="empirical", sd_slow=None, tau=None, white_mult=1.0,
             use_gnss=True, chunk=20000, keep_dense=0, slow_mode="abs"):
    """slow_mode 'abs': additive slow error in m/s (applied along the velocity direction);
    'rel': multiplicative scale error. Both derived from the same residuals."""
    if sd_slow is None:
        sd_slow = prm["sd_slow_abs"] if slow_mode == "abs" else prm["sd_slow_rel"]
    tau = prm["tau_slow_s"] if tau is None else tau
    sds = np.array([p.std() for p in pools])
    rho = prm["rho_white_240hz"]
    h = prm["h_car"] + 0.0079   # raw LiDAR height
    c_hat, d_hat = cal["fit_focal_correction"], cal["fit_height_offset_m"]
    cov = np.array([[cal["fit_se_focal"] ** 2, cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"]],
                    [cal["fit_corr"] * cal["fit_se_focal"] * cal["fit_se_offset_m"], cal["fit_se_offset_m"] ** 2]])
    # Epoch grid for scoring: every IMU step nearest to a GNSS epoch (1 Hz)
    t = prof["t"]
    ep_idx = np.searchsorted(t, prof["tg"]); ep_idx = ep_idx[(ep_idx > 0) & (ep_idx < len(t))]
    v_true_ep = np.hypot(prof["vx"][ep_idx], prof["vy"][ep_idx])
    ep_pos = {int(t_i): k for k, t_i in enumerate(ep_idx)}
    dense_idx = np.arange(0, len(t), 10)
    E, Dd, DIST = [], [], []
    left = N
    while left > 0:
        n = min(chunk, left); left -= n
        cd = rng.multivariate_normal([c_hat, d_hat], cov, size=n)
        s_static = (c_hat / cd[:, 0]) * ((h + cd[:, 1]) / (h + d_hat)) * (1 + rng.normal(0, prm["sd_h_m"], n) / (h + d_hat))
        bx = rng.normal(0, prm["sd_imu_bias_assumed"], n); by = rng.normal(0, prm["sd_imu_bias_assumed"], n)
        st = dict(slow=rng.normal(0, sd_slow, n), t_slow=prof["tf"][0], white=np.zeros(n), white_init=False)

        def white(i):
            b = prof["fbin"][i]
            if model == "empirical":
                e = pools[b][rng.integers(0, len(pools[b]), n)]
            elif model == "gaussian":
                e = rng.normal(0, sds[b], n)
            elif model == "t3":
                e = rng.standard_t(3, n) * sds[b] / np.sqrt(3)
            else:
                raise ValueError(model)
            if not st["white_init"]:
                st["white"] = e; st["white_init"] = True
            else:
                st["white"] = rho * st["white"] + np.sqrt(1 - rho ** 2) * e
            return st["white"] * white_mult

        def flow_fn(i):
            ti = prof["tf"][i]
            dt = ti - st["t_slow"]; st["t_slow"] = ti
            if dt > 0 and sd_slow > 0:
                a = np.exp(-dt / tau)
                st["slow"] = a * st["slow"] + np.sqrt(1 - a * a) * rng.normal(0, sd_slow, n)
            vx_, vy_ = prof["vfx"][i], prof["vfy"][i]
            if slow_mode == "rel":
                g = s_static * (1 + st["slow"])
            else:
                sp = np.hypot(vx_, vy_)
                g = s_static * (1 + (st["slow"] / sp if sp > 0.3 else 0.0))   # additive m/s along the motion
            return vx_ * g + white(i), vy_ * g + white(i)

        def imu_fn(i):
            return (prof["ax"][i] + bx + rng.normal(0, prm["sd_imu"], n),
                    prof["ay"][i] + by + rng.normal(0, prm["sd_imu"], n), prof["gz"][i])

        # GNSS reports the speed of ~0.65 s earlier (it lags the camera): speed(t) = |v(t + lag)| with lag < 0
        tg_true = np.hypot(np.interp(prof["tg"] + prm["lag_s"], t, prof["vx"]), np.interp(prof["tg"] + prm["lag_s"], t, prof["vy"]))

        def gnss_fn(i):
            return tg_true[i] + rng.normal(0, prm["sd_gnss"], n), prof["acc"][i]

        ests = np.zeros((len(ep_idx), n))
        dense = np.zeros((len(dense_idx), min(keep_dense, n))) if keep_dense else None
        dense_pos = {int(j): k for k, j in enumerate(dense_idx)}
        step = {"i": 0}

        def rec(tt, kf, dist):
            i = step["i"]; step["i"] += 1
            if i in ep_pos:
                ests[ep_pos[i]] = np.hypot(kf.x[0], kf.x[1])
            if dense is not None and i in dense_pos:
                dense[dense_pos[i]] = np.hypot(kf.x[0], kf.x[1])[:dense.shape[1]] - np.hypot(prof["vx"][i], prof["vy"][i])

        kf, dist = run_vectorised(prof["events"], n, flow_fn, imu_fn, gnss_fn if use_gnss else None,
                                  x0=prof["x0"], pdiag=prof["pdiag"], record_fn=rec)
        E.append(ests - v_true_ep[:, None])
        DIST.append(dist.lead)
        if dense is not None and keep_dense:
            Dd.append(dense); keep_dense = 0
    E = np.concatenate(E, 1)
    true_dist = np.trapezoid(np.hypot(prof["vx"], prof["vy"]), t)
    return E, v_true_ep, (np.concatenate(DIST) - true_dist) / true_dist, (Dd[0] if Dd else None), t[dense_idx] - prof["t0"]


def summarise(E, v_true, moving_min=2.0, sd_gnss=None, rng=None):
    m = v_true > moving_min
    e = E[m].ravel()
    a = np.abs(e)
    q = lambda p: float(np.percentile(e, p))
    out = dict(n_trials=int(E.shape[1]), n_epochs_per_trial=int(m.sum()), n_samples=int(len(e)),
               bias=float(e.mean()), sd=float(e.std()), rmse=float(np.sqrt((e ** 2).mean())),
               int68=[q(16), q(84)], int90=[q(5), q(95)], int95=[q(2.5), q(97.5)], int99=[q(0.5), q(99.5)],
               abs_p50=float(np.percentile(a, 50)), abs_p90=float(np.percentile(a, 90)), abs_p95=float(np.percentile(a, 95)),
               abs_p99=float(np.percentile(a, 99)), p_abs_gt_0p5=float((a > 0.5).mean()), p_abs_gt_1=float((a > 1.0).mean()))
    rel = (E[m] / v_true[m, None]).ravel() * 100
    out["rel_pct"] = dict(sd=float(rel.std()), int95=[float(np.percentile(rel, 2.5)), float(np.percentile(rel, 97.5))],
                          abs_p95=float(np.percentile(np.abs(rel), 95)), abs_p99=float(np.percentile(np.abs(rel), 99)))
    # 10-epoch (~10 s) mean speed error
    k = 10
    Em = E[m]
    if Em.shape[0] >= k:
        w = np.array([Em[i:i + k].mean(0) for i in range(0, Em.shape[0] - k + 1, k)]).ravel()
        out["mean_over_10s"] = dict(sd=float(w.std()), abs_p95=float(np.percentile(np.abs(w), 95)), abs_p99=float(np.percentile(np.abs(w), 99)))
    if sd_gnss is not None:
        obs = e + rng.normal(0, sd_gnss, len(e))
        out["observable_vs_gnss"] = dict(sd=float(obs.std()), rmse=float(np.sqrt((obs ** 2).mean())), abs_p95=float(np.percentile(np.abs(obs), 95)))
    return out


def main():
    style()
    t0 = time.time()
    rng = np.random.default_rng(SEED + 7)
    prm, pools, cal, good = derive_inputs()
    print(json.dumps(prm, indent=1), flush=True)
    prof = build_profile()
    FAST = os.environ.get("GSK_MC_FAST") == "1"   # run_all.py --fast: quick smoke run, not for reporting
    N = 3000 if FAST else 100_000
    E, vt, derr, dense, tdense = simulate(prof, pools, prm, cal, N, rng, keep_dense=300)
    res = dict(params=prm, N=N, seed=SEED + 7, profile=f"{RUN} t=0-{T_END:.0f} s")
    res["main"] = summarise(E, vt, sd_gnss=prm["sd_gnss"], rng=rng)
    res["main"]["distance_rel_err_pct"] = dict(mean=float(100 * derr.mean()), sd=float(100 * derr.std()),
                                               int95=[float(100 * np.percentile(derr, 2.5)), float(100 * np.percentile(derr, 97.5))])
    print("main", round(time.time() - t0), json.dumps(res["main"]), flush=True)
    save_json(res, "mc_velocity_summary.partial.json")   # checkpoint: survives an interrupted run
    np.savez_compressed(os.path.join(OUT, "mc_velocity_samples.npz"), E=E[:, :5000].astype(np.float32), v_true=vt,
                        dense=dense.astype(np.float32), tdense=tdense)

    Ns = 600 if FAST else 10_000
    sens = {"gaussian_white": dict(model="gaussian"), "student_t3_white": dict(model="t3"),
            "white_noise_x3": dict(white_mult=3.0), "no_slow_term": dict(sd_slow=0.0),
            "slow_sd_half_variance_to_reference": dict(sd_slow=prm["sd_slow_abs"] / np.sqrt(2)),
            "slow_tau_x0p5": dict(tau=prm["tau_slow_s"] * 0.5), "slow_tau_x2": dict(tau=prm["tau_slow_s"] * 2),
            "no_gnss_fusion": dict(use_gnss=False), "relative_slow_model": dict(slow_mode="rel")}
    for name, kw in sens.items():
        Es, vts, ds, _, _ = simulate(prof, pools, prm, cal, Ns, rng, **kw)
        res["sens_" + name] = summarise(Es, vts)
        res["sens_" + name]["distance_rel_err_pct_sd"] = float(100 * ds.std())
        print(name, round(time.time() - t0), {k: res["sens_" + name][k] for k in ("bias", "rmse", "int95", "abs_p99")}, flush=True)
        save_json(res, "mc_velocity_summary.partial.json")
    conv = {}
    for n in [n for n in (1000, 3000, 10000, 30000, 100000) if n <= N]:
        s_ = summarise(E[:, :n], vt); conv[n] = {k: s_[k] for k in ("rmse", "abs_p95", "abs_p99")}
    res["convergence"] = conv
    seeds = {}
    for sd in range(5):
        Es, vts, _, _, _ = simulate(prof, pools, prm, cal, 600 if FAST else 5000, np.random.default_rng(SEED + 2000 + sd))
        s_ = summarise(Es, vts); seeds[sd] = {k: s_[k] for k in ("rmse", "abs_p95", "abs_p99")}
    res["seeds_N5000"] = seeds

    # Analytic: steady-state KF velocity SD for flow-only updates at the median PSR (scalar Riccati)
    qa = 0.1; fs = 240.0; dt = 1 / fs
    psr = np.median(load_run(RUN)["flow"].quality.to_numpy()[:int(T_END * 240)])
    Rf = 1.6e-3 * (20 / max(psr, 8)) ** 2
    P = 0.0
    for _ in range(20000):
        P = P + qa * dt; P = P * Rf / (P + Rf)
    res["analytic_kf_steady_sd_white_only"] = dict(psr_median=float(psr), R_flow=Rf, sd_post=float(np.sqrt(P)),
                                                   note="filter's own 1-sigma for white noise only (it does not model slow scale error)")
    # Empirical vs MC (observable) comparison
    real = good[good.t <= T_END]
    real_e = real.err_nognss.to_numpy()
    mv = vt > 2
    sim_obs = (E[mv][:, :min(20000, E.shape[1])].ravel() + rng.normal(0, prm["sd_gnss"], mv.sum() * min(20000, E.shape[1])))
    res["empirical_vs_mc"] = dict(real_n=int(len(real_e)), real_bias=float(real_e.mean()), real_sd=float(real_e.std(ddof=1)),
                                  real_rmse=float(np.sqrt((real_e ** 2).mean())), real_abs_p95=float(np.percentile(np.abs(real_e), 95)),
                                  mc_obs_sd=float(sim_obs.std()), mc_obs_abs_p95=float(np.percentile(np.abs(sim_obs), 95)),
                                  ks_p=float(stats.ks_2samp(real_e, sim_obs[:200000]).pvalue),
                                  note="real epochs are autocorrelated (n_eff ~ 7): KS p-value overstates power")
    res["runtime_s"] = time.time() - t0
    save_json(res, "mc_velocity_summary.json")

    # ---------------- figures ----------------
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.2))
    ax = axs[0]
    bins = np.linspace(-2.5, 2.5, 101)
    ax.hist(E[mv][:, :min(20000, E.shape[1])].ravel(), bins=bins, density=True, color=C["blue"], alpha=0.5, label="MC: estimated − target")
    ax.hist(sim_obs, bins=bins, density=True, histtype="step", color=C["dark"], lw=1.2, label="MC: estimated − simulated GNSS")
    ax.hist(real_e, bins=np.linspace(-2.5, 2.5, 26), density=True, histtype="step", color=C["orange"], lw=2, label=f"real: estimated − GNSS (n={len(real_e)})")
    m_ = res["main"]
    for v in m_["int95"]:
        ax.axvline(v, color=C["blue"], ls="--", lw=0.8)
    ax.set_xlabel("velocity error (m/s)"); ax.set_ylabel("density"); ax.set_title("Velocity MC vs real (car, 0–95 s)")
    ax.legend(fontsize=7.5, loc="upper left")
    ax = axs[1]
    for j in range(min(25, dense.shape[1])):
        ax.plot(tdense, dense[:, j], color=C["blue"], lw=0.5, alpha=0.35)
    ax.plot(tdense, np.hypot(np.interp(tdense + prof["t0"], prof["t"], prof["vx"]), 0) * 0, color="k", lw=0.6)
    ax2 = ax.twinx(); ax2.plot(prof["t"] - prof["t0"], np.hypot(prof["vx"], prof["vy"]), color=C["grey"], lw=1); ax2.set_ylabel("target speed (m/s)", color=C["grey"])
    ax.set_ylim(-2, 2); ax.set_xlabel("time (s)"); ax.set_ylabel("estimated − target (m/s)"); ax.set_title("25 simulated error traces (time-correlated)")
    ax = axs[2]
    names = ["main"] + [k for k in res if k.startswith("sens_")]
    lab = {"main": "BASELINE", "sens_gaussian_white": "Gaussian frame noise", "sens_student_t3_white": "Student-t(3) frame noise",
           "sens_white_noise_x3": "frame noise ×3", "sens_no_slow_term": "no slow scale error",
           "sens_slow_sd_half_variance_to_reference": "half of slow error is GNSS's",
           "sens_slow_tau_x0p5": "slow τ ×0.5", "sens_slow_tau_x2": "slow τ ×2", "sens_no_gnss_fusion": "no GNSS fusion", "sens_relative_slow_model": "slow error ∝ speed (relative)"}
    y = np.arange(len(names))[::-1]
    for yi, nm in zip(y, names):
        r_ = res[nm]
        ax.plot(r_["int95"], [yi, yi], color=C["blue"], lw=4, solid_capstyle="butt")
        ax.plot(r_["int99"], [yi, yi], color=C["blue"], lw=1.2, alpha=0.6)
        ax.plot(r_["bias"], yi, "|", color="k", ms=10)
    ax.set_yticks(y, [lab[n] for n in names], fontsize=8); ax.axvline(0, color="k", lw=0.6)
    ax.set_xlabel("velocity error (m/s): thick 95%, thin 99%"); ax.set_title("Sensitivity of velocity bounds")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "mc_vel.png"), dpi=170); plt.close(fig)
    print(json.dumps({k: res[k] for k in ("convergence", "seeds_N5000", "analytic_kf_steady_sd_white_only", "empirical_vs_mc")}, indent=1, default=float))
    print("runtime", time.time() - t0)


def replot():
    """Redraw figures/mc_vel.png from the saved summary + samples (no re-simulation)."""
    style()
    res = json.load(open(os.path.join(OUT, "mc_velocity_summary.json")))
    z = np.load(os.path.join(OUT, "mc_velocity_samples.npz"))
    E, vt, dense, tdense = z["E"].astype(float), z["v_true"], z["dense"], z["tdense"]
    prm, _, _, good = derive_inputs()
    real_e = good[good.t <= T_END].err_nognss.to_numpy()
    mv = vt > 2
    rng = np.random.default_rng(SEED + 99)
    sim_obs = E[mv].ravel() + rng.normal(0, prm["sd_gnss"], mv.sum() * E.shape[1])
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.2))
    ax = axs[0]; bins = np.linspace(-2.5, 2.5, 101)
    ax.hist(E[mv].ravel(), bins=bins, density=True, color=C["blue"], alpha=0.5, label="MC: estimated − target")
    ax.hist(sim_obs, bins=bins, density=True, histtype="step", color=C["dark"], lw=1.2, label="MC: estimated − simulated GPS")
    ax.hist(real_e, bins=np.linspace(-2.5, 2.5, 26), density=True, histtype="step", color=C["orange"], lw=2, label=f"real: estimated − GPS (n={len(real_e)})")
    for v in res["main"]["int95"]:
        ax.axvline(v, color=C["blue"], ls="--", lw=0.8)
    ax.set_xlabel("velocity error (m/s)"); ax.set_ylabel("density"); ax.set_title(f"Velocity MC (N={res['N']:,}) vs real (car, 0–95 s)")
    ax.legend(fontsize=7.5, loc="upper left")
    ax = axs[1]
    for j in range(min(25, dense.shape[1])):
        ax.plot(tdense, dense[:, j], color=C["blue"], lw=0.5, alpha=0.35)
    ax.axhline(0, color="k", lw=0.6); ax.set_ylim(-2, 2)
    ax.set_xlabel("time (s)"); ax.set_ylabel("estimated − target (m/s)"); ax.set_title("25 simulated error traces (time-correlated)")
    ax = axs[2]
    lab = {"main": "BASELINE", "sens_gaussian_white": "Gaussian frame noise", "sens_student_t3_white": "Student-t(3) frame noise",
           "sens_white_noise_x3": "frame noise ×3", "sens_no_slow_term": "no slow drift",
           "sens_slow_sd_half_variance_to_reference": "half of drift is GPS's",
           "sens_slow_tau_x0p5": "drift τ ×0.5", "sens_slow_tau_x2": "drift τ ×2", "sens_no_gnss_fusion": "no GPS fusion",
           "sens_relative_slow_model": "drift ∝ speed (relative)"}
    names = [n for n in lab if n in res]
    y = np.arange(len(names))[::-1]
    for yi, nm in zip(y, names):
        r_ = res[nm]
        ax.plot(r_["int95"], [yi, yi], color=C["blue"], lw=4, solid_capstyle="butt")
        ax.plot(r_["int99"], [yi, yi], color=C["blue"], lw=1.2, alpha=0.6)
        ax.plot(r_["bias"], yi, "|", color="k", ms=10)
    ax.set_yticks(y, [lab[n] for n in names], fontsize=8); ax.axvline(0, color="k", lw=0.6)
    ax.set_xlabel("velocity error (m/s): thick 95%, thin 99%"); ax.set_title("Sensitivity of velocity bounds")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "mc_vel.png"), dpi=170); plt.close(fig)


if __name__ == "__main__":
    import sys
    replot() if "--replot" in sys.argv else main()
