"""Renders the Monte Carlo result JSONs as readable table images:
out/figures/mc_distance_summary.png and out/figures/mc_velocity_summary.png."""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import FIG, OUT
from plotstyle import C

BLUE, LIGHT = "#0072B2", "#F2F5F8"


def load(name):
    with open(os.path.join(OUT, name), encoding="utf-8") as f:
        return json.load(f)


def iv(a, d=2, unit=""):
    return f"{a[0]:+.{d}f} … {a[1]:+.{d}f}{unit}".replace("-", "−")


def render(sections, title, subtitle, dst, width=13):
    """sections: list of (heading, header_row, rows, col_widths)."""
    n_rows = sum(len(r) + 2.2 for _, _, r, _ in sections) + 3
    fig = plt.figure(figsize=(width, 0.32 * n_rows))
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    H = n_rows; y = H - 0.6
    ax.set_xlim(0, 1); ax.set_ylim(0, H)
    ax.text(0.02, y, title, fontsize=16, fontweight="bold", va="top", color="#1F232B")
    y -= 1.0
    ax.text(0.02, y, subtitle, fontsize=9.5, va="top", color="#5F6670")
    y -= 1.2
    for heading, header, rows, cw in sections:
        ax.text(0.02, y, heading, fontsize=12, fontweight="bold", va="top", color=BLUE)
        y -= 0.9
        cw = [w * 0.96 / sum(cw) for w in cw]   # every table spans the page width, never beyond
        rows = [[c.replace("-", "−") if c[:1] == "-" else c for c in r] for r in rows]
        xs = [0.02]
        for w in cw[:-1]:
            xs.append(xs[-1] + w)
        tot = sum(cw)
        ax.add_patch(plt.Rectangle((0.02, y - 0.75), tot, 0.85, color=BLUE, lw=0))
        for x, h in zip(xs, header):
            ax.text(x + 0.008, y - 0.33, h, fontsize=9.5, fontweight="bold", color="white", va="center")
        y -= 0.85
        for i, r in enumerate(rows):
            if i % 2 == 0:
                ax.add_patch(plt.Rectangle((0.02, y - 0.75), tot, 0.85, color=LIGHT, lw=0))
            for j, (x, v) in enumerate(zip(xs, r)):
                ax.text(x + 0.008, y - 0.33, v, fontsize=9.2, va="center", color="#1F232B",
                        fontweight="bold" if j == 0 else "normal")
            y -= 0.85
        y -= 0.5
    fig.savefig(dst, dpi=170, facecolor="white", bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)


def distance():
    d = load("mc_distance_summary.json")
    inp = d["inputs"]; vd = inp["variance_decomposition"]; pil = inp["pilot_pipeline_only"]
    lab = {"S1_repeat_run": "Repeat run (calibration fixed)", "S2_new_run": "NEW RUN (calibration uncertain) ← headline"}
    main_rows = []
    for k in ("S1_repeat_run", "S2_new_run"):
        s = d[k]
        main_rows.append([lab[k], f"{s['n']:,}", f"{s['mean_pct']:+.2f} %", f"{s['sd_pct']:.2f} %", iv(s["int95_pct"], 2, " %"),
                          iv(s["int99_pct"], 2, " %"), f"{s['abs_p95_pct']:.2f} % ({s['abs_p95_m']*100:.1f} cm)",
                          f"{100*s['p_abs_gt_1pct']:.1f} %"])
    sl = {"gaussian_flow_noise": "Gaussian camera noise", "student_t3_flow_noise": "Heavy-tailed (Student-t 3) camera noise",
          "flow_noise_x3": "Camera noise ×3", "height_sd_all_groups_0p85mm": "LiDAR SD 0.85 mm (all groups)",
          "unexplained_sd_at_chi2_upper95": "Unexplained term at 95 % upper CI", "no_unexplained_term": "No unexplained term",
          "mount_h_0p171_car": "Mount 0.171 m (car height)", "mount_h_0p30_extrapolated": "Mount 0.30 m (extrapolated)",
          "mount_h_0p50_extrapolated": "Mount 0.50 m (extrapolated)"}
    sens_rows = [[sl[k[5:]], f"{v['mean_pct']:+.2f} %", f"{v['sd_pct']:.2f} %", iv(v["int95_pct"], 2, " %"), iv(v["int99_pct"], 2, " %"),
                  f"{v['abs_p99_pct']:.2f} %"] for k, v in d.items() if k.startswith("sens_")]
    conv_rows = [[f"{int(n):,}", f"{v['sd_pct']:.3f} %", f"{v['abs_p95_pct']:.3f} %", f"{v['abs_p99_pct']:.3f} %", iv(v["int95_pct"], 3, " %")]
                 for n, v in d["convergence_S2"].items()]
    seed_rows = [[f"seed +{1000 + int(k)}", f"{v['sd_pct']:.3f} %", f"{v['abs_p95_pct']:.3f} %", f"{v['abs_p99_pct']:.3f} %", ""]
                 for k, v in d["seeds_S2_N20000"].items()]
    ev = d["empirical_vs_mc"]
    val_rows = [
        ["Real leave-one-out vs MC new run", f"SD {ev['loo_vs_S2']['emp_sd']:.2f} % vs {ev['loo_vs_S2']['mc_sd']:.2f} %",
         f"worst real run at MC percentile {100*ev['loo_vs_S2']['mc_quantile_of_emp_max']:.0f}", f"KS p = {ev['loo_vs_S2']['ks_p']:.2f} (n = 6, low power)"],
        ["Real in-sample vs MC repeat run", f"SD {ev['in_sample_vs_S1']['emp_sd']:.2f} % vs {ev['in_sample_vs_S1']['mc_sd']:.2f} %", "MC slightly conservative",
         f"KS p = {ev['in_sample_vs_S1']['ks_p']:.2f}"],
        ["Analytic (independent terms)", f"SD {d['analytic_S2_sd_pct']:.2f} % vs MC {d['S2_new_run']['sd_pct']:.2f} %",
         f"calibration-parameter part {d['analytic_cal_param_sd_pct']:.2f} %", ""],
    ]
    pp = [f"{x:+.2f}" for x in pil["per_profile_bias_pct"]]
    inp_rows = [
        ["Trials / seed", f"{inp['N_main']:,}", f"seed {inp['seed']}", "6 real pushes' timestamps, PSR, ZUPT times"],
        ["Course (truth)", f"{inp['course_m']:.4f} m", "16 ft", "push profile smoothed 0.5 s, scaled to the course"],
        ["Camera-height SD", f"{inp['sd_h_m']*1000:.2f} mm", f"LiDAR {vd['lidar_cal_groups_sd_m']*1000:.2f} mm + rounding", "repeated Measure-h, same mount"],
        ["Unexplained per-run SD", f"{inp['sd_u_pct']:.2f} %", "variance matching", f"observed scatter {vd['empirical_residual_sd_pct']:.2f} %"],
        ["Pipeline bias (normalised out)", f"{pil['raw_pipeline_bias_pct']:+.2f} %", "per push: " + ", ".join(pp) + " %", "deadband + smoothing; absorbed by calibration"],
        ["Integrated camera noise", f"{pil['sd_pct']:.2f} % (MC)", f"analytic white-noise {pil['analytic_white_noise_sd_pct']:.2f} %", "rest = push-to-push profile effect"],
        ["IMU noise / bias SD", f"{inp['imu_sd']} m/s²", f"bias {inp['imu_bias_sd_assumed']} m/s² (assumed)", "negligible effect"],
        ["Variance shares", f"height {100*vd['share_of_variance']['height']:.0f} %", f"camera {100*vd['share_of_variance']['flow']:.0f} %",
         "sum > 100 % → unexplained term set to 0"],
    ]
    render([
        ("Main results: distance error on the 4.877 m course", ["Scenario", "Trials", "Mean", "SD (1σ)", "95 % interval", "99 % interval", "|error| 95th pct", "P(|e| > 1 %)"],
         main_rows, [0.27, 0.07, 0.08, 0.08, 0.14, 0.14, 0.13, 0.08]),
        ("Sensitivity (20,000 trials each, calibration uncertain)", ["Case", "Mean", "SD", "95 % interval", "99 % interval", "|error| 99th pct"],
         sens_rows, [0.32, 0.1, 0.1, 0.17, 0.17, 0.1]),
        ("Convergence (random subsets of the 100,000) and 5 extra seeds (20,000 each)", ["N / seed", "SD", "|e| p95", "|e| p99", "95 % interval"],
         conv_rows + seed_rows, [0.18, 0.14, 0.14, 0.14, 0.36]),
        ("Validation against reality and analytics", ["Check", "Result", "Detail", "Note"], val_rows, [0.27, 0.25, 0.25, 0.19]),
        ("Inputs and where they came from", ["Input", "Value", "Detail", "Source / note"], inp_rows, [0.22, 0.17, 0.3, 0.27]),
    ], "Distance Monte Carlo: results summary (mc_distance_summary.json)",
        "Full ReferenceKF4 + odometer run on real push timestamps; intervals are prediction intervals for one new run, not confidence intervals.",
        os.path.join(FIG, "mc_distance_summary.png"))


def velocity():
    v = load("mc_velocity_summary.json")
    m, p = v["main"], v["params"]
    main_rows = [
        ["Instantaneous (true speed > 2 m/s)", f"{v['N']:,} × {m['n_epochs_per_trial']}", f"{m['bias']:+.3f}", f"{m['sd']:.3f}",
         iv(m["int95"]), iv(m["int99"]), f"{m['abs_p95']:.2f} / {m['abs_p99']:.2f}", f"{100*m['p_abs_gt_0p5']:.0f} % / {100*m['p_abs_gt_1']:.1f} %"],
        ["10 s average", "", "", f"{m['mean_over_10s']['sd']:.3f}", "", "", f"{m['mean_over_10s']['abs_p95']:.2f} / {m['mean_over_10s']['abs_p99']:.2f}", ""],
        ["Relative (% of true speed)", "", "", f"{m['rel_pct']['sd']:.1f} %", iv(m["rel_pct"]["int95"], 1, " %"), "",
         f"{m['rel_pct']['abs_p95']:.1f} / {m['rel_pct']['abs_p99']:.1f} %", ""],
        ["Distance over the 95 s profile", "", f"{m['distance_rel_err_pct']['mean']:+.2f} %", f"{m['distance_rel_err_pct']['sd']:.2f} %",
         iv(m["distance_rel_err_pct"]["int95"], 1, " %"), "", "", ""],
    ]
    sl = {"gaussian_white": "Gaussian frame noise", "student_t3_white": "Heavy-tailed (Student-t 3) frame noise",
          "white_noise_x3": "Frame noise ×3", "no_slow_term": "No slow drift (frame noise only)",
          "slow_sd_half_variance_to_reference": "Half of drift is GPS's own error (rerun, fixed)",
          "slow_tau_x0p5": "Drift correlation time ×0.5", "slow_tau_x2": "Drift correlation time ×2",
          "no_gnss_fusion": "No GPS fusion", "relative_slow_model": "Drift proportional to speed"}
    sens_rows = [["BASELINE", f"{m['bias']:+.3f}", f"{m['rmse']:.3f}", iv(m["int95"]), iv(m["int99"]), f"{m['abs_p99']:.2f}"]]
    sens_rows += [[sl[k[5:]], f"{s['bias']:+.3f}", f"{s['rmse']:.3f}", iv(s["int95"]), iv(s["int99"]), f"{s['abs_p99']:.2f}"]
                  for k, s in v.items() if k.startswith("sens_")]
    conv_rows = [[f"{int(n):,}", f"{s['rmse']:.3f}", f"{s['abs_p95']:.3f}", f"{s['abs_p99']:.3f}"] for n, s in v["convergence"].items()]
    conv_rows += [[f"seed +{2000 + int(k)} (5,000)", f"{s['rmse']:.3f}", f"{s['abs_p95']:.3f}", f"{s['abs_p99']:.3f}"] for k, s in v["seeds_N5000"].items()]
    e, a = v["empirical_vs_mc"], v["analytic_kf_steady_sd_white_only"]
    val_rows = [
        ["Mean (bias)", f"real {e['real_bias']:+.2f}", f"MC {m['bias']:+.2f}", "drift model is zero-mean; real CI includes 0"],
        ["Spread (SD)", f"real {e['real_sd']:.2f}", f"MC + GPS noise {e['mc_obs_sd']:.2f}", f"real n = {e['real_n']}, n_eff ≈ 7"],
        ["|error| 95th pct", f"real {e['real_abs_p95']:.2f}", f"MC + GPS noise {e['mc_obs_abs_p95']:.2f}", f"KS p = {e['ks_p']:.3f} (overstated: autocorrelated)"],
        ["Filter's own σ (white noise only)", f"{a['sd_post']*1000:.0f} mm/s", f"median PSR {a['psr_median']:.0f}", "≪ real error: no scale/drift state"],
    ]
    inp_rows = [
        ["Profile (truth)", v["profile"], "GPS-free filter speed, smoothed 0.5 s", "cruise, accel to 8.7 m/s, braking"],
        ["Slow drift SD / τ", f"{p['sd_slow_abs']:.2f} m/s / {p['tau_slow_s']:.1f} s", f"lag-1 corr {p['slow_rho1_at_1s']:.2f} at 1 s", f"from {p['n_epochs_used']} GPS epochs (PSR ≥ 15)"],
        ["Relative drift (alt. model)", f"{100*p['sd_slow_rel']:.1f} %", f"real rel. RMS {100*p['rel_err_rms']:.1f} %", "over-predicts spread; sensitivity only"],
        ["GPS jitter / lag", f"{p['sd_gnss']:.3f} m/s / {abs(p['lag_s']):.2f} s", "2nd difference / cross-correlation", "lags the camera"],
        ["Frame noise", "empirical pools by PSR", f"AR(1) ρ = {p['rho_white_240hz']:.2f} at 240 Hz", "car residuals vs 0.1 s median"],
        ["Camera-height SD", f"{p['sd_h_m']*1000:.2f} mm", f"mount {p['h_car']} m", "plus calibration covariance"],
        ["IMU noise / bias SD", f"{p['sd_imu']:.3f} m/s²", f"bias {p['sd_imu_bias_assumed']} m/s² (assumed)", "car, at rest"],
        ["Trials / seed", f"{v['N']:,}", f"seed {v['seed']}", "sensitivities 10,000 each"],
    ]
    render([
        ("Main results: estimated − true speed, car (m/s unless noted)", ["Quantity", "Samples", "Mean", "SD (1σ)", "95 % interval", "99 % interval", "|e| p95 / p99", "P(|e|>0.5 / >1)"],
         main_rows, [0.24, 0.1, 0.08, 0.08, 0.14, 0.14, 0.12, 0.1]),
        ("Sensitivity (10,000 trials each)", ["Case", "Mean", "RMSE", "95 % interval", "99 % interval", "|e| p99"], sens_rows, [0.33, 0.09, 0.09, 0.17, 0.17, 0.1]),
        ("Convergence (subsets of the 100,000) and 5 extra seeds", ["N / seed", "RMSE", "|e| p95", "|e| p99"], conv_rows, [0.25, 0.15, 0.15, 0.15]),
        ("Validation (real data: final drive, first 95 s, PSR ≥ 15)", ["Check", "Real", "Simulated", "Note"], val_rows, [0.24, 0.17, 0.22, 0.33]),
        ("Inputs and where they came from", ["Input", "Value", "Detail", "Source / note"], inp_rows, [0.2, 0.24, 0.27, 0.25]),
    ], "Velocity Monte Carlo: results summary (mc_velocity_summary.json)",
        "Full ReferenceKF4 (IMU + camera + GPS + ZUPT) on the final car drive's real timestamps; intervals are prediction intervals for one reading.",
        os.path.join(FIG, "mc_velocity_summary.png"))


if __name__ == "__main__":
    distance(); velocity()
    print("wrote mc_distance_summary.png, mc_velocity_summary.png")
