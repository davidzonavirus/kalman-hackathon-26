"""Collects every headline number into ONE machine-readable file, out/final_metrics.json, with the
source artefact of each value. The slides and README read their numbers from here."""
import json
import os

from common import OUT, save_json


def load(name):
    with open(os.path.join(OUT, name), encoding="utf-8") as f:
        return json.load(f)


def main():
    dc = load("distance_calibration.json")
    vs = load("velocity_summary.json")
    md = load("mc_distance_summary.json")
    mv = load("mc_velocity_summary.json")
    nz = load("noise_params.json")
    st = load("story_metrics.json")
    health = load("data_health.json")
    fe = vs["final_envelope"]
    FIN = "2026-10-04_01-59-42"
    out = {
        "_about": "Headline metrics of the forensic analysis. Every block names its source file in analysis/out/.",
        "data_health": dict(source="data_health.json", **health),
        "calibration_refit": dict(
            source="distance_calibration.json",
            focal_correction=dc["fit_focal_correction"], focal_correction_se=dc["fit_se_focal"],
            height_offset_m=dc["fit_height_offset_m"], height_offset_se_m=dc["fit_se_offset_m"], param_corr=dc["fit_corr"],
            shipped=dict(focal_correction=1.027, height_offset_m=-0.0079)),
        "distance_real": dict(
            source="distance_calibration.json, distance_runs.csv",
            course_m=4.8768, n_runs=6, scope="16 ft indoor taped course, mount 0.186-0.235 m, walking-pace pushes",
            before_any_fix_pct=dc["pre_cal_runs_5_7_err_pct"], after_first_fix_pct=dc["first_cal_runs_9_11_err_pct"],
            final_in_sample=dc["in_sample"], final_leave_one_out=dc["loo"]),
        "velocity_real": dict(
            source=f"velocity_summary.json, velocity_samples_{FIN}.csv",
            target_definition="GNSS Doppler ground speed (1 Hz), shifted by the fitted GNSS lag",
            estimate_definition="ReferenceKF4 replayed WITHOUT GNSS updates (independent of the target)",
            gnss_lag_s=vs[FIN]["lag_s"], gnss_white_noise_sd=fe["gnss_noise_sd_empirical"],
            gnss_reported_speed_acc_median_moving=vs[FIN]["gnss_speed_acc_median_moving"],
            gnss_influence_on_logged_estimate_max=vs[FIN]["gnss_influence_on_est_max"],
            envelope=fe["definition"], envelope_stats=fe["err_nognss"], envelope_bootstrap=fe["bootstrap"],
            envelope_relative_pct=fe["rel_pct"], n_effective=fe["n_effective_ar1"],
            steady=fe["steady_abs_accel_lt_0p3"], transient=fe["transient_abs_accel_ge_0p3"],
            low_psr_lt15=fe["low_psr_lt15"], segments=fe["segments"],
            whole_run_including_failures=fe["whole_run_including_failures"],
            zero_lock=dict(first_t=vs[FIN]["zero_lock_t_first"], last_t=vs[FIN]["zero_lock_t_last"], epochs=vs[FIN]["zero_lock_epochs"]),
            distance_vs_gnss=vs["final_distance_vs_gnss"],
            car_runs_all_moving_rmse={k: vs[k]["all_moving"]["est_nognss"]["rmse"] for k in vs if k.startswith("2026")},
            car_runs_band_ratio={k: {b: v["ratio_median"] for b, v in vs[k]["bins"].items()} for k in vs if k.startswith("2026")}),
        "mc_distance": dict(
            source="mc_distance_summary.json", N=md["inputs"]["N_main"], seed=md["inputs"]["seed"],
            repeat_run=md["S1_repeat_run"], new_run=md["S2_new_run"],
            variance_decomposition=md["inputs"]["variance_decomposition"], pipeline_pilot=md["inputs"]["pilot_pipeline_only"],
            analytic_sd_pct=md["analytic_S2_sd_pct"], convergence=md["convergence_S2"], seeds=md["seeds_S2_N20000"],
            empirical_vs_mc=md["empirical_vs_mc"],
            sensitivity={k[5:]: dict(sd_pct=v["sd_pct"], int95_pct=v["int95_pct"], int99_pct=v["int99_pct"], mean_pct=v["mean_pct"])
                         for k, v in md.items() if k.startswith("sens_")}),
        "mc_velocity": dict(
            source="mc_velocity_summary.json", N=mv["N"], seed=mv["seed"], profile=mv["profile"], params=mv["params"],
            main=mv["main"], convergence=mv["convergence"], seeds=mv["seeds_N5000"],
            analytic=mv["analytic_kf_steady_sd_white_only"], empirical_vs_mc=mv["empirical_vs_mc"],
            sensitivity={k[5:]: dict(bias=v["bias"], rmse=v["rmse"], int95=v["int95"], int99=v["int99"]) for k, v in mv.items() if k.startswith("sens_")}),
        "noise": dict(source="noise_params.json", flow_cart_robust_sd=nz["flow_cart"]["robust_sd"], flow_car_robust_sd=nz["flow_car"]["robust_sd"],
                      flow_kurtosis_excess=dict(cart=nz["flow_cart"]["kurtosis_excess"], car=nz["flow_car"]["kurtosis_excess"]),
                      lidar_pooled_sd_m=nz["lidar_repeatability"]["pooled_sd_m"], imu_rest_sd=dict(cart=nz["imu_cart_rest"]["sd_ax"], car=nz["imu_car_rest"]["sd_ax"])),
        "story": dict(source="story_metrics.json", **st),
    }
    save_json(out, "final_metrics.json")
    print("wrote final_metrics.json")


if __name__ == "__main__":
    main()
