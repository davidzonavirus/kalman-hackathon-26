"""Validates the Python filter port against the phone's est.csv, and infers which distance
integrator each run used (the integrator changed during the night but git has only one
code commit covering 19:41 -> 02:12, so the per-run behaviour is inferred from the data).

Writes analysis/out/replay_validation.csv."""
import os
import sys

import numpy as np
import pandas as pd

from common import OUT, load_run, run_ids
from kf import replay


def integ(t, vx, vy, mode, deadband=0.0, tau=0.0, lead=False):
    from kf import DistanceIntegrator
    d = DistanceIntegrator(deadband, tau, mode)
    for a, b, c in zip(t, vx, vy):
        d.add(a, b, c)
    return d.lead_compensated if lead else d.distance


def main(selected=None):
    rows = []
    for run in selected or run_ids():
        r = load_run(run)
        est = r["est"]
        if est.empty or r["imu"].empty:
            continue
        t, vx, vy = est["t"].to_numpy(), est["v_x"].to_numpy(), est["v_y"].to_numpy()
        logged = float(est["distance"].iloc[-1])
        # Which integrator reproduces the logged distance from the logged velocities?
        cands = {
            "forward_raw": integ(t, vx, vy, "forward"),
            "path_raw": integ(t, vx, vy, "path"),
            "path_db05": integ(t, vx, vy, "path", 0.05),
            "path_db05_tau05_lead": integ(t, vx, vy, "path", 0.05, 0.5, True),
        }
        best = min(cands, key=lambda k: abs(cands[k] - logged))
        row = dict(run=run, logged_distance=logged, **{f"int_{k}": v for k, v in cands.items()},
                   best_integrator=best, best_abs_err=abs(cands[best] - logged))
        if not r["flow"].empty:
            rep = replay(r)
            n = min(len(rep["t"]), len(est))
            dv = np.abs(rep["v_x"][:n] - vx[:n])
            row.update(replay_max_dvx=float(dv.max()), replay_med_dvx=float(np.median(dv)),
                       replay_p99_dvx=float(np.percentile(dv, 99)),
                       replay_distance=rep["final_distance"],
                       replay_dist_err=rep["final_distance"] - logged,
                       n_flow_acc=rep["flow_counts"]["accepted"], n_flow_gated=rep["flow_counts"]["gated"],
                       n_flow_skip=rep["flow_counts"]["skipped"], n_lockout=rep["flow_counts"]["lockout"])
        rows.append(row)
        print(row["run"], row["best_integrator"], f"{row['best_abs_err']:.4f}",
              {k: float("%.3g" % v) for k, v in row.items() if k.startswith("replay")}, flush=True)
    df = pd.DataFrame(rows)
    if not selected:
        df.to_csv(os.path.join(OUT, "replay_validation.csv"), index=False)
    return df


if __name__ == "__main__":
    main(sys.argv[1:] or None)
