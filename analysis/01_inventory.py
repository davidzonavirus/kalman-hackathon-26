"""Phase 1/2: inventory + data-health statistics for every phone run, LiDAR log and
dashboard session. Writes analysis/out/inventory_runs.csv, inventory_lidar.csv,
inventory_dashboard.csv and data_health.json."""
import os
from collections import Counter

import numpy as np
import pandas as pd

from common import DASH, LIDAR, OUT, load_run, run_ids, save_json, speed


def stream_stats(df, nominal=None):
    if df.empty or "t" not in df:
        return dict(n=0)
    t = df["t"].to_numpy(float)
    dt = np.diff(t)
    num = df.select_dtypes("number")
    out = dict(
        n=len(df),
        dur=float(t[-1] - t[0]) if len(t) > 1 else 0.0,
        rate_hz=float(len(t) / (t[-1] - t[0])) if len(t) > 1 and t[-1] > t[0] else np.nan,
        dt_med_ms=float(np.median(dt) * 1e3) if len(dt) else np.nan,
        dt_p99_ms=float(np.percentile(dt, 99) * 1e3) if len(dt) else np.nan,
        dt_max_ms=float(dt.max() * 1e3) if len(dt) else np.nan,
        n_dup_t=int((dt == 0).sum()),
        n_backwards=int((dt < 0).sum()),
        n_nan=int(num.isna().sum().sum()),
        n_inf=int(np.isinf(num.to_numpy(float)).sum()),
    )
    if nominal:
        out["n_gaps_gt_3x"] = int((dt > 3 * nominal).sum())
    return out


def main():
    rows = []
    for run in run_ids():
        d = load_run(run)
        m, est, flow, imu, gnss, ev = d["meta"], d["est"], d["flow"], d["imu"], d["gnss"], d["events"]
        r = dict(run=run, focal_px=m.get("focal_px"), mount_h=m.get("mount_height_m"),
                 scale_factor=m.get("scale_factor"), filter=m.get("filter_name"),
                 cfg_q_accel=m.get("q", {}).get("q_accel"))
        for name, df, nom in (("est", est, 0.01), ("flow", flow, 1 / 240), ("imu", imu, 0.01), ("gnss", gnss, 1.0)):
            for k, v in stream_stats(df, nom).items():
                r[f"{name}_{k}"] = v
        if not est.empty:
            r["final_distance"] = float(est["distance"].iloc[-1])
            r["max_est_speed"] = float(speed(est).max())
            r["max_distance"] = float(est["distance"].max())
            # Distance drops (reset_distance or lag-comp step)
            dd = np.diff(est["distance"].to_numpy())
            r["distance_drops_gt_1cm"] = int((dd < -0.01).sum())
        if not flow.empty:
            q = flow["quality"].to_numpy()
            r["flow_q_med"] = float(np.median(q))
            r["flow_frac_q_lt8"] = float((q < 8).mean())
            r["flow_h_min"] = float(flow["h"].min()); r["flow_h_max"] = float(flow["h"].max())
            r["flow_h_med"] = float(flow["h"].median())
            r["flow_max_speed"] = float(speed(flow, "vx_cam", "vy_cam").max())
        if not imu.empty:
            r["imu_acc_absmax"] = float(imu[["ax", "ay", "az"]].abs().to_numpy().max())
            r["imu_gyro_absmax"] = float(imu[["gx", "gy", "gz"]].abs().to_numpy().max())
        if not gnss.empty:
            r["gnss_max_speed"] = float(gnss["speed"].max())
            valid = gnss["speed_acc"] >= 0
            r["gnss_acc_med"] = float(gnss.loc[valid, "speed_acc"].median()) if valid.any() else np.nan
        if not ev.empty:
            c = Counter(ev["event"])
            r["n_zupt"] = c.get("zupt", 0)
            r["events"] = ";".join(f"{k}:{v}" for k, v in sorted(c.items()) if k != "zupt")
            fmt = ev.loc[ev["event"].str.contains("camera|format|fps", case=False, na=False), "value"]
            r["camera_events"] = " | ".join([str(x) for x in fmt.head(4)])
        rows.append(r)
    inv = pd.DataFrame(rows)
    inv.to_csv(os.path.join(OUT, "inventory_runs.csv"), index=False)

    # LiDAR logs
    lrows = []
    for f in sorted(os.listdir(LIDAR)):
        df = pd.read_csv(os.path.join(LIDAR, f), comment="#")
        lr = dict(file=f, n=len(df), columns=" ".join(df.columns))
        for c in df.select_dtypes("number").columns:
            if c != "t":
                lr[f"{c}_med"] = float(df[c].median()); lr[f"{c}_std"] = float(df[c].std())
        lrows.append(lr)
    lid = pd.DataFrame(lrows)
    lid.to_csv(os.path.join(OUT, "inventory_lidar.csv"), index=False)

    # Dashboard sessions
    drows = []
    for s in sorted(os.listdir(DASH)):
        p = os.path.join(DASH, s)
        files = os.listdir(p)
        dr = dict(session=s, files=" ".join(sorted(files)))
        tp = os.path.join(p, "telemetry.csv")
        if os.path.exists(tp):
            tel = pd.read_csv(tp)
            dr.update(n=len(tel), columns=" ".join(tel.columns))
            if "recv_time" in tel:
                dr["dur"] = float(tel["recv_time"].iloc[-1] - tel["recv_time"].iloc[0]) if len(tel) > 1 else 0
            if "seq" in tel and len(tel) > 1:
                ds = np.diff(tel["seq"].to_numpy())
                dr["seq_missing"] = int(ds[ds > 0].sum() - (ds > 0).sum())
                dr["seq_backwards"] = int((ds < 0).sum())
        drows.append(dr)
    dash = pd.DataFrame(drows)
    dash.to_csv(os.path.join(OUT, "inventory_dashboard.csv"), index=False)

    health = dict(
        n_runs=len(inv), n_lidar=len(lid), n_dash=len(dash),
        total_nan=int(inv.filter(like="_n_nan").sum().sum()),
        total_inf=int(inv.filter(like="_n_inf").sum().sum()),
        total_dup_t=inv.filter(like="_n_dup_t").sum().to_dict(),
        total_backwards=inv.filter(like="_n_backwards").sum().to_dict(),
        focal_px_values=sorted(set(round(x, 3) for x in inv["focal_px"].dropna())),
    )
    save_json(health, "data_health.json")
    pd.set_option("display.width", 250, "display.max_columns", 60, "display.max_rows", 100)
    print(inv[["run", "focal_px", "mount_h", "est_dur", "flow_rate_hz", "flow_dt_max_ms", "imu_rate_hz",
               "imu_dt_max_ms", "gnss_n", "final_distance", "max_est_speed", "flow_q_med", "flow_h_min",
               "flow_h_max", "distance_drops_gt_1cm"]].to_string())
    print(health)
    print(lid.to_string())
    print(dash.to_string())


if __name__ == "__main__":
    main()
