"""
replay.py - run a recorded phone run through the Kalman filter on the simulated FPGA.

    python replay.py ../../data/phone_runs/2026-10-03_22-22-18                 # whole run, ModelSim
    python replay.py <run_dir> --ticks 500 --backend iverilog                   # first 500 IMU ticks
    python replay.py <run_dir> --backend python                                 # no simulator needed

The run's imu.csv / flow.csv / gnss.csv / events.csv (ZUPTs the phone applied) are merged in
timestamp order exactly like kfreplay / Replay.swift and fed to the FPGA through its host
registers. The same run is also pushed through the float port of the Swift filter, and the
three results are compared:

    FPGA (simulated RTL)  vs  float ReferenceKF4 (kf_ref.py)  vs  the phone's own est.csv

Runs recorded before the filter's last changes (the early ones in data/) will not match the
phone's est.csv; the Swift filter changed during the hackathon. From 2026-10-03_22-18-56 on
the float port reproduces est.csv to ~1e-7 m/s.

Backends: modelsim (Quartus' simulator, one batch launch), iverilog (same, or live),
python (kf_isa.Cpu, the bit-exact model of the processor, no Verilog involved).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kf_isa  # noqa: E402
import kf_ref  # noqa: E402
import kf_rtl  # noqa: E402


def load_slice(run_dir, ticks):
    r = kf_ref.load_run(run_dir)
    if ticks:
        t1 = r["imu"][min(ticks, len(r["imu"])) - 1]["t"]
        r["imu"] = [s for s in r["imu"] if s["t"] <= t1]
        for k in ("flow", "gnss", "events", "est"):
            r[k] = [s for s in r[k] if s["t"] <= t1]
    return r


def run_fpga(r, cfg, backend: str):
    seed = kf_ref.parse_filter_state(r["events"])

    def host(be):
        f = kf_isa.FixedKF4(cfg, backend=be)
        rows = kf_ref.replay(f, r["imu"], r["flow"], r["gnss"], r["events"], seed=seed)
        return rows, f

    if backend == "python":
        rows, f = host(kf_isa.SimBackend())
        return rows, f.cycles, "instructions"
    rows, f = kf_rtl.run_batch(host, "modelsim" if backend == "modelsim" else "iverilog")
    return rows, f.backend.clocks, "FPGA clocks"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--backend", choices=("modelsim", "iverilog", "gate", "python"), default=None,
                    help="default: modelsim if installed, else iverilog, else python")
    ap.add_argument("--ticks", type=int, default=0, help="only the first N IMU samples")
    ap.add_argument("--out", help="write the FPGA estimate as CSV")
    a = ap.parse_args(argv)

    avail = kf_rtl.available()
    backend = a.backend or ("modelsim" if "modelsim" in avail else "iverilog" if "iverilog" in avail else "python")
    r = load_slice(a.run_dir, a.ticks)
    cfg = kf_ref.FilterConfig.from_meta(r["meta"])
    seed = kf_ref.parse_filter_state(r["events"])
    print(f"{Path(a.run_dir).name}: {len(r['imu'])} IMU, {len(r['flow'])} flow, {len(r['gnss'])} GNSS, "
          f"{sum(e['event'] == 'zupt' for e in r['events'])} ZUPT; config q_accel={cfg.q_accel} gate={cfg.gate}")

    t0 = time.time()
    ref = kf_ref.ReferenceKF4(cfg)
    rows_ref = kf_ref.replay(ref, r["imu"], r["flow"], r["gnss"], r["events"], seed=seed)
    t_ref = time.time() - t0
    t0 = time.time()
    rows_fpga, work, unit = run_fpga(r, cfg, backend)
    t_fpga = time.time() - t0

    def worst(x, y, lo=0):
        dv = max(max(abs(p[1] - q[1]), abs(p[2] - q[2])) for p, q in zip(x[lo:], y[lo:]))
        ds = max(max(abs(p[3] - q[3]), abs(p[4] - q[4])) for p, q in zip(x[lo:], y[lo:]))
        return dv, ds, abs(x[-1][5] - y[-1][5])

    print(f"  float port   : {t_ref:6.1f} s")
    print(f"  FPGA ({backend:8s}): {t_fpga:6.1f} s, {work:,} {unit}"
          + (f" = {work / 50e6 * 1e3:.1f} ms of FPGA time at 50 MHz" if unit == "FPGA clocks" else ""))
    dv, ds, dd = worst(rows_fpga, rows_ref)
    print(f"  FPGA vs float: max |dv| {dv:.2e} m/s   max |dsigma| {ds:.2e}   |ddistance| {dd:.2e} m")

    est = {round(e["t"], 6): e for e in r["est"]}
    if est:
        pairs = [(row, est[round(row[0], 6)]) for row in rows_fpga if round(row[0], 6) in est]
        half = len(pairs) // 2
        dv = max(max(abs(p[1] - e["v_x"]), abs(p[2] - e["v_y"])) for p, e in pairs[half:])
        dd = abs(pairs[-1][0][5] - pairs[-1][1]["distance"])
        print(f"  FPGA vs phone est.csv (2nd half of run): max |dv| {dv:.2e} m/s, "
              f"distance {pairs[-1][0][5]:.3f} m vs {pairs[-1][1]['distance']:.3f} m")
    if a.out:
        with open(a.out, "w") as f:
            f.write("t,v_x,v_y,sigma_vx,sigma_vy,distance\n")
            for row in rows_fpga:
                f.write(",".join(repr(v) for v in row) + "\n")
        print("  wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
