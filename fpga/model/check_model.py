"""
check_model.py - is the FPGA microprogram the same filter as the Swift app's?

For every recorded run in data/phone_runs, replay the sensors through
  1. kf_ref.ReferenceKF4  (float port of ReferenceKF4.swift)
  2. kf_isa.FixedKF4      (bit-exact model of the FPGA: Q16.40, saturating, microcode)
and report the worst difference. Also compares the float port with the phone's est.csv
(only meaningful for runs recorded with the final filter: 2026-10-03_22-18-56 and later).

    python check_model.py                 # all runs
    python check_model.py --max-ticks 1500 --runs 2026-10-03_22 2026-10-04
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kf_isa  # noqa: E402
import kf_ref  # noqa: E402

DATA = Path(__file__).resolve().parents[2] / "data" / "phone_runs"
FINAL_FILTER_SINCE = "2026-10-03_22-18-56"      # earlier runs were made with older filter builds


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-ticks", type=int, default=3000, help="IMU samples per run (default 3000)")
    ap.add_argument("--runs", nargs="*", help="only runs whose id starts with one of these")
    ap.add_argument("--tol", type=float, default=1e-6, help="pass bar for |dv| FPGA vs float (m/s)")
    a = ap.parse_args(argv)

    print(f"{'run':21s} {'ticks':>6s} {'flow':>6s} {'zupt':>5s} | {'FPGA-float dv':>13s} {'dsigma':>9s} {'ddist':>9s}"
          f" | {'float-est.csv dv':>16s}")
    worst, ok = 0.0, True
    for d in sorted(DATA.iterdir()):
        if a.runs and not any(d.name.startswith(p) for p in a.runs):
            continue
        r = kf_ref.load_run(d)
        if len(r["imu"]) < 50:
            continue
        n = min(len(r["imu"]), a.max_ticks)
        t1 = r["imu"][n - 1]["t"]
        imu = r["imu"][:n]
        flow = [s for s in r["flow"] if s["t"] <= t1]
        gnss = [s for s in r["gnss"] if s["t"] <= t1]
        ev = [e for e in r["events"] if e["t"] <= t1]
        cfg = kf_ref.FilterConfig.from_meta(r["meta"])
        seed = kf_ref.parse_filter_state(r["events"])
        rf = kf_ref.replay(kf_ref.ReferenceKF4(cfg), imu, flow, gnss, ev, seed=seed)
        rx = kf_ref.replay(kf_isa.FixedKF4(cfg), imu, flow, gnss, ev, seed=seed)
        dv = max(max(abs(p[1] - q[1]), abs(p[2] - q[2])) for p, q in zip(rf, rx))
        ds = max(max(abs(p[3] - q[3]), abs(p[4] - q[4])) for p, q in zip(rf, rx))
        dd = abs(rf[-1][5] - rx[-1][5])
        est = {round(e["t"], 6): e for e in r["est"]}
        cmp_est = ""
        if est and d.name >= FINAL_FILTER_SINCE:
            pairs = [(p, est[round(p[0], 6)]) for p in rf if round(p[0], 6) in est]
            h = len(pairs) // 2
            cmp_est = f"{max(max(abs(p[1] - e['v_x']), abs(p[2] - e['v_y'])) for p, e in pairs[h:]):.2e}"
        print(f"{d.name:21s} {n:6d} {len(flow):6d} {sum(e['event'] == 'zupt' for e in ev):5d} | "
              f"{dv:13.2e} {ds:9.1e} {dd:9.1e} | {cmp_est:>16s}")
        worst = max(worst, dv)
        ok &= dv <= a.tol
    print(f"worst |dv| FPGA vs float: {worst:.2e} m/s  ->  {'PASS' if ok else 'FAIL'} (bar {a.tol:g})")
    return 0 if ok else 1


if __name__ == "__main__":
    t = time.time()
    code = main()
    print(f"({time.time() - t:.0f} s)")
    sys.exit(code)
