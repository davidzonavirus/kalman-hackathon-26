"""
check_rtl.py - does the Verilog (simulated) do exactly what the Python model of it does?

Feeds the same sequence of predict / flow / GNSS / zero-velocity commands to
  * kf_isa.Cpu   (Python model of kf_cpu.v + microcode)
  * kf_cpu.v     (Verilog, simulated at clock level)
and requires every outcome, every published state value and, at the end, the whole
197-register file to be identical, bit for bit. The sequences are

  * random stress tests: huge and tiny inputs, saturation, x/0, GNSS gating, lockout recovery
  * segments of real recorded runs (the same replay as replay.py)

    python check_rtl.py                       # ModelSim if installed (Quartus), else Icarus
    python check_rtl.py --sim iverilog --quick
"""
from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kf_isa  # noqa: E402
import kf_ref  # noqa: E402
import kf_rtl  # noqa: E402

DATA = Path(__file__).resolve().parents[2] / "data" / "phone_runs"
NREG = len(kf_isa.program()[0].regs)


def stress_sequence(seed: int, n: int):
    """List of (op, args) exercising every path of the microprogram."""
    rnd = random.Random(seed)
    ops = [("predict", (0.01, 0.0, 0.0, 0.0))]
    t = 0.0
    for _ in range(n):
        k = rnd.random()
        if k < 0.45:
            t += rnd.choice([0.005, 0.01, 0.01, 0.02, 0.1, 0.5])
            a = rnd.choice([1, 1, 1, 20, 3e4])               # 3e4 saturates Q16.40 arithmetic
            ops.append(("predict", (t, rnd.gauss(0, a), rnd.gauss(0, a), rnd.gauss(0, 3))))
        elif k < 0.80:
            v = rnd.choice([0.0, 0.5, 3.0])
            out = rnd.random() < 0.1
            q = 10 ** rnd.uniform(0, 3.6) if rnd.random() > 0.05 else rnd.uniform(0, 8)
            ops.append(("flow", (t, v + rnd.gauss(0, 0.02) + (rnd.choice([-1, 1]) * 5 if out else 0),
                                 rnd.gauss(0, 0.02), q)))
        elif k < 0.90:
            ops.append(("gnss", (t, rnd.choice([0.0, 0.9, 1.5, 3.0, 3.1, 8.0, 40.0]),
                                 rnd.choice([0.0, 0.05, 0.3, 2.0]))))
        else:
            ops.append(("zupt", (t,)))
    return ops


def run_ops(be, ops, cfg):
    f = kf_isa.FixedKF4(cfg, backend=be)
    f.reset(0.0)
    trace = []
    for name, args in ops:
        if name == "predict":
            f.predict(*args); out = None
        elif name == "flow":
            out = f.update_flow(*args)
        elif name == "gnss":
            out = f.update_gnss(*args)
        else:
            out = f.update_zero_velocity(*args)
        s = f.state
        trace.append((out, s.vx, s.vy, s.bx, s.by, tuple(s.p_diag)))
    trace.append(tuple(be.read_many(list(range(NREG)))))      # whole register file
    return trace


def run_real(be, run_dir, ticks, cfg):
    r = kf_ref.load_run(run_dir)
    t1 = r["imu"][ticks - 1]["t"]
    seed = kf_ref.parse_filter_state(r["events"])
    f = kf_isa.FixedKF4(kf_ref.FilterConfig.from_meta(r["meta"]), backend=be)
    rows = kf_ref.replay(f, [s for s in r["imu"] if s["t"] <= t1], [s for s in r["flow"] if s["t"] <= t1],
                         [s for s in r["gnss"] if s["t"] <= t1], [e for e in r["events"] if e["t"] <= t1], seed=seed)
    return rows, tuple(be.read_many(list(range(NREG))))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", choices=("modelsim", "iverilog", "gate"), default=None)
    ap.add_argument("--quick", action="store_true", help="fewer / shorter tests")
    a = ap.parse_args(argv)
    sim = a.sim or ("modelsim" if "modelsim" in kf_rtl.available() else "iverilog")
    cfg = kf_ref.FilterConfig()
    ok = True
    t0 = time.time()

    seeds, n = ([1, 2], 150) if a.quick else ([1, 2, 3, 4, 5, 6], 400)
    for seed in seeds:
        ops = stress_sequence(seed, n)
        want = run_ops(kf_isa.SimBackend(), ops, cfg)
        got, = [kf_rtl.run_batch(lambda be: run_ops(be, ops, cfg), sim)]
        kinds = {}
        for (name, _), tr in zip(ops, want):
            if name != "predict" and tr[0]:
                kinds[(name, tr[0][0])] = kinds.get((name, tr[0][0]), 0) + 1
        same = got == want
        ok &= same
        print(f"stress seed {seed}: {len(ops)} commands, outcomes {dict(sorted(kinds.items()))}"
              f" -> {'bit-exact' if same else 'MISMATCH'}")

    runs = [("2026-10-03_22-22-18", 60 if a.quick else 250), ("2026-10-04_01-42-40", 40 if a.quick else 150)]
    for name, ticks in runs:
        d = DATA / name
        if not d.exists():
            continue
        want = run_real(kf_isa.SimBackend(), d, ticks, cfg)
        got = kf_rtl.run_batch(lambda be: run_real(be, d, ticks, cfg), sim)
        same = got == want
        ok &= same
        print(f"real run {name}, {ticks} ticks: {'bit-exact' if same else 'MISMATCH'}")

    print(f"RTL ({sim}) vs Python model: {'PASS' if ok else 'FAIL'}  ({time.time() - t0:.0f} s)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
