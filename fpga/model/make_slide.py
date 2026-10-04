"""
make_slide.py - one picture for a slide: the simulated FPGA vs the Python Kalman filter.

Replays a recorded run through (a) the Verilog in ModelSim (the FPGA) and (b) kf_ref.py (the
Python filter), and plots both on the same axes with their difference underneath.

    python make_slide.py                       # -> fpga/slide_fpga_vs_python.png
    python make_slide.py --run <run_dir> --ticks 1200 --out slide.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kf_isa  # noqa: E402
import kf_ref  # noqa: E402
import kf_rtl  # noqa: E402

FPGA = Path(__file__).resolve().parents[1]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=str(FPGA.parent / "data" / "phone_runs" / "2026-10-03_22-22-18"))
    ap.add_argument("--ticks", type=int, default=800, help="IMU samples to replay")
    ap.add_argument("--sim", default=None, help="modelsim | iverilog")
    ap.add_argument("--out", default=str(FPGA / "slide_fpga_vs_python.png"))
    a = ap.parse_args(argv)

    r = kf_ref.load_run(a.run)
    cfg = kf_ref.FilterConfig.from_meta(r["meta"])
    n = min(a.ticks, len(r["imu"]))
    t1 = r["imu"][n - 1]["t"]
    imu = r["imu"][:n]
    flow = [s for s in r["flow"] if s["t"] <= t1]
    gnss = [s for s in r["gnss"] if s["t"] <= t1]
    ev = [e for e in r["events"] if e["t"] <= t1]
    seed = kf_ref.parse_filter_state(r["events"])

    py = kf_ref.replay(kf_ref.ReferenceKF4(cfg), imu, flow, gnss, ev, seed=seed)
    sim = a.sim or ("modelsim" if "modelsim" in kf_rtl.available() else "iverilog")

    def host(be):
        return kf_ref.replay(kf_isa.FixedKF4(cfg, backend=be), imu, flow, gnss, ev, seed=seed)

    print(f"simulating {n} IMU ticks + {len(flow)} flow updates on the FPGA ({sim}) ...")
    fp = kf_rtl.run_batch(host, sim)

    t0 = py[0][0]
    t = [row[0] - t0 for row in py]
    vx_py, vx_fp = [row[1] for row in py], [row[1] for row in fp]
    sg_py, sg_fp = [row[3] for row in py], [row[3] for row in fp]
    err = [abs(x - y) for x, y in zip(vx_fp, vx_py)]
    worst = max(err)
    phone = {round(e["t"], 6): e["v_x"] for e in r["est"]}

    ink, grey = "#1f2933", "#8a949e"
    c_py, c_fp = "#d1495b", "#00798c"
    plt.rcParams.update({"font.family": "DejaVu Sans", "axes.edgecolor": grey, "axes.labelcolor": ink,
                         "xtick.color": ink, "ytick.color": ink, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1.3], "hspace": 0.12})
    ax1.plot(t, vx_py, color=c_py, lw=5.5, alpha=0.55, label="Python Kalman filter (kf_ref.py)", solid_capstyle="round")
    ax1.plot(t, vx_fp, color=c_fp, lw=1.6, label="FPGA design, simulated in ModelSim (Quartus)")
    ph = [(tt, phone[round(row[0], 6)]) for tt, row in zip(t, py) if round(row[0], 6) in phone]
    if ph:
        ax1.plot(*zip(*ph), color=grey, lw=1, ls=":", label="phone's own estimate (est.csv)")
    ax1.set_ylabel("forward velocity v_x (m/s)")
    ax1.legend(frameon=False, loc="lower left", fontsize=10)
    ax1.set_title("Kalman filter on the FPGA gives the same answer as the Python filter", loc="left",
                  fontsize=15, color=ink, fontweight="bold", pad=24)
    ax1.text(0, 1.03, f"recorded cart run, {n} IMU samples and {len(flow)} optical-flow updates; "
             f"largest difference {worst:.1e} m/s", transform=ax1.transAxes, fontsize=10.5, color=grey)

    ax2.semilogy(t, [max(e, 1e-13) for e in err], color=c_fp, lw=1.2)
    ax2.axhline(1e-3, color=grey, lw=0.8, ls="--")
    ax2.text(t[-1], 1.4e-3, "1 mm/s", ha="right", fontsize=9, color=grey)
    ax2.set_ylim(1e-13, 1e-1)
    ax2.set_ylabel("|FPGA - Python|\n(m/s, log)")
    ax2.set_xlabel("time (s)")
    fig.subplots_adjust(left=0.09, right=0.98, top=0.85, bottom=0.1)
    fig.savefig(a.out, dpi=200)
    print(f"worst |v_x| difference {worst:.2e} m/s, sigma {max(abs(x - y) for x, y in zip(sg_fp, sg_py)):.2e}")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
