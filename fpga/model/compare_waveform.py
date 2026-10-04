"""
compare_waveform.py - compare the ModelSim (Quartus) waveform with the Python Kalman filter.

Run make_wave.py first (it simulates the FPGA and writes both files):
    sim/build/wave/fpga_waveform.csv   what ModelSim drew: inputs and outputs after every command
    sim/build/wave/ref_commands.csv    the Python filter (kf_ref.py) after the same commands

Then:
    python compare_waveform.py

writes, next to those files,
    python_waveform.png    the Python filter alone, laid out like the ModelSim wave window
    compare_waveform.png   FPGA and Python overlaid, with the error underneath
and prints the worst difference per signal (exit code 1 if above --tol).
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DIR = Path(__file__).resolve().parents[1] / "sim" / "build" / "wave"
INK, GREY = "#1f2933", "#8a949e"
C_PY, C_FP = "#d1495b", "#00798c"

# (column, label, colour in the ModelSim window)
INPUTS = [("in_ax", "a_x in (m/s²)", "#1fa2c9"), ("in_flow_vx", "flow v_x in (m/s)", "#f28c28"),
          ("in_flow_psr", "flow quality PSR in", "#f28c28")]
OUTPUTS = [("v_x", "v_x out (m/s)", "#2e9e4f"), ("v_y", "v_y out (m/s)", "#2e9e4f"),
           ("sigma_vx", "sigma_vx out (m/s)", "#b03fb0"), ("bias_x", "accel bias_x out (m/s²)", "#c9a400")]


def load(path):
    with open(path, newline="") as f:
        return [{k: (v if k == "command" else float(v)) for k, v in r.items()} for r in csv.DictReader(f)]


def style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "axes.edgecolor": GREY, "axes.labelcolor": INK,
                         "xtick.color": INK, "ytick.color": INK, "axes.spines.top": False,
                         "axes.spines.right": False, "font.size": 9})


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(DIR))
    ap.add_argument("--tol", type=float, default=1e-6, help="pass bar for the worst difference")
    a = ap.parse_args(argv)
    d = Path(a.dir)
    fpga = load(d / "fpga_waveform.csv")
    py = load(d / "ref_commands.csv")
    if len(fpga) != len(py):
        sys.exit(f"command counts differ: FPGA {len(fpga)} vs Python {len(py)}")
    n = [r["n"] for r in fpga]
    style()

    # ---- Python alone, laid out like the ModelSim window ---------------------------------
    sig = INPUTS + OUTPUTS
    fig, axes = plt.subplots(len(sig), 1, figsize=(11, 1.15 * len(sig) + 1), sharex=True)
    for ax, (col, label, colour) in zip(axes, sig):
        ax.step(n, [r[col] for r in py], where="post", color=colour, lw=1.3)
        ax.set_ylabel(label, rotation=0, ha="right", va="center", fontsize=8.5)
        ax.margins(y=0.15)
    axes[0].set_title("Python Kalman filter (kf_ref.py): same signals as the ModelSim waveform",
                      loc="left", fontsize=12, fontweight="bold", color=INK)
    axes[-1].set_xlabel("command number (predict / flow update / zero-velocity update)")
    fig.subplots_adjust(left=0.2, right=0.98, top=0.93, bottom=0.07, hspace=0.35)
    fig.savefig(d / "python_waveform.png", dpi=170)
    plt.close(fig)

    # ---- overlay and error ------------------------------------------------------------------
    pairs = [(c, lab) for c, lab, _ in INPUTS + OUTPUTS]
    fig, axes = plt.subplots(len(OUTPUTS) + 1, 1, figsize=(11, 8.5), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1.4, 1.4, 1.4, 1.6], "hspace": 0.3})
    worst = {}
    for ax, (col, label, _) in zip(axes, OUTPUTS):
        ax.step(n, [r[col] for r in py], where="post", color=C_PY, lw=5, alpha=0.5,
                label="Python (kf_ref.py)")
        ax.step(n, [r[col] for r in fpga], where="post", color=C_FP, lw=1.2, label="FPGA, ModelSim waveform")
        ax.set_ylabel(label, fontsize=8.5)
    axes[0].legend(frameon=False, loc="best", fontsize=9)
    axes[0].set_title("ModelSim (Quartus) waveform vs Python Kalman filter", loc="left", fontsize=13,
                      fontweight="bold", color=INK)
    err_ax = axes[-1]
    for col, label, _ in OUTPUTS:
        e = [abs(f[col] - p[col]) for f, p in zip(fpga[1:], py[1:])]      # command 0 is the seed
        # sigma_vy is not in the ModelSim csv; v_y, v_x, bias and sigma_vx are
        worst[col] = max(e)
        err_ax.semilogy(n[1:], [max(v, 1e-14) for v in e], lw=1.1, label=label.split(" out")[0])
    err_ax.set_ylim(1e-14, 1e-2)
    err_ax.axhline(1e-6, color=GREY, lw=0.8, ls="--")
    err_ax.text(n[-1], 1.5e-6, "1e-6 bar", ha="right", fontsize=8, color=GREY)
    err_ax.set_ylabel("|FPGA - Python|")
    err_ax.legend(frameon=False, ncol=4, fontsize=8, loc="lower left")
    err_ax.set_xlabel("command number")
    fig.subplots_adjust(left=0.09, right=0.98, top=0.95, bottom=0.07)
    fig.savefig(d / "compare_waveform.png", dpi=170)
    plt.close(fig)

    # inputs must be identical (same recorded data into both)
    in_diff = max(abs(f[c] - p[c]) for f, p in zip(fpga, py) for c, _, _ in INPUTS)
    print(f"{len(n)} commands compared (ModelSim waveform vs Python)")
    print(f"  inputs written to the FPGA vs inputs given to Python: max difference {in_diff:.1e}")
    for col, label, _ in OUTPUTS:
        print(f"  {label:26s} worst |FPGA - Python| = {worst[col]:.2e}")
    ok = max(worst.values()) <= a.tol
    print(("PASS" if ok else "FAIL") + f" (bar {a.tol:g})")
    print("wrote", d / "python_waveform.png", "and", d / "compare_waveform.png")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
