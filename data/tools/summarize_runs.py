#!/usr/bin/env python3
"""
******************************************************************************
 @file : summarize_runs.py
 @brief : One-line summary per phone run (and optional GPS-vs-flow speed bins).
******************************************************************************
 @attention

 Copyright (c) 2026 MRacing. All rights reserved.
 MRacing is a trademark of MRacing FSAE.

 This firmware is the property of MRacing FSAE. Unauthorized use, copying,
 or distribution is prohibited.

******************************************************************************

Usage:
  python3 data/tools/summarize_runs.py                    # table of every run in data/phone_runs
  python3 data/tools/summarize_runs.py --markdown         # same, as a markdown table
  python3 data/tools/summarize_runs.py RUN_ID [--bin 4]   # flow vs GPS speed over time for one run
"""

import csv
import math
import os
import statistics as st
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "phone_runs")


def rows(run, name):
    path = os.path.join(ROOT, run, name)
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def summary(run):
    flow, est, gnss = rows(run, "flow.csv"), rows(run, "est.csv"), rows(run, "gnss.csv")
    events = rows(run, "events.csv")
    label = next((e["value"] for e in events if e["event"] == "start"), "")
    if not est:
        return dict(run=run, label=label, dur=0, dist=float("nan"), vmax=0, gmax=0, h=float("nan"), q=0, fps=0)
    t0, t1 = float(est[0]["t"]), float(est[-1]["t"])
    dur = t1 - t0
    speeds = [math.hypot(float(r["v_x"]), float(r["v_y"])) for r in est]
    hs = [float(r["h"]) for r in flow] or [float(r["h"]) for r in est]
    qs = [float(r["quality"]) for r in flow]
    return dict(
        run=run, label=label, dur=dur,
        dist=float(est[-1]["distance"]),
        vmax=max(speeds),
        gmax=max((float(r["speed"]) for r in gnss), default=0.0),
        h=st.median(hs) if hs else float("nan"),
        q=st.median(qs) if qs else 0,
        fps=len(flow) / dur if dur > 0 else 0,
    )


def table(markdown):
    runs = sorted(d for d in os.listdir(ROOT) if os.path.isdir(os.path.join(ROOT, d)))
    head = ["run", "label", "dur s", "dist m", "max v m/s", "max GPS m/s", "h m", "flow q med", "flow Hz"]
    if markdown:
        print("| " + " | ".join(head) + " |")
        print("|" + "---|" * len(head))
    else:
        print("  ".join(f"{h:>12}" for h in head))
    for run in runs:
        s = summary(run)
        cells = [s["run"], s["label"] or "", f"{s['dur']:.0f}", f"{s['dist']:.3f}", f"{s['vmax']:.2f}",
                 f"{s['gmax']:.2f}", f"{s['h']:.3f}", f"{s['q']:.0f}", f"{s['fps']:.0f}"]
        print(("| " + " | ".join(cells) + " |") if markdown else "  ".join(f"{c:>12}" for c in cells))


def bins(run, width):
    flow, gnss, est = rows(run, "flow.csv"), rows(run, "gnss.csv"), rows(run, "est.csv")
    if not flow:
        sys.exit(f"{run}: no flow.csv")
    t0 = float(flow[0]["t"])
    fl = [(float(r["t"]) - t0, math.hypot(float(r["vx_cam"]), float(r["vy_cam"])), float(r["quality"]), float(r["h"]))
          for r in flow]
    gn = [(float(r["t"]) - t0, float(r["speed"])) for r in gnss]
    es = [(float(r["t"]) - t0, math.hypot(float(r["v_x"]), float(r["v_y"]))) for r in est]
    print(f"{'t s':>5} {'GPS':>6} {'flow':>6} {'est':>6} {'est/GPS':>8} {'q med':>6} {'q>10 %':>7} {'h':>6}")
    t = 0.0
    while t < fl[-1][0]:
        f = [x for x in fl if t <= x[0] < t + width]
        g = [s for tt, s in gn if t <= tt < t + width]
        e = [s for tt, s in es if t <= tt < t + width]
        good = [x for x in f if x[2] > 10]
        gm = st.mean(g) if g else float("nan")
        em = st.mean(e) if e else float("nan")
        fm = st.median([x[1] for x in good]) if good else float("nan")
        ratio = em / gm if g and gm > 0.5 else float("nan")
        print(f"{t:5.0f} {gm:6.2f} {fm:6.2f} {em:6.2f} {ratio:8.2f} {st.median([x[2] for x in f]) if f else 0:6.0f} "
              f"{100 * len(good) / max(1, len(f)):7.0f} {st.median([x[3] for x in f]) if f else 0:6.3f}")
        t += width


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and not args[0].startswith("--"):
        width = float(args[args.index("--bin") + 1]) if "--bin" in args else 4.0
        bins(args[0], width)
    else:
        table("--markdown" in args)
