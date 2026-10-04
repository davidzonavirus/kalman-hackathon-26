"""Shared loaders and constants for the forensic analysis.

Raw data under data/ is only ever read, never written. All outputs go to analysis/out/.
"""
import json
import os

import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if os.name == "nt" and not ROOT.startswith("\\\\?\\"):
    # Windows: deep checkouts exceed MAX_PATH (260); the extended-length prefix lifts it.
    ROOT = "\\\\?\\" + ROOT
RUNS = os.path.join(ROOT, "data", "phone_runs")
DASH = os.path.join(ROOT, "data", "dashboard_logs")
LIDAR = os.path.join(ROOT, "data", "lidar")
OUT = os.path.join(ROOT, "analysis", "out")
FIG = os.path.join(OUT, "figures")
os.makedirs(FIG, exist_ok=True)

SEED = 20261004

# Ground truth course lengths (data/README.md, docs/HACKATHON_WRITEUP.md 1, 3.7, 3.8).
FT = 0.3048
COURSE_16FT = 16 * FT   # 4.8768 m
COURSE_20FT = 20 * FT   # 6.096 m

# Taped-course runs: run_id -> (course length m, group, note). Source: data/README.md tables.
TAPE_RUNS = {
    # 16 ft, 18.6 cm, BEFORE focal correction (focal_px 860.66)
    "2026-10-03_22-22-18": (COURSE_16FT, "16ft h=0.186 pre-cal", "dash run 5"),
    "2026-10-03_22-23-45": (COURSE_16FT, "16ft h=0.186 pre-cal", "dash run 6"),
    "2026-10-03_22-25-09": (COURSE_16FT, "16ft h=0.186 pre-cal", "dash run 7, ~1 m/s"),
    # 16 ft, 23.5 cm, first focal correction
    "2026-10-03_22-37-35": (COURSE_16FT, "16ft h=0.235 cal-1", "dash run 9"),
    "2026-10-03_22-38-37": (COURSE_16FT, "16ft h=0.235 cal-1", "dash run 10"),
    "2026-10-03_22-39-51": (COURSE_16FT, "16ft h=0.235 cal-1", "dash run 11, other lighting"),
    # 20 ft, 79 cm extreme mount
    "2026-10-03_22-59-53": (COURSE_20FT, "20ft h=0.79", "dash run 15, faster push"),
    "2026-10-03_23-01-53": (COURSE_20FT, "20ft h=0.79", "dash run 16"),
    "2026-10-03_23-03-20": (COURSE_20FT, "20ft h=0.79", "dash run 17"),
    "2026-10-03_23-23-07": (COURSE_20FT, "20ft h=0.79 static-sub", "dash run 18, static-pattern subtraction"),
    # 20 ft, variable ride height
    "2026-10-03_23-39-53": (COURSE_20FT, "20ft ride-height fixed-h", "ride run 1"),
    "2026-10-03_23-40-27": (COURSE_20FT, "20ft ride-height fixed-h", "ride run 2"),
    "2026-10-03_23-41-51": (COURSE_20FT, "20ft ride-height fixed-h", "ride run 3"),
    "2026-10-04_00-00-16": (COURSE_20FT, "20ft ride-height tracked", "ride run 4, expansion tracking"),
    "2026-10-04_00-01-22": (COURSE_20FT, "20ft ride-height tracked", "ride run 5, tracked h swung"),
}

CAR_RUNS = {
    "2026-10-04_01-19-31": "Car test 1 (no prediction, height tracking on)",
    "2026-10-04_01-45-16": "Car test 2 (prediction, height tracking on)",
    "2026-10-04_01-59-42": "Car test 3 (final: prediction, height fixed)",
}

# Final calibration constants (writeup 4; CameraFlowSource.swift / DepthSource.swift).
FOCAL_CORRECTION = 1.027
LIDAR_H_OFFSET = -0.0079


def run_ids():
    return sorted(d for d in os.listdir(RUNS) if os.path.isdir(os.path.join(RUNS, d)))


def load_csv(run, name):
    p = os.path.join(RUNS, run, name)
    if not os.path.exists(p) or os.path.getsize(p) == 0:
        return pd.DataFrame()
    try:
        return pd.read_csv(p)
    except pd.errors.EmptyDataError:
        return pd.DataFrame()


def load_meta(run):
    p = os.path.join(RUNS, run, "meta.json")
    if not os.path.exists(p):
        return {}
    with open(p) as f:
        return json.load(f)


def load_run(run):
    return {k: load_csv(run, k + ".csv") for k in ("est", "flow", "imu", "gnss", "depth", "events")} | {
        "meta": load_meta(run)}


def save_json(obj, name):
    def conv(o):
        if isinstance(o, (np.floating,)):
            return None if not np.isfinite(o) else float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, float) and not np.isfinite(o):
            return None
        raise TypeError(type(o))
    with open(os.path.join(OUT, name), "w") as f:
        json.dump(obj, f, indent=2, default=conv)


def speed(df, x="v_x", y="v_y"):
    return np.hypot(df[x].to_numpy(), df[y].to_numpy())
