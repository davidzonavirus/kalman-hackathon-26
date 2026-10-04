"""
kf_ref.py - double-precision Python port of the app's Kalman filter, the golden model.

Line-for-line port of GroundSpeedKit/Sources/KalmanCore/ReferenceKF4.swift (algorithm in its
header comment), plus ZuptDetector, DistanceIntegrator and Replay from the same package. Pure
standard library.

Everything in this package that is not the Swift app is checked against this file:

    kf_ref.ReferenceKF4   (float, this file)
        == Swift ReferenceKF4 / the phone's est.csv        (replay.py --compare)
    kf_isa.FixedKF4       (bit-exact fixed-point model of the FPGA microcode)
        ~= kf_ref.ReferenceKF4                              (check_model.py)
    the Verilog in ../rtl, simulated
        == kf_isa.FixedKF4, bit for bit                     (check_rtl.py)
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field, fields
from pathlib import Path

MAX_DT = 0.1


@dataclass
class FilterConfig:
    """FilterConfig in GroundSpeedFilter.swift, same names as meta.json ("q" and "r")."""
    q_accel: float = 0.1
    q_bias: float = 1e-5
    r_flow_base: float = 1.6e-3
    psr_ref: float = 20.0
    psr_min: float = 8.0
    r_zupt: float = 1e-4
    gate: float = 9.0
    p0_v: float = 0.25
    p0_b: float = 0.01
    gate_reset_count: float = 12.0

    @classmethod
    def from_meta(cls, meta: dict) -> "FilterConfig":
        """Config a recorded run was made with (meta.json "q" + "r"); missing keys = defaults."""
        known = {f.name for f in fields(cls)}
        merged = {**meta.get("q", {}), **meta.get("r", {})}
        return cls(**{k: float(v) for k, v in merged.items() if k in known})

    def flow_variance(self, quality: float) -> float:
        k = self.psr_ref / max(quality, self.psr_min)
        return self.r_flow_base * k * k


# Update outcomes: (kind, nis) with kind one of these.
ACCEPTED, GATED, SKIPPED = "accepted", "gated", "skipped"


@dataclass
class FilterState:
    t: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    bx: float = 0.0
    by: float = 0.0
    p_diag: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    initialized: bool = False

    @property
    def sigma_vx(self): return math.sqrt(max(self.p_diag[0], 0.0))
    @property
    def sigma_vy(self): return math.sqrt(max(self.p_diag[1], 0.0))
    @property
    def speed(self): return math.hypot(self.vx, self.vy)


class ReferenceKF4:
    """x = [v_x, v_y, b_x, b_y]; see ReferenceKF4.swift for the exact algorithm."""
    name = "ReferenceKF4"

    def __init__(self, config: FilterConfig | None = None):
        self.config = config or FilterConfig()
        self.state = FilterState()
        self.gate_armed = False
        self.flow_gated_run = 0
        self.x = [0.0] * 4
        self.P = [[0.0] * 4 for _ in range(4)]

    def reset(self, t: float):
        c = self.config
        self.x = [0.0] * 4
        self.P = [[0.0] * 4 for _ in range(4)]
        self.P[0][0] = self.P[1][1] = c.p0_v
        self.P[2][2] = self.P[3][3] = c.p0_b
        self.state.t = t
        self.state.initialized = True
        self.gate_armed = False
        self.flow_gated_run = 0
        self._publish()

    def seed(self, t: float, x, p_diag):
        """Start from a logged state (diagonal P) instead of reset(): replay of a run that
        began while the phone's filter was already running."""
        self.reset(t)
        self.x = list(x)
        for i in range(4):
            self.P[i][i] = p_diag[i]
        self._publish()

    def predict(self, t: float, ax: float, ay: float, r: float):
        if not self.state.initialized:
            return self.reset(t)
        dt = t - self.state.t
        if not (dt > 0 and math.isfinite(dt)):
            return
        dt = min(dt, MAX_DT)
        self.state.t = t
        P, c = self.P, self.config
        vx, vy, bx, by = self.x
        self.x[0] = vx + dt * (ax - bx + r * vy)
        self.x[1] = vy + dt * (ay - by - r * vx)
        a, d = r * dt, dt
        M = [row[:] for row in P]
        for j in range(4):
            M[0][j] = P[0][j] + a * P[1][j] - d * P[2][j]
            M[1][j] = -a * P[0][j] + P[1][j] - d * P[3][j]
        N = [row[:] for row in M]
        for i in range(4):
            N[i][0] = M[i][0] + a * M[i][1] - d * M[i][2]
            N[i][1] = -a * M[i][0] + M[i][1] - d * M[i][3]
        N[0][0] += c.q_accel * dt
        N[1][1] += c.q_accel * dt
        N[2][2] += c.q_bias * dt
        N[3][3] += c.q_bias * dt
        self.P = self._symmetrize(N)
        self._publish()

    def update_flow(self, t: float, vx: float, vy: float, quality: float):
        c, P = self.config, self.P
        if not self.state.initialized:
            return (SKIPPED, 0.0)
        if not all(map(math.isfinite, (vx, vy, quality))):
            return (SKIPPED, 0.0)
        if quality < c.psr_min:
            return (SKIPPED, 0.0)
        R = c.flow_variance(quality)
        nx = (vx - self.x[0]) ** 2 / (P[0][0] + R)
        ny = (vy - self.x[1]) ** 2 / (P[1][1] + R)
        nis = max(nx, ny)
        if self.gate_armed and nis > c.gate:
            self.flow_gated_run += 1
            if not (c.gate_reset_count > 0 and self.flow_gated_run >= c.gate_reset_count):
                return (GATED, nis)
            for i in range(2):                       # lockout recovery
                for j in range(4):
                    if j != i:
                        P[i][j] = 0.0
                        P[j][i] = 0.0
                P[i][i] = max(P[i][i], c.p0_v)
        self.flow_gated_run = 0
        self._scalar_update((1, 0, 0, 0), vx - self.x[0], R)
        self._scalar_update((0, 1, 0, 0), vy - self.x[1], R)
        self.gate_armed = True
        self._publish()
        return (ACCEPTED, nis)

    def update_gnss(self, t: float, speed: float, speed_accuracy: float):
        c = self.config
        if not self.state.initialized:
            return (SKIPPED, 0.0)
        if not (math.isfinite(speed) and math.isfinite(speed_accuracy) and speed_accuracy > 0):
            return (SKIPPED, 0.0)
        s = math.sqrt(self.x[0] ** 2 + self.x[1] ** 2)
        if not (speed > 1 and s > 1):
            return (SKIPPED, 0.0)
        h = (self.x[0] / s, self.x[1] / s, 0.0, 0.0)
        R = speed_accuracy ** 2
        nu = speed - s
        S = self._quad(h) + R
        nis = nu * nu / S
        if self.gate_armed and nis > c.gate:
            return (GATED, nis)
        self._scalar_update(h, nu, R)
        self.gate_armed = True
        self._publish()
        return (ACCEPTED, nis)

    def update_zero_velocity(self, t: float):
        if not self.state.initialized:
            return (SKIPPED, 0.0)
        R, P = self.config.r_zupt, self.P
        nx = self.x[0] ** 2 / (P[0][0] + R)
        ny = self.x[1] ** 2 / (P[1][1] + R)
        self._scalar_update((1, 0, 0, 0), -self.x[0], R)
        self._scalar_update((0, 1, 0, 0), -self.x[1], R)
        self.gate_armed = True
        self._publish()
        return (ACCEPTED, max(nx, ny))

    # -- internals ----------------------------------------------------------
    def _quad(self, h):
        return sum(h[i] * sum(self.P[i][j] * h[j] for j in range(4)) for i in range(4))

    def _scalar_update(self, h, nu, R):
        P = self.P
        ph = [sum(P[i][j] * h[j] for j in range(4)) for i in range(4)]
        S = sum(h[i] * ph[i] for i in range(4)) + R
        if not (S > 0 and math.isfinite(S)):
            return
        k = [p / S for p in ph]
        for i in range(4):
            self.x[i] += k[i] * nu
        A = [[(1.0 if i == j else 0.0) - k[i] * h[j] for j in range(4)] for i in range(4)]
        AP = [[sum(A[i][m] * P[m][j] for m in range(4)) for j in range(4)] for i in range(4)]
        Pn = [[sum(AP[i][m] * A[j][m] for m in range(4)) + R * k[i] * k[j]
               for j in range(4)] for i in range(4)]
        self.P = self._symmetrize(Pn)

    @staticmethod
    def _symmetrize(M):
        for i in range(4):
            for j in range(i + 1, 4):
                M[i][j] = M[j][i] = 0.5 * (M[i][j] + M[j][i])
        return M

    def _publish(self):
        s = self.state
        s.vx, s.vy, s.bx, s.by = self.x
        s.p_diag = [self.P[i][i] for i in range(4)]


# -- ZUPT detector and distance (separate from the filter in the app too) ------------------
class ZuptDetector:
    """ZuptDetector.swift: accel-magnitude variance over `window` s and latest flow speed."""

    def __init__(self, window=0.3, accel_var_threshold=0.0025, flow_speed_threshold=0.03,
                 flow_max_age=0.2, require_flow=False):
        self.window, self.var_thr = window, accel_var_threshold
        self.flow_thr, self.flow_max_age, self.require_flow = flow_speed_threshold, flow_max_age, require_flow
        self.reset()

    def reset(self):
        self.buf = []                       # (t, |a|)
        self.flow_t, self.flow_speed = -math.inf, math.inf
        self.is_stationary = False

    def add_flow(self, t, speed):
        self.flow_t, self.flow_speed = t, speed

    def add_imu(self, t, ax, ay, az):
        m = math.sqrt(ax * ax + ay * ay + az * az)
        self.buf = [(ts, mv) for ts, mv in self.buf if t - ts <= self.window][-511:]
        self.buf.append((t, m))
        n = len(self.buf)
        span_ok = (t - self.buf[0][0]) >= 0.8 * self.window and n >= 5
        mean = sum(v for _, v in self.buf) / n
        var = max(0.0, sum(v * v for _, v in self.buf) / n - mean * mean)
        accel_still = span_ok and var < self.var_thr
        flow_still = (self.flow_speed < self.flow_thr) if (t - self.flow_t) <= self.flow_max_age \
            else (not self.require_flow)
        self.is_stationary = accel_still and flow_still
        return self.is_stationary


class DistanceIntegrator:
    """DistanceIntegrator.swift (trapezoid); mode 'path_length' or 'forward'."""

    def __init__(self, deadband=0.0, mode="path_length", smoothing=0.0):
        self.deadband, self.mode, self.smoothing = deadband, mode, smoothing
        self.clear()

    def clear(self):
        self.distance, self.last_t, self.last_speed, self.lpx, self.lpy = 0.0, None, 0.0, 0.0, 0.0

    def reset(self):
        self.distance = 0.0

    def add(self, t, vx, vy):
        ux, uy = vx, vy
        if self.smoothing > 0:
            if self.last_t is not None:
                dt = min(max(t - self.last_t, 0.0), MAX_DT)
                a = dt / (self.smoothing + dt)
                self.lpx += a * (ux - self.lpx)
                self.lpy += a * (uy - self.lpy)
            ux, uy = self.lpx, self.lpy
        s = ux if self.mode == "forward" else math.hypot(ux, uy)
        if abs(s) < self.deadband or not math.isfinite(s):
            s = 0.0
        if self.last_t is not None:
            dt = t - self.last_t
            if dt <= 0:
                return self.distance
            self.distance += 0.5 * (s + self.last_speed) * min(dt, MAX_DT)
        self.last_t, self.last_speed = t, s
        return self.distance


# -- CSV logs and replay ---------------------------------------------------------------------
def read_csv(path) -> list[dict]:
    with open(path, newline="") as f:
        return [{k: (float(v) if k != "event" and k != "value" else v) for k, v in row.items()}
                for row in csv.DictReader(f)]


def load_run(run_dir) -> dict:
    """imu/flow/gnss/events/est rows and the config from meta.json of a data/phone_runs run."""
    d = Path(run_dir)
    run = {n: (read_csv(d / f"{n}.csv") if (d / f"{n}.csv").exists() else [])
           for n in ("imu", "flow", "gnss", "events", "est")}
    run["meta"] = json.loads((d / "meta.json").read_text()) if (d / "meta.json").exists() else {}
    return run


def parse_filter_state(events):
    """The phone logs its state at run start ("filter_state" event) so a replay can be seeded:
    returns (t, x[4], p_diag[4]) or None."""
    for e in events:
        if e["event"] == "filter_state":
            kv = dict(tok.split("=", 1) for tok in e["value"].replace(" p=", " p=").split() if "=" in tok)
            p = e["value"].split(" p=", 1)[1].split() if " p=" in e["value"] else None
            if p and len(p) == 4:
                return e["t"], [float(kv[k]) for k in ("vx", "vy", "bx", "by")], [float(v) for v in p]
    return None


def replay(filt, imu, flow, gnss, events=(), on_row=None, seed=None):
    """Replay.swift: merge by (t, kind) IMU < flow < GNSS < ZUPT; one row per IMU predict.

    `filt` is anything with predict / update_flow / update_gnss / update_zero_velocity and
    .state (ReferenceKF4 or the FPGA-backed filter). ZUPTs come from `events` rows with
    event == "zupt" (the app logs every ZUPT it applies). Returns the list of est rows
    (t, v_x, v_y, sigma_vx, sigma_vy, distance).
    """
    evs = [(s["t"], 0, i) for i, s in enumerate(imu)]
    evs += [(s["t"], 1, i) for i, s in enumerate(flow)]
    evs += [(s["t"], 2, i) for i, s in enumerate(gnss)]
    evs += [(e["t"], 3, i) for i, e in enumerate(events) if e["event"] == "zupt"]
    evs.sort()
    if seed is not None:        # (t, x, p_diag) from parse_filter_state: start mid-run like the phone
        filt.seed(*seed)
    dist = DistanceIntegrator(deadband=0.05, smoothing=0.5)
    rows, pend_t, pend_zupt = [], -math.inf, False
    for t, kind, i in evs:
        if t > pend_t:
            if pend_zupt:
                filt.update_zero_velocity(pend_t)
                pend_zupt = False
            pend_t = t
        if kind == 0:
            s = imu[i]
            if not (seed is not None and s["t"] == seed[0]):
                filt.predict(s["t"], s["ax"], s["ay"], s["gz"])
            st = filt.state
            dist.add(st.t, st.vx, st.vy)
            row = (st.t, st.vx, st.vy, st.sigma_vx, st.sigma_vy, dist.distance)
            rows.append(row)
            if on_row:
                on_row(row)
        elif kind == 1:
            s = flow[i]
            filt.update_flow(s["t"], s["vx_cam"], s["vy_cam"], s["quality"])
        elif kind == 2:
            s = gnss[i]
            filt.update_gnss(s["t"], s["speed"], s["speed_acc"])
        else:
            pend_zupt = True
    if pend_zupt:
        filt.update_zero_velocity(pend_t)
    return rows
