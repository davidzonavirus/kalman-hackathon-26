"""Python port of GroundSpeedKit's ReferenceKF4 + DistanceIntegrator and the replay rules.

Ported line-by-line from:
  GroundSpeedKit/Sources/KalmanCore/ReferenceKF4.swift   (algorithm in its header comment)
  GroundSpeedKit/Sources/KalmanCore/DistanceIntegrator.swift
  GroundSpeedKit/Sources/PhoneRuntime/SensorFusionEngine.swift (event order, deadband 0.05, tau 0.5)
Swift is not available on this machine, so `kfreplay` cannot run; 02_replay_validation.py
checks this port against the phone's own est.csv instead.
"""
import math

import numpy as np

MAX_DT = 0.1
DEFAULT_CFG = dict(q_accel=0.1, q_bias=1e-5, r_flow_base=1.6e-3, psr_ref=20.0, psr_min=8.0,
                   r_zupt=1e-4, gate=9.0, p0_v=0.25, p0_b=0.01, gate_reset_count=12)


class ReferenceKF4:
    def __init__(self, cfg=None):
        self.c = dict(DEFAULT_CFG, **(cfg or {}))
        self.init = False
        self.x = np.zeros(4)
        self.P = np.zeros((4, 4))
        self.t = 0.0
        self.armed = False
        self.gated_run = 0
        self.n_flow = dict(accepted=0, gated=0, skipped=0, lockout=0)

    def reset(self, t):
        c = self.c
        self.x[:] = 0
        self.P = np.diag([c["p0_v"], c["p0_v"], c["p0_b"], c["p0_b"]]).astype(float)
        self.t = t
        self.init = True
        self.armed = False
        self.gated_run = 0

    def seed(self, t, vx, vy, bx, by, pdiag, armed=True):
        """Start mid-stream from a logged `filter_state` event (off-diagonal P unknown -> 0)."""
        self.reset(t)
        self.x[:] = [vx, vy, bx, by]
        self.P = np.diag(pdiag).astype(float)
        self.armed = armed

    def predict(self, t, ax, ay, r):
        if not self.init:
            self.reset(t)
            return
        dt = t - self.t
        if not (dt > 0 and math.isfinite(dt)):
            return
        dt = min(dt, MAX_DT)
        self.t = t
        vx, vy, bx, by = self.x
        self.x[0] = vx + dt * (ax - bx + r * vy)
        self.x[1] = vy + dt * (ay - by - r * vx)
        a, d = r * dt, dt
        F = np.array([[1, a, -d, 0], [-a, 1, 0, -d], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        P = F @ self.P @ F.T
        P[0, 0] += self.c["q_accel"] * dt
        P[1, 1] += self.c["q_accel"] * dt
        P[2, 2] += self.c["q_bias"] * dt
        P[3, 3] += self.c["q_bias"] * dt
        self.P = 0.5 * (P + P.T)

    def _scalar(self, h, nu, R):
        ph = self.P @ h
        S = h @ ph + R
        if not (S > 0 and math.isfinite(S)):
            return
        k = ph / S
        self.x += k * nu
        A = np.eye(4) - np.outer(k, h)
        P = A @ self.P @ A.T + R * np.outer(k, k)
        self.P = 0.5 * (P + P.T)

    def flow_R(self, q):
        k = self.c["psr_ref"] / max(q, self.c["psr_min"])
        return self.c["r_flow_base"] * k * k

    def update_flow(self, vx, vy, q):
        if not self.init or not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(q)):
            self.n_flow["skipped"] += 1
            return "skipped"
        if q < self.c["psr_min"]:
            self.n_flow["skipped"] += 1
            return "skipped"
        R = self.flow_R(q)
        nis = max((vx - self.x[0]) ** 2 / (self.P[0, 0] + R), (vy - self.x[1]) ** 2 / (self.P[1, 1] + R))
        if self.armed and nis > self.c["gate"]:
            self.gated_run += 1
            if not (self.c["gate_reset_count"] > 0 and self.gated_run >= self.c["gate_reset_count"]):
                self.n_flow["gated"] += 1
                return "gated"
            for i in range(2):
                for j in range(4):
                    if j != i:
                        self.P[i, j] = 0
                        self.P[j, i] = 0
                self.P[i, i] = max(self.P[i, i], self.c["p0_v"])
            self.n_flow["lockout"] += 1
        self.gated_run = 0
        self._scalar(np.array([1.0, 0, 0, 0]), vx - self.x[0], R)
        self._scalar(np.array([0, 1.0, 0, 0]), vy - self.x[1], R)
        self.armed = True
        self.n_flow["accepted"] += 1
        return "accepted"

    def update_gnss(self, speed, acc):
        if not self.init or not (math.isfinite(speed) and math.isfinite(acc) and acc > 0):
            return "skipped"
        s = math.hypot(self.x[0], self.x[1])
        if not (speed > 1 and s > 1):
            return "skipped"
        h = np.array([self.x[0] / s, self.x[1] / s, 0, 0])
        R = acc * acc
        nu = speed - s
        S = h @ self.P @ h + R
        if self.armed and nu * nu / S > self.c["gate"]:
            return "gated"
        self._scalar(h, nu, R)
        self.armed = True
        return "accepted"

    def update_zupt(self):
        if not self.init:
            return "skipped"
        R = self.c["r_zupt"]
        self._scalar(np.array([1.0, 0, 0, 0]), -self.x[0], R)
        self._scalar(np.array([0, 1.0, 0, 0]), -self.x[1], R)
        self.armed = True
        return "accepted"


class DistanceIntegrator:
    """pathLength mode with deadband + 1st-order low-pass + lead compensation (final build)."""

    def __init__(self, deadband=0.05, smoothing=0.5, mode="path"):
        self.deadband, self.tau, self.mode = deadband, smoothing, mode
        self.clear()

    def clear(self):
        self.distance = 0.0
        self.last_t = None
        self.last_s = 0.0
        self.lpx = self.lpy = 0.0

    def add(self, t, vx, vy):
        ux, uy = vx, vy
        if self.tau > 0:
            if self.last_t is not None:
                dt = min(max(t - self.last_t, 0), MAX_DT)
                a = dt / (self.tau + dt)
                self.lpx += a * (ux - self.lpx)
                self.lpy += a * (uy - self.lpy)
            ux, uy = self.lpx, self.lpy
        s = ux if self.mode == "forward" else math.hypot(ux, uy)
        if abs(s) < self.deadband or not math.isfinite(s):
            s = 0.0
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 0:
                self.distance += 0.5 * (s + self.last_s) * min(dt, MAX_DT)
            else:
                return self.distance
        self.last_t = t
        self.last_s = s
        return self.distance

    @property
    def lead_compensated(self):
        if self.tau <= 0 or self.mode != "path":
            return self.distance
        return self.distance + self.tau * math.hypot(self.lpx, self.lpy)


def merged_events(run, flow_scale=1.0, use_gnss=True, use_zupt=True, flow_override=None):
    """Yield (t, kind, row) in replay order: IMU(0) < flow(1) < GNSS(2) < ZUPT(3) at equal t."""
    imu, flow, gnss, ev = run["imu"], run["flow"], run["gnss"], run["events"]
    if flow_override is not None:
        flow = flow_override
    items = []
    if not imu.empty:
        a = imu[["t", "ax", "ay", "gz"]].to_numpy()
        items.append((a[:, 0], np.zeros(len(a)), a[:, 1:]))
    if not flow.empty:
        f = flow[["t", "vx_cam", "vy_cam", "quality"]].to_numpy().copy()
        f[:, 1:3] *= flow_scale
        items.append((f[:, 0], np.ones(len(f)), f[:, 1:]))
    if use_gnss and not gnss.empty:
        g = gnss[["t", "speed", "speed_acc"]].to_numpy()
        items.append((g[:, 0], np.full(len(g), 2), np.c_[g[:, 1:], np.zeros(len(g))]))
    if use_zupt and not ev.empty:
        z = ev.loc[ev["event"] == "zupt", "t"].to_numpy(float)
        items.append((z, np.full(len(z), 3), np.zeros((len(z), 3))))
    t = np.concatenate([i[0] for i in items])
    k = np.concatenate([i[1] for i in items])
    v = np.concatenate([i[2] for i in items])
    order = np.lexsort((k, t))
    return t[order], k[order].astype(int), v[order]


def parse_filter_state(ev):
    row = ev.loc[ev["event"] == "filter_state"]
    if row.empty:
        return None
    t = float(row["t"].iloc[0])
    kv = {}
    p = None
    s = str(row["value"].iloc[0])
    head, _, ptxt = s.partition(" p=")
    for tok in head.split():
        k, _, v = tok.partition("=")
        kv[k] = float(v)
    p = [float(x) for x in ptxt.split()]
    return t, kv, p


def replay(run, cfg=None, flow_scale=1.0, use_gnss=True, use_zupt=True, seed_from_log=True,
           flow_override=None, deadband=0.05, smoothing=0.5, mode="path"):
    """Replays a run; returns dict of arrays at every IMU step (like est.csv)."""
    kf = ReferenceKF4(cfg)
    dist = DistanceIntegrator(deadband, smoothing, mode)
    t_arr, k_arr, v_arr = merged_events(run, flow_scale, use_gnss, use_zupt, flow_override)
    fs = parse_filter_state(run["events"]) if seed_from_log else None
    out_t, out_vx, out_vy, out_sx, out_d = [], [], [], [], []
    pend_zupt = None
    first_imu = True
    for t, k, v in zip(t_arr, k_arr, v_arr):
        if k == 3:
            kf.update_zupt()
            continue
        if k == 0:
            if first_imu:
                first_imu = False
                if fs is not None:
                    kf.seed(t, fs[1]["vx"], fs[1]["vy"], fs[1]["bx"], fs[1]["by"], fs[2])
                else:
                    kf.reset(t)
            else:
                kf.predict(t, v[0], v[1], v[2])
            dist.add(t, kf.x[0], kf.x[1])
            out_t.append(t); out_vx.append(kf.x[0]); out_vy.append(kf.x[1])
            out_sx.append(math.sqrt(max(kf.P[0, 0], 0))); out_d.append(dist.lead_compensated)
        elif k == 1:
            kf.update_flow(v[0], v[1], v[2])
        elif k == 2:
            kf.update_gnss(v[0], v[1])
    return dict(t=np.array(out_t), v_x=np.array(out_vx), v_y=np.array(out_vy), sigma_vx=np.array(out_sx),
                distance=np.array(out_d), flow_counts=dict(kf.n_flow), final_distance=dist.lead_compensated,
                raw_distance=dist.distance)
