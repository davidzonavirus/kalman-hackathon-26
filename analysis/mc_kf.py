"""ReferenceKF4 + DistanceIntegrator vectorised over N Monte Carlo trials (last axis = trial).

Same algorithm as kf.py (and ReferenceKF4.swift); every trial shares the time grid, PSR
sequence and ZUPT times of a real recorded run, but has its own measurements, state and
covariance (so gating / lockout behave per trial). `selftest()` proves equality with the
scalar port on a real run.
"""
import numpy as np

from kf import DEFAULT_CFG, MAX_DT


class VecKF:
    def __init__(self, N, cfg=None):
        self.c = dict(DEFAULT_CFG, **(cfg or {}))
        self.N = N
        self.x = np.zeros((4, N))
        self.P = np.zeros((4, 4, N))
        self.armed = np.zeros(N, bool)
        self.gated_run = np.zeros(N, int)
        self.t = None
        self.n_gated = np.zeros(N, int)

    def seed(self, t, x0, pdiag, armed=True):
        self.t = t
        self.x[:] = np.asarray(x0, float)[:, None]
        self.P[:] = 0
        for i in range(4):
            self.P[i, i] = pdiag[i]
        self.armed[:] = armed

    def predict(self, t, ax, ay, r):
        dt = t - self.t
        if not dt > 0:
            return
        dt = min(dt, MAX_DT)
        self.t = t
        vx, vy, bx, by = self.x
        nvx = vx + dt * (ax - bx + r * vy)
        nvy = vy + dt * (ay - by - r * vx)
        self.x[0], self.x[1] = nvx, nvy
        a, d = r * dt, dt
        F = np.array([[1, a, -d, 0], [-a, 1, 0, -d], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        P = np.einsum("ij,jkn,lk->iln", F, self.P, F, optimize=True)
        q = self.c
        P[0, 0] += q["q_accel"] * dt; P[1, 1] += q["q_accel"] * dt
        P[2, 2] += q["q_bias"] * dt; P[3, 3] += q["q_bias"] * dt
        self.P = 0.5 * (P + P.transpose(1, 0, 2))

    def _unit_update(self, i, nu, R, mask=None):
        """Scalar update with H = e_i, Joseph form, applied where mask is True."""
        P = self.P
        S = P[i, i] + R
        k = P[:, i] / S                                  # (4,N)
        AP = P - k[:, None, :] * P[i][None, :, :]        # (I - k e_i^T) P
        Pn = AP - AP[:, i, :][:, None, :] * k[None, :, :] + R * k[:, None, :] * k[None, :, :]
        Pn = 0.5 * (Pn + Pn.transpose(1, 0, 2))
        dx = k * nu
        if mask is None:
            self.x += dx; self.P = Pn
        else:
            self.x += np.where(mask, dx, 0.0)
            self.P = np.where(mask[None, None, :], Pn, P)

    def update_flow(self, vx, vy, q):
        c = self.c
        if not (q >= c["psr_min"]):
            return
        kk = c["psr_ref"] / max(q, c["psr_min"])
        R = c["r_flow_base"] * kk * kk
        nis = np.maximum((vx - self.x[0]) ** 2 / (self.P[0, 0] + R), (vy - self.x[1]) ** 2 / (self.P[1, 1] + R))
        gated = self.armed & (nis > c["gate"])
        self.gated_run = np.where(gated, self.gated_run + 1, 0)
        lock = gated & (c["gate_reset_count"] > 0) & (self.gated_run >= c["gate_reset_count"])
        if lock.any():
            for i in range(2):
                for j in range(4):
                    if j != i:
                        self.P[i, j] = np.where(lock, 0.0, self.P[i, j])
                        self.P[j, i] = np.where(lock, 0.0, self.P[j, i])
                self.P[i, i] = np.where(lock, np.maximum(self.P[i, i], c["p0_v"]), self.P[i, i])
            self.gated_run = np.where(lock, 0, self.gated_run)
        apply = ~gated | lock
        self.n_gated += (~apply).astype(int)
        self._unit_update(0, vx - self.x[0], R, apply)
        self._unit_update(1, vy - self.x[1], R, apply)
        self.armed |= apply

    def update_gnss(self, speed, acc):
        if not (acc > 0):
            return
        s = np.hypot(self.x[0], self.x[1])
        ok = (speed > 1) & (s > 1)
        if not ok.any():
            return
        ss = np.where(s > 0, s, 1.0)
        h = np.zeros((4, self.N)); h[0] = self.x[0] / ss; h[1] = self.x[1] / ss
        R = acc * acc
        nu = speed - s
        ph = np.einsum("ijn,jn->in", self.P, h)
        S = (h * ph).sum(0) + R
        nis = nu * nu / S
        apply = ok & ~(self.armed & (nis > self.c["gate"]))
        k = ph / S
        AP = self.P - k[:, None, :] * ph[None, :, :]
        APh = np.einsum("ijn,jn->in", AP, h)
        Pn = AP - APh[:, None, :] * k[None, :, :] + R * k[:, None, :] * k[None, :, :]
        Pn = 0.5 * (Pn + Pn.transpose(1, 0, 2))
        self.x += np.where(apply, k * nu, 0.0)
        self.P = np.where(apply[None, None, :], Pn, self.P)
        self.armed |= apply

    def update_zupt(self):
        R = self.c["r_zupt"]
        self._unit_update(0, -self.x[0], R)
        self._unit_update(1, -self.x[1], R)
        self.armed[:] = True


class VecDistance:
    def __init__(self, N, deadband=0.05, tau=0.5):
        self.db, self.tau = deadband, tau
        self.d = np.zeros(N); self.lpx = np.zeros(N); self.lpy = np.zeros(N)
        self.last_t = None; self.last_s = np.zeros(N)

    def add(self, t, vx, vy):
        if self.last_t is not None:
            dt = min(max(t - self.last_t, 0), MAX_DT)
            a = dt / (self.tau + dt)
            self.lpx += a * (vx - self.lpx); self.lpy += a * (vy - self.lpy)
        s = np.hypot(self.lpx, self.lpy)
        s = np.where(s < self.db, 0.0, s)
        if self.last_t is not None:
            dt = t - self.last_t
            if dt <= 0:
                return
            self.d += 0.5 * (s + self.last_s) * min(dt, MAX_DT)
        self.last_t = t; self.last_s = s

    @property
    def lead(self):
        return self.d + self.tau * np.hypot(self.lpx, self.lpy)


def run_vectorised(events, N, flow_fn, imu_fn, gnss_fn=None, x0=None, pdiag=None, record_fn=None, cfg=None):
    """Drive VecKF over a shared event list.

    events: (t, kind, idx, q) arrays; kind 0=IMU,1=flow,2=GNSS,3=ZUPT; idx = row in that stream.
    flow_fn(idx) -> (vx[N], vy[N]); imu_fn(idx) -> (ax[N], ay[N], r); gnss_fn(idx) -> (speed[N], acc)
    record_fn(t, kf, dist) called after each IMU step (est.csv cadence).
    """
    t_arr, k_arr, i_arr, q_arr = events
    kf = VecKF(N, cfg)
    dist = VecDistance(N)
    first = True
    for t, k, i, q in zip(t_arr, k_arr, i_arr, q_arr):
        if k == 0:
            if first:
                kf.seed(t, x0 if x0 is not None else np.zeros(4), pdiag if pdiag is not None else
                        [kf.c["p0_v"], kf.c["p0_v"], kf.c["p0_b"], kf.c["p0_b"]])
                first = False
            else:
                ax, ay, r = imu_fn(i)
                kf.predict(t, ax, ay, r)
            dist.add(t, kf.x[0], kf.x[1])
            if record_fn is not None:
                record_fn(t, kf, dist)
        elif k == 1:
            vx, vy = flow_fn(i)
            kf.update_flow(vx, vy, q)
        elif k == 2 and gnss_fn is not None:
            sp, acc = gnss_fn(i)
            kf.update_gnss(sp, acc)
        elif k == 3:
            kf.update_zupt()
    return kf, dist


def build_events(t_imu, t_flow, q_flow, t_gnss=None, t_zupt=None):
    parts = [(t_imu, np.zeros(len(t_imu)), np.arange(len(t_imu)), np.zeros(len(t_imu))),
             (t_flow, np.ones(len(t_flow)), np.arange(len(t_flow)), q_flow)]
    if t_gnss is not None and len(t_gnss):
        parts.append((t_gnss, np.full(len(t_gnss), 2), np.arange(len(t_gnss)), np.zeros(len(t_gnss))))
    if t_zupt is not None and len(t_zupt):
        parts.append((t_zupt, np.full(len(t_zupt), 3), np.arange(len(t_zupt)), np.zeros(len(t_zupt))))
    t = np.concatenate([p[0] for p in parts]); k = np.concatenate([p[1] for p in parts])
    i = np.concatenate([p[2] for p in parts]); q = np.concatenate([p[3] for p in parts])
    o = np.lexsort((k, t))
    return t[o], k[o].astype(int), i[o].astype(int), q[o]


def selftest(run_id="2026-10-03_22-37-35"):
    """Feed the REAL measurements of one run as 3 identical trials: must equal the scalar replay."""
    from common import load_run
    from kf import parse_filter_state, replay
    r = load_run(run_id)
    imu, flow, gnss, ev = r["imu"], r["flow"], r["gnss"], r["events"]
    zt = ev.loc[ev.event == "zupt", "t"].to_numpy(float)
    events = build_events(imu.t.to_numpy(), flow.t.to_numpy(), flow.quality.to_numpy(),
                          gnss.t.to_numpy() if len(gnss) else None, zt)
    fs = parse_filter_state(ev)
    N = 3
    A = imu[["ax", "ay", "gz"]].to_numpy(); Fv = flow[["vx_cam", "vy_cam"]].to_numpy()
    G = gnss[["speed", "speed_acc"]].to_numpy() if len(gnss) else None
    rec = []
    kf, dist = run_vectorised(events, N, lambda i: (np.full(N, Fv[i, 0]), np.full(N, Fv[i, 1])),
                              lambda i: (np.full(N, A[i, 0]), np.full(N, A[i, 1]), A[i, 2]),
                              (lambda i: (np.full(N, G[i, 0]), G[i, 1])) if G is not None else None,
                              x0=[fs[1]["vx"], fs[1]["vy"], fs[1]["bx"], fs[1]["by"]], pdiag=fs[2],
                              record_fn=lambda t, k, d: rec.append(k.x[0, 0]))
    rep = replay(r)
    dv = np.abs(np.array(rec) - rep["v_x"]).max()
    dd = abs(dist.lead[0] - rep["final_distance"])
    return dict(run=run_id, max_abs_dvx=float(dv), abs_ddist=float(dd))


if __name__ == "__main__":
    print(selftest())
    print(selftest("2026-10-04_01-59-42"))
