"""Replay an uploaded recording through the Kalman filter on the simulated FPGA.

The dashboard's "Replay a recording" box uploads files; this module turns them into frames and
pushes them into the dashboard as if a phone were sending them. Two kinds of upload:

* a **phone run**: the files of one ``data/phone_runs/<run>/`` folder (``imu.csv`` and ``flow.csv``
  needed; ``gnss.csv``, ``events.csv`` (for the ZUPTs) and ``meta.json`` (filter settings) used if
  present). The raw samples go to the FPGA at their recorded rate (100 Hz IMU, ~240 Hz flow): every
  sample is one command, so the FPGA gives the same answer as the Python filter to ~1e-9 m/s.
  Display frames are emitted at ~50 Hz.
* a **dashboard log**: one ``telemetry.csv`` from ``data/dashboard_logs/<session>/``. These are
  50 Hz frames, already decimated by the phone link, so the estimate differs slightly from the
  phone's (see fpga/README.md).

Playback is paced to the recording's clock (``speed`` 1 = real time) but never faster than the
simulated FPGA can compute; the simulation, not the chip, is the slow part. Standard library only.
"""
from __future__ import annotations

import csv
import io
import json
import math
import threading
import time

from . import protocol as P
from .fpga_filter import FpgaFilter, kf_isa, kf_ref, make_backend

EMIT_PERIOD = 0.0195          # one display frame per ~20 ms of recording time (50 Hz)


def parse_csv(text: str) -> list[dict]:
    rows = []
    for r in csv.DictReader(io.StringIO(text)):
        rows.append({k: (v if k in ("event", "value") else (float(v) if v not in ("", None) else math.nan))
                     for k, v in r.items()})
    return rows


def classify(files: dict[str, str]) -> tuple[str, dict]:
    """('run', {imu, flow, gnss, events, meta}) or ('log', {rows}). Raises ValueError."""
    by = {}
    for name, text in files.items():
        base = name.replace("\\", "/").rsplit("/", 1)[-1].lower()
        for key in ("imu", "flow", "gnss", "events", "meta", "telemetry"):
            if base.startswith(key):
                by[key] = text
    if "imu" in by and "flow" in by:
        return "run", {
            "imu": parse_csv(by["imu"]), "flow": parse_csv(by["flow"]),
            "gnss": parse_csv(by["gnss"]) if "gnss" in by else [],
            "events": parse_csv(by["events"]) if "events" in by else [],
            "meta": json.loads(by["meta"]) if "meta" in by else {}}
    if "telemetry" in by:
        rows = parse_csv(by["telemetry"])
        need = {"t", "v_x", "ax", "ay", "gz", "flow_vx", "flow_quality", "status"}
        if not rows or not need <= set(rows[0]):
            raise ValueError("telemetry.csv needs the v2 columns (ax, ay, gz, flow_vx, ...)")
        return "log", {"rows": rows}
    names = ", ".join(sorted(files)) or "nothing"
    raise ValueError(f"need imu.csv + flow.csv (a phone run) or telemetry.csv (a dashboard log); got {names}")


class ReplayJob(threading.Thread):
    def __init__(self, deliver, files: dict[str, str], name: str = "recording", speed: float = 1.0,
                 backend: str = "auto", log=print):
        super().__init__(daemon=True, name="replay")
        self.kind, self.data = classify(files)          # raises ValueError for a bad upload
        self.deliver, self.name, self.speed, self.log = deliver, name, max(speed, 0.01), log
        self.backend = make_backend(backend, log)
        self.running = True
        self.done = False
        self.error = ""
        self.progress = 0.0
        self.frames = 0
        self.t_start = time.time()
        self.rec_time = 0.0
        self.counts = {"flow_accepted": 0, "flow_gated": 0, "flow_skipped": 0, "zupt": 0, "gnss": 0}

    # -- control / status --------------------------------------------------------------
    def stop(self):
        self.running = False

    def status(self) -> dict:
        clocks = getattr(self.backend, "clocks", None)
        return {"name": self.name, "kind": self.kind, "backend": self.backend.name,
                "progress": round(self.progress, 3), "frames": self.frames, "done": self.done,
                "running": self.running and not self.done, "error": self.error,
                "recording_s": round(self.rec_time, 1), "wall_s": round(time.time() - self.t_start, 1),
                "fpga_clocks": clocks if clocks is not None else None, **self.counts}

    def run(self):
        try:
            if self.kind == "run":
                self._run_phone()
            else:
                self._run_log()
        except Exception as e:                          # noqa: BLE001 - report in the UI
            self.error = repr(e)
            self.log("replay failed:", e)
        finally:
            self.done = True
            try:
                self.backend.close()
            except Exception:                           # noqa: BLE001
                pass

    # -- phone run: raw samples, every one a command --------------------------------------
    def _run_phone(self):
        d = self.data
        imu, flow, gnss, events = d["imu"], d["flow"], d["gnss"], d["events"]
        cfg = kf_ref.FilterConfig.from_meta(d["meta"])
        filt = kf_isa.FixedKF4(cfg, backend=self.backend)
        seed = kf_ref.parse_filter_state(events)
        evs = [(s["t"], 0, i) for i, s in enumerate(imu)]
        evs += [(s["t"], 1, i) for i, s in enumerate(flow)]
        evs += [(s["t"], 2, i) for i, s in enumerate(gnss)]
        evs += [(e["t"], 3, i) for i, e in enumerate(events) if e.get("event") == "zupt"]
        evs.sort()
        if not evs or not imu:
            raise ValueError("no IMU samples")
        if seed is not None:
            filt.seed(*seed)
        dist = kf_ref.DistanceIntegrator(deadband=0.05, smoothing=0.5)
        net = kf_ref.DistanceIntegrator(mode="forward")
        t0, wall0 = evs[0][0], time.time()
        st = {"fq": 0.0, "h": 0.0, "fvx": 0.0, "fvy": 0.0, "acc_t": -1e9, "gate": False, "zupt_t": -1e9,
              "gnss_t": -1e9, "ax": 0.0, "ay": 0.0, "gz": 0.0}
        pend_t, pend_zupt, last_emit, seq = -math.inf, False, -1e9, 0

        def frame(t, rec=True):
            s = filt.state
            status = P.IMU_OK | P.FILTER_INIT | (P.RECORDING if rec else 0)
            if not st["gate"] and t - st["acc_t"] < 0.1:
                status |= P.FLOW_OK
            if st["gate"]:
                status |= P.FLOW_GATED
            if t - st["gnss_t"] < 2.0:
                status |= P.GNSS_OK
            if t - st["zupt_t"] < 0.1:
                status |= P.ZUPT
            return dict(seq=seq, t=t, v_x=s.vx, v_y=s.vy, sigma_vx=s.sigma_vx, sigma_vy=s.sigma_vy,
                        distance=dist.distance, flow_quality=st["fq"], h=st["h"], status=status,
                        battery=255, torch=0, ax=st["ax"], ay=st["ay"], gz=st["gz"], flow_vx=st["fvx"],
                        flow_vy=st["fvy"], net_forward=net.distance, fmt="replay", version=2)

        def flush_zupt():
            nonlocal pend_zupt
            if pend_zupt:
                filt.update_zero_velocity(pend_t)
                st["zupt_t"] = pend_t
                self.counts["zupt"] += 1
                pend_zupt = False

        n = len(evs)
        for k, (t, kind, i) in enumerate(evs):
            if not self.running:
                break
            if t > pend_t:
                flush_zupt()
                pend_t = t
            if kind == 0:
                s = imu[i]
                filt.predict(s["t"], s["ax"], s["ay"], s["gz"])
                sx = filt.state
                dist.add(sx.t, sx.vx, sx.vy)
                net.add(sx.t, sx.vx, sx.vy)
                st.update(ax=s["ax"], ay=s["ay"], gz=s["gz"])
                if t - last_emit >= EMIT_PERIOD:
                    wait = wall0 + (t - t0) / self.speed - time.time()
                    if wait > 0:
                        time.sleep(wait)
                    self.deliver(frame(t), ("replay", 0))
                    seq += 1
                    self.frames += 1
                    last_emit = t
                    self.rec_time = t - t0
                    self.progress = k / n
            elif kind == 1:
                s = flow[i]
                kind_, _ = filt.update_flow(s["t"], s["vx_cam"], s["vy_cam"], s["quality"])
                st.update(fq=s["quality"], h=s.get("h", 0.0), fvx=s["vx_cam"], fvy=s["vy_cam"])
                if kind_ == kf_ref.ACCEPTED:
                    st.update(acc_t=s["t"], gate=False)
                    self.counts["flow_accepted"] += 1
                elif kind_ == kf_ref.GATED:
                    st["gate"] = True
                    self.counts["flow_gated"] += 1
                else:
                    self.counts["flow_skipped"] += 1
            elif kind == 2:
                s = gnss[i]
                filt.update_gnss(s["t"], s["speed"], s["speed_acc"])
                st["gnss_t"] = s["t"]
                self.counts["gnss"] += 1
            else:
                pend_zupt = True
        flush_zupt()
        if self.running:
            self.deliver(frame(evs[-1][0], rec=False), ("replay", 0))   # run ends: recording bit off
            self.frames += 1
            self.progress = 1.0

    # -- dashboard log: 50 Hz frames through the same stage logic the live dashboard uses --------
    def _run_log(self):
        rows = self.data["rows"]
        flt = FpgaFilter(self.backend)
        t0, wall0 = rows[0]["t"], time.time()
        keys = ("t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "flow_quality", "h", "ax", "ay",
                "gz", "flow_vx", "flow_vy", "net_forward")
        for i, r in enumerate(rows):
            if not self.running:
                break
            t = r["t"]
            wait = wall0 + (t - t0) / self.speed - time.time()
            if wait > 0:
                time.sleep(wait)
            f = {k: (0.0 if math.isnan(r.get(k, math.nan)) else r[k]) for k in keys}
            f.update(seq=i, status=int(r["status"]), battery=255, torch=0, fmt="replay", version=2)
            flt.process(f)
            self.deliver(f, ("replay", 0))
            self.frames += 1
            self.rec_time = t - t0
            self.progress = (i + 1) / len(rows)
