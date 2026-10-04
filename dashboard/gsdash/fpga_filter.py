"""The Kalman filter on a simulated FPGA.

By default the dashboard shows the estimate the phone computed (ReferenceKF4 in Swift).
With ``--fpga`` it ignores that estimate and recomputes it from the raw sensor columns of
the 72-byte v2 frame (a_x, a_y, yaw rate, flow v_x / v_y / PSR) on the FPGA design in
``fpga/`` (``kf_cpu.v`` plus its microcode), simulated at clock level. Python is only the
host CPU of that board: it writes the inputs into the coprocessor's registers, starts a
command (predict / flow update / zero-velocity update), and reads the state back.

What the FPGA filter sees per frame:

* predict     : every frame (a_x, a_y, gyro z, dt from the frame timestamps, clamped to 0.1 s)
* flow update : when the frame carries a new flow sample (FLOW_OK set and the flow columns
                changed since the last frame); the filter skips low-PSR samples itself
* ZUPT        : while the phone's ZUPT status bit is set (the detector stays on the phone)
* GNSS        : not in the frame, so not used

It overwrites v_x, v_y, sigma_vx, sigma_vy, distance and net_forward in the frame (the phone's
originals are kept under ``frame["phone"]``) and sets the FLOW_GATED / FILTER_INIT status bits
from the FPGA's own outcomes.

If the simulation cannot keep up, frames that queue up behind the newest one are only
predicted (no measurement updates), so the display never lags by more than a few frames.
"""
from __future__ import annotations

import math
import queue
import sys
import threading
import time
from pathlib import Path

from . import protocol as P

_MODEL = Path(__file__).resolve().parents[2] / "fpga" / "model"
if str(_MODEL) not in sys.path:
    sys.path.insert(0, str(_MODEL))

import kf_isa      # noqa: E402
import kf_ref      # noqa: E402
import kf_rtl      # noqa: E402


def make_backend(kind: str = "auto", log=print):
    """'rtl' (simulated FPGA, needs a Verilog simulator), 'model' (Python model of the same
    processor, no simulator needed), or 'auto' = rtl if a simulator is found."""
    if kind in ("auto", "rtl", "iverilog"):
        try:
            return kf_rtl.RtlBackend("iverilog")
        except Exception as e:                       # noqa: BLE001
            if kind != "auto":
                raise
            log(f"fpga: no Icarus Verilog simulator ({e}); using the Python model of the processor")
    return kf_isa.SimBackend()


class FpgaFilter:
    """Frame -> estimate, using the simulated FPGA."""

    def __init__(self, backend, config: kf_ref.FilterConfig | None = None):
        self.backend = backend
        self.kf = kf_isa.FixedKF4(config, backend=backend)
        self.dist = kf_ref.DistanceIntegrator(deadband=0.05, smoothing=0.5)
        self.net = kf_ref.DistanceIntegrator(mode="forward")
        self.last_flow = None
        self.last_phone_dist = 0.0
        self.frames = 0

    def reset_distance(self):
        self.dist.reset()
        self.net.reset()

    def process(self, f: dict, full: bool = True) -> dict:
        """Update `f` in place from the FPGA and return it. full=False: predict only."""
        if f.get("ax") is None:                      # v1 frame: no raw sensor columns
            return f
        kf = self.kf
        t = f["t"]
        f["phone"] = {k: f.get(k) for k in ("v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "net_forward")}
        # the phone zeroed its own odometer (Zero / Start / its own button): follow it
        pd = f.get("distance") or 0.0
        if pd < self.last_phone_dist - 0.5:
            self.reset_distance()
        self.last_phone_dist = pd

        status = f["status"]
        kf.predict(t, f["ax"], f["ay"], f["gz"])
        gated = False
        if full:
            flow = (f["flow_vx"], f["flow_vy"], f["flow_quality"])
            if status & P.FLOW_OK and flow != self.last_flow and all(map(math.isfinite, flow)):
                kind, _ = kf.update_flow(t, *flow)
                gated = kind == kf_ref.GATED
            self.last_flow = flow
            if status & P.ZUPT:
                kf.update_zero_velocity(t)
        s = kf.state
        f["v_x"], f["v_y"] = s.vx, s.vy
        f["sigma_vx"], f["sigma_vy"] = s.sigma_vx, s.sigma_vy
        f["distance"] = self.dist.add(s.t, s.vx, s.vy)
        f["net_forward"] = self.net.add(s.t, s.vx, s.vy)
        status |= P.FILTER_INIT
        f["status"] = (status | P.FLOW_GATED) if gated else (status & ~P.FLOW_GATED)
        self.frames += 1
        return f


class FpgaStage(threading.Thread):
    """Worker between the UDP receiver and the dashboard: frames in, FPGA estimates out."""

    MAX_BACKLOG = 3

    def __init__(self, sink, backend: str = "auto", config=None, log=print):
        super().__init__(daemon=True, name="fpga-kf")
        self.sink, self.log = sink, log
        self.backend = make_backend(backend, log)
        self.filter = FpgaFilter(self.backend, config)
        self.q: queue.Queue = queue.Queue()
        self.running = True
        self.caught_up_dropped = 0
        self.t_start = time.time()
        self.busy_s = 0.0
        self.last_error = ""
        log(f"fpga: Kalman filter on the simulated FPGA ({self.backend.name})")

    def submit(self, frame: dict, addr):
        self.q.put((frame, addr))

    def reset_distance(self):
        self.filter.reset_distance()

    def run(self):
        while self.running:
            try:
                frame, addr = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()
            behind = self.q.qsize() > self.MAX_BACKLOG
            try:
                self.filter.process(frame, full=not behind)
            except Exception as e:                   # noqa: BLE001 - keep the dashboard alive
                self.last_error = repr(e)
                self.log("fpga: filter error:", e)
            if behind:
                self.caught_up_dropped += 1
            self.busy_s += time.time() - t0
            self.sink(frame, addr)

    def stop(self):
        self.running = False
        try:
            self.backend.close()
        except Exception:                            # noqa: BLE001
            pass

    def status(self) -> dict:
        up = max(time.time() - self.t_start, 1e-9)
        clocks = getattr(self.backend, "clocks", None)
        return {
            "backend": self.backend.name,
            "frames": self.filter.frames,
            "backlog": self.q.qsize(),
            "predict_only_frames": self.caught_up_dropped,
            "load": round(self.busy_s / up, 2),
            "fpga_clocks": clocks if clocks is not None else self.filter.kf.cycles,
            "last_error": self.last_error,
        }
