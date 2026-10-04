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
* odometer    : distance and net forward are integrated here; they freeze when the run stops
                (RECORDING bit falls, or Stop tracking) and restart from zero on the next start

It overwrites v_x, v_y, sigma_vx, sigma_vy, distance and net_forward in the frame (the phone's
originals are kept under ``frame["phone"]``) and sets the FLOW_GATED / FILTER_INIT status bits
from the FPGA's own outcomes.

If the simulation cannot keep up, frames queued behind newer ones are answered with the current
estimate without running the FPGA, and the newest frame gets the full predict (over the whole
gap) + flow update: measurement updates continue at the rate the simulation sustains and the
display never lags by more than a frame or two.

Every estimate is also sent back to the phone the frame came from (PROTOCOL.md §4), so the
phone shows the FPGA's numbers too.
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
        self.hold = False               # distance frozen: a run was stopped
        self.skipped = 0                # frames answered without running the FPGA (behind)
        self.prev_rec = None

    def reset_distance(self):
        self.dist.reset()
        self.net.reset()

    def set_hold(self, on: bool):
        """Freeze (True) or resume (False) the odometer: Stop tracking / recording stopped."""
        self.hold = on

    def process(self, f: dict, full: bool = True, skip: bool = False) -> dict:
        """Update `f` in place from the FPGA and return it.

        full=False: predict only. skip=True: run nothing on the FPGA, answer with its current
        estimate (used for frames queued behind newer ones when the simulation falls behind:
        the next processed frame's predict then covers the whole gap)."""
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

        if kf.state.initialized and t < kf.state.t - 1.0:   # clock went back: new session
            kf.reset(t)                                      # (phone restarted, log replayed)
            self.reset_distance()
            self.hold, self.prev_rec, self.last_flow = False, None, None
        status = f["status"]
        rec = bool(status & P.RECORDING)
        if self.prev_rec is not None:
            if rec and not self.prev_rec:            # a run started: odometer from zero
                self.reset_distance()
                self.hold = False
            elif self.prev_rec and not rec:          # the run stopped: hold its distance
                self.hold = True
        self.prev_rec = rec
        gated = False
        if skip:
            s = kf.state
            f["v_x"], f["v_y"] = s.vx, s.vy
            f["sigma_vx"], f["sigma_vy"] = s.sigma_vx, s.sigma_vy
            if not self.hold:                        # keep the odometer continuous in time
                self.dist.add(t, s.vx, s.vy)
                self.net.add(t, s.vx, s.vy)
            f["distance"] = self.dist.distance
            f["net_forward"] = self.net.distance
            f["status"] = (status | P.FILTER_INIT) & ~P.FLOW_GATED
            self.frames += 1
            self.skipped += 1
            return f
        kf.predict(t, f["ax"], f["ay"], f["gz"])
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
        if not self.hold:
            self.dist.add(s.t, s.vx, s.vy)
            self.net.add(s.t, s.vx, s.vy)
        f["distance"] = self.dist.distance
        f["net_forward"] = self.net.distance
        status |= P.FILTER_INIT
        f["status"] = (status | P.FLOW_GATED) if gated else (status & ~P.FLOW_GATED)
        self.frames += 1
        return f


class FpgaStage(threading.Thread):
    """Worker between the UDP receiver and the dashboard: frames in, FPGA estimates out."""

    # More than this many newer frames waiting: answer this one with the current estimate and
    # let the newest frame's predict + flow update cover the gap ("latest wins").
    MAX_BACKLOG = 1

    def __init__(self, sink, backend: str = "auto", config=None, log=print, reply=None):
        """sink(frame, addr): the dashboard. reply(payload, raw_addr): send the FPGA's estimate
        back to the phone the frame came from (None = do not return estimates)."""
        super().__init__(daemon=True, name="fpga-kf")
        self.sink, self.log, self.reply = sink, log, reply
        self.returned = 0
        self.backend = make_backend(backend, log)
        self.filter = FpgaFilter(self.backend, config)
        self.q: queue.Queue = queue.Queue()
        self.running = True
        self.t_start = time.time()
        self.busy_s = 0.0
        self.last_error = ""
        log(f"fpga: Kalman filter on the simulated FPGA ({self.backend.name})")

    def submit(self, frame: dict, addr, reply_addr=None):
        self.q.put((frame, addr, reply_addr))

    def reset_distance(self):
        self.filter.reset_distance()
        self.filter.set_hold(False)

    def hold_distance(self):
        self.filter.set_hold(True)

    def run(self):
        while self.running:
            try:
                frame, addr, reply_addr = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            t0 = time.time()
            behind = self.q.qsize() > self.MAX_BACKLOG
            ok = False
            try:
                self.filter.process(frame, skip=behind)
                ok = "phone" in frame                # a v2 frame the FPGA actually estimated
            except Exception as e:                   # noqa: BLE001 - keep the dashboard alive
                self.last_error = repr(e)
                self.log("fpga: filter error:", e)
            self.busy_s += time.time() - t0
            self.sink(frame, addr)
            if ok and self.reply is not None and reply_addr is not None:
                payload = P.encode_fpga_estimate(frame, self.backend.name)
                if payload is not None:
                    self.reply(payload, reply_addr)
                    self.returned += 1

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
            "skipped_frames": self.filter.skipped,
            "load": round(self.busy_s / up, 2),
            "fpga_clocks": clocks if clocks is not None else self.filter.kf.cycles,
            "estimates_returned": self.returned,
            "last_error": self.last_error,
        }
